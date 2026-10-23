#include "pc_protocol.h"
#include "motor_host.h"
#include "motor_control.h"
#include "motor_system.h"
#include "motor_parameters.h"
#include "motor_current_loop.h"
#include "motor_speed_loop.h"
#include "motor_position_loop.h"
#include "motor_debug.h"
#include "motor_encoder.h"
#include "mt6826s.h"
#include "motor_calibration.h"
#include "motor_identify.h"
#include "motor_encoder_calibration.h"
#include "usart.h"
#include <math.h>
#include <string.h>

#define SOF0                              0xA5U // 协议帧头第一个字节
#define SOF1                              0x5AU // 协议帧头第二个字节
#define TYPE_COMMAND                      1U    // 命令帧类型
#define TYPE_RESPONSE                     2U    // 命令应答帧类型
#define TYPE_STATUS                       3U    // 周期状态帧类型
#define TYPE_TELEMETRY                    4U    // 遥测帧类型
#define TYPE_CALIBRATION                  5U    // 标定采样帧类型
#define RX_RING_SIZE                      2048U // UART ISR 到前台解析器的固定接收容量，单位字节
#define MAX_FRAME_SIZE                    (10U + PC_PROTOCOL_MAX_PAYLOAD) // 完整帧容量：8 字节头 + 载荷 + 2 字节 CRC
#define PC_FEATURE_COGGING_GAIN_10X       (1UL << 8U)                     // HELLO 能力位：COGGING_GAIN_10X
#define PC_FEATURE_CALIBRATION            (1UL << 9U)                     // HELLO 能力位：CALIBRATION
#define PC_FEATURE_FRICTION_CALIBRATION   (1UL << 10U)                    // HELLO 能力位：FRICTION_CALIBRATION
#define PC_FEATURE_FRICTION_FLASH         (1UL << 11U)                    // HELLO 能力位：FRICTION_FLASH
#define PC_FEATURE_SPEED_PI               (1UL << 12U)                    // HELLO 能力位：SPEED_PI
#define PC_FEATURE_POINT_CAL              (1UL << 13U)                    // HELLO 能力位：POINT_CAL
#define PC_FEATURE_POSITION_FRICTION      (1UL << 14U)                    // HELLO 能力位：POSITION_FRICTION
#define PC_FEATURE_CAL_REPLAY             (1UL << 15U)                    // HELLO 能力位：CAL_REPLAY
#define PC_FEATURE_POINT_CAL_256          (1UL << 16U)                    // HELLO 能力位：POINT_CAL_256
#define PC_FEATURE_POINT_QUALITY_RETRY    (1UL << 17U)                    // HELLO 能力位：POINT_QUALITY_RETRY
#define PC_FEATURE_POINT_8COUNT_30S       (1UL << 18U)                    // HELLO 能力位：POINT_8COUNT_30S
#define PC_FEATURE_POINT_SINGLE_PAIR_WAIT (1UL << 19U)                    // HELLO 能力位：POINT_SINGLE_PAIR_WAIT
#define PC_FEATURE_POINT_512_4COUNT       (1UL << 20U)                    // HELLO 能力位：POINT_512_4COUNT
#define PC_FEATURE_POINT_SETTLE_1500      (1UL << 21U) /* 历史能力位；自 1.12.12 起不在 HELLO 中声明 */
#define PC_FEATURE_POINT_3COUNT           (1UL << 22U) // HELLO 能力位：POINT_3COUNT
#define PC_FEATURE_POINT_AVERAGE          (1UL << 23U) // HELLO 能力位：POINT_AVERAGE
#define PC_FEATURE_ENCODER_AUTOCAL        (1UL << 24U) // HELLO 能力位：ENCODER_AUTOCAL
#define PC_FEATURE_POINT_2COUNT           (1UL << 25U) // HELLO 能力位：POINT_2COUNT
#define PC_FEATURE_POINT_AVERAGE_POSITION (1UL << 26U) // HELLO 能力位：POINT_AVERAGE_POSITION

enum {
    CMD_HELLO = 0x01,
    CMD_GET_STATUS = 0x02,
    CMD_CLAIM = 0x03,
    CMD_RELEASE = 0x04,
    CMD_HEARTBEAT = 0x05,
    CMD_START = 0x10,
    CMD_STOP = 0x11,
    CMD_CLEAR_FAULT = 0x12,
    CMD_IDENTIFY = 0x13,
    CMD_SET_MODE = 0x20,
    CMD_SET_IQ = 0x21,
    CMD_SET_IQ_LIMIT = 0x22,
    CMD_SET_HAPTIC = 0x23,
    CMD_GET_PARAMS = 0x24,
    CMD_SET_FRICTION = 0x25,
    CMD_GET_FRICTION = 0x26,
    CMD_SET_SPEED = 0x27,
    CMD_SET_COGGING_CONFIG = 0x28,
    CMD_GET_COGGING_CONFIG = 0x29,
    CMD_COGGING_TABLE_BEGIN = 0x2A,
    CMD_COGGING_TABLE_CHUNK = 0x2B,
    CMD_COGGING_TABLE_COMMIT = 0x2C,
    CMD_GET_COGGING_TABLE_CHUNK = 0x2D,
    CMD_SAVE_COGGING = 0x2E,
    CMD_FORCE_DRAG_250 = 0x30,
    CMD_CAL_INFO = 0x31,
    CMD_CAL_START = 0x32,
    CMD_CAL_ABORT = 0x33,
    CMD_CAL_STATUS = 0x34,
    CMD_SAVE_FRICTION = 0x35,
    CMD_SET_SPEED_PI = 0x36,
    CMD_GET_SPEED_PI = 0x37,
    CMD_CAL_REPLAY = 0x38,
    CMD_ENCODER_CAL_START = 0x39,
    CMD_ENCODER_CAL_STATUS = 0x3A
};

/* UART 接收 ISR 生产 s_ring，前台协议任务消费并执行命令。
 * s_tx 由 DMA 占用至发送回调；应答与流数据使用独立固定槽，发布/启动保持原临界区。
 * s_token_* 属于主机连接，s_last_request/response 属于重放缓存，周期时间戳单位 ms。
 */
static volatile uint8_t s_ring[RX_RING_SIZE];
static volatile uint16_t s_head, s_tail;
static uint8_t s_parse[MAX_FRAME_SIZE];
static uint16_t s_parse_len;
static uint8_t s_tx[MAX_FRAME_SIZE];
static uint8_t s_tx_pending[MAX_FRAME_SIZE];
static uint8_t s_tx_stream[MAX_FRAME_SIZE];
static volatile uint16_t s_tx_pending_len, s_tx_stream_len;
static volatile uint8_t s_tx_busy;
static uint32_t s_token;
static uint32_t s_token_counter;
static uint32_t s_crc_errors, s_overflows, s_telemetry_drops;
static uint16_t s_telemetry_seq, s_status_seq;
static uint32_t s_sample_seq, s_last_status_ms, s_last_telemetry_ms;
static uint8_t s_last_request[PC_PROTOCOL_MAX_PAYLOAD];
static uint16_t s_last_request_len, s_last_request_seq;
static uint8_t s_last_response[PC_PROTOCOL_MAX_PAYLOAD];
static uint16_t s_last_response_len;

// 计算协议 CRC-16/CCITT-FALSE：初值 0xFFFF，多项式 0x1021，不包含帧头字节。
static uint16_t crc16(const uint8_t *p, uint16_t n)
{
    static const uint16_t nibble[16] = {0x0000U, 0x1021U, 0x2042U, 0x3063U, 0x4084U, 0x50A5U, 0x60C6U, 0x70E7U,
                                        0x8108U, 0x9129U, 0xA14AU, 0xB16BU, 0xC18CU, 0xD1ADU, 0xE1CEU, 0xF1EFU};
    uint16_t crc = 0xFFFFU;
    while (n-- != 0U) {
        crc ^= (uint16_t)(*p++) << 8U;
        crc = (uint16_t)((crc << 4U) ^ nibble[crc >> 12U]);
        crc = (uint16_t)((crc << 4U) ^ nibble[crc >> 12U]);
    }
    return crc;
}

/* HELLO 和 STOP 沿用可重复执行语义；与 HEARTBEAT 一同排除于重放缓存。
 * 新连接重启序号时可重新执行握手和停机，不命中上一连接的缓存。 */
static uint8_t command_uses_replay_cache(uint8_t cmd)
{
    return (cmd != CMD_HEARTBEAT) && (cmd != CMD_HELLO) && (cmd != CMD_STOP);
}

// 把 16 位整数写成协议小端字节；调用方负责提供至少 2 字节输出空间。
static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8U);
}

// 把 32 位整数写成协议小端字节；调用方负责提供至少 4 字节输出空间。
static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8U);
    p[2] = (uint8_t)(v >> 16U);
    p[3] = (uint8_t)(v >> 24U);
}

// 按小端读取 16 位字段；帧长度须在调用前检查。
static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8U);
}

// 按小端读取有符号 16 位字段，保留补码解释方式。
static int16_t get_i16(const uint8_t *p)
{
    return (int16_t)get_u16(p);
}

// 按小端读取 32 位字段；帧长度须在调用前检查。
static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) | ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U);
}

// 按既有 IEEE 754 字节格式读取 float，避免未对齐指针解引用。
static float get_f32(const uint8_t *p)
{
    float v;
    memcpy(&v, p, 4U);
    return v;
}

// 按既有 IEEE 754 字节格式写 float；输入验证由对应命令负责。
static void put_f32(uint8_t *p, float v)
{
    memcpy(p, &v, 4U);
}

// 构造版本、类型、序号、长度和 CRC；调用方负责完整帧容量及合法载荷长度。
static uint16_t make_frame(uint8_t *dst, uint8_t type, uint16_t seq, const uint8_t *payload, uint16_t length)
{
    dst[0] = SOF0;
    dst[1] = SOF1;
    dst[2] = PC_PROTOCOL_VERSION;
    dst[3] = type;
    put_u16(&dst[4], seq);
    put_u16(&dst[6], length);
    if (length != 0U) {
        memcpy(&dst[8], payload, length);
    }
    put_u16(&dst[8U + length], crc16(&dst[2], (uint16_t)(6U + length)));
    return (uint16_t)(10U + length);
}

/* HAL 无法启动 DMA 时保留排队数据，由后续前台任务按原规则重试。 */
static void start_queued_tx(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!s_tx_busy && huart2.gState == HAL_UART_STATE_READY) {
        uint16_t n = s_tx_pending_len ? s_tx_pending_len : s_tx_stream_len;
        if (n) {
            uint8_t priority = s_tx_pending_len != 0U;
            memcpy(s_tx, priority ? s_tx_pending : s_tx_stream, n);
            s_tx_busy = 1U;
            if (HAL_UART_Transmit_DMA(&huart2, s_tx, n) == HAL_OK) {
                if (priority) {
                    s_tx_pending_len = 0U;
                } else {
                    s_tx_stream_len = 0U;
                }
            } else {
                s_tx_busy = 0U;
            }
        }
    }
    if (!mask) {
        __enable_irq();
    }
}

// 前台构造并发布固定容量帧，按原规则区分应答优先槽与数据流槽；成功返回 1。
static uint8_t queue_frame(uint8_t type, uint16_t seq, const uint8_t *payload, uint16_t length, uint8_t priority)
{
    uint8_t frame[MAX_FRAME_SIZE];
    uint16_t n = make_frame(frame, type, seq, payload, length);
    uint8_t accepted = 0U;
    /* 构帧和 CRC 计算可与 DMA 并行；发布和启动沿用临界区，
     * 防止 UART 完成 ISR 干扰；应答使用独立优先槽。 */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if ((s_tx_busy == 0U) && (huart2.gState == HAL_UART_STATE_READY)) {
        memcpy(s_tx, frame, n);
        s_tx_busy = 1U;
        if (HAL_UART_Transmit_DMA(&huart2, s_tx, n) == HAL_OK) {
            accepted = 1U;
        } else {
            s_tx_busy = 0U;
        }
    } else if ((priority != 0U) && (s_tx_pending_len == 0U)) {
        memcpy(s_tx_pending, frame, n);
        s_tx_pending_len = n;
        accepted = 1U;
    } else if ((priority == 0U) && (s_tx_stream_len == 0U) &&
               (type == TYPE_STATUS || type == TYPE_CALIBRATION ||
                (type == TYPE_TELEMETRY && (motor_calibration_is_active() || motor_calibration_has_samples())))) {
        memcpy(s_tx_stream, frame, n);
        s_tx_stream_len = n;
        accepted = 1U;
    }
    if (!mask) {
        __enable_irq();
    }
    return accepted;
}

// 按既有字段偏移构造状态载荷，保持状态码、位宽和小端格式。
static uint16_t build_status(uint8_t *p)
{
    motor_control_snapshot_t s;
    motor_system_get_control_snapshot(&s);
    put_u32(&p[0], HAL_GetTick());
    p[4] = (uint8_t)s.state;
    p[5] = (uint8_t)s.mode;
    p[6] = (uint8_t)s.source;
    p[7] = (uint8_t)s.owner;
    p[8] = s.run_requested;
    p[9] = (uint8_t)motor_parameters_get_status();
    p[10] = motor_encoder_is_data_fresh(2U);
    p[11] = mt6826s_get_status();
    uint32_t faults = 0U;
    if (p[10] == 0U) {
        faults |= 1U;
    }
    if (p[9] == (uint8_t)MOTOR_PARAMETERS_ERROR) {
        faults |= 2U;
    }
    put_u32(&p[12], faults);
    p[16] = 0U;
    p[17] = p[18] = p[19] = 0U;
    put_u32(&p[20], s.config_revision);
    put_f32(&p[24], s.iq_limit_a);
    put_u32(&p[28], s_crc_errors);
    put_u32(&p[32], s_overflows);
    put_u32(&p[36], s_telemetry_drops);
    return 40U;
}

// 构造命令应答并按原规则维护重放缓存；保留状态码和应答长度。
static void response(uint16_t seq, uint8_t cmd, motor_command_result_t result, const uint8_t *body, uint16_t body_len)
{
    uint8_t p[PC_PROTOCOL_MAX_PAYLOAD];
    motor_control_snapshot_t s;
    motor_system_get_control_snapshot(&s);
    p[0] = cmd;
    p[1] = (uint8_t)result;
    put_u32(&p[2], s_token);
    put_u32(&p[6], s.config_revision);
    if (body_len != 0U) {
        memcpy(&p[10], body, body_len);
    }
    uint16_t response_len = (uint16_t)(10U + body_len);
    if (command_uses_replay_cache(cmd)) {
        s_last_response_len = response_len;
        memcpy(s_last_response, p, response_len);
    }
    (void)queue_frame(TYPE_RESPONSE, seq, p, response_len, 1U);
}

// 检查主机令牌和控制权；令牌不能代替各命令的状态检查。
static uint8_t token_ok(uint32_t token)
{
    return (s_token != 0U) && (token == s_token) && (motor_system_get_control_owner() == MOTOR_OWNER_HOST);
}

// 按原字段布局序列化标定状态，返回载荷字节数。
static uint16_t build_cal_status(uint8_t *p)
{
    motor_cal_status_t s;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    motor_calibration_get_status(&s);
    if (!mask) {
        __enable_irq();
    }
    memset(p, 0, 32U);
    put_u32(p, s.session);
    put_u32(p + 4, s.produced);
    put_u32(p + 8, s.dropped);
    p[12] = s.phase;
    p[13] = s.reason;
    p[14] = s.speed_index;
    p[15] = s.reverse;
    put_u16(p + 16, s.accepted_turns);
    put_u16(p + 18, s.rejected_turns);
    p[20] = s.passed_mask;
    p[21] = s.mode;
    memcpy(p + 22, s.speed_reasons, 3U);
    put_f32(p + 28, s.target_rpm);
    return 32U;
}

// 按原字段布局序列化编码器自校准状态，返回载荷字节数。
static uint16_t build_encoder_cal_status(uint8_t *p)
{
    motor_encoder_cal_status_t s;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    motor_encoder_cal_get_status(&s);
    if (!mask) {
        __enable_irq();
    }
    memset(p, 0, 24U);
    p[0] = 1U;
    p[1] = s.phase;
    p[2] = s.reason;
    p[3] = s.flags;
    p[4] = s.chip_status;
    p[5] = 4U;
    put_u32(p + 8, s.elapsed_ms);
    put_u32(p + 12, s.calibrated_counts);
    put_f32(p + 16, s.measured_rpm);
    put_f32(p + 20, 250.0f);
    return 24U;
}

// 构造标定能力、质量门限和编译信息，保留协商版本与字段偏移。
static uint16_t build_cal_info(uint8_t *p)
{
    motor_control_snapshot_t s;
    motor_friction_config_t f;
    motor_cogging_config_t c;
    motor_system_get_control_snapshot(&s);
    motor_system_get_friction_config(&f);
    motor_system_get_cogging_config(&c);
    motor_identified_params_t m = motor_identify_get_result();
    put_u16(p, MOTOR_CAL_SCHEMA);
    put_u16(p + 2, sizeof(motor_cal_sample_t));
    put_u32(p + 4, s.config_revision);
    put_u16(p + 8, m.pole_pairs);
    p[10] = (uint8_t)m.uvw_dir;
    p[11] = (uint8_t)(f.enabled | (c.enabled << 1U));
    float v[32] = {s.iq_limit_a,
                   fminf(s.iq_limit_a, .20f),
                   speed_pid.kp,
                   speed_pid.ki,
                   speed_pid.kd,
                   speed_est.filter_alpha,
                   MOTOR_ROTOR_INERTIA_KG_M2,
                   MOTOR_TORQUE_CONSTANT_NM_PER_A,
                   m.zero_angle_offset,
                   m.resistance,
                   m.inductance,
                   f.coulomb_iq_a,
                   f.viscous_iq_a_per_rpm,
                   f.smooth_speed_rpm,
                   f.max_iq_a,
                   c.gain,
                   c.max_iq_a,
                   c.fade_start_rpm,
                   c.fade_end_rpm,
                   (float)c.phase_offset_counts,
                   g_foc_state.pi_d.kp,
                   g_foc_state.pi_d.ki,
                   g_foc_state.pi_d.kd,
                   g_foc_state.pi_q.kp,
                   g_foc_state.pi_q.ki,
                   g_foc_state.pi_q.kd,
                   g_foc_state.params.bias_u_volts,
                   g_foc_state.params.bias_w_volts,
                   g_foc_state.params.shunt_ohms,
                   g_foc_state.params.gain,
                   g_foc_state.params.vref_volts,
                   g_foc_state.params.adc_max};
    memcpy(p + 12, v, sizeof(v));
    p[140] = (uint8_t)s.mode;
    p[141] = (uint8_t)s.source;
    put_u16(p + 142, 7U);
    memset(p + 144, 0, 24U);
    memcpy(p + 144, __DATE__ " " __TIME__, sizeof(__DATE__ " " __TIME__));
    put_u32(p + 168, motor_cogging_get_table_revision());
    put_u32(p + 172, motor_cogging_get_active_table_crc());
    if (motor_calibration_is_point_mode()) {
        put_u16(p, motor_calibration_point_schema());
        put_u16(p + 2, 68U);
        put_f32(p + 176, g_pi_pos.kp);
        put_f32(p + 180, g_pi_pos.ki);
        put_f32(p + 184, g_pi_pos.kd);
        if (motor_calibration_point_repeats() != 3U || motor_calibration_point_retry_until_good() ||
            motor_calibration_point_policy()) {
            p[188] = motor_calibration_point_repeats();
            p[189] = motor_calibration_point_retry_until_good();
            put_u16(p + 190, 0U);
            if (motor_calibration_point_policy()) {
                p[192] = motor_calibration_point_policy();
                p[193] = 0U;
                put_u16(p + 194, 0U);
                return 196U;
            }
            return 192U;
        }
        return 188U;
    }
    return 176U;
}

// 按当前采集/待发送状态判断命令是否允许，保持原命令白名单。
static uint8_t cal_command_allowed(uint8_t cmd)
{
    return cmd == CMD_HEARTBEAT || cmd == CMD_RELEASE || cmd == CMD_GET_PARAMS || cmd == CMD_GET_FRICTION ||
           cmd == CMD_GET_COGGING_CONFIG || cmd == CMD_GET_COGGING_TABLE_CHUNK || cmd == CMD_CAL_INFO ||
           cmd == CMD_CAL_STATUS || cmd == CMD_CAL_ABORT || cmd == CMD_GET_SPEED_PI || cmd == CMD_CAL_REPLAY;
}

_Static_assert(sizeof(motor_cal_sample_t) == 64U, "Calibration wire sample layout changed");

// 编码 12 字节批头和 64/68 字节记录；发送消费与历史回放由调用方决定。
static uint16_t build_calibration_batch(uint8_t *out, const motor_cal_status_t *status,
                                       const motor_cal_sample_t *samples, const float *targets, uint8_t count)
{
    uint8_t point = status->mode == 5U;
    uint16_t size = point ? 68U : 64U;
    put_u16(out, point ? motor_calibration_point_schema() : MOTOR_CAL_SCHEMA);
    put_u32(out + 2, status->session);
    out[6] = count;
    out[7] = 0U;
    put_u32(out + 8, status->dropped);
    for (uint8_t i = 0; i < count; i++) {
        memcpy(out + 12 + i * size, &samples[i], 64U);
        if (point) {
            put_f32(out + 12 + i * size + 64, targets[i]);
        }
    }
    return (uint16_t)(12U + count * size);
}

// 处理连接、查询和无令牌停机命令，保持原有权限语义。
static uint8_t handle_connection_command(uint8_t cmd, uint16_t bn, uint8_t *out,
                                         uint16_t *out_n, motor_command_result_t *result)
{
    motor_command_result_t r = MOTOR_CMD_OK;
    switch (cmd) {
    case CMD_HELLO:
    {
        if (bn != 0U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            out[0] = 1U;
            out[1] = 13U;
            out[2] = 3U;
            put_u16(&out[3], 7U);
            put_u32(&out[5], HAL_GetUIDw0());
            put_u32(&out[9], HAL_GetUIDw1());
            put_u32(&out[13], HAL_GetUIDw2());
            put_u32(&out[17], 0xFFU);
            put_u32(&out[21], 0xFFU | PC_FEATURE_COGGING_GAIN_10X | PC_FEATURE_CALIBRATION |
                                  PC_FEATURE_FRICTION_CALIBRATION | PC_FEATURE_FRICTION_FLASH | PC_FEATURE_SPEED_PI |
                                  PC_FEATURE_POINT_CAL | PC_FEATURE_POSITION_FRICTION | PC_FEATURE_CAL_REPLAY |
                                  PC_FEATURE_POINT_CAL_256 | PC_FEATURE_POINT_QUALITY_RETRY |
                                  PC_FEATURE_POINT_8COUNT_30S | PC_FEATURE_POINT_SINGLE_PAIR_WAIT |
                                  PC_FEATURE_POINT_512_4COUNT | PC_FEATURE_POINT_3COUNT | PC_FEATURE_POINT_AVERAGE |
                                  PC_FEATURE_ENCODER_AUTOCAL | PC_FEATURE_POINT_2COUNT |
                                  PC_FEATURE_POINT_AVERAGE_POSITION);
            put_f32(&out[25], MOTOR_TORQUE_CURRENT_LIMIT_A);
            put_f32(&out[29], MOTOR_HOST_DEFAULT_IQ_LIMIT_A);
            put_u16(&out[33], PC_PROTOCOL_MAX_PAYLOAD);
            put_u16(&out[35], MOTOR_HOST_HEARTBEAT_TIMEOUT_MS);
            *out_n = 37U;
        }
        break;
    }
    case CMD_GET_STATUS:
    {
        if (bn != 0U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            *out_n = build_status(out);
        }
        break;
    }
    case CMD_ENCODER_CAL_STATUS:
    {
        if (bn != 0U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            *out_n = build_encoder_cal_status(out);
        }
        break;
    }
    case CMD_CLAIM:
    {
        if (bn != 0U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            r = motor_system_claim_host();
            if (r == MOTOR_CMD_OK) {
                ++s_token_counter;
                if (s_token_counter == 0U) {
                    ++s_token_counter;
                }
                s_token = s_token_counter;
                motor_system_host_heartbeat();
                put_u32(out, s_token);
                *out_n = 4U;
            }
        }
        break;
    }
    case CMD_STOP:
    {
        motor_system_release_host();
        s_token = 0U;
        break;
    }
    case CMD_CLEAR_FAULT:
    {
        r = motor_system_clear_fault() ? MOTOR_CMD_OK : MOTOR_CMD_NOT_READY;
        break;
    }
    default: return 0U;
    }
    *result = r;
    return 1U;
}

// 处理标定管理与编码器校准命令，保留既有校验顺序与应答字段。
static motor_command_result_t handle_calibration_command(uint8_t cmd, const uint8_t *b,
                                                            uint16_t bn, uint8_t *out, uint16_t *out_n)
{
    motor_command_result_t r = MOTOR_CMD_OK;
    switch (cmd) {
    case CMD_CAL_INFO:
    {
        if (bn != 0U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            *out_n = build_cal_info(out);
        }
        break;
    }
    case CMD_ENCODER_CAL_START:
    {
        if (bn != 0U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            r = motor_system_host_start_encoder_calibration();
            *out_n = build_encoder_cal_status(out);
        }
        break;
    }
    case CMD_CAL_START:
    {
        uint8_t mode, repeats, retry_until_good, policy;
        uint16_t points;
        if (bn != 1U && bn != 3U && bn != 5U && bn != 6U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else if (!motor_calibration_decode_start_policy(b, bn, &mode, &points, &repeats, &retry_until_good,
                                                          &policy)) {
            r = MOTOR_CMD_INVALID_VALUE;
        } else {
            r = motor_system_host_start_calibration(mode, points, repeats, retry_until_good, policy);
            *out_n = build_cal_status(out);
        }
        break;
    }
    case CMD_CAL_ABORT:
    {
        if (bn != 0U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            motor_system_abort_calibration();
            *out_n = build_cal_status(out);
        }
        break;
    }
    case CMD_CAL_STATUS:
    {
        if (bn != 0U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            *out_n = build_cal_status(out);
        }
        break;
    }
    default: return MOTOR_CMD_UNSUPPORTED;
    }
    return r;
}

// 处理标定历史回放命令，保留既有校验顺序与应答字段。
static motor_command_result_t handle_calibration_replay_command(uint8_t cmd, const uint8_t *b,
                                                            uint16_t bn, uint8_t *out, uint16_t *out_n)
{
    motor_command_result_t r = MOTOR_CMD_OK;
    switch (cmd) {
    case CMD_CAL_REPLAY:
    {
        if (bn != 9U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            motor_cal_sample_t records[3];
            float targets[3];
            motor_cal_status_t status;
            uint32_t mask = __get_PRIMASK();
            __disable_irq();
            motor_calibration_get_status(&status);
            uint8_t count =
                get_u32(b) == status.session ? motor_calibration_replay(get_u32(b + 4), b[8], records, targets) : 0U;
            if (!mask) {
                __enable_irq();
            }
            if (!count) {
                r = MOTOR_CMD_NOT_READY;
            } else {
                *out_n = build_calibration_batch(out, &status, records, targets, count);
            }
        }
        break;
    }
    default: return MOTOR_CMD_UNSUPPORTED;
    }
    return r;
}

// 处理启停与辨识命令，保留既有校验顺序与应答字段。
static motor_command_result_t handle_control_command(uint8_t cmd, const uint8_t *b,
                                                            uint16_t bn, uint8_t *out, uint16_t *out_n)
{
    motor_command_result_t r = MOTOR_CMD_OK;
    switch (cmd) {
    case CMD_IDENTIFY:
    {
        if (bn != 0U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else if (motor_encoder_is_data_fresh(2U) == 0U) {
            r = MOTOR_CMD_NOT_READY;
        } else {
            r = motor_system_identify_and_save() ? MOTOR_CMD_OK : MOTOR_CMD_BUSY;
        }
        break;
    }
    case CMD_RELEASE:
    {
        motor_system_release_host();
        s_token = 0U;
        break;
    }
    case CMD_HEARTBEAT:
    {
        motor_system_host_heartbeat();
        *out_n = build_status(out);
        break;
    }
    case CMD_START:
    {
        r = motor_system_host_start();
        break;
    }
    case CMD_FORCE_DRAG_250:
    {
        if (bn != 0U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            r = motor_system_host_start_force_drag250();
        }
        break;
    }
    default: return MOTOR_CMD_UNSUPPORTED;
    }
    return r;
}

// 处理模式与主机目标命令，保留既有校验顺序与应答字段。
static motor_command_result_t handle_targets_command(uint8_t cmd, const uint8_t *b,
                                                            uint16_t bn, uint8_t *out, uint16_t *out_n)
{
    motor_command_result_t r = MOTOR_CMD_OK;
    switch (cmd) {
    case CMD_SET_MODE:
    {
        if (bn != 2U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            r = motor_system_host_set_mode(b[0], b[1]);
        }
        break;
    }
    case CMD_SET_IQ:
    {
        float accepted = 0.0f;
        float slew = MOTOR_HOST_IQ_SLEW_A_PER_S;
        if ((bn != 4U) && (bn != 8U)) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            if (bn == 8U) {
                slew = get_f32(&b[4]);
            }
            r = motor_system_host_set_iq(get_f32(b), slew);
            if (r == MOTOR_CMD_OK) {
                accepted = motor_system_get_last_accepted_iq();
            }
            put_f32(out, accepted);
            *out_n = 4U;
        }
        break;
    }
    case CMD_SET_IQ_LIMIT:
    {
        if (bn != 4U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            r = motor_system_host_set_iq_limit(get_f32(b));
            put_f32(out, motor_system_get_control_owner() == MOTOR_OWNER_HOST ? get_f32(b) : 0.0f);
            *out_n = 4U;
        }
        break;
    }
    case CMD_SET_SPEED:
    {
        float accepted = 0.0f;
        if ((bn != 4U) && (bn != 8U)) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            float slew = (bn == 8U) ? get_f32(&b[4]) : MOTOR_HOST_SPEED_SLEW_RPM_PER_S;
            r = motor_system_host_set_speed(get_f32(b), slew);
            if (r == MOTOR_CMD_OK) {
                accepted = motor_system_get_last_accepted_speed();
            }
            put_f32(out, accepted);
            put_f32(&out[4], slew);
            *out_n = 8U;
        }
        break;
    }
    default: return MOTOR_CMD_UNSUPPORTED;
    }
    return r;
}

// 处理触觉配置与参数查询命令，保留既有校验顺序与应答字段。
static motor_command_result_t handle_haptic_command(uint8_t cmd, const uint8_t *b,
                                                            uint16_t bn, uint8_t *out, uint16_t *out_n)
{
    motor_command_result_t r = MOTOR_CMD_OK;
    switch (cmd) {
    case CMD_SET_HAPTIC:
    {
        motor_haptic_params_t hp;
        if (bn != 22U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            hp.spring_k_a_per_rad = get_f32(&b[0]);
            hp.damping_b_a_per_rad_s = get_f32(&b[4]);
            hp.detent_k_a_per_rad = get_f32(&b[8]);
            hp.limit_k_a_per_rad = get_f32(&b[12]);
            hp.limit_half_range_deg = get_f32(&b[16]);
            hp.detent_count = get_u16(&b[20]);
            r = motor_system_host_set_haptic_params(hp.spring_k_a_per_rad, hp.damping_b_a_per_rad_s,
                                                    hp.detent_k_a_per_rad, hp.limit_k_a_per_rad,
                                                    hp.limit_half_range_deg, hp.detent_count);
        }
        break;
    }
    case CMD_GET_PARAMS:
    {
        motor_control_snapshot_t cs;
        motor_haptic_params_t hp;
        motor_system_get_control_snapshot(&cs);
        motor_system_get_haptic_params(&hp);
        out[0] = (uint8_t)cs.mode;
        out[1] = (uint8_t)cs.source;
        put_f32(&out[2], cs.iq_limit_a);
        put_f32(&out[6], MOTOR_HOST_IQ_SLEW_A_PER_S);
        put_f32(&out[10], hp.spring_k_a_per_rad);
        put_f32(&out[14], hp.damping_b_a_per_rad_s);
        put_f32(&out[18], hp.detent_k_a_per_rad);
        put_f32(&out[22], hp.limit_k_a_per_rad);
        put_f32(&out[26], hp.limit_half_range_deg);
        put_u16(&out[30], hp.detent_count);
        *out_n = 32U;
        break;
    }
    default: return MOTOR_CMD_UNSUPPORTED;
    }
    return r;
}

// 处理摩擦补偿配置命令，保留既有校验顺序与应答字段。
static motor_command_result_t handle_friction_command(uint8_t cmd, const uint8_t *b,
                                                            uint16_t bn, uint8_t *out, uint16_t *out_n)
{
    motor_command_result_t r = MOTOR_CMD_OK;
    switch (cmd) {
    case CMD_SET_FRICTION:
    case CMD_SAVE_FRICTION:
    {
        motor_friction_config_t fc;
        if (bn != 17U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            fc.enabled = b[0];
            fc.coulomb_iq_a = get_f32(&b[1]);
            fc.viscous_iq_a_per_rpm = get_f32(&b[5]);
            fc.smooth_speed_rpm = get_f32(&b[9]);
            fc.max_iq_a = get_f32(&b[13]);
            r = cmd == CMD_SAVE_FRICTION
                    ? motor_system_host_save_friction_config(fc.enabled, fc.coulomb_iq_a, fc.viscous_iq_a_per_rpm,
                                                             fc.smooth_speed_rpm, fc.max_iq_a)
                    : motor_system_host_set_friction_config(fc.enabled, fc.coulomb_iq_a, fc.viscous_iq_a_per_rpm,
                                                            fc.smooth_speed_rpm, fc.max_iq_a);
            if (r == MOTOR_CMD_OK) {
                memcpy(out, b, 17U);
                *out_n = 17U;
            }
        }
        break;
    }
    case CMD_GET_FRICTION:
    {
        motor_friction_config_t fc;
        if (bn != 0U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            motor_system_get_friction_config(&fc);
            out[0] = fc.enabled;
            put_f32(&out[1], fc.coulomb_iq_a);
            put_f32(&out[5], fc.viscous_iq_a_per_rpm);
            put_f32(&out[9], fc.smooth_speed_rpm);
            put_f32(&out[13], fc.max_iq_a);
            *out_n = 17U;
        }
        break;
    }
    default: return MOTOR_CMD_UNSUPPORTED;
    }
    return r;
}

// 处理速度 PI 配置命令，保留既有校验顺序与应答字段。
static motor_command_result_t handle_speed_pi_command(uint8_t cmd, const uint8_t *b,
                                                            uint16_t bn, uint8_t *out, uint16_t *out_n)
{
    motor_command_result_t r = MOTOR_CMD_OK;
    switch (cmd) {
    case CMD_SET_SPEED_PI:
    case CMD_GET_SPEED_PI:
    {
        if (bn != (cmd == CMD_SET_SPEED_PI ? 8U : 0U)) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            if (cmd == CMD_SET_SPEED_PI) {
                r = motor_system_host_set_speed_pi(get_f32(b), get_f32(b + 4));
            }
            if (r == MOTOR_CMD_OK) {
                put_f32(out, speed_pid.kp);
                put_f32(out + 4, speed_pid.ki);
                put_f32(out + 8, speed_pid.kd);
                *out_n = 12U;
            }
        }
        break;
    }
    default: return MOTOR_CMD_UNSUPPORTED;
    }
    return r;
}

// 处理齿槽补偿配置命令，保留既有校验顺序与应答字段。
static motor_command_result_t handle_cogging_config_command(uint8_t cmd, const uint8_t *b,
                                                            uint16_t bn, uint8_t *out, uint16_t *out_n)
{
    motor_command_result_t r = MOTOR_CMD_OK;
    switch (cmd) {
    case CMD_SET_COGGING_CONFIG:
    {
        motor_cogging_config_t cc;
        if (bn != 19U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            cc.enabled = b[0];
            cc.gain = get_f32(&b[1]);
            cc.max_iq_a = get_f32(&b[5]);
            cc.fade_start_rpm = get_f32(&b[9]);
            cc.fade_end_rpm = get_f32(&b[13]);
            cc.phase_offset_counts = get_u16(&b[17]);
            r = motor_system_host_set_cogging_config(cc.enabled, cc.gain, cc.max_iq_a, cc.fade_start_rpm,
                                                     cc.fade_end_rpm, cc.phase_offset_counts);
            if (r == MOTOR_CMD_OK) {
                memcpy(out, b, 19U);
                *out_n = 19U;
            }
        }
        break;
    }
    case CMD_GET_COGGING_CONFIG:
    {
        motor_cogging_config_t cc;
        motor_control_snapshot_t cs;
        if (bn != 0U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            motor_system_get_cogging_config(&cc);
            motor_system_get_control_snapshot(&cs);
            out[0] = cc.enabled;
            put_f32(&out[1], cc.gain);
            put_f32(&out[5], cc.max_iq_a);
            put_f32(&out[9], cc.fade_start_rpm);
            put_f32(&out[13], cc.fade_end_rpm);
            put_u16(&out[17], cc.phase_offset_counts);
            put_u32(&out[19], cs.cogging_table_revision);
            put_u32(&out[23], cs.cogging_table_crc);
            out[27] = motor_cogging_is_persisted();
            *out_n = 28U;
        }
        break;
    }
    default: return MOTOR_CMD_UNSUPPORTED;
    }
    return r;
}

// 处理齿槽表事务与保存命令，保留既有校验顺序与应答字段。
static motor_command_result_t handle_cogging_table_command(uint8_t cmd, const uint8_t *b,
                                                            uint16_t bn, uint8_t *out, uint16_t *out_n)
{
    motor_command_result_t r = MOTOR_CMD_OK;
    switch (cmd) {
    case CMD_COGGING_TABLE_BEGIN:
    {
        uint16_t transaction = 0U;
        if (bn != 6U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            r = motor_system_host_begin_cogging_table(get_u16(&b[0]), get_u32(&b[2]));
            if (r == MOTOR_CMD_OK) {
                transaction = motor_cogging_get_transaction_id();
            }
            if (r == MOTOR_CMD_OK) {
                put_u16(out, transaction);
                *out_n = 2U;
            }
        }
        break;
    }
    case CMD_COGGING_TABLE_CHUNK:
    {
        int16_t values[MOTOR_COGGING_CHUNK_MAX_POINTS];
        if ((bn < 5U) || (b[4] == 0U) || (b[4] > MOTOR_COGGING_CHUNK_MAX_POINTS) ||
            (bn != (uint16_t)(5U + 2U * b[4]))) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            for (uint16_t i = 0U; i < b[4]; ++i) {
                values[i] = get_i16(&b[5U + 2U * i]);
            }
            r = motor_system_host_write_cogging_chunk(get_u16(&b[0]), get_u16(&b[2]), b[4], values);
            if (r == MOTOR_CMD_OK) {
                put_u16(&out[0], get_u16(&b[2]));
                out[2] = b[4];
                *out_n = 3U;
            }
        }
        break;
    }
    case CMD_COGGING_TABLE_COMMIT:
    {
        motor_control_snapshot_t cs;
        if (bn != 2U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            r = motor_system_host_commit_cogging_table(get_u16(b));
            if (r == MOTOR_CMD_OK) {
                motor_system_get_control_snapshot(&cs);
                put_u32(&out[0], cs.cogging_table_revision);
                put_u32(&out[4], cs.cogging_table_crc);
                *out_n = 8U;
            }
        }
        break;
    }
    case CMD_GET_COGGING_TABLE_CHUNK:
    {
        int16_t values[MOTOR_COGGING_CHUNK_MAX_POINTS];
        if ((bn != 3U) || (b[2] == 0U) || (b[2] > MOTOR_COGGING_CHUNK_MAX_POINTS)) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            r = motor_system_host_read_cogging_chunk(get_u16(&b[0]), b[2], values);
            if (r == MOTOR_CMD_OK) {
                put_u16(&out[0], get_u16(&b[0]));
                out[2] = b[2];
                for (uint16_t i = 0U; i < b[2]; ++i) {
                    put_u16(&out[3U + 2U * i], (uint16_t)values[i]);
                }
                *out_n = (uint16_t)(3U + 2U * b[2]);
            }
        }
        break;
    }
    case CMD_SAVE_COGGING:
    {
        motor_control_snapshot_t cs;
        if (bn != 0U) {
            r = MOTOR_CMD_INVALID_LENGTH;
        } else {
            r = motor_system_host_save_cogging();
            motor_system_get_control_snapshot(&cs);
            if (r == MOTOR_CMD_OK) {
                put_u32(&out[0], cs.cogging_table_revision);
                put_u32(&out[4], cs.cogging_table_crc);
                out[8] = motor_cogging_is_persisted();
                *out_n = 9U;
            }
        }
        break;
    }
    default: return MOTOR_CMD_UNSUPPORTED;
    }
    return r;
}

// 仅在令牌和校准门控通过后分发业务；未知命令维持原错误码。
static motor_command_result_t handle_owned_command(uint8_t cmd, const uint8_t *b, uint16_t bn,
                                                   uint8_t *out, uint16_t *out_n)
{
    switch (cmd) {
    case CMD_CAL_INFO:
    case CMD_ENCODER_CAL_START:
    case CMD_CAL_START:
    case CMD_CAL_ABORT:
    case CMD_CAL_STATUS:
        return handle_calibration_command(cmd, b, bn, out, out_n);
    case CMD_CAL_REPLAY:
        return handle_calibration_replay_command(cmd, b, bn, out, out_n);
    case CMD_IDENTIFY:
    case CMD_RELEASE:
    case CMD_HEARTBEAT:
    case CMD_START:
    case CMD_FORCE_DRAG_250:
        return handle_control_command(cmd, b, bn, out, out_n);
    case CMD_SET_MODE:
    case CMD_SET_IQ:
    case CMD_SET_IQ_LIMIT:
    case CMD_SET_SPEED:
        return handle_targets_command(cmd, b, bn, out, out_n);
    case CMD_SET_HAPTIC:
    case CMD_GET_PARAMS:
        return handle_haptic_command(cmd, b, bn, out, out_n);
    case CMD_SET_FRICTION:
    case CMD_SAVE_FRICTION:
    case CMD_GET_FRICTION:
        return handle_friction_command(cmd, b, bn, out, out_n);
    case CMD_SET_SPEED_PI:
    case CMD_GET_SPEED_PI:
        return handle_speed_pi_command(cmd, b, bn, out, out_n);
    case CMD_SET_COGGING_CONFIG:
    case CMD_GET_COGGING_CONFIG:
        return handle_cogging_config_command(cmd, b, bn, out, out_n);
    case CMD_COGGING_TABLE_BEGIN:
    case CMD_COGGING_TABLE_CHUNK:
    case CMD_COGGING_TABLE_COMMIT:
    case CMD_GET_COGGING_TABLE_CHUNK:
    case CMD_SAVE_COGGING:
        return handle_cogging_table_command(cmd, b, bn, out, out_n);
    default: return MOTOR_CMD_UNSUPPORTED;
    }
}

// 校验请求、控制权和校准互斥后分派业务，再统一回复和缓存。
static void handle_command(uint16_t seq, const uint8_t *p, uint16_t n)
{
    if (n < 5U) {
        return;
    }
    uint8_t cmd = p[0];
    uint32_t token = get_u32(&p[1]);
    const uint8_t *b = &p[5];
    uint16_t bn = (uint16_t)(n - 5U);
    motor_command_result_t r = MOTOR_CMD_OK;
    uint8_t out[PC_PROTOCOL_MAX_PAYLOAD - 10U];
    uint16_t out_n = 0U;
    if (command_uses_replay_cache(cmd) && (seq == s_last_request_seq) && (n == s_last_request_len)) {
        if (memcmp(p, s_last_request, n) == 0) {
            (void)queue_frame(TYPE_RESPONSE, seq, s_last_response, s_last_response_len, 1U);
        } else {
            motor_control_snapshot_t cs;
            uint8_t conflict[10];
            motor_system_get_control_snapshot(&cs);
            conflict[0] = cmd;
            conflict[1] = 11U;
            put_u32(&conflict[2], s_token);
            put_u32(&conflict[6], cs.config_revision);
            (void)queue_frame(TYPE_RESPONSE, seq, conflict, sizeof(conflict), 1U);
        }
        return;
    }
    if (handle_connection_command(cmd, bn, out, &out_n, &r)) {
        // 无令牌命令已处理，继续走统一回复和请求去重缓存。
    } else if (!token_ok(token)) {
        r = MOTOR_CMD_NOT_OWNER;
    } else if ((motor_encoder_cal_is_active() || motor_encoder_cal_requires_power_cycle()) &&
               !cal_command_allowed(cmd)) {
        r = motor_encoder_cal_requires_power_cycle() ? MOTOR_CMD_NOT_READY : MOTOR_CMD_BUSY;
    } else if (motor_calibration_is_active() && !cal_command_allowed(cmd)) {
        r = MOTOR_CMD_BUSY;
    } else {
        r = handle_owned_command(cmd, b, bn, out, &out_n);
    }
    response(seq, cmd, r, out, out_n);
    if (command_uses_replay_cache(cmd)) {
        s_last_request_seq = seq;
        s_last_request_len = n;
        memcpy(s_last_request, p, n);
    }
}


// 复位解析、发送、令牌及重放缓存，保留协议初始序号和诊断计数。
void pc_protocol_init(void)
{
    s_head = s_tail = s_parse_len = 0U;
    s_tx_busy = 0U;
    s_tx_pending_len = s_tx_stream_len = 0U;
    s_token = 0U;
    s_token_counter = 0U;
    s_crc_errors = s_overflows = s_telemetry_drops = 0U;
    s_last_request_len = 0U;
    s_last_request_seq = 0U;
    s_last_response_len = 0U;
    s_sample_seq = 0U;
    s_telemetry_seq = s_status_seq = 0U;
    s_last_status_ms = s_last_telemetry_ms = HAL_GetTick();
}

// 接收 ISR 将明确长度的字节片段写入固定环形队列；前台负责解析命令。
void pc_protocol_feed_from_isr(const uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0U; i < length; ++i) {
        uint16_t next = (uint16_t)((s_head + 1U) % RX_RING_SIZE);
        if (next == s_tail) {
            ++s_overflows;
            break;
        }
        s_ring[s_head] = data[i];
        s_head = next;
    }
}

// 前台推进解析、DMA 重试和周期状态发送；时间参数单位 ms。
void pc_protocol_task(void)
{
    start_queued_tx();
    uint16_t budget = 512U;
    while ((s_tail != s_head) && (budget-- != 0U)) {
        uint8_t byte = s_ring[s_tail];
        s_tail = (uint16_t)((s_tail + 1U) % RX_RING_SIZE);
        if ((s_parse_len == 0U) && (byte != SOF0)) {
            continue;
        }
        if ((s_parse_len == 1U) && (byte != SOF1)) {
            s_parse_len = (byte == SOF0) ? 1U : 0U;
            continue;
        }
        s_parse[s_parse_len++] = byte;
        if (s_parse_len >= 8U) {
            uint16_t payload_len = get_u16(&s_parse[6]);
            if ((payload_len > PC_PROTOCOL_MAX_PAYLOAD) || (s_parse_len >= (uint16_t)sizeof(s_parse))) {
                s_parse_len = 0U;
                continue;
            }
            uint16_t frame_len = (uint16_t)(10U + payload_len);
            if (s_parse_len == frame_len) {
                uint16_t got = get_u16(&s_parse[8U + payload_len]);
                uint16_t calc = crc16(&s_parse[2], (uint16_t)(6U + payload_len));
                if ((got == calc) && (s_parse[2] == PC_PROTOCOL_VERSION) && (s_parse[3] == TYPE_COMMAND)) {
                    handle_command(get_u16(&s_parse[4]), &s_parse[8], payload_len);
                } else {
                    ++s_crc_errors;
                }
                s_parse_len = 0U;
            }
        }
    }
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_last_status_ms) >= 100U) {
        uint8_t p[40];
        uint16_t next_status_seq = (uint16_t)(s_status_seq + 1U);
        /* Periodic status is best effort; never occupy the reserved command ACK slot.
         * A busy DMA may skip this update. HEARTBEAT/CAL_STATUS still reply at priority. */
        s_last_status_ms = now;
        if (queue_frame(TYPE_STATUS, next_status_seq, p, build_status(p), 0U) != 0U) {
            s_status_seq = next_status_seq;
            return;
        }
    }
    uint8_t streaming = motor_calibration_is_active() || motor_calibration_has_samples();
    if ((!s_tx_busy || streaming) && !s_tx_stream_len &&
        (uint32_t)(now - s_last_telemetry_ms) >=
            ((motor_calibration_is_active() || motor_calibration_has_samples()) ? 50U : 5U)) {
        s_last_telemetry_ms = now;
        pc_protocol_send_telemetry();
        return;
    }
    if (motor_calibration_has_samples() && !s_tx_stream_len) {
        uint8_t payload[216];
        motor_cal_sample_t samples[3];
        motor_cal_status_t status;
        uint8_t count = motor_calibration_peek(samples, 3U);
        motor_calibration_get_status(&status);
        float targets[3];
        motor_calibration_peek_targets(targets, count);
        uint16_t payload_len = build_calibration_batch(payload, &status, samples, targets, count);
        if ((count == 3U || !motor_calibration_is_active()) &&
            queue_frame(TYPE_CALIBRATION, 0U, payload, payload_len, 0U)) {
            motor_calibration_consume(count);
        }
    }
}

// 按既有遥测字段序列化观测量，保留采集记录配对及发送成功后的消费次序。
void pc_protocol_send_telemetry(void)
{
    uint8_t p[126];
    motor_control_snapshot_t s;
    motor_system_get_control_snapshot(&s);
    put_u16(&p[0], 7U);
    put_u32(&p[2], HAL_GetTick());
    put_u32(&p[6], ++s_sample_seq);
    float values[29] = {motor_system_get_debug_pot_target(),
                        g_foc_state.pi_q.target,
                        g_foc_state.park.q,
                        g_foc_state.park.d,
                        g_motor_system.run_data.speed_rpm,
                        g_foc_state.pi_q.output,
                        (float)motor_encoder_get_raw_angle() * MOTOR_ENCODER_RAD_PER_COUNT,
                        s.continuous_angle_rad,
                        g_foc_state.target_q,
                        s.relative_center_angle_rad,
                        s.target_speed_rpm,
                        s.speed_loop_iq_a,
                        s.friction_iq_a,
                        s.speed_loop_iq_a + s.friction_iq_a,
                        s.cogging_iq_a,
                        s.cogging_effective_gain,
                        (float)s.cogging_table_revision,
                        (float)(s.cogging_table_crc & 0xFFFFU),
                        (float)(s.cogging_table_crc >> 16U),
                        s.position_target_counts,
                        s.position_actual_counts,
                        s.position_error_counts,
                        s.position_target_speed_rpm,
                        g_foc_state.electrical_angle_rad,
                        speed_est.instant_speed_rpm,
                        (float)motor_encoder_get_raw_angle(),
                        (float)mt6826s_get_status(),
                        (float)mt6826s_get_crc_error_count(),
                        (float)mt6826s_get_transfer_error_count()};
    memcpy(&p[10], values, sizeof(values));
    if (queue_frame(TYPE_TELEMETRY, ++s_telemetry_seq, p, sizeof(p), 0U) == 0U) {
        ++s_telemetry_drops;
    }
}

// UART 发送完成 ISR 释放当前 DMA 槽并按原优先级启动排队帧。
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) {
        return;
    }
    s_tx_busy = 0U;
    start_queued_tx();
}

// 读取累计接收 CRC 错误次数。
uint32_t pc_protocol_get_crc_error_count(void)
{
    return s_crc_errors;
}

// 读取接收环形队列溢出次数。
uint32_t pc_protocol_get_rx_overflow_count(void)
{
    return s_overflows;
}

// 读取遥测发送未被接受的累计次数。
uint32_t pc_protocol_get_telemetry_drop_count(void)
{
    return s_telemetry_drops;
}
