#include "motor_system.h"
#include "motor_current_loop.h"
#include "board_profile.h"
#include "motor_speed_loop.h"
#include "motor_position_loop.h"
#include "motor_feedforward.h"
#include "motor_trajectory.h"
#include "motor_encoder.h"
#include "tim.h"
#include <math.h>
#include <stddef.h>
#include "user_io.h"
#include "motor_identify.h"
#include "svpwm.h"
#include "motor_music.h"
#include "motor_parameters.h"
#include "motor_sensorless.h"
#include "motor_debug.h"

#define MOTOR_ENCODER_MAX_SAMPLE_AGE_MS  2U
#define MOTOR_CURRENT_LOOP_CONTROL_BW_HZ 250.0f
#define MOTOR_CURRENT_LOOP_MUSIC_BW_HZ  2000.0f

// 电位器低通滤波系数 (一阶 EMA, 1kHz 更新率)
// α 越小滤波越强、响应越慢; α=1.0 则无滤波
// α=0.10f → 截止频率 ~16Hz, 适合人手旋转电位器
// α=0.05f → 截止频率 ~8Hz,  更平滑但略有滞后感
#define POT_LPF_ALPHA  0.10f

// 电机系统运行状态。
MotorSystem g_motor_system = {
    .state = MOTOR_STATE_WAIT_PARAMETERS,
};

static volatile uint8_t g_run_requested = 0U;
/* 故障采用锁存方式，传感器恢复后仍需调用 ClearFault。 */
static volatile uint8_t g_fault_latched = 0U;
/* 主循环接口和 1ms 中断共用；volatile 保证读取最新值，多变量切换另用临界区保护。 */
static volatile MotorControlMode g_control_mode = MOTOR_DEFAULT_CONTROL_MODE;
static volatile float g_torque_current_a = 0.0f; // 限幅后的 Iq 给定，供力矩分支送入电流环
static volatile MotorInputSource g_input_source = MOTOR_INPUT_POT;
static volatile MotorControlOwner g_control_owner = MOTOR_OWNER_LOCAL;
static volatile uint32_t g_host_last_heartbeat_ms = 0U;
static volatile uint32_t g_config_revision = 0U;
static volatile float g_host_iq_limit_a = MOTOR_HOST_DEFAULT_IQ_LIMIT_A;
static float g_host_iq_applied_a = 0.0f;
static float g_host_speed_target_rpm = 0.0f;
static float g_host_speed_applied_rpm = 0.0f;
static float g_debug_speed_loop_iq_a = 0.0f;
static float g_debug_friction_iq_a = 0.0f;
static float g_debug_cogging_iq_a = 0.0f;
static float g_continuous_angle_rad = 0.0f;
static float g_haptic_center_rad = 0.0f;
static uint16_t g_last_angle_counts = 0U;
static uint8_t g_angle_initialized = 0U;
static MotorHapticParams g_haptic_params = {
    0.03f, 0.002f, 0.05f, 0.05f, 90.0f, 24U
};

static void Motor_System_ResetOuterLoops(void);
static void Motor_System_ForceSafeStop(void);
static void Motor_System_UpdateOperatingState(void);
static void Motor_System_TuneCurrentLoopBandwidth(float bandwidth_hz);
static void Motor_System_ResetControlHistory(void);

// 电位器滤波量参与力矩/位置控制，同时供 VOFA+ 只读观测。
static float g_pot_target_filtered = 0.0f;         // 反向映射并滤波后的 ADC 计数，供位置/力矩模式共用
static uint8_t g_pot_filter_initialized = 0U;      // 首次采样直接装入，避免从零滤波产生虚假反向力矩

// 浮点数绝对值
static float Motor_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* 初始化系统状态与目标值，清除上次运行残留的力矩和电位器滤波状态。
 * 默认选择力矩模式，但实际输出仍须等待参数有效、编码器新鲜且状态机允许运行。
 * 保留外环初始化，是为了以后切回位置模式时可直接使用，并非默认执行外环。
 */
void Motor_System_Init(void)
{
    g_motor_system.state = MOTOR_STATE_WAIT_PARAMETERS;
    g_run_requested = 0U;
    g_fault_latched = 0U;

    g_control_mode = MOTOR_DEFAULT_CONTROL_MODE;
    g_input_source = MOTOR_INPUT_POT;
    g_control_owner = MOTOR_OWNER_LOCAL;
    g_host_iq_limit_a = MOTOR_HOST_DEFAULT_IQ_LIMIT_A;
    g_host_iq_applied_a = 0.0f;
    g_host_speed_target_rpm = 0.0f;
    g_host_speed_applied_rpm = 0.0f;
    g_debug_speed_loop_iq_a = 0.0f;
    g_debug_friction_iq_a = 0.0f;
    g_debug_cogging_iq_a = 0.0f;
    g_config_revision = 0U;
    g_angle_initialized = 0U;
    g_torque_current_a = 0.0f;
    g_motor_system.run_data.target_torque_nm = 0.0f;
    g_pot_filter_initialized = 0U;

    // 初始目标电流归零
    g_foc_state.target_q = 0.0f;
    g_foc_state.target_d = 0.0f;
    
    // 初始化速度估计器 
    // 参数为低通滤波系数 filter_alpha (0.001 ~ 1.0)
    // 越接近 0 (如 0.08f)：信号越平滑，抗噪声能力强，低速越稳，但响应会有滞后
    // 越接近 1 (如 0.80f)：几乎无滤波，对转速变化响应极快，但低速极容易受噪声抖动
    Motor_SpeedEstimator_Init(0.4f);
    
    // 初始化速度环 PID (参数已移至 motor_speed_loop.h)
    Motor_SpeedLoop_Init();

    // 初始化位置环 PID
    Motor_PositionLoop_Init();
    Motor_Feedforward_FrictionInit();
    Motor_Cogging_Init();

    // 仅初始化音乐模块；当前自动播放宏为 0，需要主动调用播放接口才会发声。
    Motor_Music_Init();

    // Sensorless estimation is observation-only and disabled by default.
    Motor_Sensorless_Init();
}

MotorState Motor_System_GetState(void)
{
    return g_motor_system.state;
}

uint8_t Motor_System_StartControl(void)
{
#if !BOARD_SENSORED_CONTROL_ENABLE
    return 0U;
#endif
    if ((Motor_Parameters_IsReady() == 0U) ||
        (g_fault_latched != 0U) ||
        (Motor_Encoder_IsDataFresh(MOTOR_ENCODER_MAX_SAMPLE_AGE_MS) == 0U)) {
        return 0U;
    }

    g_run_requested = 1U;
    return 1U;
}

void Motor_System_StopControl(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_run_requested = 0U;
    Motor_System_ForceSafeStop();
    g_motor_system.state = MOTOR_STATE_STOPPED;
    if (primask == 0U) {
        __enable_irq();
    }
}

/* 为什么需要：集中切换控制权，防止旧的速度积分或力矩目标带入新模式。
 * 功能：检查模式；发生切换时清零力矩、复位外环，再更新所选模式。
 * 参数：mode 为位置或力矩模式；返回 1 表示接受，0 表示模式无效。
 * 不改变运行请求、不自动停音乐；电位器模式下一周期会重新读取当前旋钮给定。
 */
uint8_t Motor_System_SetControlMode(MotorControlMode mode)
{
    if ((mode < MOTOR_CONTROL_POSITION) || (mode > MOTOR_CONTROL_LIMIT)) {
        return 0U;
    }
    if ((g_control_owner == MOTOR_OWNER_HOST) ||
        (g_motor_system.state == MOTOR_STATE_SENSORED_RUN)) {
        return 0U;
    }

    // 切换包含多个共享变量，暂时屏蔽中断，避免 1ms 控制任务看到切换一半的状态。
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (g_control_mode != mode) {
        g_torque_current_a = 0.0f;
        Motor_System_ResetOuterLoops();
        g_control_mode = mode;
        g_input_source = (mode <= MOTOR_CONTROL_TORQUE)
                             ? MOTOR_INPUT_POT : MOTOR_INPUT_INTERNAL;
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

MotorInputSource Motor_System_GetInputSource(void) { return g_input_source; }
MotorControlOwner Motor_System_GetControlOwner(void) { return g_control_owner; }

static uint8_t Motor_System_IsModeSourceValid(MotorControlMode mode,
                                               MotorInputSource source)
{
    if (mode == MOTOR_CONTROL_POSITION) return source == MOTOR_INPUT_POT;
    if (mode == MOTOR_CONTROL_TORQUE) {
        return (source == MOTOR_INPUT_POT) || (source == MOTOR_INPUT_HOST);
    }
    if (mode == MOTOR_CONTROL_SPEED) return source == MOTOR_INPUT_HOST;
    return ((mode >= MOTOR_CONTROL_FREE) && (mode <= MOTOR_CONTROL_LIMIT) &&
            (source == MOTOR_INPUT_INTERNAL));
}

MotorCommandResult Motor_System_ClaimHost(void)
{
    if ((g_motor_system.state == MOTOR_STATE_IDENTIFYING) ||
        (g_motor_system.state == MOTOR_STATE_MUSIC)) return MOTOR_CMD_BUSY;
    if (g_motor_system.state == MOTOR_STATE_FAULT) return MOTOR_CMD_FAULT_ACTIVE;
    if ((g_motor_system.state != MOTOR_STATE_STOPPED) &&
        (g_motor_system.state != MOTOR_STATE_WAIT_PARAMETERS)) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (g_control_owner == MOTOR_OWNER_HOST) return MOTOR_CMD_BUSY;
    g_control_owner = MOTOR_OWNER_HOST;
    g_host_iq_limit_a = MOTOR_HOST_DEFAULT_IQ_LIMIT_A;
    g_host_last_heartbeat_ms = HAL_GetTick();
    return MOTOR_CMD_OK;
}

void Motor_System_ReleaseHost(void)
{
    Motor_System_StopControl();
    g_control_owner = MOTOR_OWNER_LOCAL;
    g_control_mode = MOTOR_CONTROL_TORQUE;
    g_input_source = MOTOR_INPUT_POT;
    g_host_iq_limit_a = MOTOR_HOST_DEFAULT_IQ_LIMIT_A;
    g_torque_current_a = 0.0f;
    g_host_iq_applied_a = 0.0f;
    g_host_speed_target_rpm = 0.0f;
    g_host_speed_applied_rpm = 0.0f;
    ++g_config_revision;
}

void Motor_System_HostHeartbeat(void)
{
    if (g_control_owner == MOTOR_OWNER_HOST) g_host_last_heartbeat_ms = HAL_GetTick();
}

MotorCommandResult Motor_System_HostStart(void)
{
    if (g_control_owner != MOTOR_OWNER_HOST) return MOTOR_CMD_NOT_OWNER;
    if (g_motor_system.state == MOTOR_STATE_FAULT) return MOTOR_CMD_FAULT_ACTIVE;
    if (Motor_Parameters_IsBusy() != 0U) return MOTOR_CMD_BUSY;
    if ((g_control_mode >= MOTOR_CONTROL_FREE) &&
        (g_control_mode <= MOTOR_CONTROL_SPEED)) {
        int8_t direction = Motor_Identify_GetResult().uvw_dir;
        if ((direction != 1) && (direction != -1)) return MOTOR_CMD_NOT_READY;
    }
    if (Motor_System_StartControl() == 0U) return MOTOR_CMD_NOT_READY;
    g_torque_current_a = 0.0f;
    g_host_iq_applied_a = 0.0f;
    g_host_speed_target_rpm = 0.0f;
    g_host_speed_applied_rpm = 0.0f;
    Motor_Cogging_RuntimeStart();
    if ((g_control_mode >= MOTOR_CONTROL_SPRING) &&
        (g_control_mode <= MOTOR_CONTROL_LIMIT)) {
        g_haptic_center_rad = g_continuous_angle_rad;
    }
    g_host_last_heartbeat_ms = HAL_GetTick();
    return MOTOR_CMD_OK;
}

MotorCommandResult Motor_System_HostSetMode(MotorControlMode mode,
                                            MotorInputSource source)
{
    if (g_control_owner != MOTOR_OWNER_HOST) return MOTOR_CMD_NOT_OWNER;
    if (g_motor_system.state != MOTOR_STATE_STOPPED) return MOTOR_CMD_MUST_STOP_FIRST;
    if (Motor_System_IsModeSourceValid(mode, source) == 0U) {
        return MOTOR_CMD_INVALID_COMBINATION;
    }
    if ((g_control_mode != mode) || (g_input_source != source)) {
        g_control_mode = mode;
        g_input_source = source;
        g_torque_current_a = 0.0f;
        g_host_iq_applied_a = 0.0f;
        g_host_speed_target_rpm = 0.0f;
        g_host_speed_applied_rpm = 0.0f;
        Motor_System_ResetOuterLoops();
        ++g_config_revision;
    }
    return MOTOR_CMD_OK;
}

MotorCommandResult Motor_System_HostSetIq(float iq_a, float *accepted_iq_a)
{
    if (g_control_owner != MOTOR_OWNER_HOST) return MOTOR_CMD_NOT_OWNER;
    if ((g_control_mode != MOTOR_CONTROL_TORQUE) ||
        (g_input_source != MOTOR_INPUT_HOST)) return MOTOR_CMD_INVALID_COMBINATION;
    if (g_motor_system.state != MOTOR_STATE_SENSORED_RUN) return MOTOR_CMD_NOT_RUNNING;
    if (!isfinite(iq_a)) return MOTOR_CMD_INVALID_VALUE;
    if (iq_a > g_host_iq_limit_a) iq_a = g_host_iq_limit_a;
    if (iq_a < -g_host_iq_limit_a) iq_a = -g_host_iq_limit_a;
    g_torque_current_a = iq_a;
    if (accepted_iq_a != NULL) *accepted_iq_a = iq_a;
    return MOTOR_CMD_OK;
}

MotorCommandResult Motor_System_HostSetIqLimit(float limit_a)
{
    if (g_control_owner != MOTOR_OWNER_HOST) return MOTOR_CMD_NOT_OWNER;
    if (g_motor_system.state != MOTOR_STATE_STOPPED) return MOTOR_CMD_MUST_STOP_FIRST;
    if (!isfinite(limit_a) || (limit_a < 0.02f) ||
        (limit_a > MOTOR_TORQUE_CURRENT_LIMIT_A)) return MOTOR_CMD_INVALID_VALUE;
    g_host_iq_limit_a = limit_a;
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

MotorCommandResult Motor_System_HostSetHapticParams(const MotorHapticParams *p)
{
    if (g_control_owner != MOTOR_OWNER_HOST) return MOTOR_CMD_NOT_OWNER;
    if (g_motor_system.state != MOTOR_STATE_STOPPED) return MOTOR_CMD_MUST_STOP_FIRST;
    if ((p == NULL) || !isfinite(p->spring_k_a_per_rad) ||
        !isfinite(p->damping_b_a_per_rad_s) || !isfinite(p->detent_k_a_per_rad) ||
        !isfinite(p->limit_k_a_per_rad) || !isfinite(p->limit_half_range_deg) ||
        (p->spring_k_a_per_rad < 0.0f) || (p->spring_k_a_per_rad > 5.0f) ||
        (p->damping_b_a_per_rad_s < 0.0f) || (p->damping_b_a_per_rad_s > 1.0f) ||
        (p->detent_k_a_per_rad < 0.0f) || (p->detent_k_a_per_rad > 5.0f) ||
        (p->limit_k_a_per_rad < 0.0f) || (p->limit_k_a_per_rad > 5.0f) ||
        (p->limit_half_range_deg < 5.0f) || (p->limit_half_range_deg > 180.0f) ||
        (p->detent_count < 1U) || (p->detent_count > 128U)) return MOTOR_CMD_INVALID_VALUE;
    g_haptic_params = *p;
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

void Motor_System_GetHapticParams(MotorHapticParams *p)
{
    if (p != NULL) *p = g_haptic_params;
}

MotorCommandResult Motor_System_HostSetFrictionConfig(const MotorFrictionConfig *p)
{
    if (g_control_owner != MOTOR_OWNER_HOST) return MOTOR_CMD_NOT_OWNER;
    if (g_motor_system.state != MOTOR_STATE_STOPPED) return MOTOR_CMD_MUST_STOP_FIRST;
    if (Motor_Feedforward_IsFrictionConfigValid(p) == 0U) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    Motor_Feedforward_SetFrictionConfig(p);
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

void Motor_System_GetFrictionConfig(MotorFrictionConfig *p)
{
    Motor_Feedforward_GetFrictionConfig(p);
}

MotorCommandResult Motor_System_HostSetCoggingConfig(const MotorCoggingConfig *p)
{
    if (g_control_owner != MOTOR_OWNER_HOST) return MOTOR_CMD_NOT_OWNER;
    if (g_motor_system.state != MOTOR_STATE_STOPPED) return MOTOR_CMD_MUST_STOP_FIRST;
    if (Motor_Cogging_IsConfigValid(p) == 0U) return MOTOR_CMD_INVALID_VALUE;
    Motor_Cogging_SetConfig(p);
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

MotorCommandResult Motor_System_HostBeginCoggingTable(uint16_t count,
                                                      uint32_t expected_crc,
                                                      uint16_t *transaction_id)
{
    if (g_control_owner != MOTOR_OWNER_HOST) return MOTOR_CMD_NOT_OWNER;
    if (g_motor_system.state != MOTOR_STATE_STOPPED) return MOTOR_CMD_MUST_STOP_FIRST;
    if (count != MOTOR_COGGING_TABLE_SIZE) return MOTOR_CMD_INVALID_VALUE;
    if (Motor_Cogging_TableBegin(count, expected_crc, transaction_id) == 0U) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    return MOTOR_CMD_OK;
}

MotorCommandResult Motor_System_HostWriteCoggingChunk(uint16_t transaction_id,
                                                      uint16_t offset,
                                                      uint8_t count,
                                                      const int16_t *values)
{
    if (g_control_owner != MOTOR_OWNER_HOST) return MOTOR_CMD_NOT_OWNER;
    if (g_motor_system.state != MOTOR_STATE_STOPPED) return MOTOR_CMD_MUST_STOP_FIRST;
    return Motor_Cogging_TableWriteChunk(transaction_id, offset, count, values)
               ? MOTOR_CMD_OK : MOTOR_CMD_INVALID_VALUE;
}

MotorCommandResult Motor_System_HostCommitCoggingTable(uint16_t transaction_id)
{
    if (g_control_owner != MOTOR_OWNER_HOST) return MOTOR_CMD_NOT_OWNER;
    if (g_motor_system.state != MOTOR_STATE_STOPPED) return MOTOR_CMD_MUST_STOP_FIRST;
    if (Motor_Cogging_TableCommit(transaction_id) == 0U) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

MotorCommandResult Motor_System_HostReadCoggingChunk(uint16_t offset,
                                                     uint8_t count,
                                                     int16_t *values)
{
    if (g_control_owner != MOTOR_OWNER_HOST) return MOTOR_CMD_NOT_OWNER;
    return Motor_Cogging_TableReadChunk(offset, count, values)
               ? MOTOR_CMD_OK : MOTOR_CMD_INVALID_VALUE;
}

MotorCommandResult Motor_System_HostSaveCogging(void)
{
    if (g_control_owner != MOTOR_OWNER_HOST) return MOTOR_CMD_NOT_OWNER;
    if (g_motor_system.state != MOTOR_STATE_STOPPED) return MOTOR_CMD_MUST_STOP_FIRST;
    return Motor_Cogging_SaveToFlash() ? MOTOR_CMD_OK : MOTOR_CMD_NOT_READY;
}

void Motor_System_GetCoggingConfig(MotorCoggingConfig *p)
{
    Motor_Cogging_GetConfig(p);
}

MotorCommandResult Motor_System_HostSetSpeed(float speed_rpm,
                                             float *accepted_speed_rpm)
{
    if (g_control_owner != MOTOR_OWNER_HOST) return MOTOR_CMD_NOT_OWNER;
    if ((g_control_mode != MOTOR_CONTROL_SPEED) ||
        (g_input_source != MOTOR_INPUT_HOST)) return MOTOR_CMD_INVALID_COMBINATION;
    if (g_motor_system.state != MOTOR_STATE_SENSORED_RUN) return MOTOR_CMD_NOT_RUNNING;
    if (!isfinite(speed_rpm)) return MOTOR_CMD_INVALID_VALUE;
    if (speed_rpm > MOTOR_HOST_SPEED_MAX_RPM) speed_rpm = MOTOR_HOST_SPEED_MAX_RPM;
    if (speed_rpm < -MOTOR_HOST_SPEED_MAX_RPM) speed_rpm = -MOTOR_HOST_SPEED_MAX_RPM;
    if ((speed_rpm == 0.0f) ||
        ((g_host_speed_target_rpm * speed_rpm) < 0.0f)) {
        /* 停止或换向时清除旧方向积分，避免静摩擦挣脱后的反向冲击。 */
        PID_Reset(&speed_pid);
    }
    g_host_speed_target_rpm = speed_rpm;
    if (accepted_speed_rpm != NULL) *accepted_speed_rpm = speed_rpm;
    return MOTOR_CMD_OK;
}

void Motor_System_GetControlSnapshot(MotorControlSnapshot *s)
{
    if (s == NULL) return;
    s->state = g_motor_system.state;
    s->mode = g_control_mode;
    s->source = g_input_source;
    s->owner = g_control_owner;
    s->run_requested = g_run_requested;
    s->iq_limit_a = g_host_iq_limit_a;
    s->continuous_angle_rad = g_continuous_angle_rad;
    s->relative_center_angle_rad = ((g_control_mode >= MOTOR_CONTROL_SPRING) &&
                                    (g_control_mode <= MOTOR_CONTROL_LIMIT))
                                       ? g_continuous_angle_rad - g_haptic_center_rad : 0.0f;
    s->target_speed_rpm = g_host_speed_applied_rpm;
    s->speed_loop_iq_a = g_debug_speed_loop_iq_a;
    s->friction_iq_a = g_debug_friction_iq_a;
    s->cogging_iq_a = g_debug_cogging_iq_a;
    s->cogging_effective_gain = Motor_Cogging_GetEffectiveGain();
    s->cogging_table_revision = Motor_Cogging_GetTableRevision();
    s->cogging_table_crc = Motor_Cogging_GetActiveTableCrc();
    s->config_revision = g_config_revision;
}

/* 提供统一的模式查询入口，避免其他模块直接依赖内部变量。
 * 返回当前选择的模式；停机、故障或播放音乐时也保留此模式值。
 */
MotorControlMode Motor_System_GetControlMode(void)
{
    return g_control_mode;
}

/* 为什么需要：为手动调试和 N·m 接口提供统一的电流限幅及运行条件检查。
 * 功能：接收有符号 Iq（A），拒绝 NaN/无穷大，并限制在允许电流范围。
 * 仅手动给定配置且处于正常力矩运行时接受；返回 1 表示接受，0 表示拒绝。
 * 拒绝时清零存储目标；若正处于力矩模式，还立即清零当前 FOC 的 q 轴目标。
 * 默认电位器配置下不应调用此接口：即使调用清零，下一周期仍由电位器重新给定。
 */
uint8_t Motor_System_SetTorqueCurrent(float iq_a)
{
    uint8_t valid = isfinite(iq_a) ? 1U : 0U;
    if (valid == 0U) {
        iq_a = 0.0f;
    } else if (iq_a > MOTOR_TORQUE_CURRENT_LIMIT_A) {
        iq_a = MOTOR_TORQUE_CURRENT_LIMIT_A;
    } else if (iq_a < -MOTOR_TORQUE_CURRENT_LIMIT_A) {
        iq_a = -MOTOR_TORQUE_CURRENT_LIMIT_A;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if ((g_input_source != MOTOR_INPUT_HOST) ||
        (g_control_mode != MOTOR_CONTROL_TORQUE) ||
        (g_motor_system.state != MOTOR_STATE_SENSORED_RUN) ||
        (g_run_requested == 0U) || (g_fault_latched != 0U)) {
        valid = 0U;
        iq_a = 0.0f;
    }
    g_torque_current_a = iq_a;
    if ((valid == 0U) && (g_control_mode == MOTOR_CONTROL_TORQUE)) {
        g_foc_state.target_q = 0.0f;
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return valid;
}

/* 为什么需要：FOC 电流环接收安培，上层使用 N·m，必须统一换算并限制电流。
 * 参数：torque_nm 为目标电磁力矩（N·m），符号沿用 Iq 方向。
 * 返回：按 Iq=T/Kt 换算并限幅的电流（A）；参数或 Kt 无效时返回 0。
 * 先与 Kt*电流上限比较，再做除法，避免极小 Kt 或极大输入导致除法溢出。
 * 此函数只计算，不修改运行状态；电位器和手动力矩入口共用相同换算规则。
 */
static float Motor_System_TorqueToIq(float torque_nm)
{
    const float kt = MOTOR_TORQUE_CONSTANT_NM_PER_A;
    if (!isfinite(kt) || (kt <= 0.0f) || !isfinite(torque_nm)) {
        return 0.0f;
    }
    float max_torque_nm = kt * MOTOR_TORQUE_CURRENT_LIMIT_A;
    if (torque_nm >= max_torque_nm) {
        return MOTOR_TORQUE_CURRENT_LIMIT_A;
    }
    if (torque_nm <= -max_torque_nm) {
        return -MOTOR_TORQUE_CURRENT_LIMIT_A;
    }
    return torque_nm / kt;
}

/* 为什么需要：允许上层按 N·m 给定，无需在业务代码中重复计算 Kt 和限流。
 * 功能：检查输入与 Kt，经 TorqueToIq 换算后交给 SetTorqueCurrent 执行。
 * 参数单位为 N·m；返回 1 表示接受（可能限幅），0 表示参数或运行条件不满足。
 * 仅 MOTOR_TORQUE_USE_POT=0 时用于手动控制，不替代默认电位器输入。
 */
uint8_t Motor_System_SetTorqueNm(float torque_nm)
{
    const float kt = MOTOR_TORQUE_CONSTANT_NM_PER_A;
    if (!isfinite(kt) || (kt <= 0.0f) || !isfinite(torque_nm)) {
        (void)Motor_System_SetTorqueCurrent(0.0f);
        return 0U;
    }
    return Motor_System_SetTorqueCurrent(Motor_System_TorqueToIq(torque_nm));
}

#if MOTOR_TORQUE_USE_POT
/* 为什么需要：将单路电位器变成可正反向调节的力矩旋钮，并抑制中点抖动。
 * 参数：filtered_pot 是经过4095-raw反向映射和低通滤波的12位ADC计数。
 * 返回：中点死区内为 0；死区外线性映射至 +/- MOTOR_TORQUE_POT_MAX_NM。
 * 减去死区后再归一化，使刚离开死区时从零连续增加，不会突然跳到一个非零力矩。
 * 原始 ADC 越小，反向映射后的给定越正；该符号不承诺实际轴端顺/逆时针方向。
 * 只生成力矩目标，电流限幅由后续 TorqueToIq 负责。
 */
static float Motor_System_PotTorqueNm(float filtered_pot)
{
    float offset = filtered_pot - MOTOR_TORQUE_POT_CENTER;
    float magnitude = Motor_AbsFloat(offset) - MOTOR_TORQUE_POT_DEADBAND;
    if (magnitude <= 0.0f) {
        return 0.0f;
    }
    float normalized = magnitude /
                       (MOTOR_TORQUE_POT_CENTER - MOTOR_TORQUE_POT_DEADBAND);
    if (normalized > 1.0f) {
        normalized = 1.0f;
    }
    return ((offset > 0.0f) ? normalized : -normalized) * MOTOR_TORQUE_POT_MAX_NM;
}
#endif

uint8_t Motor_System_ClearFault(void)
{
    if ((Motor_Parameters_IsReady() == 0U) ||
        (Motor_Encoder_IsDataFresh(MOTOR_ENCODER_MAX_SAMPLE_AGE_MS) == 0U)) {
        return 0U;
    }

    g_fault_latched = 0U;
    return 1U;
}

uint8_t Motor_System_IdentifyAndSave(void)
{
    if ((Motor_Parameters_IsBusy() != 0U) ||
        (Motor_Encoder_IsDataFresh(MOTOR_ENCODER_MAX_SAMPLE_AGE_MS) == 0U)) {
        return 0U;
    }

    g_fault_latched = 0U;
    g_run_requested = 1U;
    return Motor_Parameters_IdentifyAndSave();
}

uint8_t Motor_System_PlaySong(const MotorMusicNote *song,
                              uint16_t note_count,
                              uint16_t play_count)
{
    if ((song == NULL) || (note_count == 0U) || (play_count == 0U) ||
        (Motor_Parameters_IsReady() == 0U) ||
        (g_fault_latched != 0U) ||
        (Motor_CurrentLoop_IsEnabled() == 0U)) {
        return 0U;
    }

    Motor_Music_StartTimes(song, note_count, play_count);
    return 1U;
}

uint8_t Motor_System_PlaySongLoop(const MotorMusicNote *song,
                                  uint16_t note_count)
{
    if ((song == NULL) || (note_count == 0U) ||
        (Motor_Parameters_IsReady() == 0U) ||
        (g_fault_latched != 0U) ||
        (Motor_CurrentLoop_IsEnabled() == 0U)) {
        return 0U;
    }

    Motor_Music_Start(song, note_count, 1U);
    return 1U;
}

void Motor_System_StopMusic(void)
{
    Motor_Music_Stop();
}

uint8_t Motor_System_EnableSensorlessObserver(uint8_t enable)
{
    if (enable == 0U) {
        Motor_Sensorless_Enable(0U);
        return 1U;
    }

    if ((g_motor_system.state != MOTOR_STATE_SENSORED_RUN) ||
        (Motor_CurrentLoop_IsEnabled() == 0U)) {
        return 0U;
    }

    Motor_Sensorless_Enable(1U);
    return 1U;
}

/* 为什么需要：位置/速度环历史复位在多处重复，集中为一处以免顺序漂移。
 * 功能：按固定顺序清目标速度、速度 PID、位置环和轨迹规划器历史量。
 * 参数：无；调用上下文：仅由模式切换、进入音乐和非运行清理复用。
 * 副作用：会清 PID 积分与轨迹状态；不修改目标力矩、q/d 给定。
 */
static void Motor_System_ResetControlHistory(void)
{
    Motor_SpeedLoop_SetTarget(0.0f);
    PID_Reset(&speed_pid);
    Motor_PositionLoop_Reset();
    Motor_Trajectory_Clear();
    g_host_speed_target_rpm = 0.0f;
    g_host_speed_applied_rpm = 0.0f;
    g_debug_speed_loop_iq_a = 0.0f;
    g_debug_friction_iq_a = 0.0f;
    g_debug_cogging_iq_a = 0.0f;
}

static void Motor_System_ResetOuterLoops(void)
{
    g_motor_system.run_data.target_torque_nm = 0.0f;
    /* 清掉位置/速度环历史量，避免功能切换后积分残留。 */
    Motor_System_ResetControlHistory();
    g_foc_state.target_q = 0.0f;
    g_foc_state.target_d = 0.0f;
}

static void Motor_System_TuneCurrentLoopBandwidth(float bandwidth_hz)
{
    MotorIdentifiedParams id = Motor_Identify_GetResult();

    Motor_CurrentLoop_AutoTunePIDWithBandwidth(id.resistance,
                                               id.inductance,
                                               SYSTEM_BUS_VOLTAGE,
                                               bandwidth_hz);
}

static void Motor_System_ForceSafeStop(void)
{
    /* 所有功能统一从这里撤销转矩输出。 */
    Motor_Cogging_RuntimeStop();
    g_torque_current_a = 0.0f;
    Motor_System_ResetOuterLoops();
    if (Motor_Music_IsPlaying() != 0U) {
        Motor_Music_Stop();
    }
    if (Motor_Sensorless_IsEnabled() != 0U) {
        Motor_Sensorless_Enable(0U);
    }
    if (Motor_CurrentLoop_IsEnabled() != 0U) {
        Motor_CurrentLoop_Enable(0U);
    }
    if (SVPWM_IsEnabled() != 0U) {
        SVPWM_Disable();
    }
}

static void Motor_System_UpdateOperatingState(void)
{
#if !BOARD_SENSORED_CONTROL_ENABLE
    /* Development foundation only: no sensorless startup/handover yet. */
    Motor_System_ForceSafeStop();
    g_motor_system.state = MOTOR_STATE_STOPPED;
    return;
#endif
    MotorParametersStatus parameter_status = Motor_Parameters_GetStatus();

    /*
     * Identification owns the PWM after its start routine has stopped all
     * closed-loop features. Do not clear its open-loop duty every 1 ms.
     */
    if (Motor_Parameters_IsBusy() != 0U) {
        g_torque_current_a = 0.0f;
        g_motor_system.state = MOTOR_STATE_IDENTIFYING;
        return;
    }

    if (parameter_status == MOTOR_PARAMETERS_ERROR) {
        g_fault_latched = 1U;
    }

    if (parameter_status == MOTOR_PARAMETERS_NO_DATA) {
        Motor_System_ForceSafeStop();
        g_motor_system.state = MOTOR_STATE_WAIT_PARAMETERS;
        return;
    }

    /* MT6826S SPI是当前实际换相角度源，数据过期必须立即停机。 */
    if ((g_fault_latched != 0U) ||
        (Motor_Encoder_IsDataFresh(MOTOR_ENCODER_MAX_SAMPLE_AGE_MS) == 0U)) {
        g_fault_latched = 1U;
        Motor_System_ForceSafeStop();
        g_motor_system.state = MOTOR_STATE_FAULT;
        return;
    }

    if (g_run_requested == 0U) {
        Motor_System_ForceSafeStop();
        g_motor_system.state = MOTOR_STATE_STOPPED;
        return;
    }

    /* 参数和传感器均有效后，统一完成整定与闭环使能。 */
    if (Motor_CurrentLoop_IsEnabled() == 0U) {
        MotorIdentifiedParams id = Motor_Identify_GetResult();

        Motor_System_TuneCurrentLoopBandwidth(
            MOTOR_CURRENT_LOOP_CONTROL_BW_HZ);
        (void)Motor_Sensorless_ConfigureMotor(id.resistance,
                                              id.inductance,
                                              id.pole_pairs);
        SVPWM_Enable();
        Motor_CurrentLoop_Enable(1U);
    }

    g_motor_system.state = (Motor_Music_IsPlaying() != 0U)
                               ? MOTOR_STATE_MUSIC
                               : MOTOR_STATE_SENSORED_RUN;
}

/* 为什么需要：电位器采样、反向映射与滤波是力矩/位置模式共用的公共输入。
 * 功能：读取 ADC，做 4095-raw 反向映射，首次直接装入并做一阶 EMA 滤波。
 * 参数：无；返回映射到15位编码器范围的目标位置计数(0~32767)。
 * 副作用：更新 run_data.pot_raw、g_pot_target_filtered 与电位器初始化标记。
 * 调用上下文：每次 1ms 任务开始时由 Task 调用一次。
 */
static float Motor_System_UpdatePotTarget(void)
{
    // ========== 步骤 2: 读取电位器给定 ==========
    // ADC 原始值 → 反向映射 (4095-raw) → 低通滤波。
    // 力矩模式将该计数映射成 N·m；保留的位置模式将该计数作为位置目标。
    g_motor_system.run_data.pot_raw = Pot_ReadRaw();

    float target_pos_raw = 4095.0f - (float)g_motor_system.run_data.pot_raw;
    /* 首次直接装入采样，避免中点电位器从滤波初值 0 爬升造成反向力矩。 */
    if (g_pot_filter_initialized == 0U) {
        g_pot_target_filtered = target_pos_raw;
        g_pot_filter_initialized = 1U;
    }
    // 一阶 EMA 低通滤波: filtered += α * (raw - filtered)
    g_pot_target_filtered += POT_LPF_ALPHA * (target_pos_raw - g_pot_target_filtered);
    return g_pot_target_filtered *
           (MOTOR_ENCODER_COUNTS_PER_REV_F / 4096.0f);
}

/* 为什么需要：力矩模式给定换算独立成函数，使 Task 只表达模式调度。
 * 功能：按配置从电位器（或手动保存值）生成受限 Iq，写 q/d 目标并反算调试力矩。
 * 参数：无；调用上下文：状态为 SENSORED_RUN 且模式为 TORQUE 时，由 Task 调用。
 * 副作用：更新 g_torque_current_a、g_foc_state.target_q/target_d，
 *         并可能更新 run_data.target_torque_nm。
 */
static void Motor_System_RunTorqueMode(void)
{
    if (g_input_source == MOTOR_INPUT_POT) {
        g_torque_current_a = Motor_System_TorqueToIq(
            Motor_System_PotTorqueNm(g_pot_target_filtered));
        g_host_iq_applied_a = g_torque_current_a;
    } else {
        float max_step = MOTOR_IQ_SLEW_A_PER_S * MOTOR_SYSTEM_TASK_DT_SEC;
        if (g_torque_current_a > g_host_iq_applied_a + max_step) {
            g_host_iq_applied_a += max_step;
        } else if (g_torque_current_a < g_host_iq_applied_a - max_step) {
            g_host_iq_applied_a -= max_step;
        } else {
            g_host_iq_applied_a = g_torque_current_a;
        }
    }
    g_foc_state.target_q = g_host_iq_applied_a;
    g_foc_state.target_d = 0.0f;
    if (isfinite(MOTOR_TORQUE_CONSTANT_NM_PER_A) &&
        (MOTOR_TORQUE_CONSTANT_NM_PER_A > 0.0f)) {
        // 从限幅后的 Iq 反算目标力矩用于调试；这是模型估算，不是力矩传感器读数。
        g_motor_system.run_data.target_torque_nm =
            g_host_iq_applied_a * MOTOR_TORQUE_CONSTANT_NM_PER_A;
    }
}

/* 为什么需要：位置/速度级联计算独立成函数，使 Task 只表达模式调度。
 * 功能：用目标位置与实际位置跑位置环和速度环，直接生成 Iq。
 * 参数：target_pos 为电位器目标位置计数；current_rpm 为按电磁正方向统一后的转速。
 * 调用上下文：状态为 SENSORED_RUN 且未选择力矩模式时，由 Task 调用。
 * 副作用：更新位置/速度 PID 和 g_foc_state.target_q。
 */
static void Motor_System_RunPositionMode(float target_pos, float current_rpm)
{
    float actual_pos = (float)Motor_Encoder_GetRawAngle();

    /* SPEED 模式会按会话限流收紧速度 PI 输出；回到位置模式时恢复位置环基线。 */
    speed_pid.out_max = MOTOR_SPEED_PID_OUT_MAX;
    speed_pid.out_min = MOTOR_SPEED_PID_OUT_MIN;

    // 恢复无负载时期的纯级联结构：位置误差直接生成目标机械转速。
    float target_mech_rpm =
        Motor_PositionLoop_Run(target_pos, actual_pos);

    // 将机械期望转速乘以 uvw_dir 统一符号后给到速度环，防止正反馈。
    // target_signed_rpm 是按电磁转矩正方向统一符号的机械 RPM，没有乘极对数。
    float uvw_dir = (float)Motor_Identify_GetResult().uvw_dir;
    float target_signed_rpm = target_mech_rpm * uvw_dir;

    // 位置环输出的目标速度直接传给速度环。
    Motor_SpeedLoop_SetTarget(target_signed_rpm);

    // 速度 PI 直接输出 Iq；无负载位置模式不叠加摩擦或惯性前馈。
    g_debug_speed_loop_iq_a = Motor_SpeedLoop_Update(current_rpm);
    g_debug_friction_iq_a = 0.0f;
    g_debug_cogging_iq_a = 0.0f;
    g_foc_state.target_q = g_debug_speed_loop_iq_a;
}

static float Motor_System_ClampHostIq(float iq)
{
    if (!isfinite(iq)) return 0.0f;
    if (iq > g_host_iq_limit_a) return g_host_iq_limit_a;
    if (iq < -g_host_iq_limit_a) return -g_host_iq_limit_a;
    return iq;
}

static float Motor_System_SlewHostIq(float target_iq, float slew_a_per_s)
{
    float max_step = slew_a_per_s * MOTOR_SYSTEM_TASK_DT_SEC;
    if (target_iq > g_host_iq_applied_a + max_step) {
        g_host_iq_applied_a += max_step;
    } else if (target_iq < g_host_iq_applied_a - max_step) {
        g_host_iq_applied_a -= max_step;
    } else {
        g_host_iq_applied_a = target_iq;
    }
    return g_host_iq_applied_a;
}

static void Motor_System_RunSpeedMode(float mechanical_speed_rpm,
                                      int8_t identified_direction,
                                      uint16_t mechanical_angle_counts)
{
    const float max_step = MOTOR_HOST_SPEED_SLEW_RPM_PER_S *
                           MOTOR_SYSTEM_TASK_DT_SEC;
    if (g_host_speed_target_rpm > g_host_speed_applied_rpm + max_step) {
        g_host_speed_applied_rpm += max_step;
    } else if (g_host_speed_target_rpm < g_host_speed_applied_rpm - max_step) {
        g_host_speed_applied_rpm -= max_step;
    } else {
        g_host_speed_applied_rpm = g_host_speed_target_rpm;
    }

    /* SPEED按目标方向提前给摩擦前馈。实际速度为零时也能提供起步电流，
     * 避免只能等待积分累积后突然挣脱。FREE仍使用实际速度，防止静止自驱。 */
    g_debug_friction_iq_a = Motor_Feedforward_FrictionCompensation(
        g_host_speed_applied_rpm) * (float)identified_direction;
    g_debug_cogging_iq_a = Motor_Cogging_Compensation(
        mechanical_angle_counts, mechanical_speed_rpm);
    float feedforward_iq = g_debug_friction_iq_a + g_debug_cogging_iq_a;

    /* 将前馈占用的电流余量反馈给PI抗饱和，防止总Iq已经限幅时积分仍增长。 */
    speed_pid.out_max = g_host_iq_limit_a - feedforward_iq;
    speed_pid.out_min = -g_host_iq_limit_a - feedforward_iq;
    Motor_SpeedLoop_SetTarget(g_host_speed_applied_rpm * (float)identified_direction);
    g_debug_speed_loop_iq_a = Motor_SpeedLoop_Update(
        mechanical_speed_rpm * (float)identified_direction);
    float iq = Motor_System_ClampHostIq(
        g_debug_speed_loop_iq_a + feedforward_iq);
    g_foc_state.target_q = Motor_System_SlewHostIq(
        iq, MOTOR_SPEED_IQ_SLEW_A_PER_S);
    g_foc_state.target_d = 0.0f;
}

// 周期任务：位置模式执行外环；力矩模式直接将给定 Iq 送入电流环。
// 调用频率: 1kHz (由 TIM2 中断驱动, dt = 1ms)
// 默认链路: 电位器 → 目标力矩(N·m) → Iq=T/Kt → FOC → SVPWM → 电机
void Motor_System_Task(void)
{
    static uint8_t music_was_active = 0U;
#if MOTOR_MUSIC_AUTOPLAY_DEMO
    static uint8_t demo_autoplay_checked = 0U;
#endif

    // ========== 步骤 1: 速度估算（20/10/5 ms自适应滚动位置窗） ==========
    // 始终每1ms更新反馈；带迟滞切换窗口，兼顾低速分辨率和中高速延迟。
    if (Motor_Encoder_IsDataFresh(MOTOR_ENCODER_MAX_SAMPLE_AGE_MS) != 0U) {
        g_motor_system.run_data.speed_rpm =
            Motor_SpeedEstimator_UpdateAdaptive(g_motor_system.run_data.speed_rpm);
    } else {
        g_motor_system.run_data.speed_rpm = 0.0f;
    }
    
    // 如果编码器接线/电机相序不同，编码器读数的正反方向可能会和 Iq 的正扭矩方向相反。
    // 我们必须用系统辨识出的 uvw_dir (1 或 -1) 来把转速的正负号与电机电磁正方向统一，否则会导致 PID 变成正反馈（越差越使劲）！
    int8_t identified_direction = Motor_Identify_GetResult().uvw_dir;
    if ((identified_direction != 1) && (identified_direction != -1)) {
        identified_direction = 1;
    }
    float current_rpm = g_motor_system.run_data.speed_rpm *
                        (float)identified_direction;

    /* 在机械坐标中展开单圈角度，供触觉模式跨零点连续计算。 */
    uint16_t angle_counts = Motor_Encoder_GetRawAngle();
    if (g_angle_initialized == 0U) {
        g_last_angle_counts = angle_counts;
        g_continuous_angle_rad = (float)angle_counts * MOTOR_ENCODER_RAD_PER_COUNT;
        g_angle_initialized = 1U;
    } else {
        int32_t delta = (int32_t)angle_counts - (int32_t)g_last_angle_counts;
        if (delta > (int32_t)MOTOR_ENCODER_HALF_REV_U32) {
            delta -= (int32_t)MOTOR_ENCODER_COUNTS_PER_REV_U32;
        }
        if (delta < -(int32_t)MOTOR_ENCODER_HALF_REV_U32) {
            delta += (int32_t)MOTOR_ENCODER_COUNTS_PER_REV_U32;
        }
        g_continuous_angle_rad += (float)delta * MOTOR_ENCODER_RAD_PER_COUNT;
        g_last_angle_counts = angle_counts;
    }

    if ((g_control_owner == MOTOR_OWNER_HOST) &&
        ((uint32_t)(HAL_GetTick() - g_host_last_heartbeat_ms) >
         MOTOR_HOST_HEARTBEAT_TIMEOUT_MS)) {
        Motor_System_ReleaseHost();
    }

    if (current_rpm > 20.0f) {
        Motor_Sensorless_SetDirection(1);
    } else if (current_rpm < -20.0f) {
        Motor_Sensorless_SetDirection(-1);
    }
    
    float target_pos = Motor_System_UpdatePotTarget();
    g_foc_state.target_d = 0.0f;
    Motor_Cogging_Task1ms(
        (g_motor_system.state == MOTOR_STATE_SENSORED_RUN) &&
        ((g_control_mode == MOTOR_CONTROL_SPEED) ||
         (g_control_mode == MOTOR_CONTROL_FREE)));

#if MOTOR_MUSIC_AUTOPLAY_DEMO
    if ((g_motor_system.state == MOTOR_STATE_SENSORED_RUN) &&
        (demo_autoplay_checked == 0U)) {
        demo_autoplay_checked = 1U;
        Motor_Music_StartDemo();
        g_motor_system.state = MOTOR_STATE_MUSIC;
    }
#endif

    // 音乐播放时由音频模块独占 q 轴给定，暂停正常位置/力矩输出以免相互叠加。
    if (g_motor_system.state == MOTOR_STATE_MUSIC) {
        g_motor_system.run_data.target_torque_nm = 0.0f;
        if (music_was_active == 0U) {
            g_torque_current_a = 0.0f;
            Motor_System_TuneCurrentLoopBandwidth(
                MOTOR_CURRENT_LOOP_MUSIC_BW_HZ);
            Motor_System_ResetControlHistory();
            music_was_active = 1U;
        }

        Motor_Music_Task1ms();
        g_foc_state.target_q = 0.0f;
        g_foc_state.target_d = 0.0f;
        return;
    }

    if (music_was_active != 0U) {
        Motor_System_TuneCurrentLoopBandwidth(
            MOTOR_CURRENT_LOOP_CONTROL_BW_HZ);
    }
    music_was_active = 0U;
    g_motor_system.run_data.target_torque_nm = 0.0f;

    /* 直接力矩分支：状态机确认正常有感运行后，才将力矩给定送入电流环。
     * 默认每 1ms 从电位器生成 N·m，再换算成受限的 Iq；手动配置则使用保存的 Iq。
     * Id 固定为 0，并提前返回，防止下方位置/速度环和前馈覆盖直接力矩给定。
     * 因为没有速度环，此模式不保持某个转速，也不保持某个位置。
     */
    if (g_motor_system.state == MOTOR_STATE_SENSORED_RUN) {
        if (g_control_mode == MOTOR_CONTROL_TORQUE) {
            g_debug_speed_loop_iq_a = 0.0f;
            g_debug_friction_iq_a = 0.0f;
            g_debug_cogging_iq_a = 0.0f;
            Motor_System_RunTorqueMode();
            return;
        }
        if (g_control_mode == MOTOR_CONTROL_SPEED) {
            Motor_System_RunSpeedMode(g_motor_system.run_data.speed_rpm,
                                      identified_direction, angle_counts);
            return;
        }
        if ((g_control_mode >= MOTOR_CONTROL_FREE) &&
            (g_control_mode <= MOTOR_CONTROL_LIMIT)) {
            const float omega = g_motor_system.run_data.speed_rpm *
                                (6.28318530718f / 60.0f);
            const float rel = g_continuous_angle_rad - g_haptic_center_rad;
            float i_mech = 0.0f;
            if (g_control_mode == MOTOR_CONTROL_FREE) {
                i_mech = Motor_Feedforward_FrictionCompensation(
                    g_motor_system.run_data.speed_rpm);
            } else if (g_control_mode == MOTOR_CONTROL_DAMPING) {
                i_mech = -g_haptic_params.damping_b_a_per_rad_s * omega;
            } else if (g_control_mode == MOTOR_CONTROL_SPRING) {
                i_mech = -g_haptic_params.spring_k_a_per_rad * rel
                         -g_haptic_params.damping_b_a_per_rad_s * omega;
            } else if (g_control_mode == MOTOR_CONTROL_DETENT) {
                float spacing = 6.28318530718f / (float)g_haptic_params.detent_count;
                float nearest = roundf(rel / spacing) * spacing;
                i_mech = -g_haptic_params.detent_k_a_per_rad * (rel - nearest)
                         -g_haptic_params.damping_b_a_per_rad_s * omega;
            } else if (g_control_mode == MOTOR_CONTROL_LIMIT) {
                float half = g_haptic_params.limit_half_range_deg * 0.01745329252f;
                if (rel > half) {
                    i_mech = -g_haptic_params.limit_k_a_per_rad * (rel - half)
                             -g_haptic_params.damping_b_a_per_rad_s * omega;
                } else if (rel < -half) {
                    i_mech = -g_haptic_params.limit_k_a_per_rad * (rel + half)
                             -g_haptic_params.damping_b_a_per_rad_s * omega;
                }
            }
            g_debug_friction_iq_a = (g_control_mode == MOTOR_CONTROL_FREE)
                                        ? i_mech * (float)identified_direction
                                        : 0.0f;
            g_debug_cogging_iq_a = (g_control_mode == MOTOR_CONTROL_FREE)
                ? Motor_Cogging_Compensation(
                      angle_counts, g_motor_system.run_data.speed_rpm)
                : 0.0f;
            float iq = i_mech * (float)identified_direction +
                       g_debug_cogging_iq_a;
            if (!isfinite(iq)) iq = 0.0f;
            if (iq > g_host_iq_limit_a) iq = g_host_iq_limit_a;
            if (iq < -g_host_iq_limit_a) iq = -g_host_iq_limit_a;
            iq = Motor_System_SlewHostIq(iq, MOTOR_IQ_SLEW_A_PER_S);
            g_debug_speed_loop_iq_a = 0.0f;
            g_foc_state.target_q = iq;
            g_foc_state.target_d = 0.0f;
            return;
        }
    }
    
    // 3. FOC 闭环开始工作后，开始让位置环介入产生速度，速度环介入产生 Iq
    // 控制链路: 电位器目标 → 位置环 (P) → 目标机械转速 → 乘 uvw_dir → 目标电磁转速
    //          → 速度环 (PI) → Iq 电流 → FOC 电流环
    if (g_motor_system.state == MOTOR_STATE_SENSORED_RUN)
    {
        Motor_System_RunPositionMode(target_pos, current_rpm);
    }
    else
    {
        // 辨识未完成: 清零所有 PID 积分和电流输出，防止误动作
        Motor_System_ResetControlHistory();
        g_foc_state.target_q = 0.0f;
        g_foc_state.target_d = 0.0f;
    }
    
    // D 轴弱磁控制 (Field Weakening) 策略 — 当前禁用
    // 弱磁用于高速时主动注入负 Id 来压低反电动势，扩展转速范围
    //g_foc_state.target_d = Motor_SpeedLoop_FieldWeakening(current_rpm);
}

// ---------------------------------------------------------
// 状态灯 2Hz 闪烁任务 (设计为 1ms 调用一次)
// ---------------------------------------------------------
static void Motor_StatusLed_Task(void)
{
    // 频率 2Hz = 周期 500ms = 亮 250ms，灭 250ms
    static uint16_t led_cnt = 0;
    led_cnt++;
    if (led_cnt >= 250) {
        led_cnt = 0;
        Led_Toggle();
    }
}

// ---------------------------------------------------------
// 定时器更新中断回调函数 (TIM2, 假设为 1000Hz / 1ms 周期)
// 这是整个电机控制系统的"心跳"中断，所有实时控制逻辑在此统一调度:
//   1. 辨识阶段: 执行开环辨识任务
//   2. 辨识刚完成: 自动整定电流环 PI 参数 + 使能 SVPWM + 开启 FOC 闭环
//   3. 正常运行时: 执行 Motor_System_Task (位置/速度/Iq 级联控制)
//   4. 每 250ms 翻转一次状态 LED (2Hz 闪烁)
// 注意: 中断中不得执行耗时操作 (如 OLED 刷新、VOFA 发送等)，那些应放在 main 循环中
// ---------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
        Motor_Parameters_ControlTask1ms();
        Motor_System_UpdateOperatingState();
        
        // 调用系统普通任务 (1ms 周期刷新电位器和目标)
        Motor_System_Task();
        // ================================================
        // ============ 状态灯 2Hz 闪烁 ============
        Motor_StatusLed_Task();
        // =========================================
        
    }
}

/* 仅供前台 VOFA 观测：读取不关中断、不做 I/O、不调用算法。 */
float Motor_System_GetDebugPotTarget(void)
{
    return g_pot_target_filtered;
}

