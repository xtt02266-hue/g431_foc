#include "motor_system.h"
#include "motor_songs.h"
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
#include "motor_debug.h"
#include "motor_calibration.h"
#include "motor_friction_storage.h"
#include "mt6826s.h"
#include "motor_encoder_calibration.h"

#define MOTOR_ENCODER_MAX_SAMPLE_AGE_MS  2U      // 控制路径允许的有效编码器样本最大年龄，单位 ms
#define MOTOR_CURRENT_LOOP_CONTROL_BW_HZ 75.0f   // 正常控制时电流 PI 调谐带宽，单位 Hz
#define MOTOR_CURRENT_LOOP_MUSIC_BW_HZ   2000.0f // 音乐模式电流 PI 调谐带宽，单位 Hz

// 电位器低通滤波系数 (一阶 EMA, 1kHz 更新率)
// α 越小滤波越强、响应越慢; α=1.0 则无滤波
// α=0.10f → 截止频率 ~16Hz, 适合人手旋转电位器
// α=0.05f → 截止频率 ~8Hz,  更平滑但略有滞后感
#define POT_LPF_ALPHA 0.10f

// 电机系统运行状态。
motor_system_t g_motor_system = {
    .state = MOTOR_STATE_WAIT_PARAMETERS,
};

/* 运行请求、控制权和给定由前台命令与 TIM2 的 1 ms 控制路径共用。
 * 现有模式切换/标定恢复临界区及读取次序保持原样；此处仅记录所有权，不新增同步行为。
 * g_debug_* 是控制周期发布的观测值，g_cal_saved_* 保存标定前配置；时间戳单位 ms。
 */
static volatile uint8_t g_run_requested = 0U;
/* 故障采用锁存方式，传感器恢复后仍需调用 ClearFault。 */
static volatile uint8_t g_fault_latched = 0U;
/* 主循环接口和 1ms 中断共用；volatile 保证读取最新值，多变量切换另用临界区保护。 */
static volatile motor_control_mode_t g_control_mode = MOTOR_DEFAULT_CONTROL_MODE;
static volatile float g_torque_current_a = 0.0f; // 限幅后的 Iq 给定，供力矩分支送入电流环
static volatile motor_input_source_t g_input_source = MOTOR_INPUT_POT;
static volatile motor_control_owner_t g_control_owner = MOTOR_OWNER_LOCAL;
static volatile uint32_t g_host_last_heartbeat_ms = 0U;
static volatile uint32_t g_config_revision = 0U;
static volatile uint8_t g_force_drag_active = 0U;
static float g_force_drag_phase_rad = 0.0f;
static uint16_t g_force_drag_align_ticks = 0U;
static volatile float g_host_iq_limit_a = MOTOR_HOST_DEFAULT_IQ_LIMIT_A;
static float g_host_iq_applied_a = 0.0f;
static float g_host_iq_slew_a_per_s = MOTOR_HOST_IQ_SLEW_A_PER_S;
static float g_host_speed_target_rpm = 0.0f;
static float g_host_speed_applied_rpm = 0.0f;
static float g_host_speed_slew_rpm_per_s = MOTOR_HOST_SPEED_SLEW_RPM_PER_S;
/* 前台命令保存已接受数值，供协议应答读取；ISR 不写，避免停机抢占改变应答。
 * 与原输出指针一样，只在命令成功时更新；电流单位 A，速度单位 rpm。
 */
static float s_last_accepted_iq_a;
static float s_last_accepted_speed_rpm;
static float g_debug_speed_loop_iq_a = 0.0f;
static float g_debug_position_target_counts = 0.0f;
static float g_debug_position_actual_counts = 0.0f;
static float g_debug_position_error_counts = 0.0f;
static float g_debug_position_target_speed_rpm = 0.0f;
static float g_debug_friction_iq_a = 0.0f;
static float g_debug_cogging_iq_a = 0.0f;
static float g_continuous_angle_rad = 0.0f;
static float g_haptic_center_rad = 0.0f;
static uint16_t g_last_angle_counts = 0U;
static uint8_t g_angle_initialized = 0U;
static motor_haptic_params_t g_haptic_params = {0.03f, 0.002f, 0.05f, 0.05f, 90.0f, 24U};

static void motor_system_reset_outer_loops(void);
static void motor_system_force_safe_stop(void);
static void motor_system_update_operating_state(void);
static void motor_system_tune_current_loop_bandwidth(float bandwidth_hz);
static void motor_system_reset_control_history(void);
static uint8_t g_cal_saved_valid;
static motor_cogging_config_t g_cal_saved_cogging;
static motor_friction_config_t g_cal_saved_friction;
static float g_cal_saved_limit;
static float g_cal_saved_speed_slew;
static motor_control_mode_t g_cal_saved_mode;
static motor_input_source_t g_cal_saved_source;
static uint16_t g_cal_control_flags;
static uint32_t g_cal_last_crc, g_cal_last_transfer;
static float g_cal_saved_pos_max, g_cal_saved_pos_min;

/* 调用方已屏蔽中断；输出停止后再恢复标定前配置。 */
static void motor_system_restore_calibration(void)
{
    if (!g_cal_saved_valid) {
        return;
    }
    motor_cogging_set_config(g_cal_saved_cogging.enabled, g_cal_saved_cogging.gain, g_cal_saved_cogging.max_iq_a,
                             g_cal_saved_cogging.fade_start_rpm, g_cal_saved_cogging.fade_end_rpm,
                             g_cal_saved_cogging.phase_offset_counts);
    motor_feedforward_set_friction_config(g_cal_saved_friction.enabled, g_cal_saved_friction.coulomb_iq_a,
                                          g_cal_saved_friction.viscous_iq_a_per_rpm,
                                          g_cal_saved_friction.smooth_speed_rpm, g_cal_saved_friction.max_iq_a);
    g_host_iq_limit_a = g_cal_saved_limit;
    g_host_speed_slew_rpm_per_s = g_cal_saved_speed_slew;
    g_control_mode = g_cal_saved_mode;
    g_input_source = g_cal_saved_source;
    g_pi_pos.out_max = g_cal_saved_pos_max;
    g_pi_pos.out_min = g_cal_saved_pos_min;
    g_cal_saved_valid = 0U;
    ++g_config_revision;
}

// 电位器滤波量参与力矩/位置控制，同时供 VOFA+ 只读观测。
static float g_pot_target_filtered = 0.0f;    // 反向映射并滤波后的 ADC 计数，供位置/力矩模式共用
static uint8_t g_pot_filter_initialized = 0U; // 首次采样直接装入，避免从零滤波产生虚假反向力矩

// 浮点数绝对值
static float motor_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* 初始化系统状态与目标值，清除上次运行残留的力矩和电位器滤波状态。
 * 默认选择力矩模式，但实际输出仍须等待参数有效、编码器新鲜且状态机允许运行。
 * 保留外环初始化，是为了以后切回位置模式时可直接使用，并非默认执行外环。
 */
void motor_system_init(void)
{
    g_motor_system.state = MOTOR_STATE_WAIT_PARAMETERS;
    g_run_requested = 0U;
    g_fault_latched = 0U;

    g_control_mode = MOTOR_DEFAULT_CONTROL_MODE;
    g_input_source = MOTOR_INPUT_POT;
    g_control_owner = MOTOR_OWNER_LOCAL;
    g_host_iq_limit_a = MOTOR_HOST_DEFAULT_IQ_LIMIT_A;
    g_host_iq_applied_a = 0.0f;
    g_host_iq_slew_a_per_s = MOTOR_HOST_IQ_SLEW_A_PER_S;
    g_host_speed_target_rpm = 0.0f;
    g_host_speed_applied_rpm = 0.0f;
    g_host_speed_slew_rpm_per_s = MOTOR_HOST_SPEED_SLEW_RPM_PER_S;
    g_debug_speed_loop_iq_a = 0.0f;
    g_debug_position_target_counts = 0.0f;
    g_debug_position_actual_counts = 0.0f;
    g_debug_position_error_counts = 0.0f;
    g_debug_position_target_speed_rpm = 0.0f;
    g_debug_friction_iq_a = 0.0f;
    g_debug_cogging_iq_a = 0.0f;
    g_config_revision = 0U;
    g_force_drag_active = 0U;
    g_force_drag_phase_rad = 0.0f;
    g_force_drag_align_ticks = 0U;
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
    motor_speed_estimator_init(0.4f);

    // 初始化速度环 PID (参数已移至 motor_speed_loop.h)
    motor_speed_loop_init();

    // 初始化位置环 PID
    motor_position_loop_init();
    motor_feedforward_friction_init();
    motor_cogging_init();

    // 仅初始化音乐模块；当前自动播放宏为 0，需要主动调用播放接口才会发声。
    motor_music_init();
}

// 以 uint8_t 读取系统运行状态，数值对应 MOTOR_STATE_*。
uint8_t motor_system_get_state(void)
{
    return g_motor_system.state;
}

// 检查参数、故障、自校准和编码器新鲜度后接受运行请求；成功返回 1。
uint8_t motor_system_start_control(void)
{
    if (motor_encoder_cal_is_active() || motor_encoder_cal_requires_power_cycle()) {
        return 0U;
    }
#if !BOARD_SENSORED_CONTROL_ENABLE
    return 0U;
#endif
    if ((motor_parameters_is_ready() == 0U) || (g_fault_latched != 0U) ||
        (motor_encoder_is_data_fresh(MOTOR_ENCODER_MAX_SAMPLE_AGE_MS) == 0U)) {
        return 0U;
    }

    g_run_requested = 1U;
    return 1U;
}

// 在恢复原中断状态的临界区内撤销运行、归零输出并恢复标定前配置。
void motor_system_stop_control(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    motor_calibration_abort(CAL_CANCELLED);
    g_run_requested = 0U;
    motor_system_force_safe_stop();
    g_motor_system.state = MOTOR_STATE_STOPPED;
    motor_system_restore_calibration();
    if (primask == 0U) {
        __enable_irq();
    }
}

// 通过统一停机路径取消标定，保持输出撤销与配置恢复次序。
void motor_system_abort_calibration(void)
{
    motor_system_stop_control();
}

// 检查主机控制权和启动条件，保存原配置后启动指定采集；返回 MOTOR_CMD_*。
uint8_t motor_system_host_start_calibration(uint8_t mode, uint16_t point_count, uint8_t repeats,
                                            uint8_t retry_until_good, uint8_t policy)
{
    if (motor_encoder_cal_is_active()) {
        return MOTOR_CMD_BUSY;
    }
    if (motor_encoder_cal_requires_power_cycle()) {
        return MOTOR_CMD_NOT_READY;
    }
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (mode > 5U) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    if (motor_calibration_is_active() || motor_calibration_has_samples() || motor_music_is_playing() ||
        motor_parameters_is_busy()) {
        return MOTOR_CMD_BUSY;
    }
    int8_t direction = motor_identify_get_result().uvw_dir;
    if ((direction != 1 && direction != -1) || !motor_parameters_is_ready() || !motor_encoder_is_data_fresh(2U)) {
        return MOTOR_CMD_NOT_READY;
    }
    motor_cogging_config_t verification;
    motor_cogging_get_config(&verification);
    if ((mode == 2U || mode == 3U) &&
        (!verification.enabled || verification.gain <= 0.0f || verification.gain > 1.0f ||
         verification.max_iq_a <= 0.0f || verification.max_iq_a > .010001f || !motor_cogging_get_table_revision())) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!motor_calibration_begin_with_policy(mode, point_count, repeats, retry_until_good, policy)) {
        if (!mask) {
            __enable_irq();
        }
        return MOTOR_CMD_BUSY;
    }
    motor_cogging_get_config(&g_cal_saved_cogging);
    motor_feedforward_get_friction_config(&g_cal_saved_friction);
    g_cal_saved_limit = g_host_iq_limit_a;
    g_cal_saved_speed_slew = g_host_speed_slew_rpm_per_s;
    g_cal_saved_mode = g_control_mode;
    g_cal_saved_source = g_input_source;
    g_cal_saved_pos_max = g_pi_pos.out_max;
    g_cal_saved_pos_min = g_pi_pos.out_min;
    g_cal_saved_valid = 1U;
    motor_cogging_config_t disabled = g_cal_saved_cogging;
    if (mode != 3U) {
        disabled.enabled = 0U;
    }
    motor_cogging_set_config(disabled.enabled, disabled.gain, disabled.max_iq_a, disabled.fade_start_rpm,
                             disabled.fade_end_rpm, disabled.phase_offset_counts);
    if (mode == 4U || mode == 5U) {
        motor_friction_config_t friction_disabled = g_cal_saved_friction;
        friction_disabled.enabled = 0U;
        motor_feedforward_set_friction_config(friction_disabled.enabled, friction_disabled.coulomb_iq_a,
                                              friction_disabled.viscous_iq_a_per_rpm,
                                              friction_disabled.smooth_speed_rpm, friction_disabled.max_iq_a);
    }
    g_control_mode = mode == 5U ? MOTOR_CONTROL_POSITION : MOTOR_CONTROL_SPEED;
    g_input_source = MOTOR_INPUT_HOST;
    g_host_iq_limit_a = fminf(g_host_iq_limit_a, .20f);
    motor_system_reset_outer_loops();
    if (mode == 5U) {
        g_pi_pos.out_max = 30.0f;
        g_pi_pos.out_min = -30.0f;
    }
    g_cal_control_flags = 0U;
    g_cal_last_crc = mt6826s_get_crc_error_count();
    g_cal_last_transfer = mt6826s_get_transfer_error_count();
    g_host_speed_slew_rpm_per_s = 30.0f;
    g_host_speed_target_rpm = g_host_speed_applied_rpm = 0.0f;
    g_host_iq_applied_a = 0.0f;
    g_host_last_heartbeat_ms = HAL_GetTick();
    motor_cogging_runtime_start();
    g_run_requested = 1U;
    ++g_config_revision;
    if (!mask) {
        __enable_irq();
    }
    return MOTOR_CMD_OK;
}

/* 为什么需要：集中切换控制权，防止旧的速度积分或力矩目标带入新模式。
 * 功能：检查模式；发生切换时清零力矩、复位外环，再更新所选模式。
 * 参数：mode 为 POSITION 至 LIMIT 范围的模式；返回 1 表示接受，0 表示请求被拒绝。
 * 不改变运行请求、不自动停音乐；电位器模式下一周期会重新读取当前旋钮给定。
 */
uint8_t motor_system_set_control_mode(uint8_t mode)
{
    if ((mode < MOTOR_CONTROL_POSITION) || (mode > MOTOR_CONTROL_LIMIT)) {
        return 0U;
    }
    if ((g_control_owner == MOTOR_OWNER_HOST) || (g_motor_system.state == MOTOR_STATE_SENSORED_RUN)) {
        return 0U;
    }

    // 切换包含多个共享变量，暂时屏蔽中断，避免 1ms 控制任务看到切换一半的状态。
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (g_control_mode != mode) {
        g_torque_current_a = 0.0f;
        motor_system_reset_outer_loops();
        g_control_mode = (motor_control_mode_t)mode;
        g_input_source = (mode <= MOTOR_CONTROL_TORQUE) ? MOTOR_INPUT_POT : MOTOR_INPUT_INTERNAL;
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

// 读取当前给定来源；不切换控制权或运行状态。
uint8_t motor_system_get_input_source(void)
{
    return g_input_source;
}

// 读取当前本地/主机控制权，供协议令牌校验使用。
uint8_t motor_system_get_control_owner(void)
{
    return g_control_owner;
}

// 检查控制模式与给定来源的合法组合，沿用原有模式范围。
static uint8_t motor_system_is_mode_source_valid(uint8_t mode, uint8_t source)
{
    if (mode == MOTOR_CONTROL_POSITION) {
        return source == MOTOR_INPUT_POT;
    }
    if (mode == MOTOR_CONTROL_TORQUE) {
        return (source == MOTOR_INPUT_POT) || (source == MOTOR_INPUT_HOST);
    }
    if (mode == MOTOR_CONTROL_SPEED) {
        return source == MOTOR_INPUT_HOST;
    }
    return ((mode >= MOTOR_CONTROL_FREE) && (mode <= MOTOR_CONTROL_LIMIT) && (source == MOTOR_INPUT_INTERNAL));
}

// 在允许状态下交接主机控制权并初始化心跳和限幅；返回 MOTOR_CMD_*。
uint8_t motor_system_claim_host(void)
{
    if ((g_motor_system.state == MOTOR_STATE_IDENTIFYING) || (g_motor_system.state == MOTOR_STATE_MUSIC)) {
        return MOTOR_CMD_BUSY;
    }
    if (g_motor_system.state == MOTOR_STATE_FAULT) {
        return MOTOR_CMD_FAULT_ACTIVE;
    }
    if ((g_motor_system.state != MOTOR_STATE_STOPPED) && (g_motor_system.state != MOTOR_STATE_WAIT_PARAMETERS)) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (g_control_owner == MOTOR_OWNER_HOST) {
        return MOTOR_CMD_BUSY;
    }
    g_control_owner = MOTOR_OWNER_HOST;
    g_host_iq_limit_a = MOTOR_HOST_DEFAULT_IQ_LIMIT_A;
    g_host_iq_slew_a_per_s = MOTOR_HOST_IQ_SLEW_A_PER_S;
    g_host_last_heartbeat_ms = HAL_GetTick();
    return MOTOR_CMD_OK;
}

// 先停机再恢复本地力矩/电位器控制权，清除主机给定历史。
void motor_system_release_host(void)
{
    motor_system_stop_control();
    g_control_owner = MOTOR_OWNER_LOCAL;
    g_control_mode = MOTOR_CONTROL_TORQUE;
    g_input_source = MOTOR_INPUT_POT;
    g_host_iq_limit_a = MOTOR_HOST_DEFAULT_IQ_LIMIT_A;
    g_torque_current_a = 0.0f;
    g_host_iq_applied_a = 0.0f;
    g_host_iq_slew_a_per_s = MOTOR_HOST_IQ_SLEW_A_PER_S;
    g_host_speed_target_rpm = 0.0f;
    g_host_speed_applied_rpm = 0.0f;
    ++g_config_revision;
}

// 主机拥有控制权时更新 HAL 毫秒心跳，不改变本地控制状态。
void motor_system_host_heartbeat(void)
{
    if (g_control_owner == MOTOR_OWNER_HOST) {
        g_host_last_heartbeat_ms = HAL_GetTick();
    }
}

// 检查主机控制权、参数和方向后请求运行，并清除上次主机目标。
uint8_t motor_system_host_start(void)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state == MOTOR_STATE_FAULT) {
        return MOTOR_CMD_FAULT_ACTIVE;
    }
    if (motor_parameters_is_busy() != 0U) {
        return MOTOR_CMD_BUSY;
    }
    if ((g_control_mode >= MOTOR_CONTROL_FREE) && (g_control_mode <= MOTOR_CONTROL_SPEED)) {
        int8_t direction = motor_identify_get_result().uvw_dir;
        if ((direction != 1) && (direction != -1)) {
            return MOTOR_CMD_NOT_READY;
        }
    }
    if (motor_system_start_control() == 0U) {
        return MOTOR_CMD_NOT_READY;
    }
    g_torque_current_a = 0.0f;
    g_host_iq_applied_a = 0.0f;
    g_host_speed_target_rpm = 0.0f;
    g_host_speed_applied_rpm = 0.0f;
    motor_cogging_runtime_start();
    if ((g_control_mode >= MOTOR_CONTROL_SPRING) && (g_control_mode <= MOTOR_CONTROL_LIMIT)) {
        g_haptic_center_rad = g_continuous_angle_rad;
    }
    g_host_last_heartbeat_ms = HAL_GetTick();
    return MOTOR_CMD_OK;
}

// 接受诊断强拖请求：先固定相位对齐，再按原斜率升至 250 rpm。
uint8_t motor_system_host_start_force_drag250(void)
{
    if (motor_encoder_cal_is_active()) {
        return MOTOR_CMD_BUSY;
    }
    if (motor_encoder_cal_requires_power_cycle()) {
        return MOTOR_CMD_NOT_READY;
    }
    uint32_t primask;

    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state == MOTOR_STATE_FAULT) {
        return MOTOR_CMD_FAULT_ACTIVE;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (motor_parameters_is_busy() != 0U) {
        return MOTOR_CMD_BUSY;
    }
    if ((motor_parameters_is_ready() == 0U) || (motor_encoder_is_data_fresh(MOTOR_ENCODER_MAX_SAMPLE_AGE_MS) == 0U)) {
        return MOTOR_CMD_NOT_READY;
    }

    /* 使用固定相位先把转子吸到已知位置，再缓升至 250 rpm 旋转磁场。
     * 这里故意不采用辨识出的零点、极对数和方向：GB4310 已知为 14 极对，
     * 这样强拖结果不会被待排查的编码器换相角或辨识参数污染。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    motor_system_reset_outer_loops();
    g_force_drag_phase_rad = 0.0f;
    g_force_drag_align_ticks = MOTOR_FORCE_DRAG_ALIGN_MS;
    g_force_drag_active = 1U;
    g_run_requested = 1U;
    g_host_speed_target_rpm = MOTOR_FORCE_DRAG_SPEED_RPM;
    g_host_speed_applied_rpm = 0.0f;
    g_torque_current_a = 0.0f;
    if (primask == 0U) {
        __enable_irq();
    }
    g_host_last_heartbeat_ms = HAL_GetTick();
    return MOTOR_CMD_OK;
}

// 停机状态下先保存禁用齿槽配置，再请求强拖并启动编码器自校准。
uint8_t motor_system_host_start_encoder_calibration(void)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (motor_encoder_cal_is_active() || motor_calibration_is_active() || motor_calibration_has_samples() ||
        motor_parameters_is_busy()) {
        return MOTOR_CMD_BUSY;
    }
    if (motor_encoder_cal_requires_power_cycle()) {
        return MOTOR_CMD_NOT_READY;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (!motor_parameters_is_ready() || !motor_encoder_is_data_fresh(2U) || mt6826s_get_status() != 0U) {
        return MOTOR_CMD_NOT_READY;
    }
    /* 保留表及 CRC，在修改传感器映射前先保存禁用配置，
     * 避免重新上电时旧映射对应的补偿表自动使能。 */
    motor_cogging_config_t cogging;
    motor_cogging_get_config(&cogging);
    cogging.enabled = 0U;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    motor_cogging_set_config(cogging.enabled, cogging.gain, cogging.max_iq_a, cogging.fade_start_rpm,
                             cogging.fade_end_rpm, cogging.phase_offset_counts);
    ++g_config_revision;
    uint8_t saved = motor_cogging_save_to_flash();
    if (!mask) {
        __enable_irq();
    }
    if (!saved) {
        return MOTOR_CMD_NOT_READY;
    }
    motor_command_result_t r = motor_system_host_start_force_drag250();
    if (r == MOTOR_CMD_OK) {
        motor_encoder_cal_begin();
    }
    return r;
}

// 主机停机时校验并切换模式/来源；保持错误码和外环复位规则。
uint8_t motor_system_host_set_mode(uint8_t mode, uint8_t source)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (motor_system_is_mode_source_valid(mode, source) == 0U) {
        return MOTOR_CMD_INVALID_COMBINATION;
    }
    if ((g_control_mode != mode) || (g_input_source != source)) {
        g_control_mode = (motor_control_mode_t)mode;
        g_input_source = (motor_input_source_t)source;
        g_torque_current_a = 0.0f;
        g_host_iq_applied_a = 0.0f;
        g_host_speed_target_rpm = 0.0f;
        g_host_speed_applied_rpm = 0.0f;
        motor_system_reset_outer_loops();
        ++g_config_revision;
    }
    return MOTOR_CMD_OK;
}

// 主机力矩运行时设置 Iq（A）及斜率 A/s；返回 MOTOR_CMD_*。
uint8_t motor_system_host_set_iq(float iq_a, float slew_a_per_s)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if ((g_control_mode != MOTOR_CONTROL_TORQUE) || (g_input_source != MOTOR_INPUT_HOST)) {
        return MOTOR_CMD_INVALID_COMBINATION;
    }
    if (g_motor_system.state != MOTOR_STATE_SENSORED_RUN) {
        return MOTOR_CMD_NOT_RUNNING;
    }
    if (!isfinite(iq_a) || !isfinite(slew_a_per_s) || (slew_a_per_s < 0.0f) ||
        (slew_a_per_s > MOTOR_HOST_IQ_SLEW_MAX_A_PER_S)) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    if (iq_a > g_host_iq_limit_a) {
        iq_a = g_host_iq_limit_a;
    }
    if (iq_a < -g_host_iq_limit_a) {
        iq_a = -g_host_iq_limit_a;
    }
    g_torque_current_a = iq_a;
    g_host_iq_slew_a_per_s = slew_a_per_s;
    s_last_accepted_iq_a = iq_a;
    return MOTOR_CMD_OK;
}

// 读取最近成功命令接受的 Iq，不读取可能被 ISR 清零的运行目标。
float motor_system_get_last_accepted_iq(void)
{
    return s_last_accepted_iq_a;
}

// 主机停机时设置 Iq 限幅 A；保留原有数值边界与配置修订号。
uint8_t motor_system_host_set_iq_limit(float limit_a)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (!isfinite(limit_a) || (limit_a < 0.02f) || (limit_a > MOTOR_TORQUE_CURRENT_LIMIT_A)) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    g_host_iq_limit_a = limit_a;
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

// 主机停机时校验触觉参数；刚度单位 A/rad、阻尼 A/(rad/s)、限位角度 deg。
uint8_t motor_system_host_set_haptic_params(float spring_k_a_per_rad, float damping_b_a_per_rad_s,
                                            float detent_k_a_per_rad, float limit_k_a_per_rad,
                                            float limit_half_range_deg, uint16_t detent_count)
{
    const motor_haptic_params_t p = {spring_k_a_per_rad, damping_b_a_per_rad_s, detent_k_a_per_rad,
                                     limit_k_a_per_rad,  limit_half_range_deg,  detent_count};

    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (!isfinite(p.spring_k_a_per_rad) || !isfinite(p.damping_b_a_per_rad_s) || !isfinite(p.detent_k_a_per_rad) ||
        !isfinite(p.limit_k_a_per_rad) || !isfinite(p.limit_half_range_deg) || (p.spring_k_a_per_rad < 0.0f) ||
        (p.spring_k_a_per_rad > 5.0f) || (p.damping_b_a_per_rad_s < 0.0f) || (p.damping_b_a_per_rad_s > 1.0f) ||
        (p.detent_k_a_per_rad < 0.0f) || (p.detent_k_a_per_rad > 5.0f) || (p.limit_k_a_per_rad < 0.0f) ||
        (p.limit_k_a_per_rad > 5.0f) || (p.limit_half_range_deg < 5.0f) || (p.limit_half_range_deg > 180.0f) ||
        (p.detent_count < 1U) || (p.detent_count > 128U)) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    g_haptic_params = p;
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

// 复制当前触觉配置供协议读取；空指针沿用原有跳过行为。
void motor_system_get_haptic_params(motor_haptic_params_t *p)
{
    if (p != NULL) {
        *p = g_haptic_params;
    }
}

// 主机停机时校验并采用摩擦配置；不触发 Flash 保存。
uint8_t motor_system_host_set_friction_config(uint8_t enabled, float coulomb_iq_a, float viscous_iq_a_per_rpm,
                                              float smooth_speed_rpm, float max_iq_a)
{
    const motor_friction_config_t p = {enabled, coulomb_iq_a, viscous_iq_a_per_rpm, smooth_speed_rpm, max_iq_a};

    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (motor_feedforward_is_friction_config_valid(&p) == 0U) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    motor_feedforward_set_friction_config(p.enabled, p.coulomb_iq_a, p.viscous_iq_a_per_rpm, p.smooth_speed_rpm,
                                          p.max_iq_a);
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

// 读取摩擦配置，保持下层复制接口与空指针处理规则。
void motor_system_get_friction_config(motor_friction_config_t *p)
{
    motor_feedforward_get_friction_config(p);
}

// 主机停机且采集空闲时保存摩擦配置；保持错误返回与更新次序。
uint8_t motor_system_host_save_friction_config(uint8_t enabled, float coulomb_iq_a, float viscous_iq_a_per_rpm,
                                               float smooth_speed_rpm, float max_iq_a)
{
    const motor_friction_config_t p = {enabled, coulomb_iq_a, viscous_iq_a_per_rpm, smooth_speed_rpm, max_iq_a};

    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (motor_calibration_is_active() || motor_calibration_has_samples() || motor_parameters_is_busy()) {
        return MOTOR_CMD_BUSY;
    }
    if (!motor_feedforward_is_friction_config_valid(&p)) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    if (!motor_friction_storage_save(&p)) {
        return MOTOR_CMD_NOT_READY;
    }
    motor_feedforward_set_friction_config(p.enabled, p.coulomb_iq_a, p.viscous_iq_a_per_rpm, p.smooth_speed_rpm,
                                          p.max_iq_a);
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

// 主机停机时校验并采用齿槽配置，保持现有电流/增益边界。
uint8_t motor_system_host_set_cogging_config(uint8_t enabled, float gain, float max_iq_a, float fade_start_rpm,
                                             float fade_end_rpm, uint16_t phase_offset_counts)
{
    const motor_cogging_config_t p = {enabled, gain, max_iq_a, fade_start_rpm, fade_end_rpm, phase_offset_counts};

    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (motor_cogging_is_config_valid(&p) == 0U) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    motor_cogging_set_config(p.enabled, p.gain, p.max_iq_a, p.fade_start_rpm, p.fade_end_rpm, p.phase_offset_counts);
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

// 主机停机时开始固定容量表事务；输出事务号，错误返回 MOTOR_CMD_*。
uint8_t motor_system_host_begin_cogging_table(uint16_t count, uint32_t expected_crc)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (count != MOTOR_COGGING_TABLE_SIZE) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    if (motor_cogging_table_begin(count, expected_crc) == 0U) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    return MOTOR_CMD_OK;
}

// 校验主机控制权和停机状态后写入带长度的表片段；不提交活动表。
uint8_t motor_system_host_write_cogging_chunk(uint16_t transaction_id, uint16_t offset, uint8_t count,
                                              const int16_t *values)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    return motor_cogging_table_write_chunk(transaction_id, offset, count, values) ? MOTOR_CMD_OK
                                                                                  : MOTOR_CMD_INVALID_VALUE;
}

// 主机停机时提交收齐且 CRC 正确的表，并更新配置修订号。
uint8_t motor_system_host_commit_cogging_table(uint16_t transaction_id)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (motor_cogging_table_commit(transaction_id) == 0U) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

// 主机取得控制权后读取活动表片段；保持允许读取的状态规则。
uint8_t motor_system_host_read_cogging_chunk(uint16_t offset, uint8_t count, int16_t *values)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    return motor_cogging_table_read_chunk(offset, count, values) ? MOTOR_CMD_OK : MOTOR_CMD_INVALID_VALUE;
}

// 主机停机时保存活动齿槽表；返回保存结果对应的既有状态码。
uint8_t motor_system_host_save_cogging(void)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    return motor_cogging_save_to_flash() ? MOTOR_CMD_OK : MOTOR_CMD_NOT_READY;
}

// 复制当前齿槽补偿配置用于诊断与协议应答。
void motor_system_get_cogging_config(motor_cogging_config_t *p)
{
    motor_cogging_get_config(p);
}

// 主机速度运行时设置 rpm 及 rpm/s 斜率；零速/换向按原规则复位速度积分。
uint8_t motor_system_host_set_speed(float speed_rpm, float slew_rpm_per_s)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if ((g_control_mode != MOTOR_CONTROL_SPEED) || (g_input_source != MOTOR_INPUT_HOST)) {
        return MOTOR_CMD_INVALID_COMBINATION;
    }
    if (g_motor_system.state != MOTOR_STATE_SENSORED_RUN) {
        return MOTOR_CMD_NOT_RUNNING;
    }
    if (!isfinite(speed_rpm) || !isfinite(slew_rpm_per_s) || (slew_rpm_per_s < 0.0f) ||
        (slew_rpm_per_s > MOTOR_HOST_SPEED_SLEW_MAX_RPM_PER_S)) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    if (speed_rpm > MOTOR_HOST_SPEED_MAX_RPM) {
        speed_rpm = MOTOR_HOST_SPEED_MAX_RPM;
    }
    if (speed_rpm < -MOTOR_HOST_SPEED_MAX_RPM) {
        speed_rpm = -MOTOR_HOST_SPEED_MAX_RPM;
    }
    if ((speed_rpm == 0.0f) || ((g_host_speed_target_rpm * speed_rpm) < 0.0f)) {
        /* 停止或换向时清除旧方向积分，避免静摩擦挣脱后的反向冲击。 */
        pid_reset(&speed_pid);
    }
    g_host_speed_target_rpm = speed_rpm;
    g_host_speed_slew_rpm_per_s = slew_rpm_per_s;
    s_last_accepted_speed_rpm = speed_rpm;
    return MOTOR_CMD_OK;
}

// 读取最近成功命令接受的 rpm，保留协议应答与该命令的对应关系。
float motor_system_get_last_accepted_speed(void)
{
    return s_last_accepted_speed_rpm;
}

// 更新速度 PI；运行时保持输出连续性，停机时按原规则复位积分。
uint8_t motor_system_host_set_speed_pi(float kp, float ki)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (motor_calibration_is_active()) {
        return MOTOR_CMD_BUSY;
    }
    uint8_t running = g_motor_system.state == MOTOR_STATE_SENSORED_RUN;
    if (g_motor_system.state != MOTOR_STATE_STOPPED && !running) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (running && (g_control_mode != MOTOR_CONTROL_SPEED || g_input_source != MOTOR_INPUT_HOST)) {
        return MOTOR_CMD_INVALID_COMBINATION;
    }
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint8_t valid = motor_speed_loop_set_pi(kp, ki, running);
    if (valid) {
        ++g_config_revision;
    }
    if (!mask) {
        __enable_irq();
    }
    return valid ? MOTOR_CMD_OK : MOTOR_CMD_INVALID_VALUE;
}

// 按原读取顺序复制状态与观测量；保留既有同步方式和字段布局。
void motor_system_get_control_snapshot(motor_control_snapshot_t *s)
{
    if (s == NULL) {
        return;
    }
    s->state = g_motor_system.state;
    s->mode = g_control_mode;
    s->source = g_input_source;
    s->owner = g_control_owner;
    s->run_requested = g_run_requested;
    s->iq_limit_a = g_host_iq_limit_a;
    s->continuous_angle_rad = g_continuous_angle_rad;
    s->relative_center_angle_rad = ((g_control_mode >= MOTOR_CONTROL_SPRING) && (g_control_mode <= MOTOR_CONTROL_LIMIT))
                                       ? g_continuous_angle_rad - g_haptic_center_rad
                                       : 0.0f;
    s->target_speed_rpm = g_host_speed_applied_rpm;
    s->speed_loop_iq_a = g_debug_speed_loop_iq_a;
    s->position_target_counts = g_debug_position_target_counts;
    s->position_actual_counts = g_debug_position_actual_counts;
    s->position_error_counts = g_debug_position_error_counts;
    s->position_target_speed_rpm = g_debug_position_target_speed_rpm;
    s->friction_iq_a = g_debug_friction_iq_a;
    s->cogging_iq_a = g_debug_cogging_iq_a;
    s->cogging_effective_gain = motor_cogging_get_effective_gain();
    s->cogging_table_revision = motor_cogging_get_table_revision();
    s->cogging_table_crc = motor_cogging_get_active_table_crc();
    s->config_revision = g_config_revision;
}

/* 提供统一的模式查询入口，避免其他模块直接依赖内部变量。
 * 返回当前选择的模式；停机、故障或播放音乐时也保留此模式值。
 */
uint8_t motor_system_get_control_mode(void)
{
    return g_control_mode;
}

/* 为什么需要：为手动调试和 N·m 接口提供统一的电流限幅及运行条件检查。
 * 功能：接收有符号 Iq（A），拒绝 NaN/无穷大，并限制在允许电流范围。
 * 仅 HOST 来源且处于正常力矩运行时接受；返回 1 表示接受，0 表示拒绝。
 * 拒绝时清零存储目标；若正处于力矩模式，还立即清零当前 FOC 的 q 轴目标。
 * 默认电位器配置下不应调用此接口：即使调用清零，下一周期仍由电位器重新给定。
 */
uint8_t motor_system_set_torque_current(float iq_a)
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
    if ((g_input_source != MOTOR_INPUT_HOST) || (g_control_mode != MOTOR_CONTROL_TORQUE) ||
        (g_motor_system.state != MOTOR_STATE_SENSORED_RUN) || (g_run_requested == 0U) || (g_fault_latched != 0U)) {
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
static float motor_system_torque_to_iq(float torque_nm)
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
uint8_t motor_system_set_torque_nm(float torque_nm)
{
    const float kt = MOTOR_TORQUE_CONSTANT_NM_PER_A;
    if (!isfinite(kt) || (kt <= 0.0f) || !isfinite(torque_nm)) {
        (void)motor_system_set_torque_current(0.0f);
        return 0U;
    }
    return motor_system_set_torque_current(motor_system_torque_to_iq(torque_nm));
}

#if MOTOR_TORQUE_USE_POT
/* 为什么需要：将单路电位器变成可正反向调节的力矩旋钮，并抑制中点抖动。
 * 参数：filtered_pot 是经过4095-raw反向映射和低通滤波的12位ADC计数。
 * 返回：中点死区内为 0；死区外线性映射至 +/- MOTOR_TORQUE_POT_MAX_NM。
 * 减去死区后再归一化，使刚离开死区时从零连续增加，不会突然跳到一个非零力矩。
 * 原始 ADC 越小，反向映射后的给定越正；该符号不承诺实际轴端顺/逆时针方向。
 * 只生成力矩目标，电流限幅由后续 TorqueToIq 负责。
 */
static float motor_system_pot_torque_nm(float filtered_pot)
{
    float offset = filtered_pot - MOTOR_TORQUE_POT_CENTER;
    float magnitude = motor_abs_float(offset) - MOTOR_TORQUE_POT_DEADBAND;
    if (magnitude <= 0.0f) {
        return 0.0f;
    }
    float normalized = magnitude / (MOTOR_TORQUE_POT_CENTER - MOTOR_TORQUE_POT_DEADBAND);
    if (normalized > 1.0f) {
        normalized = 1.0f;
    }
    return ((offset > 0.0f) ? normalized : -normalized) * MOTOR_TORQUE_POT_MAX_NM;
}
#endif

// 传感器与参数恢复后主动解除故障锁存；保留既有接受条件。
uint8_t motor_system_clear_fault(void)
{
    if ((motor_parameters_is_ready() == 0U) || (motor_encoder_is_data_fresh(MOTOR_ENCODER_MAX_SAMPLE_AGE_MS) == 0U)) {
        return 0U;
    }

    g_fault_latched = 0U;
    return 1U;
}

// 通过参数模块请求辨识与保存；成功仅表示请求被接受。
uint8_t motor_system_identify_and_save(void)
{
    if (motor_encoder_cal_is_active() || motor_encoder_cal_requires_power_cycle()) {
        return 0U;
    }
    if ((motor_parameters_is_busy() != 0U) || (motor_encoder_is_data_fresh(MOTOR_ENCODER_MAX_SAMPLE_AGE_MS) == 0U)) {
        return 0U;
    }

    g_fault_latched = 0U;
    g_run_requested = 1U;
    return motor_parameters_identify_and_save();
}

// 按曲目 ID 请求有限次数播放；外部曲目先经数据登记，缓冲区须保持有效。
uint8_t motor_system_play_song(uint8_t song_id, uint16_t play_count)
{
    uint16_t note_count;
    const motor_music_note_t *song = motor_songs_get_score(song_id, &note_count);

    if (motor_encoder_cal_is_active() || motor_encoder_cal_requires_power_cycle()) {
        return 0U;
    }
    if ((song == NULL) || (note_count == 0U) || (play_count == 0U) || (motor_parameters_is_ready() == 0U) ||
        (g_fault_latched != 0U) || (motor_current_loop_is_enabled() == 0U)) {
        return 0U;
    }

    motor_music_start_times(song_id, play_count);
    return 1U;
}

// 请求循环播放曲目，沿用音乐期间暂停正常闭环控制的规则。
uint8_t motor_system_play_song_loop(uint8_t song_id)
{
    uint16_t note_count;
    const motor_music_note_t *song = motor_songs_get_score(song_id, &note_count);

    if (motor_encoder_cal_is_active() || motor_encoder_cal_requires_power_cycle()) {
        return 0U;
    }
    if ((song == NULL) || (note_count == 0U) || (motor_parameters_is_ready() == 0U) || (g_fault_latched != 0U) ||
        (motor_current_loop_is_enabled() == 0U)) {
        return 0U;
    }

    motor_music_start(song_id, 1U);
    return 1U;
}

// 撤销音乐播放请求；系统状态仍由原定时更新路径处理。
void motor_system_stop_music(void)
{
    motor_music_stop();
}

/* 为什么需要：位置/速度环历史复位在多处重复，集中为一处以免顺序漂移。
 * 功能：按固定顺序清目标速度、速度 PID、位置环和轨迹规划器历史量。
 * 参数：无；调用上下文：仅由模式切换、进入音乐和非运行清理复用。
 * 副作用：会清 PID 积分与轨迹状态；不修改目标力矩、q/d 给定。
 */
static void motor_system_reset_control_history(void)
{
    motor_speed_loop_set_target(0.0f);
    pid_reset(&speed_pid);
    motor_position_loop_reset();
    motor_trajectory_clear();
    g_host_speed_target_rpm = 0.0f;
    g_host_speed_applied_rpm = 0.0f;
    g_debug_speed_loop_iq_a = 0.0f;
    g_debug_position_target_counts = 0.0f;
    g_debug_position_actual_counts = 0.0f;
    g_debug_position_error_counts = 0.0f;
    g_debug_position_target_speed_rpm = 0.0f;
    g_debug_friction_iq_a = 0.0f;
    g_debug_cogging_iq_a = 0.0f;
}

// 清零速度/位置环和相关给定历史，避免旧积分进入下一种控制模式。
static void motor_system_reset_outer_loops(void)
{
    g_motor_system.run_data.target_torque_nm = 0.0f;
    /* 清掉位置/速度环历史量，避免功能切换后积分残留。 */
    motor_system_reset_control_history();
    g_foc_state.target_q = 0.0f;
    g_foc_state.target_d = 0.0f;
}

// 按指定带宽 Hz 重新配置电流 PI；保持原有 R/L 与阻尼参数。
static void motor_system_tune_current_loop_bandwidth(float bandwidth_hz)
{
    motor_identified_params_t id = motor_identify_get_result();

    motor_current_loop_auto_tune_pid_with_bandwidth(id.resistance, id.inductance, SYSTEM_BUS_VOLTAGE, bandwidth_hz);
}

// 统一撤销运行输出及相关历史，按现有次序禁用电流环与 PWM。
static void motor_system_force_safe_stop(void)
{
    motor_encoder_cal_notify_stop();
    /* 所有功能统一从这里撤销转矩输出。 */
    g_force_drag_active = 0U;
    g_force_drag_align_ticks = 0U;
    motor_open_loop_drive(0.0f, 0.0f);
    motor_cogging_runtime_stop();
    g_torque_current_a = 0.0f;
    motor_system_reset_outer_loops();
    if (motor_music_is_playing() != 0U) {
        motor_music_stop();
    }
    if (motor_current_loop_is_enabled() != 0U) {
        motor_current_loop_enable(0U);
    }
    if (svpwm_is_enabled() != 0U) {
        svpwm_disable();
    }
}

// 按故障、参数、音乐和运行请求确定系统状态，保留判定优先级。
static void motor_system_update_operating_state(void)
{
    if (motor_encoder_cal_requires_power_cycle()) {
        g_run_requested = 0U;
        motor_system_force_safe_stop();
        g_motor_system.state = MOTOR_STATE_STOPPED;
        return;
    }
#if !BOARD_SENSORED_CONTROL_ENABLE
    /* 当前无感配置仅提供开发基础，尚无启动和闭环切换流程。 */
    motor_system_force_safe_stop();
    g_motor_system.state = MOTOR_STATE_STOPPED;
    return;
#endif
    motor_parameters_status_t parameter_status = motor_parameters_get_status();

    /*
     * 辨识启动路径先停止闭环功能，然后由辨识模块拥有 PWM。
     * 保留辨识期间开环占空比，不在 1 ms 状态更新中覆盖它。
     */
    if (motor_parameters_is_busy() != 0U) {
        g_torque_current_a = 0.0f;
        g_motor_system.state = MOTOR_STATE_IDENTIFYING;
        return;
    }

    if (parameter_status == MOTOR_PARAMETERS_ERROR) {
        g_fault_latched = 1U;
    }

    if (parameter_status == MOTOR_PARAMETERS_NO_DATA) {
        motor_system_force_safe_stop();
        g_motor_system.state = MOTOR_STATE_WAIT_PARAMETERS;
        return;
    }

    /* MT6826S SPI是当前实际换相角度源，数据过期必须立即停机。 */
    if ((g_fault_latched != 0U) || (motor_encoder_is_data_fresh(MOTOR_ENCODER_MAX_SAMPLE_AGE_MS) == 0U)) {
        g_fault_latched = 1U;
        motor_system_force_safe_stop();
        g_motor_system.state = MOTOR_STATE_FAULT;
        return;
    }

    if (g_run_requested == 0U) {
        motor_system_force_safe_stop();
        g_motor_system.state = MOTOR_STATE_STOPPED;
        return;
    }

    /* 强拖是独立诊断路径：保留编码器新鲜度、参数和通信超时保护，
     * 但明确关闭闭环电流调制，PWM 只由 1 ms 开环旋转磁场更新。
     */
    if (g_force_drag_active != 0U) {
        if (motor_current_loop_is_enabled() != 0U) {
            motor_current_loop_enable(0U);
        }
        if (svpwm_is_enabled() == 0U) {
            svpwm_enable();
        }
        g_motor_system.state = MOTOR_STATE_SENSORED_RUN;
        return;
    }

    /* 参数和传感器均有效后，统一完成整定与闭环使能。 */
    if (motor_current_loop_is_enabled() == 0U) {
        motor_system_tune_current_loop_bandwidth(MOTOR_CURRENT_LOOP_CONTROL_BW_HZ);
        svpwm_enable();
        motor_current_loop_enable(1U);
    }

    g_motor_system.state = (motor_music_is_playing() != 0U) ? MOTOR_STATE_MUSIC : MOTOR_STATE_SENSORED_RUN;
}

/* 为什么需要：电位器采样、反向映射与滤波是力矩/位置模式共用的公共输入。
 * 功能：读取 ADC，做 4095-raw 反向映射，首次直接装入并做一阶 EMA 滤波。
 * 参数：无；返回映射到15位编码器范围的目标位置计数(0~32767)。
 * 副作用：更新 run_data.pot_raw、g_pot_target_filtered 与电位器初始化标记。
 * 调用上下文：每次 1ms 任务开始时由 Task 调用一次。
 */
static float motor_system_update_pot_target(void)
{
    // ========== 步骤 2: 读取电位器给定 ==========
    // ADC 原始值 → 反向映射 (4095-raw) → 低通滤波。
    // 力矩模式将该计数映射成 N·m；保留的位置模式将该计数作为位置目标。
    g_motor_system.run_data.pot_raw = pot_read_raw();

    float target_pos_raw = 4095.0f - (float)g_motor_system.run_data.pot_raw;
    /* 首次直接装入采样，避免中点电位器从滤波初值 0 爬升造成反向力矩。 */
    if (g_pot_filter_initialized == 0U) {
        g_pot_target_filtered = target_pos_raw;
        g_pot_filter_initialized = 1U;
    }
    // 一阶 EMA 低通滤波: filtered += α * (raw - filtered)
    g_pot_target_filtered += POT_LPF_ALPHA * (target_pos_raw - g_pot_target_filtered);
    return g_pot_target_filtered * (MOTOR_ENCODER_COUNTS_PER_REV_F / 4096.0f);
}

/* 为什么需要：力矩模式给定换算独立成函数，使 Task 只表达模式调度。
 * 功能：按配置从电位器（或手动保存值）生成受限 Iq，写 q/d 目标并反算调试力矩。
 * 参数：无；调用上下文：状态为 SENSORED_RUN 且模式为 TORQUE 时，由 Task 调用。
 * 副作用：更新 g_torque_current_a、g_foc_state.target_q/target_d，
 *         并可能更新 run_data.target_torque_nm。
 */
static void motor_system_run_torque_mode(void)
{
    if (g_input_source == MOTOR_INPUT_POT) {
        g_torque_current_a = motor_system_torque_to_iq(motor_system_pot_torque_nm(g_pot_target_filtered));
        g_host_iq_applied_a = g_torque_current_a;
    } else {
        float max_step = g_host_iq_slew_a_per_s * MOTOR_SYSTEM_TASK_DT_SEC;
        if (g_host_iq_slew_a_per_s <= 0.0f) {
            g_host_iq_applied_a = g_torque_current_a;
        } else if (g_torque_current_a > g_host_iq_applied_a + max_step) {
            g_host_iq_applied_a += max_step;
        } else if (g_torque_current_a < g_host_iq_applied_a - max_step) {
            g_host_iq_applied_a -= max_step;
        } else {
            g_host_iq_applied_a = g_torque_current_a;
        }
    }
    g_foc_state.target_q = g_host_iq_applied_a;
    g_foc_state.target_d = 0.0f;
    if (isfinite(MOTOR_TORQUE_CONSTANT_NM_PER_A) && (MOTOR_TORQUE_CONSTANT_NM_PER_A > 0.0f)) {
        // 从限幅后的 Iq 反算目标力矩用于调试；这是模型估算，不是力矩传感器读数。
        g_motor_system.run_data.target_torque_nm = g_host_iq_applied_a * MOTOR_TORQUE_CONSTANT_NM_PER_A;
    }
}

/* 为什么需要：位置/速度级联计算独立成函数，使 Task 只表达模式调度。
 * 功能：用目标位置与实际位置跑位置环和速度环，直接生成 Iq。
 * 参数：target_pos 为电位器目标位置计数；current_rpm 为按电磁正方向统一后的转速。
 * 调用上下文：状态为 SENSORED_RUN 且未选择力矩模式时，由 Task 调用。
 * 副作用：更新位置/速度 PID 和 g_foc_state.target_q。
 */
static void motor_system_run_position_mode(float target_pos, float current_rpm)
{
    float actual_pos = (float)motor_encoder_get_raw_angle();
    float position_error = target_pos - actual_pos;
    if (position_error > MOTOR_ENCODER_HALF_REV_F) {
        position_error -= MOTOR_ENCODER_COUNTS_PER_REV_F;
    } else if (position_error < -MOTOR_ENCODER_HALF_REV_F) {
        position_error += MOTOR_ENCODER_COUNTS_PER_REV_F;
    }
    g_debug_position_target_counts = target_pos;
    g_debug_position_actual_counts = actual_pos;
    g_debug_position_error_counts = position_error;

    float iq_max = g_cal_saved_valid && motor_calibration_is_point_mode() ? g_host_iq_limit_a : MOTOR_SPEED_PID_OUT_MAX;
    float iq_min =
        g_cal_saved_valid && motor_calibration_is_point_mode() ? -g_host_iq_limit_a : MOTOR_SPEED_PID_OUT_MIN;

    // 位置误差生成目标机械速度，也作为摩擦模型的输入。
    float target_mech_rpm = motor_position_loop_run(target_pos, actual_pos);
    g_debug_position_target_speed_rpm = target_mech_rpm;

    // 将机械期望转速乘以 uvw_dir 统一符号后给到速度环，防止正反馈。
    // target_signed_rpm 是按电磁转矩正方向统一符号的机械 RPM，没有乘极对数。
    float uvw_dir = (float)motor_identify_get_result().uvw_dir;
    float target_signed_rpm = target_mech_rpm * uvw_dir;

    /* 逐点标定仅在接近目标时使用已保存的摩擦辅助。
     * 辅助关闭并重新完成稳定窗口后才开始正式采样。 */
    float friction_iq;
    if (g_cal_saved_valid && motor_calibration_is_point_mode()) {
        friction_iq = motor_calibration_point_assist()
                          ? motor_feedforward_friction_with_config(&g_cal_saved_friction, target_mech_rpm) * uvw_dir
                          : 0.0f;
    } else {
        friction_iq = motor_feedforward_friction_compensation(target_mech_rpm) * uvw_dir;
    }
    friction_iq = fmaxf(iq_min, fminf(iq_max, friction_iq));
    g_debug_friction_iq_a = friction_iq;
    speed_pid.out_max = iq_max - friction_iq;
    speed_pid.out_min = iq_min - friction_iq;

    // 位置环输出的目标速度直接传给速度环。
    motor_speed_loop_set_target(target_signed_rpm);

    // 前馈占用的余量反馈给速度PI抗饱和；总电流仍服从原位置模式限幅。
    g_debug_speed_loop_iq_a = motor_speed_loop_update(current_rpm);
    g_debug_cogging_iq_a = 0.0f;
    g_foc_state.target_q = fmaxf(iq_min, fminf(iq_max, g_debug_speed_loop_iq_a + friction_iq));
    g_foc_state.target_d = 0.0f;
}

// 将主机 Iq（A）夹在当前正负电流限幅内。
static float motor_system_clamp_host_iq(float iq)
{
    if (!isfinite(iq)) {
        return 0.0f;
    }
    if (iq > g_host_iq_limit_a) {
        return g_host_iq_limit_a;
    }
    if (iq < -g_host_iq_limit_a) {
        return -g_host_iq_limit_a;
    }
    return iq;
}

// 以 1 ms 步长和 A/s 斜率更新已应用 Iq；斜率为零时沿用立即给定语义。
static float motor_system_slew_host_iq(float target_iq, float slew_a_per_s)
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

// 1 ms 速度模式计算速度 PI 和摩擦/齿槽前馈 Iq（A），保留叠加、限幅及发布次序。
static void motor_system_run_speed_mode(float mechanical_speed_rpm, int8_t identified_direction,
                                        uint16_t mechanical_angle_counts)
{
    const float max_step = g_host_speed_slew_rpm_per_s * MOTOR_SYSTEM_TASK_DT_SEC;
    if (g_host_speed_slew_rpm_per_s <= 0.0f) {
        g_host_speed_applied_rpm = g_host_speed_target_rpm;
    } else if (g_host_speed_target_rpm > g_host_speed_applied_rpm + max_step) {
        g_host_speed_applied_rpm += max_step;
    } else if (g_host_speed_target_rpm < g_host_speed_applied_rpm - max_step) {
        g_host_speed_applied_rpm -= max_step;
    } else {
        g_host_speed_applied_rpm = g_host_speed_target_rpm;
    }

    /* SPEED按目标方向提前给摩擦前馈。实际速度为零时也能提供起步电流，
     * 避免只能等待积分累积后突然挣脱。FREE仍使用实际速度，防止静止自驱。 */
    g_debug_friction_iq_a =
        motor_feedforward_friction_compensation(g_host_speed_applied_rpm) * (float)identified_direction;
    g_debug_cogging_iq_a = motor_cogging_compensation(mechanical_angle_counts, mechanical_speed_rpm);
    float feedforward_iq = g_debug_friction_iq_a + g_debug_cogging_iq_a;

    /* 将前馈占用的电流余量反馈给PI抗饱和，防止总Iq已经限幅时积分仍增长。 */
    speed_pid.out_max = g_host_iq_limit_a - feedforward_iq;
    speed_pid.out_min = -g_host_iq_limit_a - feedforward_iq;
    motor_speed_loop_set_target(g_host_speed_applied_rpm * (float)identified_direction);
    g_debug_speed_loop_iq_a = motor_speed_loop_update(mechanical_speed_rpm * (float)identified_direction);
    float iq = motor_system_clamp_host_iq(g_debug_speed_loop_iq_a + feedforward_iq);
    g_cal_control_flags = (fabsf(iq) >= g_host_iq_limit_a - 1e-6f) ? CAL_LIMIT : 0U;
    g_foc_state.target_q = motor_system_slew_host_iq(iq, MOTOR_SPEED_IQ_SLEW_A_PER_S);
    if (fabsf(g_foc_state.target_q - iq) > 1e-6f) {
        g_cal_control_flags |= CAL_SLEW;
    }
    g_foc_state.target_d = 0.0f;
}

// 周期任务：位置模式执行外环；力矩模式直接将给定 Iq 送入电流环。
// 调用频率: 1kHz (由 TIM2 中断驱动, dt = 1ms)
// 默认链路: 电位器 → 目标力矩(N·m) → Iq=T/Kt → FOC → SVPWM → 电机
void motor_system_task(void)
{
    static uint8_t music_was_active = 0U;
#if MOTOR_MUSIC_AUTOPLAY_DEMO
    static uint8_t demo_autoplay_checked = 0U;
#endif

    // ========== 步骤 1: 速度估算（20/10/5 ms自适应滚动位置窗） ==========
    // 始终每1ms更新反馈；带迟滞切换窗口，兼顾低速分辨率和中高速延迟。
    if (motor_encoder_is_data_fresh(MOTOR_ENCODER_MAX_SAMPLE_AGE_MS) != 0U) {
        g_motor_system.run_data.speed_rpm = motor_speed_estimator_update_adaptive(g_motor_system.run_data.speed_rpm);
    } else {
        g_motor_system.run_data.speed_rpm = 0.0f;
    }

    // 如果编码器接线/电机相序不同，编码器读数的正反方向可能会和 Iq 的正扭矩方向相反。
    // 我们必须用系统辨识出的 uvw_dir (1 或 -1) 来把转速的正负号与电机电磁正方向统一，否则会导致 PID 变成正反馈（越差越使劲）！
    int8_t identified_direction = motor_identify_get_result().uvw_dir;
    if ((identified_direction != 1) && (identified_direction != -1)) {
        identified_direction = 1;
    }
    float current_rpm = g_motor_system.run_data.speed_rpm * (float)identified_direction;

    /* 在机械坐标中展开单圈角度，供触觉模式跨零点连续计算。 */
    uint16_t angle_counts = motor_encoder_get_raw_angle();
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
        ((uint32_t)(HAL_GetTick() - g_host_last_heartbeat_ms) > MOTOR_HOST_HEARTBEAT_TIMEOUT_MS)) {
        motor_system_release_host();
    }

    if (g_cal_saved_valid) {
        if (g_fault_latched && motor_parameters_get_status() != MOTOR_PARAMETERS_ERROR) {
            motor_calibration_abort(CAL_SENSOR);
        }
        if (g_motor_system.state != MOTOR_STATE_SENSORED_RUN || g_control_owner != MOTOR_OWNER_HOST ||
            !g_run_requested) {
            motor_calibration_abort(CAL_CONTROL_LOST);
        }
        if (motor_calibration_is_active()) {
            if (motor_calibration_is_point_mode()) {
                uint32_t crc = mt6826s_get_crc_error_count(), transfer = mt6826s_get_transfer_error_count();
                uint16_t point_flags = g_cal_control_flags;
                if (crc != g_cal_last_crc || transfer != g_cal_last_transfer) {
                    point_flags |= CAL_BAD_SENSOR;
                }
                g_cal_last_crc = crc;
                g_cal_last_transfer = transfer;
                motor_calibration_step_point(angle_counts, g_motor_system.run_data.speed_rpm, point_flags,
                                             motor_encoder_is_data_fresh(2U));
            } else {
                float next_target =
                    motor_calibration_step(angle_counts, g_motor_system.run_data.speed_rpm, g_host_speed_applied_rpm,
                                           g_cal_control_flags, motor_encoder_is_data_fresh(2U));
                if (next_target != g_host_speed_target_rpm) {
                    pid_reset(&speed_pid);
                }
                g_host_speed_target_rpm = next_target;
            }
        }
        if (!motor_calibration_is_active()) {
            uint32_t mask = __get_PRIMASK();
            __disable_irq();
            g_run_requested = 0U;
            motor_system_force_safe_stop();
            if (g_motor_system.state != MOTOR_STATE_FAULT) {
                g_motor_system.state = MOTOR_STATE_STOPPED;
            }
            motor_system_restore_calibration();
            if (!mask) {
                __enable_irq();
            }
            return;
        }
    }

    if ((g_force_drag_active != 0U) && (g_motor_system.state == MOTOR_STATE_SENSORED_RUN)) {
        g_foc_state.target_q = 0.0f;
        g_foc_state.target_d = 0.0f;
        g_debug_speed_loop_iq_a = 0.0f;
        g_debug_friction_iq_a = 0.0f;
        g_debug_cogging_iq_a = 0.0f;
        g_motor_system.run_data.target_torque_nm = 0.0f;

        float drive_amplitude = MOTOR_FORCE_DRAG_ALIGN_AMPLITUDE;
        if (g_force_drag_align_ticks != 0U) {
            --g_force_drag_align_ticks;
            g_host_speed_applied_rpm = 0.0f;
        } else {
            const float speed_step = MOTOR_FORCE_DRAG_RAMP_RPM_PER_S * MOTOR_SYSTEM_TASK_DT_SEC;
            if (g_host_speed_applied_rpm < MOTOR_FORCE_DRAG_SPEED_RPM) {
                g_host_speed_applied_rpm += speed_step;
                if (g_host_speed_applied_rpm > MOTOR_FORCE_DRAG_SPEED_RPM) {
                    g_host_speed_applied_rpm = MOTOR_FORCE_DRAG_SPEED_RPM;
                }
            }
            const float phase_step = g_host_speed_applied_rpm * (6.28318530718f / 60.0f) *
                                     (float)MOTOR_FORCE_DRAG_POLE_PAIRS * MOTOR_SYSTEM_TASK_DT_SEC;
            g_force_drag_phase_rad += phase_step;
            while (g_force_drag_phase_rad >= 6.28318530718f) {
                g_force_drag_phase_rad -= 6.28318530718f;
            }
            while (g_force_drag_phase_rad < 0.0f) {
                g_force_drag_phase_rad += 6.28318530718f;
            }
            drive_amplitude =
                MOTOR_FORCE_DRAG_ALIGN_AMPLITUDE + (MOTOR_FORCE_DRAG_RUN_AMPLITUDE - MOTOR_FORCE_DRAG_ALIGN_AMPLITUDE) *
                                                       (g_host_speed_applied_rpm / MOTOR_FORCE_DRAG_SPEED_RPM);
        }
        motor_open_loop_drive(g_force_drag_phase_rad, drive_amplitude);
        return;
    }

    float target_pos = motor_system_update_pot_target();
    g_foc_state.target_d = 0.0f;
    motor_cogging_task1ms((g_motor_system.state == MOTOR_STATE_SENSORED_RUN) &&
                          ((g_control_mode == MOTOR_CONTROL_SPEED) || (g_control_mode == MOTOR_CONTROL_FREE)));

#if MOTOR_MUSIC_AUTOPLAY_DEMO
    if ((g_motor_system.state == MOTOR_STATE_SENSORED_RUN) && (demo_autoplay_checked == 0U) &&
        !motor_calibration_is_active()) {
        demo_autoplay_checked = 1U;
        motor_music_start_demo();
        g_motor_system.state = MOTOR_STATE_MUSIC;
    }
#endif

    // 音乐播放时由音频模块独占 q 轴给定，暂停正常位置/力矩输出以免相互叠加。
    if (g_motor_system.state == MOTOR_STATE_MUSIC) {
        g_motor_system.run_data.target_torque_nm = 0.0f;
        if (music_was_active == 0U) {
            g_torque_current_a = 0.0f;
            motor_system_tune_current_loop_bandwidth(MOTOR_CURRENT_LOOP_MUSIC_BW_HZ);
            motor_system_reset_control_history();
            music_was_active = 1U;
        }

        motor_music_task1ms();
        g_foc_state.target_q = 0.0f;
        g_foc_state.target_d = 0.0f;
        return;
    }

    if (music_was_active != 0U) {
        motor_system_tune_current_loop_bandwidth(MOTOR_CURRENT_LOOP_CONTROL_BW_HZ);
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
            motor_system_run_torque_mode();
            return;
        }
        if (g_control_mode == MOTOR_CONTROL_SPEED || (g_cal_saved_valid && motor_calibration_is_point_mode())) {
            if (motor_calibration_is_point_mode() && g_cal_saved_valid) {
                motor_system_run_position_mode(motor_calibration_point_target(), current_rpm);
                speed_pid.out_max = g_host_iq_limit_a;
                speed_pid.out_min = -g_host_iq_limit_a;
                float iq = g_foc_state.target_q;
                g_cal_control_flags = fabsf(iq) >= g_host_iq_limit_a - 1e-6f ? CAL_LIMIT : 0U;
                g_foc_state.target_q = fmaxf(-g_host_iq_limit_a, fminf(g_host_iq_limit_a, iq));
            } else {
                motor_system_run_speed_mode(g_motor_system.run_data.speed_rpm, identified_direction, angle_counts);
            }
            motor_cal_status_t capture_status;
            motor_calibration_get_status(&capture_status);
            if (motor_calibration_is_active() &&
                (!motor_calibration_is_point_mode() || capture_status.phase == CAL_CAPTURE)) {
                motor_cal_sample_t sample = {0};
                sample.tick_ms = HAL_GetTick();
                sample.flags = motor_calibration_sample_flags() | g_cal_control_flags;
                sample.target_rpm =
                    motor_calibration_is_point_mode() ? g_debug_position_target_speed_rpm : g_host_speed_applied_rpm;
                sample.speed_rpm = g_motor_system.run_data.speed_rpm;
                sample.window_rpm = speed_est.instant_speed_rpm;
                sample.pi_iq = g_debug_speed_loop_iq_a;
                sample.friction_iq = g_debug_friction_iq_a;
                sample.cogging_iq = g_debug_cogging_iq_a;
                sample.sensor_status = mt6826s_get_status();
                sample.window_ticks = motor_speed_estimator_get_window_ticks();
                sample.sensor_age_ms = motor_encoder_get_sample_age_ms();
                sample.crc_errors = mt6826s_get_crc_error_count();
                sample.transfer_errors = mt6826s_get_transfer_error_count();
                if (!motor_encoder_is_data_fresh(2U)) {
                    sample.flags |= CAL_BAD_SENSOR;
                }
                uint32_t mask = __get_PRIMASK();
                __disable_irq();
                motor_calibration_publish_control(&sample);
                if (!mask) {
                    __enable_irq();
                }
            }
            return;
        }
        if ((g_control_mode >= MOTOR_CONTROL_FREE) && (g_control_mode <= MOTOR_CONTROL_LIMIT)) {
            const float omega = g_motor_system.run_data.speed_rpm * (6.28318530718f / 60.0f);
            const float rel = g_continuous_angle_rad - g_haptic_center_rad;
            float i_mech = 0.0f;
            if (g_control_mode == MOTOR_CONTROL_FREE) {
                i_mech = motor_feedforward_free_compensation(g_motor_system.run_data.speed_rpm);
            } else if (g_control_mode == MOTOR_CONTROL_DAMPING) {
                i_mech = -g_haptic_params.damping_b_a_per_rad_s * omega;
            } else if (g_control_mode == MOTOR_CONTROL_SPRING) {
                i_mech = -g_haptic_params.spring_k_a_per_rad * rel - g_haptic_params.damping_b_a_per_rad_s * omega;
            } else if (g_control_mode == MOTOR_CONTROL_DETENT) {
                float spacing = 6.28318530718f / (float)g_haptic_params.detent_count;
                float nearest = roundf(rel / spacing) * spacing;
                i_mech = -g_haptic_params.detent_k_a_per_rad * (rel - nearest) -
                         g_haptic_params.damping_b_a_per_rad_s * omega;
            } else if (g_control_mode == MOTOR_CONTROL_LIMIT) {
                float half = g_haptic_params.limit_half_range_deg * 0.01745329252f;
                if (rel > half) {
                    i_mech = -g_haptic_params.limit_k_a_per_rad * (rel - half) -
                             g_haptic_params.damping_b_a_per_rad_s * omega;
                } else if (rel < -half) {
                    i_mech = -g_haptic_params.limit_k_a_per_rad * (rel + half) -
                             g_haptic_params.damping_b_a_per_rad_s * omega;
                }
            }
            g_debug_friction_iq_a =
                (g_control_mode == MOTOR_CONTROL_FREE) ? i_mech * (float)identified_direction : 0.0f;
            g_debug_cogging_iq_a = (g_control_mode == MOTOR_CONTROL_FREE)
                                       ? motor_cogging_compensation(angle_counts, g_motor_system.run_data.speed_rpm)
                                       : 0.0f;
            float iq = i_mech * (float)identified_direction + g_debug_cogging_iq_a;
            if (!isfinite(iq)) {
                iq = 0.0f;
            }
            if (iq > g_host_iq_limit_a) {
                iq = g_host_iq_limit_a;
            }
            if (iq < -g_host_iq_limit_a) {
                iq = -g_host_iq_limit_a;
            }
            iq = motor_system_slew_host_iq(iq, MOTOR_IQ_SLEW_A_PER_S);
            g_debug_speed_loop_iq_a = 0.0f;
            g_foc_state.target_q = iq;
            g_foc_state.target_d = 0.0f;
            return;
        }
    }

    // 3. FOC 闭环开始工作后，开始让位置环介入产生速度，速度环介入产生 Iq
    // 控制链路: 电位器目标 → 位置环 (P) → 目标机械转速 → 乘 uvw_dir → 目标电磁转速
    //          → 速度环 (PI) → Iq 电流 → FOC 电流环
    if (g_motor_system.state == MOTOR_STATE_SENSORED_RUN) {
        motor_system_run_position_mode(target_pos, current_rpm);
    } else {
        // 辨识未完成: 清零所有 PID 积分和电流输出，防止误动作
        motor_system_reset_control_history();
        g_foc_state.target_q = 0.0f;
        g_foc_state.target_d = 0.0f;
    }

    // D 轴弱磁控制 (Field Weakening) 策略 — 当前禁用
    // 弱磁用于高速时主动注入负 Id 来压低反电动势，扩展转速范围
    //g_foc_state.target_d = motor_speed_loop_field_weakening(current_rpm);
}

// 状态灯 2Hz 闪烁任务 (设计为 1ms 调用一次)
static void motor_status_led_task(void)
{
    // 频率 2Hz = 周期 500ms = 亮 250ms，灭 250ms
    static uint16_t led_cnt = 0;
    led_cnt++;
    if (led_cnt >= 250) {
        led_cnt = 0;
        led_toggle();
    }
}

// 定时器更新中断回调函数 (TIM2, 假设为 1000Hz / 1ms 周期)
// 这是整个电机控制系统的"心跳"中断，所有实时控制逻辑在此统一调度:
//   1. 辨识阶段: 执行开环辨识任务
//   2. 辨识刚完成: 自动整定电流环 PI 参数 + 使能 SVPWM + 开启 FOC 闭环
//   3. 正常运行时: 执行 motor_system_task (位置/速度/Iq 级联控制)
//   4. 每 250ms 翻转一次状态 LED (2Hz 闪烁)
// 注意: 中断中不得执行耗时操作 (如 OLED 刷新、VOFA 发送等)，那些应放在 main 循环中
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2) {
        motor_parameters_control_task1ms();
        motor_system_update_operating_state();

        // 调用系统普通任务 (1ms 周期刷新电位器和目标)
        motor_system_task();
        // ============ 状态灯 2Hz 闪烁 ============
        motor_status_led_task();
    }
}

/* 仅供前台 VOFA 观测：读取不关中断、不做 I/O、不调用算法。 */
float motor_system_get_debug_pot_target(void)
{
    return g_pot_target_filtered;
}
