#include "motor_control.h"
#include "motor_context.h"
#include "motor_config.h"
#include "motor_host.h"
#include "motor_songs.h"
#include "motor_current_loop.h"
#include "motor_speed_loop.h"
#include "motor_position_loop.h"
#include "motor_friction.h"
#include "motor_trajectory.h"
#include "motor_encoder.h"
#include "motor_identify.h"
#include "svpwm.h"
#include "motor_music.h"
#include "motor_parameters.h"
#include "motor_calibration.h"
#include "motor_encoder_calibration.h"
#include <math.h>
#include <stddef.h>

// 浮点数绝对值
float motor_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* 初始化系统状态与目标值，清除上次运行残留的力矩和电位器滤波状态。
 * 默认选择力矩模式，但实际输出仍须等待参数有效、编码器新鲜且状态机允许运行。
 * 保留外环初始化，是为了以后切回位置模式时可直接使用，并非默认执行外环。
 */
void motor_control_init(void)
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
void motor_system_reset_control_history(void)
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
void motor_system_reset_outer_loops(void)
{
    g_motor_system.run_data.target_torque_nm = 0.0f;
    /* 清掉位置/速度环历史量，避免功能切换后积分残留。 */
    motor_system_reset_control_history();
    g_foc_state.target_q = 0.0f;
    g_foc_state.target_d = 0.0f;
}

// 按指定带宽 Hz 重新配置电流 PI；保持原有 R/L 与阻尼参数。
void motor_system_tune_current_loop_bandwidth(float bandwidth_hz)
{
    motor_identified_params_t id = motor_identify_get_result();

    motor_current_loop_auto_tune_pid_with_bandwidth(id.resistance, id.inductance, SYSTEM_BUS_VOLTAGE, bandwidth_hz);
}

// 统一撤销运行输出及相关历史，按现有次序禁用电流环与 PWM。
void motor_system_force_safe_stop(void)
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
void motor_control_update_state(void)
{
    if (motor_encoder_cal_requires_power_cycle()) {
        g_run_requested = 0U;
        motor_system_force_safe_stop();
        g_motor_system.state = MOTOR_STATE_STOPPED;
        return;
    }
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
