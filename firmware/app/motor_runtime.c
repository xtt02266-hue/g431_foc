#include "motor_control.h"
#include "motor_host.h"
#include "motor_context.h"
#include "motor_songs.h"
#include "motor_current_loop.h"
#include "motor_speed_loop.h"
#include "motor_encoder.h"
#include "motor_identify.h"
#include "motor_parameters.h"
#include "motor_music.h"
#include "motor_calibration.h"
#include "mt6826s.h"
#include <math.h>
#include <stddef.h>

// TIM2 独占更新的上次机械角计数，用于 15 位角度解回绕；初始化标记由生命周期复位。
static uint16_t g_last_angle_counts = 0U;
static uint8_t g_angle_initialized = 0U;
// TIM2 按编码器计数累计连续机械位置，避免浮点逐拍累加丢失小位移。
static int64_t s_continuous_angle_counts = 0;
static uint32_t g_cal_last_crc, g_cal_last_transfer;

// 生命周期复位初始化标记，下一周期重新建立连续角度起点。
void motor_runtime_reset_feedback(void)
{
    g_angle_initialized = 0U;
    s_continuous_angle_counts = 0;
}

// 获取连续机械位置计数；仅限 TIM2 或已屏蔽中断的调用方，64 位读取不保证原子性。
int64_t motor_runtime_get_position_counts(void)
{
    return s_continuous_angle_counts;
}

// 标定会话在原启动临界区建立传感器错误基线。
void motor_runtime_begin_calibration(void)
{
    g_cal_control_flags = 0U;
    g_cal_last_crc = mt6826s_get_crc_error_count();
    g_cal_last_transfer = mt6826s_get_transfer_error_count();
}

typedef struct {
    float control_speed_rpm;
    uint16_t angle_counts;
    int8_t identified_direction;
} motor_runtime_feedback_t;

// 更新本周期速度和连续机械角；这里只封装原读取顺序，不构成跨中断快照。
static void motor_runtime_update_feedback(motor_runtime_feedback_t *feedback)
{
    // 始终每1ms更新反馈；带迟滞切换窗口，兼顾低速分辨率和中高速延迟。
    if (motor_encoder_is_data_fresh(MOTOR_ENCODER_MAX_SAMPLE_AGE_MS) != 0U) {
        g_motor_system.run_data.speed_rpm = motor_speed_estimator_update_adaptive(g_motor_system.run_data.speed_rpm);
    } else {
        g_motor_system.run_data.speed_rpm = 0.0f;
    }

    // 将机械转速符号转换为电磁方向，避免外环形成正反馈。
    int8_t identified_direction = motor_identify_get_result().uvw_dir;
    if ((identified_direction != 1) && (identified_direction != -1)) {
        identified_direction = 1;
    }
    float current_rpm = g_motor_system.run_data.speed_rpm * (float)identified_direction;

    /* 在机械坐标中展开单圈角度，供触觉模式跨零点连续计算。 */
    uint16_t angle_counts = motor_encoder_get_raw_angle();
    if (g_angle_initialized == 0U) {
        g_last_angle_counts = angle_counts;
        s_continuous_angle_counts = (int64_t)angle_counts;
        g_angle_initialized = 1U;
    } else {
        int32_t delta = (int32_t)angle_counts - (int32_t)g_last_angle_counts;
        if (delta > (int32_t)MOTOR_ENCODER_HALF_REV_U32) {
            delta -= (int32_t)MOTOR_ENCODER_COUNTS_PER_REV_U32;
        }
        if (delta < -(int32_t)MOTOR_ENCODER_HALF_REV_U32) {
            delta += (int32_t)MOTOR_ENCODER_COUNTS_PER_REV_U32;
        }
        s_continuous_angle_counts += (int64_t)delta;
        g_last_angle_counts = angle_counts;
    }
    // 连续弧度仅作近似遥测；触觉控制先在整数计数域减去参考点，再转换为弧度。
    g_continuous_angle_rad = (float)s_continuous_angle_counts * MOTOR_ENCODER_RAD_PER_COUNT;

    feedback->control_speed_rpm = current_rpm;
    feedback->angle_counts = angle_counts;
    feedback->identified_direction = identified_direction;
}

// 主机心跳超时后通过统一入口停止并释放控制权。
static void motor_runtime_check_host_timeout(void)
{
    if ((g_control_owner == MOTOR_OWNER_HOST) &&
        ((uint32_t)(HAL_GetTick() - g_host_last_heartbeat_ms) > MOTOR_HOST_HEARTBEAT_TIMEOUT_MS)) {
        motor_system_release_host();
    }
}

// 推进标定并在结束时停机恢复；返回 1 表示本周期已处理完毕。
static uint8_t motor_runtime_step_calibration(const motor_runtime_feedback_t *feedback)
{
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
                motor_calibration_step_point(feedback->angle_counts, g_motor_system.run_data.speed_rpm, point_flags,
                                             motor_encoder_is_data_fresh(2U));
            } else {
                float next_target =
                    motor_calibration_step(feedback->angle_counts, g_motor_system.run_data.speed_rpm, g_host_speed_applied_rpm,
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
            return 1U;
        }
    }

    return 0U;
}

// 推进强拖对齐、斜坡和开环输出；返回 1 表示本周期已处理完毕。
static uint8_t motor_runtime_handle_force_drag(void)
{
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
        return 1U;
    }

    return 0U;
}

// 处理音乐进入、播放和退出带宽；返回 1 表示本周期已处理完毕。
static uint8_t motor_runtime_handle_music(void)
{
    static uint8_t music_was_active = 0U;
#if MOTOR_MUSIC_AUTOPLAY_DEMO
    static uint8_t demo_autoplay_checked = 0U;
    if ((g_motor_system.state == MOTOR_STATE_SENSORED_RUN) && (demo_autoplay_checked == 0U) &&
        !motor_calibration_is_active()) {
        demo_autoplay_checked = 1U;
        motor_music_start_demo();
        g_motor_system.state = MOTOR_STATE_MUSIC;
    }
#endif

    // 音乐模块独占调制电流；正常外环只负责进入/退出时的历史和带宽。
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
        return 1U;
    }
    if (music_was_active != 0U) {
        motor_system_tune_current_loop_bandwidth(MOTOR_CURRENT_LOOP_CONTROL_BW_HZ);
    }
    music_was_active = 0U;
    g_motor_system.run_data.target_torque_nm = 0.0f;
    return 0U;
}

// 在速度/点位控制之后发布采样及传感器质量字段，保留原临界区。
static void motor_runtime_publish_calibration_sample(void)
{
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
}

// 按既有优先级计算力矩、速度、点位、触觉或位置输出。
static void motor_runtime_run_control_mode(const motor_runtime_feedback_t *feedback, float target_pos)
{
    // 点位标定保留原有覆盖优先级，其采样在控制输出发布后进行。
    if (g_motor_system.state == MOTOR_STATE_SENSORED_RUN) {
        if (g_control_mode == MOTOR_CONTROL_TORQUE) {
            g_debug_speed_loop_iq_a = 0.0f;
            // 力矩给定叠加本周期反馈对应的补偿，再发布受限总电流。
            motor_system_run_torque_mode(g_motor_system.run_data.speed_rpm, feedback->identified_direction,
                                         feedback->angle_counts);
            return;
        }
        if (g_control_mode == MOTOR_CONTROL_SPEED || (g_cal_saved_valid && motor_calibration_is_point_mode())) {
            if (motor_calibration_is_point_mode() && g_cal_saved_valid) {
                motor_system_run_position_mode(motor_calibration_point_target(), feedback->control_speed_rpm);
                speed_pid.out_max = g_host_iq_limit_a;
                speed_pid.out_min = -g_host_iq_limit_a;
                float iq = g_foc_state.target_q;
                g_cal_control_flags = fabsf(iq) >= g_host_iq_limit_a - 1e-6f ? CAL_LIMIT : 0U;
                g_foc_state.target_q = fmaxf(-g_host_iq_limit_a, fminf(g_host_iq_limit_a, iq));
            } else {
                motor_system_run_speed_mode(g_motor_system.run_data.speed_rpm, feedback->identified_direction,
                                             feedback->angle_counts);
            }
            motor_runtime_publish_calibration_sample();
            return;
        }
        if ((g_control_mode >= MOTOR_CONTROL_FREE) && (g_control_mode <= MOTOR_CONTROL_LIMIT)) {
            motor_haptic_task(feedback->identified_direction, feedback->angle_counts);
            return;
        }
    }

    // 普通位置模式执行位置/速度外环；停机仍按原规则每周期清理控制历史。
    if (g_motor_system.state == MOTOR_STATE_SENSORED_RUN) {
        motor_system_run_position_mode(target_pos, feedback->control_speed_rpm);
    } else {
        // 辨识未完成: 清零所有 PID 积分和电流输出，防止误动作
        motor_system_reset_control_history();
        g_foc_state.target_q = 0.0f;
        g_foc_state.target_d = 0.0f;
    }
}

// TIM2 以 1 kHz 调用；各提前返回保留原有输出所有权和执行顺序。
void motor_control_task(void)
{
    motor_runtime_feedback_t feedback;
    // 更新本周期转速和连续机械角，供后续标定与控制使用。
    motor_runtime_update_feedback(&feedback);
    // 检查主机心跳；超时则停机并释放主机控制权。
    motor_runtime_check_host_timeout();
    // 推进标定；若已完成结束处理，本周期不再执行后续控制。
    if (motor_runtime_step_calibration(&feedback)) {
        return;
    }
    // 执行强拖开环输出；接管本周期后跳过普通模式控制。
    if (motor_runtime_handle_force_drag()) {
        return;
    }
    // 采样并滤波电位器，更新公共输入并生成目标位置计数。
    float target_pos = motor_system_update_pot_target();
    // 普通控制的 d 轴电流目标保持为 0 A。
    g_foc_state.target_d = 0.0f;
    // 所有正常闭环模式共享齿槽渐入；标定是否补偿仍由会话配置和采样阶段决定。
    motor_cogging_task1ms((g_motor_system.state == MOTOR_STATE_SENSORED_RUN) &&
                          (g_control_mode >= MOTOR_CONTROL_POSITION) && (g_control_mode <= MOTOR_CONTROL_SPEED));
    // 处理音乐播放及带宽切换；播放时独占输出并跳过普通模式控制。
    if (motor_runtime_handle_music()) {
        return;
    }
    // 按当前模式执行力矩、速度、位置或触觉控制，并发布电流目标。
    motor_runtime_run_control_mode(&feedback, target_pos);
}
