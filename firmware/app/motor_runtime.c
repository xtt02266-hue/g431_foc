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

// 周期任务：位置模式执行外环；力矩模式直接将给定 Iq 送入电流环。
// 调用频率: 1kHz (由 TIM2 中断驱动, dt = 1ms)
// 默认链路: 电位器 → 目标力矩(N·m) → Iq=T/Kt → FOC → SVPWM → 电机
void motor_control_task(void)
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
            motor_haptic_task(identified_direction, angle_counts);
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
