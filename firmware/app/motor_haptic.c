#include "motor_context.h"
#include "motor_current_loop.h"
#include <math.h>

/* TIM2 在 FREE/DAMPING/SPRING/DETENT/LIMIT 模式调用；保留原力矩模型、补偿叠加和 Iq 斜率。
 * direction 为已校验的换相方向 ±1，angle_counts 为机械编码器计数；速度和角度从周期状态读取。 */
void motor_haptic_task(int8_t identified_direction, uint16_t angle_counts)
{
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
        i_mech = -g_haptic_params.detent_k_a_per_rad * (rel - nearest) - g_haptic_params.damping_b_a_per_rad_s * omega;
    } else if (g_control_mode == MOTOR_CONTROL_LIMIT) {
        float half = g_haptic_params.limit_half_range_deg * 0.01745329252f;
        if (rel > half) {
            i_mech = -g_haptic_params.limit_k_a_per_rad * (rel - half) - g_haptic_params.damping_b_a_per_rad_s * omega;
        } else if (rel < -half) {
            i_mech = -g_haptic_params.limit_k_a_per_rad * (rel + half) - g_haptic_params.damping_b_a_per_rad_s * omega;
        }
    }
    g_debug_friction_iq_a = (g_control_mode == MOTOR_CONTROL_FREE) ? i_mech * (float)identified_direction : 0.0f;
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
