#include "motor_context.h"
#include "motor_current_loop.h"
#include "motor_encoder.h"
#include <math.h>

#define MOTOR_HAPTIC_CENTER_DEADBAND_RAD 0.00261799388f // 中心静区 0.15°，减小静止时的编码器抖动响应
#define MOTOR_HAPTIC_SPRING_PROGRESS_RAD 1.57079632679f // 弹簧在 90° 偏转内逐渐增硬，单位 rad
#define MOTOR_HAPTIC_DAMPING_TIME_S      0.025f         // 将刚度换算为附加阻尼的时间尺度，单位 s
#define MOTOR_HAPTIC_DETENT_DEADBAND     0.02f          // 档心静区占档距的比例，上限为中心静区
#define MOTOR_HAPTIC_DETENT_STEP_UNITS   ((int64_t)MOTOR_ENCODER_COUNTS_PER_REV_U32) // 缩放计数域的一档间距
#define MOTOR_HAPTIC_DETENT_SNAP_UNITS   ((55LL * MOTOR_HAPTIC_DETENT_STEP_UNITS) / 100LL) // 55% 档距阈值
#define MOTOR_HAPTIC_LIMIT_PROGRESS_RAD  0.1745329252f  // 软墙每 10° 穿透逐渐增硬，单位 rad
#define MOTOR_HAPTIC_LIMIT_CONTACT_RAD   0.00872664626f // 向外阻尼在墙外 0.5° 内连续建立，单位 rad
#define MOTOR_HAPTIC_PROFILE_SLEW_A_S    20.0f          // 三种位置触觉模式的总电流变化率，单位 A/s

/* 参考点和档位历史由 TIM2 更新；生命周期在原临界区内复位。
 * 控制先在整数计数域求相对位置；档内误差单位为编码器计数乘每圈档数。
 */
static uint8_t s_reference_valid;
static motor_control_mode_t s_reference_mode;
static int64_t s_reference_counts;
static int64_t s_detent_last_position_counts;
static int64_t s_detent_error_units;

// 失效参考点并清除档位与输出斜率历史，下一次触觉运行重新建立中心。
void motor_haptic_reset(void)
{
    s_reference_valid = 0U;
    s_detent_error_units = 0;
    g_host_iq_applied_a = 0.0f;
}

// 读取有效触觉中心的整数相对计数；仅限 TIM2 或调用方已屏蔽中断时使用。
int64_t motor_haptic_get_relative_counts(void)
{
    if ((s_reference_valid == 0U) || (s_reference_mode != g_control_mode)) {
        return 0;
    }
    return motor_runtime_get_position_counts() - s_reference_counts;
}

// 从中心两侧连续扣除静区，静区内恢复力为零，角度单位 rad。
static float remove_center_deadband(float angle_rad, float deadband_rad)
{
    if (angle_rad > deadband_rad) {
        return angle_rad - deadband_rad;
    }
    if (angle_rad < -deadband_rad) {
        return angle_rad + deadband_rad;
    }
    return 0.0f;
}

// 计算带中心静区和渐硬特性的扭簧电流，阻尼使用实际机械角速度 rad/s。
static float calculate_spring_current(float relative_angle_rad, float speed_rad_s)
{
    float error_rad = remove_center_deadband(relative_angle_rad, MOTOR_HAPTIC_CENTER_DEADBAND_RAD);
    float progress = fminf(fabsf(error_rad) / MOTOR_HAPTIC_SPRING_PROGRESS_RAD, 1.0f);
    float damping = g_haptic_params.damping_b_a_per_rad_s +
                    g_haptic_params.spring_k_a_per_rad * MOTOR_HAPTIC_DAMPING_TIME_S;
    return -g_haptic_params.spring_k_a_per_rad * error_rad * (1.0f + progress * progress) - damping * speed_rad_s;
}

/* 用有界整数相位保持档心；实际相邻控制拍的机械计数差在半圈内。
 * 先按整数档距消去整档，再把档内误差换算为 rad，长期旋转不累加浮点舍入。
 */
static float calculate_detent_current(int64_t position_counts, float speed_rad_s)
{
    float spacing_rad = 6.28318530718f / (float)g_haptic_params.detent_count;
    int64_t delta_counts = position_counts - s_detent_last_position_counts;
    s_detent_last_position_counts = position_counts;
    s_detent_error_units += delta_counts * (int64_t)g_haptic_params.detent_count;
    if (s_detent_error_units > MOTOR_HAPTIC_DETENT_SNAP_UNITS) {
        int64_t distance = s_detent_error_units - MOTOR_HAPTIC_DETENT_SNAP_UNITS;
        int64_t steps = 1 + (distance - 1) / MOTOR_HAPTIC_DETENT_STEP_UNITS;
        s_detent_error_units -= steps * MOTOR_HAPTIC_DETENT_STEP_UNITS;
    } else if (s_detent_error_units < -MOTOR_HAPTIC_DETENT_SNAP_UNITS) {
        int64_t distance = -(s_detent_error_units + MOTOR_HAPTIC_DETENT_SNAP_UNITS);
        int64_t steps = 1 + (distance - 1) / MOTOR_HAPTIC_DETENT_STEP_UNITS;
        s_detent_error_units += steps * MOTOR_HAPTIC_DETENT_STEP_UNITS;
    }
    float error_rad = (float)s_detent_error_units * MOTOR_ENCODER_RAD_PER_COUNT /
                      (float)g_haptic_params.detent_count;
    float deadband_rad = fminf(spacing_rad * MOTOR_HAPTIC_DETENT_DEADBAND, MOTOR_HAPTIC_CENTER_DEADBAND_RAD);
    error_rad = remove_center_deadband(error_rad, deadband_rad);
    float progress = fminf(fabsf(error_rad) / (0.5f * spacing_rad), 1.0f);
    float damping = g_haptic_params.damping_b_a_per_rad_s +
                    g_haptic_params.detent_k_a_per_rad * MOTOR_HAPTIC_DAMPING_TIME_S;
    return -g_haptic_params.detent_k_a_per_rad * error_rad * (1.0f + progress * progress) - damping * speed_rad_s;
}

// 软墙仅在越界时向内恢复，并只阻尼继续越界的速度；返回时不施加向外阻力。
static float calculate_limit_current(float relative_angle_rad, float speed_rad_s)
{
    float half_range_rad = g_haptic_params.limit_half_range_deg * 0.01745329252f;
    float penetration_rad = fabsf(relative_angle_rad) - half_range_rad;
    if (penetration_rad <= 0.0f) {
        return 0.0f;
    }
    float side = (relative_angle_rad > 0.0f) ? 1.0f : -1.0f;
    float progress = fminf(penetration_rad / MOTOR_HAPTIC_LIMIT_PROGRESS_RAD, 2.0f);
    float contact = fminf(penetration_rad / MOTOR_HAPTIC_LIMIT_CONTACT_RAD, 1.0f);
    float outward_speed_rad_s = fmaxf(side * speed_rad_s, 0.0f);
    float damping = g_haptic_params.damping_b_a_per_rad_s +
                    g_haptic_params.limit_k_a_per_rad * MOTOR_HAPTIC_DAMPING_TIME_S;
    float restoring_iq_a = g_haptic_params.limit_k_a_per_rad * penetration_rad * (1.0f + progress * progress);
    return -side * (restoring_iq_a + damping * outward_speed_rad_s * contact);
}

/* TIM2 在 FREE/DAMPING/SPRING/DETENT/LIMIT 模式调用；各模式叠加摩擦和齿槽补偿。
 * identified_direction 为已校验的换相方向 ±1，angle_counts 为机械编码器计数。
 */
void motor_haptic_task(int8_t identified_direction, uint16_t angle_counts)
{
    // 只读取本周期已发布的计数，不重新访问编码器硬件。
    int64_t position_counts = motor_runtime_get_position_counts();
    uint8_t position_profile = (g_control_mode >= MOTOR_CONTROL_SPRING) && (g_control_mode <= MOTOR_CONTROL_LIMIT);
    if (position_profile != 0U) {
        if ((s_reference_valid == 0U) || (s_reference_mode != g_control_mode)) {
            g_haptic_center_rad = g_continuous_angle_rad;
            s_reference_counts = position_counts;
            s_detent_last_position_counts = position_counts;
            s_detent_error_units = 0;
            s_reference_mode = g_control_mode;
            s_reference_valid = 1U;
        }
    } else {
        s_reference_valid = 0U;
    }
    const float omega = g_motor_system.run_data.speed_rpm * (6.28318530718f / 60.0f);
    const float rel = (float)(position_counts - s_reference_counts) * MOTOR_ENCODER_RAD_PER_COUNT;
    float i_mech = 0.0f;
    float friction_iq_mech;
    if (g_control_mode == MOTOR_CONTROL_FREE) {
        // 自由模式沿用摩擦减小阻尼的原模型，其它模式保留各自的虚拟阻尼。
        friction_iq_mech = motor_feedforward_free_compensation(g_motor_system.run_data.speed_rpm);
    } else {
        // 手动触觉按实际机械速度补偿寄生摩擦，静止时不凭目标方向主动起步。
        friction_iq_mech = motor_feedforward_friction_compensation(g_motor_system.run_data.speed_rpm);
    }
    if (g_control_mode == MOTOR_CONTROL_DAMPING) {
        i_mech = -g_haptic_params.damping_b_a_per_rad_s * omega;
    } else if (g_control_mode == MOTOR_CONTROL_SPRING) {
        // 偏离启动中心越远，恢复力逐渐增大，附加阻尼抑制回中过冲。
        i_mech = calculate_spring_current(rel, omega);
    } else if (g_control_mode == MOTOR_CONTROL_DETENT) {
        // 保持当前档心直到越过迟滞阈值，避免半档附近反复切换。
        i_mech = calculate_detent_current(position_counts, omega);
    } else if (g_control_mode == MOTOR_CONTROL_LIMIT) {
        // 边界内不产生虚拟墙力，越界后渐硬且允许顺畅退回。
        i_mech = calculate_limit_current(rel, omega);
    }
    g_debug_friction_iq_a = friction_iq_mech * (float)identified_direction;
    // 齿槽表已使用电磁 Iq 方向，不能再次乘换相方向。
    g_debug_cogging_iq_a = motor_cogging_compensation(angle_counts, g_motor_system.run_data.speed_rpm);
    float iq = i_mech * (float)identified_direction + g_debug_friction_iq_a + g_debug_cogging_iq_a;
    if (!isfinite(iq)) {
        iq = 0.0f;
    }
    if (iq > g_host_iq_limit_a) {
        iq = g_host_iq_limit_a;
    }
    if (iq < -g_host_iq_limit_a) {
        iq = -g_host_iq_limit_a;
    }
    // 位置触觉使用有界快速响应；自由和纯阻尼模式保持原输出斜率。
    float slew_a_per_s = position_profile ? MOTOR_HAPTIC_PROFILE_SLEW_A_S : MOTOR_IQ_SLEW_A_PER_S;
    iq = motor_system_slew_host_iq(iq, slew_a_per_s);
    g_debug_speed_loop_iq_a = 0.0f;
    g_foc_state.target_q = iq;
    g_foc_state.target_d = 0.0f;
    return;
}
