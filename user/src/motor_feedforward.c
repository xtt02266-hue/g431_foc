#include "motor_feedforward.h"

// ================================================================
// 前馈补偿参数定义
// ================================================================

// 摩擦补偿：Iq_ff = gain * v / (|v| + c)
// 公式特性: 连续奇函数，零速附近平滑过渡，高速趋近固定增益
//   v     — 目标电磁转速 (RPM)
//   c     — 低速平滑系数，越大零速附近补偿越柔和
#define MOTOR_FRICTION_COMP_GAIN_A     0.032f     // 摩擦补偿最大趋近电流 (A)
#define MOTOR_FRICTION_COMP_C_RPM      15.0f     // 低速平滑系数 (RPM)
#define MOTOR_FRICTION_COMP_LIMIT_A    0.03f    // 摩擦补偿单独限幅 (A)，防止低速推力过大

// 惯性补偿：Iq_ff = gain * accel
// accel 来自轨迹规划器输出的目标加速度 (RPM/s)
#define MOTOR_INERTIA_COMP_GAIN_A_PER_RPM_S 0.000006f // 惯性补偿增益 A/(RPM/s)
#define MOTOR_INERTIA_COMP_LIMIT_A     0.15f     // 惯性补偿单独限幅 (A)，防止加速度前馈过猛

// 浮点数绝对值 (内部工具函数)
static float Motor_Feedforward_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

// 浮点数限幅 (内部工具函数): 将 value 限制在 [min_value, max_value] 区间内
static float Motor_Feedforward_ClampFloat(float value,
                                          float min_value,
                                          float max_value)
{
    if (value > max_value) {
        return max_value;
    } else if (value < min_value) {
        return min_value;
    } else {
        return value;
    }
}

// 摩擦力前馈补偿电流
// 公式: Iq_friction = GAIN * v / (|v| + C)，再限幅到 ±LIMIT
// 参数:
//   speed_rpm — 目标电磁转速 (RPM)，应使用速度环目标值而非带噪声的实测值
// 返回: 摩擦补偿电流 (A)
float Motor_Feedforward_FrictionIq(float speed_rpm)
{
    float denominator =
        Motor_Feedforward_AbsFloat(speed_rpm) +
        MOTOR_FRICTION_COMP_C_RPM;

    float iq =
        MOTOR_FRICTION_COMP_GAIN_A *
        speed_rpm /
        denominator;

    return Motor_Feedforward_ClampFloat(iq,
                                        -MOTOR_FRICTION_COMP_LIMIT_A,
                                        MOTOR_FRICTION_COMP_LIMIT_A);
}

// 惯性力前馈补偿电流
// 公式: Iq_inertia = GAIN * accel，再限幅到 ±LIMIT
// 参数:
//   accel_rpm_s — 目标加速度 (RPM/s)，来自轨迹规划器
// 返回: 惯性补偿电流 (A)
float Motor_Feedforward_InertiaIq(float accel_rpm_s)
{
    float iq =
        MOTOR_INERTIA_COMP_GAIN_A_PER_RPM_S *
        accel_rpm_s;

    return Motor_Feedforward_ClampFloat(iq,
                                        -MOTOR_INERTIA_COMP_LIMIT_A,
                                        MOTOR_INERTIA_COMP_LIMIT_A);
}

// 前馈合成函数：速度环 PID + 摩擦前馈 + 惯性前馈，最后统一限幅
// 参数:
//   speed_loop_iq — 速度环 PID 输出 (A)
//   friction_iq   — Motor_Feedforward_FrictionIq 的输出
//   inertia_iq    — Motor_Feedforward_InertiaIq 的输出
//   min_iq, max_iq — 总输出限幅范围 (A)
// 返回: 限幅后的最终目标 Iq (A)
float Motor_Feedforward_ApplyIq(float speed_loop_iq,
                                float friction_iq,
                                float inertia_iq,
                                float min_iq,
                                float max_iq)
{
    float iq =
        speed_loop_iq + friction_iq +inertia_iq;

    return Motor_Feedforward_ClampFloat(iq,
                                        min_iq,
                                        max_iq);
}

MotorFeedforwardResult Motor_Feedforward_Calculate(float speed_loop_iq,
                                                   float target_speed_rpm,
                                                   float target_accel_rpm_s,
                                                   float min_iq,
                                                   float max_iq)
{
    MotorFeedforwardResult result;

    result.speed_loop_iq = speed_loop_iq;
    result.accel_rpm_s = target_accel_rpm_s;
    result.friction_iq =
        Motor_Feedforward_FrictionIq(target_speed_rpm);
    result.inertia_iq =
        Motor_Feedforward_InertiaIq(target_accel_rpm_s);
    result.output_iq =
        Motor_Feedforward_ApplyIq(result.speed_loop_iq,
                                  result.friction_iq,
                                  result.inertia_iq,
                                  min_iq,
                                  max_iq);

    return result;
}
