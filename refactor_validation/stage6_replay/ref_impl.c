/* Stage 6 参考实现：逐字取自 Stage 0 保存的 baseline/user/src/motor_system.c
 * 中的 Motor_AbsFloat / Motor_SpeedEstimator_GetPeriodTicks /
 * Motor_UpdateSpeedEstimatorAdaptive。除把最后一个函数改为可调用的
 * Adaptive_Call 包装外，函数体不做任何改动。 */
#include "motor_speed_loop.h"
#include "motor_config.h"
#include "as5600.h"
#include "motor_system.h"

MotorSystem g_motor_system;

// 浮点数绝对值
static float Motor_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint16_t Motor_SpeedEstimator_GetPeriodTicks(float rpm)
{
    float abs_rpm = Motor_AbsFloat(rpm);

    if (abs_rpm >= SPEED_EST_HIGH_RPM_THRESHOLD) {
        return SPEED_EST_MAX_PERIOD_TICKS;
    } else if (abs_rpm >= SPEED_EST_MID_RPM_THRESHOLD) {
        return SPEED_EST_HIGH_PERIOD_TICKS;
    } else if (abs_rpm >= SPEED_EST_LOW_RPM_THRESHOLD) {
        return SPEED_EST_MID_PERIOD_TICKS;
    } else {
        return SPEED_EST_LOW_PERIOD_TICKS;
    }
}

// 自适应采样率速度估算器
// 策略: 根据当前估算转速动态调整编码器采样间隔 (分频比)
//   低速 (<50 RPM):  每 20ms 采样一次 (50Hz)，用更长窗口抑制量化噪声
//   中速 (50~200):   每 5ms 采样一次 (200Hz)
//   中高速 (200~500): 每 2ms 采样一次 (500Hz)
//   高速 (>500 RPM): 每 1ms 采样一次 (1000Hz)，保证响应速度
// 这样在低速时避免了 AS5600 12-bit 编码器因采样过快导致的量化抖动
static float Motor_UpdateSpeedEstimatorAdaptive(void)
{
    static uint16_t ticks = 0;
    static float dt_acc = 0.0f;

    if (!speed_est.initialized) {
        ticks = 0;
        dt_acc = 0.0f;
        g_motor_system.run_data.speed_rpm =
            Motor_SpeedEstimator_Update(AS5600_ReadRawAngle(), MOTOR_SYSTEM_TASK_DT_SEC);
        return g_motor_system.run_data.speed_rpm;
    }

    ticks++;
    dt_acc += MOTOR_SYSTEM_TASK_DT_SEC;

    if (ticks >= Motor_SpeedEstimator_GetPeriodTicks(g_motor_system.run_data.speed_rpm)) {
        g_motor_system.run_data.speed_rpm =
            Motor_SpeedEstimator_Update(AS5600_ReadRawAngle(), dt_acc);
        ticks = 0;
        dt_acc = 0.0f;
    }

    return g_motor_system.run_data.speed_rpm;
}

/* 回放入口：先把发布速度写入系统量（与真实 fresh 分支一致），再调用旧调度。 */
float Adaptive_Call(float published_speed_rpm)
{
    g_motor_system.run_data.speed_rpm = published_speed_rpm;
    return Motor_UpdateSpeedEstimatorAdaptive();
}
