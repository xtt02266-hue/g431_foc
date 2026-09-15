#include "motor_speed_loop.h"
#include "motor_config.h"
#include "as5600.h"
#include "pid.h"

// 速度环PID控制器实例
PID_Controller speed_pid;

// 速度测算器实例
MotorSpeedEstimator speed_est;

// 假设速度环控制周期为 1ms，与 motor_system 里的定时分频配平
#define SPEED_LOOP_DT  0.001f

/**
 * @brief 速度估计器初始化
 * @param filter_alpha 低通滤波系数 (0.0~1.0，值越小越平滑但延迟越大)
 */
void Motor_SpeedEstimator_Init(float filter_alpha) {
    speed_est.last_angle_raw = 0;
    speed_est.speed_rpm = 0.0f;
    speed_est.filter_alpha = filter_alpha;
    speed_est.initialized = 0;
}

/**
 * @brief 更新并计算当前速度 (RPM)
 * dt_seconds 必须传入距离上一次实际测速的累计时间
 */
float Motor_SpeedEstimator_Update(uint16_t current_angle_raw, float dt_seconds) {
    if (dt_seconds <= 0.0f) return speed_est.speed_rpm;
    
    // 第一次读数只建立基准，不计算速度，避免启动时产生虚假差值。
    if (!speed_est.initialized) {
        speed_est.last_angle_raw = current_angle_raw;
        speed_est.speed_rpm = 0.0f;
        speed_est.initialized = 1;
        return 0.0f;
    }

    /*
     * 为什么改为相邻两次“实际测速时刻”直接差分：外层已经根据速度选择 1~20ms
     * 的采样周期，旧代码又叠加 4 点窗口，使低速反馈覆盖约 80ms，造成明显相位滞后。
     * 现在低速仍用最长 20ms 的计数窗口抑制 12 位量化噪声，但不再额外等待四个窗口。
     */
    int32_t delta = (int32_t)current_angle_raw -
                    (int32_t)speed_est.last_angle_raw;
    
    // 4. 处理编码器过零点 (0 -> 4095 或 4095 -> 0)
    if (delta > 2048) {
        delta -= 4096;
    } else if (delta < -2048) {
        delta += 4096;
    }
    
    speed_est.last_angle_raw = current_angle_raw;
    
    // dt_seconds 是外层累计的真实测速间隔，直接用于机械转速换算。
    float instant_rpm = ((float)delta * 60.0f) /
                        (4096.0f * dt_seconds);
    
    // 6. 一阶低通滤波 (EMA)，进一步平滑
    speed_est.speed_rpm = speed_est.filter_alpha * instant_rpm 
                        + (1.0f - speed_est.filter_alpha) * speed_est.speed_rpm;
                        
    return speed_est.speed_rpm;
}

/* 为什么需要：自适应采样率调度原本在系统模块，依赖系统全局量；迁入速度模块后
 * 只依赖估算器自身状态和 AS5600，避免系统模块与测速实现相互耦合。
 * 功能：按传入的发布速度选择编码器采样分频，仅在到期周期读取 AS5600 并更新速度。
 * 参数：rpm 为用于分频判断的速度；返回对应的采样周期 tick 数。
 * 注意：使用传入值而非 speed_est.speed_rpm，二者在 stale 恢复等场景不恒等。
 */
static uint16_t Motor_SpeedEstimator_GetPeriodTicks(float rpm)
{
    float abs_rpm = (rpm < 0.0f) ? -rpm : rpm;

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
float Motor_SpeedEstimator_UpdateAdaptive(float published_speed_rpm)
{
    static uint16_t ticks = 0;
    static float dt_acc = 0.0f;

    if (!speed_est.initialized) {
        ticks = 0;
        dt_acc = 0.0f;
        return Motor_SpeedEstimator_Update(AS5600_ReadRawAngle(), MOTOR_SYSTEM_TASK_DT_SEC);
    }

    ticks++;
    dt_acc += MOTOR_SYSTEM_TASK_DT_SEC;

    if (ticks >= Motor_SpeedEstimator_GetPeriodTicks(published_speed_rpm)) {
        float updated_speed_rpm = Motor_SpeedEstimator_Update(AS5600_ReadRawAngle(), dt_acc);
        ticks = 0;
        dt_acc = 0.0f;
        return updated_speed_rpm;
    }

    return published_speed_rpm;
}

// ======================= PID 闭环部分 =======================

/**
 * @brief 速度环初始化
 */
void Motor_SpeedLoop_Init(void) {
    // 调用底层的 PID 初始化
    PID_Init(&speed_pid, 
             MOTOR_SPEED_PID_KP, 
             MOTOR_SPEED_PID_KI, 
             MOTOR_SPEED_PID_KD, 
             MOTOR_SPEED_PID_OUT_MAX, 
             MOTOR_SPEED_PID_OUT_MIN, 
             SPEED_LOOP_DT);
}

/**
 * @brief 设置目标速度
 * @param target_speed_rpm 期望的目标速度(RPM)
 */
void Motor_SpeedLoop_SetTarget(float target_speed_rpm) {
    speed_pid.target = target_speed_rpm;
}

/**
 * @brief 速度环控制周期更新
 * @param current_speed_rpm 当前的实际速度反馈(RPM)
 * @return float 计算出的目标电流 (Iq_ref)
 */
float Motor_SpeedLoop_Update(float current_speed_rpm) {
    // 1. 更新当前测量值
    speed_pid.measure = current_speed_rpm;
    
    // 2. 运行 PID 计算
    PID_Calculate(&speed_pid);
    
    // 3. 返回计算的输出值 (作为电流环的 Iq 目标值)
    return speed_pid.output;
}


/**
 * @brief  弱磁控制 (Field Weakening) 策略，根据当前转速调整 D 轴电流以实现更高的速度。
 * @param  current_rpm 当前实际转速
 * @retval 
 */
float Motor_SpeedLoop_FieldWeakening(float current_rpm)
{

    float fw_rpm_threshold = 1000.0f;
    float fw_gain = 0.001f;  
    float fw_max_current = -1.0f; 

    float target_d = 0.0f;
    
    float abs_rpm = current_rpm;
    if (abs_rpm < 0.0f) abs_rpm = -abs_rpm;
    
    if (abs_rpm > fw_rpm_threshold)
    {
        target_d = -(abs_rpm - fw_rpm_threshold) * fw_gain;

        if (target_d < fw_max_current)
        {
            target_d = fw_max_current;
        }
    }
    else
    {
        target_d = 0.0f;
    }
    
    return target_d;
}

