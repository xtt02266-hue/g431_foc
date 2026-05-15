#include "motor_speed_loop.h"
#include "pid.h"

// 速度环PID控制器实例
PID_Controller speed_pid;

// 速度测算器实例
MotorSpeedEstimator speed_est;

// 假设速度环控制周期为 1ms，与 motor_system 里的定时分频配平
#define SPEED_LOOP_DT  0.001f

// ======================= 速度测算部分 =======================

/**
 * @brief 速度估计器初始化
 * @param filter_alpha 低通滤波系数 (0.0~1.0，值越小越平滑但延迟越大，默认建议 0.1~0.2)
 */
void Motor_SpeedEstimator_Init(float filter_alpha) {
    speed_est.last_angle_raw = 0;
    speed_est.speed_rpm = 0.0f;
    speed_est.filter_alpha = filter_alpha;
    speed_est.initialized = 0;
}

/**
 * @brief 更新并计算当前速度 (RPM)
 * @param current_angle_raw 当前编码器原始读数 (12位，0~4095)
 * @param dt_seconds 两次调用的时间间隔(秒)，例如 0.001f
 * @return float 计算并滤波后的转速 (RPM)
 */
float Motor_SpeedEstimator_Update(uint16_t current_angle_raw, float dt_seconds) {
    if (dt_seconds <= 0.0f) return speed_est.speed_rpm;
    
    // 初始化第一次读数，避免系统刚启动时跳变产生巨大的初始虚假速度
    if (!speed_est.initialized) {
        speed_est.last_angle_raw = current_angle_raw;
        speed_est.speed_rpm = 0.0f;
        speed_est.initialized = 1;
        return 0.0f;
    }

    // 1. 计算原始数据增量
    int32_t delta = (int32_t)current_angle_raw - (int32_t)speed_est.last_angle_raw;
    
    // 2. 处理编码器过零点 (0 -> 4095 或 4095 -> 0)
    if (delta > 2048) {
        delta -= 4096;
    } else if (delta < -2048) {
        delta += 4096;
    }
    
    speed_est.last_angle_raw = current_angle_raw;
    
// 3. 计算机械瞬时速度
    // delta (LSB)
    // 根据差值直接计算，放大避免被截断
    // 一圈 4096 LSB。dt_seconds (例如 0.010s)
    // RPM = (delta / 4096.0f) * (1.0f / dt_seconds) * 60.0f
    float instant_rpm = ((float)delta * 60.0f) / (4096.0f * dt_seconds);
    
    // 4. 一阶低通滤波 (EMA)
    speed_est.speed_rpm = speed_est.filter_alpha * instant_rpm 
                        + (1.0f - speed_est.filter_alpha) * speed_est.speed_rpm;
                        
    return speed_est.speed_rpm;
}

// ======================= PID 闭环部分 =======================

/**
 * @brief 速度环初始化
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param max_current 最大输出电流(作为电流环的Iq限幅)
 */
void Motor_SpeedLoop_Init(float kp, float ki, float kd, float max_current) {
    // 调用底层的 PID 初始化
    PID_Init(&speed_pid, kp, ki, kd, max_current, -max_current, SPEED_LOOP_DT);
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
