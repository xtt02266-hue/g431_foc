#include "motor_speed_loop.h"
#include "pid.h"

// 速度环PID控制器实例
PID_Controller speed_pid;

// 速度测算器实例
MotorSpeedEstimator speed_est;

// 假设速度环控制周期为 1ms，与 motor_system 里的定时分频配平
#define SPEED_LOOP_DT  0.001f

// 缩小滑动窗口，降低速度反馈中的硬件级相位滞后！
#define SPEED_WINDOW_SIZE 4 // 低速自适应采样时会拉长等效测速窗口，降低量化抖动
static uint16_t angle_history[SPEED_WINDOW_SIZE];
static float dt_history[SPEED_WINDOW_SIZE];
static uint8_t history_idx = 0;

/**
 * @brief 速度估计器初始化
 * @param filter_alpha 低通滤波系数 (0.0~1.0，值越小越平滑但延迟越大，默认建议 0.1~0.2)
 */
void Motor_SpeedEstimator_Init(float filter_alpha) {
    speed_est.last_angle_raw = 0;
    speed_est.speed_rpm = 0.0f;
    speed_est.filter_alpha = filter_alpha;
    speed_est.initialized = 0;
    for (int i = 0; i < SPEED_WINDOW_SIZE; i++) {
        angle_history[i] = 0;
        dt_history[i] = 0.0f;
    }
    history_idx = 0;
}

/**
 * @brief 更新并计算当前速度 (RPM)
 * dt_seconds 必须传入距离上一次实际测速的累计时间
 */
float Motor_SpeedEstimator_Update(uint16_t current_angle_raw, float dt_seconds) {
    if (dt_seconds <= 0.0f) return speed_est.speed_rpm;
    
    // 初始化第一次读数：填满整个缓冲区，避免刚启动时算错差值
    if (!speed_est.initialized) {
        for (int i = 0; i < SPEED_WINDOW_SIZE; i++) {
            angle_history[i] = current_angle_raw;
            dt_history[i] = 0.0f;
        }
        history_idx = 0;
        
        speed_est.last_angle_raw = current_angle_raw;
        speed_est.speed_rpm = 0.0f;
        speed_est.initialized = 1;
        return 0.0f;
    }

    // 1. 获取 SPEED_WINDOW_SIZE 次实际测速前记录的历史角度
    uint16_t oldest_angle = angle_history[history_idx];
    
    // 2. 将当前最新角度存入历史缓冲区覆盖旧值，并推进索引
    angle_history[history_idx] = current_angle_raw;
    dt_history[history_idx] = dt_seconds;
    history_idx++;
    if (history_idx >= SPEED_WINDOW_SIZE) {
        history_idx = 0;
    }

    // 3. 计算测速窗口内的原始数据增量
    int32_t delta = (int32_t)current_angle_raw - (int32_t)oldest_angle;
    
    // 4. 处理编码器过零点 (0 -> 4095 或 4095 -> 0)
    if (delta > 2048) {
        delta -= 4096;
    } else if (delta < -2048) {
        delta += 4096;
    }
    
    speed_est.last_angle_raw = current_angle_raw;
    
    // 5. 计算机械瞬时速度
    float actual_dt = 0.0f;
    for (int i = 0; i < SPEED_WINDOW_SIZE; i++) {
        actual_dt += dt_history[i];
    }
    if (actual_dt <= 0.0f) return speed_est.speed_rpm;
    float instant_rpm = ((float)delta * 60.0f) / (4096.0f * actual_dt);
    
    // 6. 一阶低通滤波 (EMA)，进一步平滑
    speed_est.speed_rpm = speed_est.filter_alpha * instant_rpm 
                        + (1.0f - speed_est.filter_alpha) * speed_est.speed_rpm;
                        
    return speed_est.speed_rpm;
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

