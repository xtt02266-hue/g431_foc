#include "motor_speed_loop.h"
#include "motor_config.h"
#include "motor_encoder.h"
#include "pid.h"
#include <math.h>

// 速度环PID控制器实例
PID_Controller speed_pid;

// 速度测算器实例
MotorSpeedEstimator speed_est;

/*
 * 滚动窗保存展开后的机械角度计数。每次覆盖的元素正好来自
 * SPEED_EST_WINDOW_TICKS 个系统周期之前，因此不需要在不同转速区间
 * 切换测速分频，也不会在 50/200/500 rpm 处产生反馈分辨率突变。
 */
static int32_t g_speed_position_history[SPEED_EST_WINDOW_TICKS];
static int32_t g_speed_unwrapped_counts = 0;
static uint16_t g_speed_history_index = 0U;
static uint16_t g_speed_history_valid_ticks = 0U;
static uint16_t g_speed_window_ticks = SPEED_EST_WINDOW_TICKS;
uint8_t Motor_SpeedEstimator_GetWindowTicks(void) { return (uint8_t)g_speed_window_ticks; }

// 假设速度环控制周期为 1ms，与 motor_system 里的定时分频配平
#define SPEED_LOOP_DT  0.001f

/**
 * @brief 速度估计器初始化
 * @param filter_alpha 低通滤波系数 (0.0~1.0，值越小越平滑但延迟越大)
 */
void Motor_SpeedEstimator_Init(float filter_alpha) {
    speed_est.last_angle_raw = 0;
    speed_est.speed_rpm = 0.0f;
    speed_est.instant_speed_rpm = 0.0f;
    speed_est.filter_alpha = filter_alpha;
    speed_est.initialized = 0;
    g_speed_unwrapped_counts = 0;
    g_speed_history_index = 0U;
    g_speed_history_valid_ticks = 0U;
    g_speed_window_ticks = SPEED_EST_WINDOW_TICKS;
    for (uint16_t i = 0U; i < SPEED_EST_WINDOW_TICKS; ++i) {
        g_speed_position_history[i] = 0;
    }
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

    /* 直接差分接口保留给独立调用；系统1 kHz任务使用下方固定滚动窗接口。 */
    int32_t delta = (int32_t)current_angle_raw -
                    (int32_t)speed_est.last_angle_raw;
    
    // 4. 处理15位编码器的0/32767过零点。
    if (delta > (int32_t)MOTOR_ENCODER_HALF_REV_U32) {
        delta -= (int32_t)MOTOR_ENCODER_COUNTS_PER_REV_U32;
    } else if (delta < -(int32_t)MOTOR_ENCODER_HALF_REV_U32) {
        delta += (int32_t)MOTOR_ENCODER_COUNTS_PER_REV_U32;
    }
    
    speed_est.last_angle_raw = current_angle_raw;
    
    // dt_seconds 是相邻两次调用的真实测速间隔，直接用于机械转速换算。
    float instant_rpm = ((float)delta * 60.0f) /
                        (MOTOR_ENCODER_COUNTS_PER_REV_F * dt_seconds);
    
    // 6. 一阶低通滤波 (EMA)，进一步平滑
    speed_est.speed_rpm = speed_est.filter_alpha * instant_rpm 
                        + (1.0f - speed_est.filter_alpha) * speed_est.speed_rpm;
                        
    return speed_est.speed_rpm;
}

/*
 * 自适应滚动窗测速：
 * - 每 1 ms 对相邻原始角度解回绕并累计为连续位置；
 * - 低/中/高速分别使用20/10/5 ms位置差；
 * - 每 1 ms 产生新结果，之后再经过原有 EMA。
 *
 * 窗口切换由当前发布速度决定并带迟滞；与旧分频实现不同，切换窗口不会让
 * 反馈停止更新。启动阶段使用逐步增长的有效窗口，避免先固定输出零。
 */
float Motor_SpeedEstimator_UpdateAdaptive(float published_speed_rpm)
{
    float abs_rpm = (published_speed_rpm < 0.0f)
                        ? -published_speed_rpm : published_speed_rpm;
    if (g_speed_window_ticks == SPEED_EST_WINDOW_TICKS) {
        if (abs_rpm > SPEED_EST_LOW_TO_MID_RPM) {
            g_speed_window_ticks = SPEED_EST_MID_WINDOW_TICKS;
        }
    } else if (g_speed_window_ticks == SPEED_EST_MID_WINDOW_TICKS) {
        if (abs_rpm < SPEED_EST_MID_TO_LOW_RPM) {
            g_speed_window_ticks = SPEED_EST_WINDOW_TICKS;
        } else if (abs_rpm > SPEED_EST_MID_TO_HIGH_RPM) {
            g_speed_window_ticks = SPEED_EST_HIGH_WINDOW_TICKS;
        }
    } else if (abs_rpm < SPEED_EST_HIGH_TO_MID_RPM) {
        g_speed_window_ticks = SPEED_EST_MID_WINDOW_TICKS;
    }

    uint16_t current_angle_raw = Motor_Encoder_GetRawAngle();

    if (!speed_est.initialized) {
        speed_est.last_angle_raw = current_angle_raw;
        speed_est.speed_rpm = 0.0f;
        speed_est.instant_speed_rpm = 0.0f;
        speed_est.initialized = 1U;
        g_speed_unwrapped_counts = (int32_t)current_angle_raw;
        g_speed_history_index = 0U;
        g_speed_history_valid_ticks = 0U;
        g_speed_window_ticks = SPEED_EST_WINDOW_TICKS;
        for (uint16_t i = 0U; i < SPEED_EST_WINDOW_TICKS; ++i) {
            g_speed_position_history[i] = g_speed_unwrapped_counts;
        }
        return 0.0f;
    }

    int32_t step_delta = (int32_t)current_angle_raw -
                         (int32_t)speed_est.last_angle_raw;
    if (step_delta > (int32_t)MOTOR_ENCODER_HALF_REV_U32) {
        step_delta -= (int32_t)MOTOR_ENCODER_COUNTS_PER_REV_U32;
    } else if (step_delta < -(int32_t)MOTOR_ENCODER_HALF_REV_U32) {
        step_delta += (int32_t)MOTOR_ENCODER_COUNTS_PER_REV_U32;
    }
    speed_est.last_angle_raw = current_angle_raw;
    g_speed_unwrapped_counts += step_delta;

    uint16_t new_history_index = g_speed_history_index + 1U;
    if (new_history_index >= SPEED_EST_WINDOW_TICKS) {
        new_history_index = 0U;
    }

    uint16_t effective_window_ticks = g_speed_window_ticks;
    if (effective_window_ticks > (uint16_t)(g_speed_history_valid_ticks + 1U)) {
        effective_window_ticks = (uint16_t)(g_speed_history_valid_ticks + 1U);
    }
    uint16_t old_history_index = (uint16_t)(
        (new_history_index + SPEED_EST_WINDOW_TICKS - effective_window_ticks) %
        SPEED_EST_WINDOW_TICKS);
    int32_t old_position = g_speed_position_history[old_history_index];
    g_speed_position_history[new_history_index] = g_speed_unwrapped_counts;
    g_speed_history_index = new_history_index;

    if (g_speed_history_valid_ticks < SPEED_EST_WINDOW_TICKS) {
        g_speed_history_valid_ticks++;
    }

    if (g_speed_history_valid_ticks == 0U) {
        return speed_est.speed_rpm;
    }

    float dt_seconds = (float)effective_window_ticks *
                       MOTOR_SYSTEM_TASK_DT_SEC;
    float instant_rpm =
        ((float)(g_speed_unwrapped_counts - old_position) * 60.0f) /
        (MOTOR_ENCODER_COUNTS_PER_REV_F * dt_seconds);

    speed_est.instant_speed_rpm = instant_rpm;
    speed_est.speed_rpm = speed_est.filter_alpha * instant_rpm +
        (1.0f - speed_est.filter_alpha) * speed_est.speed_rpm;
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

/* Caller excludes the 1 kHz control task while updating both coefficients. */
uint8_t Motor_SpeedLoop_SetPI(float kp, float ki, uint8_t running) {
    if (!isfinite(kp) || !isfinite(ki) || kp<0.0f || kp>0.1f || ki<0.0f || ki>1.0f) return 0U;
    if (running) {
        float error=speed_pid.target-speed_pid.measure;
        /* integral stores its current contribution in amperes, not raw error.
         * Offset the change in P to retain the instantaneous P+I command. */
        speed_pid.integral+=(speed_pid.kp-kp)*error;
        if (speed_pid.integral>speed_pid.out_max) speed_pid.integral=speed_pid.out_max;
        if (speed_pid.integral<speed_pid.out_min) speed_pid.integral=speed_pid.out_min;
    } else PID_Reset(&speed_pid);
    speed_pid.kp=kp;speed_pid.ki=ki;
    return 1U;
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

