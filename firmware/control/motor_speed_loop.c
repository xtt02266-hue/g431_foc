#include "motor_speed_loop.h"
#include "motor_config.h"
#include "motor_encoder.h"
#include "pid.h"
#include <math.h>

// 速度环PID控制器实例
pid_controller_t speed_pid;

// 速度测算器实例
motor_speed_estimator_t speed_est;

/*
 * 滚动窗保存按 uint32_t 模运算累计的机械角度计数，避免长运行时有符号溢出。
 * 最大 20 ms 窗口内的位置差远小于 INT32_MAX，跨累计计数回绕仍可恢复短距离。
 * 每次覆盖的元素正好来自
 * SPEED_EST_WINDOW_TICKS 个系统周期之前，因此不需要在不同转速区间
 * 切换测速分频，也不会在 50/200/500 rpm 处产生反馈分辨率突变。
 */
static uint32_t g_speed_position_history[SPEED_EST_WINDOW_TICKS];
static uint32_t g_speed_unwrapped_counts = 0U;
static uint16_t g_speed_history_index = 0U;
static uint16_t g_speed_history_valid_ticks = 0U;
static uint16_t g_speed_window_ticks = SPEED_EST_WINDOW_TICKS;

// 将模 2^32 的窗口差分解码为有符号短距离，避免超范围无符号转有符号转换。
static int32_t calculate_position_delta(uint32_t position_counts, uint32_t previous_position_counts)
{
    uint32_t delta = position_counts - previous_position_counts;
    if (delta <= (uint32_t)INT32_MAX) {
        return (int32_t)delta;
    }
    return -(int32_t)(UINT32_MAX - delta) - 1;
}

// 读取当前速度估计窗口长度，单位 1 ms 周期数；不更新窗口或转速。
uint8_t motor_speed_estimator_get_window_ticks(void)
{
    return (uint8_t)g_speed_window_ticks;
}

// 假设速度环控制周期为 1ms，与 motor_system 里的定时分频配平
#define SPEED_LOOP_DT 0.001f

/**
 * @brief 速度估计器初始化
 * @param filter_alpha 低通滤波系数 (0.0~1.0，值越小越平滑但延迟越大)
 */
void motor_speed_estimator_init(float filter_alpha)
{
    speed_est.last_angle_raw = 0;
    speed_est.speed_rpm = 0.0f;
    speed_est.instant_speed_rpm = 0.0f;
    speed_est.filter_alpha = filter_alpha;
    speed_est.initialized = 0;
    g_speed_unwrapped_counts = 0U;
    g_speed_history_index = 0U;
    g_speed_history_valid_ticks = 0U;
    g_speed_window_ticks = SPEED_EST_WINDOW_TICKS;
    for (uint16_t i = 0U; i < SPEED_EST_WINDOW_TICKS; ++i) {
        g_speed_position_history[i] = 0U;
    }
}

/**
 * @brief 更新并计算当前速度 (RPM)
 * dt_seconds 必须传入距离上一次实际测速的累计时间
 */
float motor_speed_estimator_update(uint16_t current_angle_raw, float dt_seconds)
{
    if (dt_seconds <= 0.0f) {
        return speed_est.speed_rpm;
    }

    // 第一次读数只建立基准，不计算速度，避免启动时产生虚假差值。
    if (!speed_est.initialized) {
        speed_est.last_angle_raw = current_angle_raw;
        speed_est.speed_rpm = 0.0f;
        speed_est.initialized = 1;
        return 0.0f;
    }

    /* 直接差分接口保留给独立调用；系统1 kHz任务使用下方固定滚动窗接口。 */
    int32_t delta = (int32_t)current_angle_raw - (int32_t)speed_est.last_angle_raw;

    // 4. 处理15位编码器的0/32767过零点。
    if (delta > (int32_t)MOTOR_ENCODER_HALF_REV_U32) {
        delta -= (int32_t)MOTOR_ENCODER_COUNTS_PER_REV_U32;
    } else if (delta < -(int32_t)MOTOR_ENCODER_HALF_REV_U32) {
        delta += (int32_t)MOTOR_ENCODER_COUNTS_PER_REV_U32;
    }

    speed_est.last_angle_raw = current_angle_raw;

    // dt_seconds 是相邻两次调用的真实测速间隔，直接用于机械转速换算。
    float instant_rpm = ((float)delta * 60.0f) / (MOTOR_ENCODER_COUNTS_PER_REV_F * dt_seconds);

    // 6. 一阶低通滤波 (EMA)，进一步平滑
    speed_est.speed_rpm = speed_est.filter_alpha * instant_rpm + (1.0f - speed_est.filter_alpha) * speed_est.speed_rpm;

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
float motor_speed_estimator_update_adaptive(float published_speed_rpm)
{
    float abs_rpm = (published_speed_rpm < 0.0f) ? -published_speed_rpm : published_speed_rpm;
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

    uint16_t current_angle_raw = motor_encoder_get_raw_angle();

    if (!speed_est.initialized) {
        speed_est.last_angle_raw = current_angle_raw;
        speed_est.speed_rpm = 0.0f;
        speed_est.instant_speed_rpm = 0.0f;
        speed_est.initialized = 1U;
        g_speed_unwrapped_counts = (uint32_t)current_angle_raw;
        g_speed_history_index = 0U;
        g_speed_history_valid_ticks = 0U;
        g_speed_window_ticks = SPEED_EST_WINDOW_TICKS;
        for (uint16_t i = 0U; i < SPEED_EST_WINDOW_TICKS; ++i) {
            g_speed_position_history[i] = g_speed_unwrapped_counts;
        }
        return 0.0f;
    }

    int32_t step_delta = (int32_t)current_angle_raw - (int32_t)speed_est.last_angle_raw;
    if (step_delta > (int32_t)MOTOR_ENCODER_HALF_REV_U32) {
        step_delta -= (int32_t)MOTOR_ENCODER_COUNTS_PER_REV_U32;
    } else if (step_delta < -(int32_t)MOTOR_ENCODER_HALF_REV_U32) {
        step_delta += (int32_t)MOTOR_ENCODER_COUNTS_PER_REV_U32;
    }
    speed_est.last_angle_raw = current_angle_raw;
    g_speed_unwrapped_counts += (uint32_t)step_delta;

    uint16_t new_history_index = g_speed_history_index + 1U;
    if (new_history_index >= SPEED_EST_WINDOW_TICKS) {
        new_history_index = 0U;
    }

    uint16_t effective_window_ticks = g_speed_window_ticks;
    if (effective_window_ticks > (uint16_t)(g_speed_history_valid_ticks + 1U)) {
        effective_window_ticks = (uint16_t)(g_speed_history_valid_ticks + 1U);
    }
    uint16_t old_history_index =
        (uint16_t)((new_history_index + SPEED_EST_WINDOW_TICKS - effective_window_ticks) % SPEED_EST_WINDOW_TICKS);
    uint32_t old_position = g_speed_position_history[old_history_index];
    g_speed_position_history[new_history_index] = g_speed_unwrapped_counts;
    g_speed_history_index = new_history_index;

    if (g_speed_history_valid_ticks < SPEED_EST_WINDOW_TICKS) {
        g_speed_history_valid_ticks++;
    }

    if (g_speed_history_valid_ticks == 0U) {
        return speed_est.speed_rpm;
    }

    float dt_seconds = (float)effective_window_ticks * MOTOR_SYSTEM_TASK_DT_SEC;
    int32_t window_delta = calculate_position_delta(g_speed_unwrapped_counts, old_position);
    float instant_rpm =
        ((float)window_delta * 60.0f) / (MOTOR_ENCODER_COUNTS_PER_REV_F * dt_seconds);

    speed_est.instant_speed_rpm = instant_rpm;
    speed_est.speed_rpm = speed_est.filter_alpha * instant_rpm + (1.0f - speed_est.filter_alpha) * speed_est.speed_rpm;
    return speed_est.speed_rpm;
}

// ======================= PID 闭环部分 =======================

/**
 * @brief 速度环初始化
 */
void motor_speed_loop_init(void)
{
    // 调用底层的 PID 初始化
    pid_init(&speed_pid, MOTOR_SPEED_PID_KP, MOTOR_SPEED_PID_KI, MOTOR_SPEED_PID_KD, MOTOR_SPEED_PID_OUT_MAX,
             MOTOR_SPEED_PID_OUT_MIN, SPEED_LOOP_DT);
}

/**
 * @brief 设置目标速度
 * @param target_speed_rpm 期望的目标速度(RPM)
 */
void motor_speed_loop_set_target(float target_speed_rpm)
{
    speed_pid.target = target_speed_rpm;
}

/* 调用方在同步更新两个系数时屏蔽 1 kHz 控制任务。 */
uint8_t motor_speed_loop_set_pi(float kp, float ki, uint8_t running)
{
    if (!isfinite(kp) || !isfinite(ki) || kp < 0.0f || kp > 0.1f || ki < 0.0f || ki > 1.0f) {
        return 0U;
    }
    if (running) {
        float error = speed_pid.target - speed_pid.measure;
        /* integral 保存积分电流贡献 A，而非原始误差累加值。
         * 以积分项补偿比例项变化，保持更新瞬间 P+I 给定连续。 */
        speed_pid.integral += (speed_pid.kp - kp) * error;
        if (speed_pid.integral > speed_pid.out_max) {
            speed_pid.integral = speed_pid.out_max;
        }
        if (speed_pid.integral < speed_pid.out_min) {
            speed_pid.integral = speed_pid.out_min;
        }
    } else {
        pid_reset(&speed_pid);
    }
    speed_pid.kp = kp;
    speed_pid.ki = ki;
    return 1U;
}

/**
 * @brief 速度环控制周期更新
 * @param current_speed_rpm 当前的实际速度反馈(RPM)
 * @return float 计算出的目标电流 (Iq_ref)
 */
float motor_speed_loop_update(float current_speed_rpm)
{
    // 1. 更新当前测量值
    speed_pid.measure = current_speed_rpm;

    // 2. 运行 PID 计算
    pid_calculate(&speed_pid);

    // 3. 返回计算的输出值 (作为电流环的 Iq 目标值)
    return speed_pid.output;
}

/**
 * @brief  弱磁控制 (Field Weakening) 策略，根据当前转速调整 D 轴电流以实现更高的速度。
 * @param  current_rpm 当前实际转速
 * @retval 
 */
float motor_speed_loop_field_weakening(float current_rpm)
{

    float fw_rpm_threshold = 1000.0f;
    float fw_gain = 0.001f;
    float fw_max_current = -1.0f;

    float target_d = 0.0f;

    float abs_rpm = current_rpm;
    if (abs_rpm < 0.0f) {
        abs_rpm = -abs_rpm;
    }

    if (abs_rpm > fw_rpm_threshold) {
        target_d = -(abs_rpm - fw_rpm_threshold) * fw_gain;

        if (target_d < fw_max_current) {
            target_d = fw_max_current;
        }
    } else {
        target_d = 0.0f;
    }

    return target_d;
}
