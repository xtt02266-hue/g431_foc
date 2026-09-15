#include "motor_position_loop.h"

// 位置环的 PID 控制器实例
PID_Controller g_pi_pos;

#define MOTOR_POSITION_COUNTS_PER_REV   4096.0f
#define MOTOR_POSITION_HALF_REV_COUNTS  2048.0f

/*
 * 位置 D 项不能直接对 AS5600 的 0~4095 原始值求导：转子正向跨过零点时，
 * 原始读数会从 4095 跳到 0，普通差分会误认为位置瞬间倒退了 4095 counts。
 * 这里保存一份连续展开的位置，仅供 PID 的测量微分使用；位置误差仍按最短路径计算。
 */
static float g_position_unwrapped = 0.0f;
static float g_position_last_wrapped = 0.0f;
static uint8_t g_position_unwrap_initialized = 0U;

static float Motor_PositionLoop_ShortestError(float target_position,
                                              float actual_position)
{
    float error = target_position - actual_position;

    if (error > MOTOR_POSITION_HALF_REV_COUNTS) {
        error -= MOTOR_POSITION_COUNTS_PER_REV;
    } else if (error < -MOTOR_POSITION_HALF_REV_COUNTS) {
        error += MOTOR_POSITION_COUNTS_PER_REV;
    }

    return error;
}

/**
 * @brief  位置环初始化
 * @param  无
 * @retval 无
 */
void Motor_PositionLoop_Init(void)
{
    // 假设位置环在更低频率运行，例如 1kHz， dt 为 0.001s（请根据你的实际任务调度周期修改 dt 参数）
    const float pid_dt = 0.001f; 

    PID_Init(&g_pi_pos, 
             MOTOR_POSITION_PID_KP, MOTOR_POSITION_PID_KI, MOTOR_POSITION_PID_KD, 
             MOTOR_POSITION_PID_OUT_MAX, MOTOR_POSITION_PID_OUT_MIN, pid_dt);
             
    Motor_PositionLoop_Reset();
}

/*
 * 为什么需要这个函数：除了 PID 积分和微分历史，位置环还保存了跨零展开状态。
 * 如果停机或切换模式时只调用 PID_Reset，重新进入位置模式后旧位置可能造成 D 项冲击。
 * 本函数统一清除两类状态，让下一次 Run 从当前编码器位置平滑开始。
 */
void Motor_PositionLoop_Reset(void)
{
    PID_Reset(&g_pi_pos);
    g_pi_pos.target = 0.0f;
    g_position_unwrapped = 0.0f;
    g_position_last_wrapped = 0.0f;
    g_position_unwrap_initialized = 0U;
}

/**
 * @brief  位置环运行一次
 * @param  target_position: 期望目标角度 / 位置
 * @param  actual_position: 实际当前角度 / 位置
 * @retval 计算得到的目标速度 (传给速度环的 reference)
 */
float Motor_PositionLoop_Run(float target_position, float actual_position)
{
    float position_error =
        Motor_PositionLoop_ShortestError(target_position,
                                         actual_position);

    /*
     * 第一次运行时将连续位置对齐到当前实测值，并同步 PID 的微分历史，
     * 避免刚进入位置模式时把当前位置当作一次巨大跳变。
     */
    if (g_position_unwrap_initialized == 0U) {
        g_position_unwrapped = actual_position;
        g_position_last_wrapped = actual_position;
        g_pi_pos.measure = g_position_unwrapped;
        g_pi_pos.prev_measure = g_position_unwrapped;
        g_position_unwrap_initialized = 1U;
    } else {
        /* 将本周期机械位移限制到最短跨零差值，例如 4095->0 等价于 +1。 */
        float position_delta =
            Motor_PositionLoop_ShortestError(actual_position,
                                             g_position_last_wrapped);
        g_position_unwrapped += position_delta;
        g_position_last_wrapped = actual_position;
    }

    // PID 误差仍等于环形最短位置误差，D 项则对连续展开的位置求导。
    g_pi_pos.target = g_position_unwrapped + position_error;
    g_pi_pos.measure = g_position_unwrapped;
    
    // 执行PID计算 (计算结果储存在 g_pi_pos.output 内)
    PID_Calculate(&g_pi_pos);
    
    // 返回计算输出 (通常这作为速度环的 target)
    return g_pi_pos.output;
}
