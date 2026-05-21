#include "motor_position_loop.h"

// 位置环的 PID 控制器实例
PID_Controller g_pi_pos;

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
             
    PID_Reset(&g_pi_pos);
    g_pi_pos.target = 0.0f;
}

/**
 * @brief  位置环运行一次
 * @param  target_position: 期望目标角度 / 位置
 * @param  actual_position: 实际当前角度 / 位置
 * @retval 计算得到的目标速度 (传给速度环的 reference)
 */
float Motor_PositionLoop_Run(float target_position, float actual_position)
{
    // 处理 0~4095 过零点“最短路径”问题，避免在 0 和 4095 之间来回疯抖
    float pos_error = target_position - actual_position;
    if (pos_error > 2048.0f) {
        actual_position += 4096.0f;
    } else if (pos_error < -2048.0f) {
        actual_position -= 4096.0f;
    }

    // 设置PID的输入参数
    g_pi_pos.target = target_position;
    g_pi_pos.measure = actual_position;
    
    // 执行PID计算 (计算结果储存在 g_pi_pos.output 内)
    PID_Calculate(&g_pi_pos);
    
    // 返回计算输出 (通常这作为速度环的 target)
    return g_pi_pos.output;
}
