#include "motor_system.h"
#include "motor_publicdata.h"

// 电机系统运行状态与目标值缓存。
MotorSystem g_motor_system = {
    .state = MOTOR_STATE_STOPPED,
    .pot_raw = 0U,
    .target = 0U,
};

// 电位器原始值映射到目标量（占位实现）。
static uint16_t Motor_MapPotToTarget(uint16_t pot_raw)
{
    return pot_raw;
}

// 初始化系统状态与目标值。
void Motor_System_Init(void)
{
    g_motor_system.state = MOTOR_STATE_STOPPED;
    g_motor_system.pot_raw = g_motor_publicdata.pot_raw;
    g_motor_system.target = Motor_MapPotToTarget(g_motor_system.pot_raw);
}

// 周期任务：更新输入与目标值。
void Motor_System_Task(void)
{
    g_motor_system.pot_raw = g_motor_publicdata.pot_raw;
    g_motor_system.target = Motor_MapPotToTarget(g_motor_system.pot_raw);
}

// ---------------------------------------------------------
// 定时器更新中断回调函数 (TIM2, 假设为 1000Hz / 1ms 周期)
// ---------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 请确保在 stm32g4xx_it.c 中调用了 HAL_TIM_IRQHandler(&htim2)
    // 且 htim2 已被正确初始化并启动：HAL_TIM_Base_Start_IT(&htim2);
    // 同时如果要在其他文件中使用外部的 htim2，需要包含对应的头文件 (例如 #include "tim.h")
    
    // extern TIM_HandleTypeDef htim2; // 开启这行如果下方报错未定义
    
    // if (htim == &htim2)
    if (htim->Instance == TIM2)
    {
        // 调用辨识状态机 (1ms 周期调度)
        // Motor_Identify_Task();
        
        // 调用系统普通任务 (1ms 周期刷新电位器和目标)
        Motor_System_Task();
        
        // 其他需要在 1ms 下执行的逻辑，如按键扫描延时、速度环更新等
    }
}
