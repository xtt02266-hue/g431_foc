#include "motor_system.h"
#include "motor_context.h"
#include "motor_parameters.h"
#include "user_io.h"
#include "tim.h"

static void motor_status_led_task(void);

// 系统初始化入口：交由控制生命周期按原顺序初始化算法和状态。
void motor_system_init(void)
{
    motor_control_init();
}

// TIM2 的 1 ms 控制入口；保持运行周期内部的反馈、保护、标定和模式执行顺序。
void motor_system_task(void)
{
    motor_control_task();
}

// 状态灯 2Hz 闪烁任务 (设计为 1ms 调用一次)
static void motor_status_led_task(void)
{
    // 频率 2Hz = 周期 500ms = 亮 250ms，灭 250ms
    static uint16_t led_cnt = 0;
    led_cnt++;
    if (led_cnt >= 250) {
        led_cnt = 0;
        led_toggle();
    }
}

// 定时器更新中断回调函数 (TIM2, 假设为 1000Hz / 1ms 周期)
// 这是整个电机控制系统的"心跳"中断，所有实时控制逻辑在此统一调度:
//   1. 辨识阶段: 执行开环辨识任务
//   2. 辨识刚完成: 自动整定电流环 PI 参数 + 使能 SVPWM + 开启 FOC 闭环
//   3. 正常运行时: 执行 motor_control_task (位置/速度/Iq 级联控制)
//   4. 每 250ms 翻转一次状态 LED (2Hz 闪烁)
// 注意: 中断中不得执行耗时操作 (如 OLED 刷新、VOFA 发送等)，那些应放在 main 循环中
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2) {
        // 推进电机参数辨识，处理辨识完成或失败状态。
        motor_parameters_control_task1ms();
        // 检查参数、编码器和故障状态，决定运行或安全停机。
        motor_control_update_state();

        // 执行 1 ms 电机任务，更新反馈、目标和控制输出。
        motor_system_task();
        // 按 2 Hz 节奏翻转状态灯。
        motor_status_led_task();
    }
}
