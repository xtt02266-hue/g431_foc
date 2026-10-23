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

// TIM2 每 1 ms 依次推进辨识、更新状态、执行外环并更新 LED。
// 辨识结束仍需显式 START；Flash 保存、协议发送和设备寄存器事务在前台执行。
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
