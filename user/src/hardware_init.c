#include "hardware_init.h"
#include "adc.h"
#include "tim.h"
#include "user_io.h"
#include "motor_current_loop.h"

// OLED_Init 由显示驱动实现，这里做前置声明。
void OLED_Init(void);

// 硬件外设初始化与启动时序。
void hardware_init(void)
{
    // 1. 启动常规通道 DMA 搬运（读取电位器等慢速信号）。
    // 说明：HAL_ADC_Start_DMA 内部已启动 ADC，本处不再额外调用 HAL_ADC_Start。
    (void)UserIO_StartDma();

    // 2. 启动 TIM1 通道 4（ADC 触发源若配置为 CC4）。
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); 

    // 3. 启动 TIM1 的 6 路互补 PWM 输出（驱动三相半桥）。
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    // 4. 启动 ADC 注入通道中断，使其响应 TIM1 触发进行采样。
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    Motor_CurrentLoop_Init();

    OLED_Init();
    HAL_Delay(50);
}