#include "hardware_init.h"
#include "adc.h"
#include "tim.h"
#include "user_io.h"
#include "motor_current_loop.h"

// OLED_Init 由显示驱动实现，这里做前置声明。
void OLED_Init(void);

void hardware_init(void)
{
    // 1. 启动常规通道的 DMA 搬运（读取电位器等慢速信号）
    // 【修改点】：去掉了 HAL_ADC_Start(&hadc1); 
    // 解释：HAL_ADC_Start_DMA 内部已经包含了启动 ADC 的动作。连续调用两者容易导致 HAL 库状态机锁死报错。
    (void)UserIO_StartDma();

    // 2. 启动 TIM1 的通道 4（如果你在 CubeMX 里选了 CC4 作为 ADC 的触发源）
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); 

    // 3. 启动 TIM1 的 6 路互补 PWM 输出 (驱动 DRV8300 的三个半桥)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    // 4. 【核心缺失项】启动 ADC 注入通道中断
    // 解释：你的代码里忘记让 ADC 去监听 TIM1 的触发信号了！加上这句，ADC 才会像狙击手一样潜伏，等待 TIM1 的指令瞬间采样 U/W 相电流。
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    Motor_CurrentLoop_Init();

    OLED_Init();
    HAL_Delay(50);
}