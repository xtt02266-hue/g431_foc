#include "hardware_init.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "i2c.h"
#include "user_io.h"
#include "motor_current_loop.h"
#include "motor_publicdata.h"

// OLED_Init 由显示驱动实现，这里做前置声明。
void OLED_Init(void);

// 硬件外设初始化与启动时序。
void hardware_init(void)
{
    // 0. 执行 ADC 内部自校准以提高采样精度（必须在 ADC 启动前调用）
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    // 1. 启动常规通道 DMA 搬运（读取电位器等慢速信号）
    // 为了避免单数据高频循环转换导致 DMA 中断风暴卡死，这里不使用 HAL_ADC_Start_DMA 附带的软件中断。
    // 使用寄存器直接启动 ADC 和 DMA 请求。由于 CubeMX 配置了 DMA 循环模式，这就可以实现纯硬件后台搬运。
    HAL_DMA_Start(hadc2.DMA_Handle, (uint32_t)&hadc2.Instance->DR, (uint32_t)&g_motor_publicdata.pot_raw, 1U);
    SET_BIT(hadc2.Instance->CFGR, ADC_CFGR_DMACFG); // 设为 DMA 循环模式
    SET_BIT(hadc2.Instance->CFGR, ADC_CFGR_DMAEN);  // 开启 ADC2 的 DMA 请求功能
    HAL_ADC_Start(&hadc2);                          // 开启 ADC2

    // 2. 首先配置并开启 ADC 注入通道，等待被 TIM1 触发
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    Motor_CurrentLoop_Init();

    // 3. 启动 TIM1 通道 4（用作 ADC 的触发信号 CC4）
    HAL_TIM_Base_Init(&htim1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

    // 4. 启动 TIM1 的 6 路互补 PWM 输出（驱动三相半桥）。

    // 5. 启动 TIM2 (用于 1ms / 1000Hz 周期任务调度，如 AS5600 慢速读取、目标值更新等)
    HAL_TIM_Base_Start_IT(&htim2);

    // 6. 启动串口接收 (示例：按需开启串口空闲中断/DMA接收，如与上位机通信)
    // 假设你有全局接收缓存 rx_buffer，请解开注释并修改：
    // HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, RX_BUFFER_SIZE); 
    // 或 HAL_UART_Receive_IT(&huart2, &rx_data, 1);

    // 7. 启动 I2C DMA (如果对于 AS5600 你编写了基于 DMA 的无阻塞读取逻辑)
    // 注意：如果是普通阻塞式读写(HAL_I2C_Master_Transmit)，无需在此处 Init 外启动。
    // 如果使用 DMA 周期读取，请将请求动作发在这里或 TIM2 任务中。
    // HAL_I2C_Master_Receive_DMA(&hi2c1, AS5600_Address | 1, i2c_rx_buffer, 2);

    OLED_Init();
    HAL_Delay(50);
}
