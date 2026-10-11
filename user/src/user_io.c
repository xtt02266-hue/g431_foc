#include "user_io.h"
#include "adc.h"
#include "gpio.h"

volatile uint16_t g_user_pot_raw = 0;

#define POT_ADC_HANDLE (&hadc2) // 电位器由 ADC2 DMA 独占采样

#define BUTTON1_GPIO_PORT GPIOB       // 按键 1 GPIO 端口
#define BUTTON1_PIN       GPIO_PIN_10 // 按键 1 对应 PB10
#define BUTTON2_GPIO_PORT GPIOB       // 按键 2 GPIO 端口
#define BUTTON2_PIN       GPIO_PIN_11 // 按键 2 对应 PB11

#define LED_GPIO_PORT GPIOB      // 状态 LED GPIO 端口
#define LED_PIN       GPIO_PIN_6 // 状态 LED 对应 PB6，高电平点亮

// 启动 ADC2 DMA 电位器采样；失败返回 HAL 状态，缓冲区持续有效。
HAL_StatusTypeDef user_io_start_dma(void)
{
    HAL_StatusTypeDef status = HAL_DMA_Start(POT_ADC_HANDLE->DMA_Handle, (uint32_t)&POT_ADC_HANDLE->Instance->DR,
                                             (uint32_t)&g_user_pot_raw, 1U);

    if (status != HAL_OK) {
        return status;
    }

    SET_BIT(POT_ADC_HANDLE->Instance->CFGR, ADC_CFGR_DMACFG);
    SET_BIT(POT_ADC_HANDLE->Instance->CFGR, ADC_CFGR_DMAEN);

    return HAL_ADC_Start(POT_ADC_HANDLE);
}

// 读取 DMA 发布的电位器原始 ADC 计数，不发起新采样。
uint16_t pot_read_raw(void)
{
    return g_user_pot_raw;
}

// 读取按键 1 原始引脚电平，不包含软件去抖。
uint8_t button1_read_level(void)
{
    return (HAL_GPIO_ReadPin(BUTTON1_GPIO_PORT, BUTTON1_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

// 读取按键 2 原始引脚电平，不包含软件去抖。
uint8_t button2_read_level(void)
{
    return (HAL_GPIO_ReadPin(BUTTON2_GPIO_PORT, BUTTON2_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

// 按原有效电平设置板载 LED，不改变引脚配置。
void led_set(uint8_t on)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// 翻转板载 LED 当前电平，供既有运行状态指示使用。
void led_toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
}
