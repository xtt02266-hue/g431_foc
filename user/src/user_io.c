#include "user_io.h"
#include "adc.h"
#include "gpio.h"
#include "motor_publicdata.h"

// PC4 外部电位器：ADC2 + DMA 循环采样。
#define POT_ADC_HANDLE (&hadc2)

// 外部按键：读取原始电平（无滤波）。
#define BUTTON1_GPIO_PORT GPIOB
#define BUTTON1_PIN GPIO_PIN_10
#define BUTTON2_GPIO_PORT GPIOB
#define BUTTON2_PIN GPIO_PIN_11

// 状态灯：共阴极，PB6 高电平点亮。
#define LED_GPIO_PORT GPIOB
#define LED_PIN GPIO_PIN_6

// 启动电位器 ADC 的 DMA 循环采样（实际上现改为在硬件初始化中底层手动启动无中断 DMA）
// 为了保持接口兼容性，保留此函数，但当前可以为空。
HAL_StatusTypeDef UserIO_StartDma(void)
{
    return HAL_OK;
}

// 返回 12 位 ADC 原始值（DMA 持续更新）。
uint16_t Pot_ReadRaw(void)
{
    return g_motor_publicdata.pot_raw;
}

// 引脚高电平返回 1，低电平返回 0。
uint8_t Button1_ReadLevel(void)
{
    return (HAL_GPIO_ReadPin(BUTTON1_GPIO_PORT, BUTTON1_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

// 引脚高电平返回 1，低电平返回 0。
uint8_t Button2_ReadLevel(void)
{
    return (HAL_GPIO_ReadPin(BUTTON2_GPIO_PORT, BUTTON2_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

// 控制灯亮灭；非 0 表示点亮（GPIO 高）。
void Led_Set(uint8_t on)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// 反转 LED 状态。
void Led_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
}
