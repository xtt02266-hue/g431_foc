#include "user_io.h"
#include "adc.h"
#include "gpio.h"

volatile uint16_t g_user_pot_raw = 0;

#define POT_ADC_HANDLE (&hadc2)

#define BUTTON1_GPIO_PORT GPIOB
#define BUTTON1_PIN GPIO_PIN_10
#define BUTTON2_GPIO_PORT GPIOB
#define BUTTON2_PIN GPIO_PIN_11

#define LED_GPIO_PORT GPIOB
#define LED_PIN GPIO_PIN_6

HAL_StatusTypeDef UserIO_StartDma(void)
{
    HAL_StatusTypeDef status =
        HAL_DMA_Start(POT_ADC_HANDLE->DMA_Handle,
                      (uint32_t)&POT_ADC_HANDLE->Instance->DR,
                      (uint32_t)&g_user_pot_raw,
                      1U);

    if (status != HAL_OK) {
        return status;
    }

    SET_BIT(POT_ADC_HANDLE->Instance->CFGR, ADC_CFGR_DMACFG);
    SET_BIT(POT_ADC_HANDLE->Instance->CFGR, ADC_CFGR_DMAEN);

    return HAL_ADC_Start(POT_ADC_HANDLE);
}

uint16_t Pot_ReadRaw(void)
{
    return g_user_pot_raw;
}

uint8_t Button1_ReadLevel(void)
{
    return (HAL_GPIO_ReadPin(BUTTON1_GPIO_PORT, BUTTON1_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

uint8_t Button2_ReadLevel(void)
{
    return (HAL_GPIO_ReadPin(BUTTON2_GPIO_PORT, BUTTON2_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

void Led_Set(uint8_t on)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Led_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
}
