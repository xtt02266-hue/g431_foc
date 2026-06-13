#include "mt6826s.h"
#include "spi.h"
#include "gpio.h"

#define MT6826S_CS_GPIO_PORT GPIOA
#define MT6826S_CS_GPIO_PIN  GPIO_PIN_4

static uint8_t g_mt6826s_ok = 0U;
static uint16_t g_mt6826s_last_angle = 0U;

static void MT6826S_Select(void)
{
    HAL_GPIO_WritePin(MT6826S_CS_GPIO_PORT,
                      MT6826S_CS_GPIO_PIN,
                      GPIO_PIN_RESET);
}

static void MT6826S_Unselect(void)
{
    HAL_GPIO_WritePin(MT6826S_CS_GPIO_PORT,
                      MT6826S_CS_GPIO_PIN,
                      GPIO_PIN_SET);
}

void MT6826S_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    MT6826S_Unselect();

    GPIO_InitStruct.Pin = MT6826S_CS_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MT6826S_CS_GPIO_PORT, &GPIO_InitStruct);

    MT6826S_Unselect();
    g_mt6826s_ok = 0U;
    g_mt6826s_last_angle = 0U;
}

uint16_t MT6826S_ReadRawAngle15(void)
{
    uint8_t tx_data[2] = {0x00U, 0x00U};
    uint8_t rx_data[2] = {0x00U, 0x00U};

    MT6826S_Select();
    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(&hspi1,
                                tx_data,
                                rx_data,
                                2U,
                                2U);
    MT6826S_Unselect();

    if (status != HAL_OK) {
        g_mt6826s_ok = 0U;
        return g_mt6826s_last_angle;
    }

    uint16_t frame =
        ((uint16_t)rx_data[0] << 8) |
        (uint16_t)rx_data[1];

    g_mt6826s_last_angle = (frame >> 1) & MT6826S_ANGLE_MAX_15BIT;
    g_mt6826s_ok = 1U;

    return g_mt6826s_last_angle;
}

uint8_t MT6826S_IsOk(void)
{
    return g_mt6826s_ok;
}
