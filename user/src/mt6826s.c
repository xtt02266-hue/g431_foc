#include "mt6826s.h"
#include "spi.h"
#include "gpio.h"

#define MT6826S_CS_GPIO_PORT GPIOA
#define MT6826S_CS_GPIO_PIN  GPIO_PIN_4

/* Burst-read command 1010b + 12-bit start address 0x003. */
#define MT6826S_BURST_COMMAND_HIGH  0xA0U
#define MT6826S_BURST_COMMAND_LOW   0x03U
#define MT6826S_BURST_FRAME_BYTES   6U
#define MT6826S_CRC8_POLYNOMIAL     0x07U

static uint8_t g_mt6826s_ok = 0U;
static uint16_t g_mt6826s_last_angle = 0U;
static uint8_t g_mt6826s_status = 0U;
static uint32_t g_mt6826s_crc_error_count = 0U;
static uint32_t g_mt6826s_transfer_error_count = 0U;

static uint8_t MT6826S_CalculateCrc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;
    for (uint8_t byte_index = 0U; byte_index < length; ++byte_index) {
        crc ^= data[byte_index];
        for (uint8_t bit_index = 0U; bit_index < 8U; ++bit_index) {
            crc = (crc & 0x80U) != 0U
                      ? (uint8_t)((crc << 1) ^ MT6826S_CRC8_POLYNOMIAL)
                      : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

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
    g_mt6826s_status = 0U;
    g_mt6826s_crc_error_count = 0U;
    g_mt6826s_transfer_error_count = 0U;
}

uint16_t MT6826S_ReadRawAngle15(void)
{
    uint8_t tx_data[MT6826S_BURST_FRAME_BYTES] = {
        MT6826S_BURST_COMMAND_HIGH,
        MT6826S_BURST_COMMAND_LOW,
        0x00U, 0x00U, 0x00U, 0x00U
    };
    uint8_t rx_data[MT6826S_BURST_FRAME_BYTES] = {0U};

    MT6826S_Select();
    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(&hspi1,
                                tx_data,
                                rx_data,
                                MT6826S_BURST_FRAME_BYTES,
                                1U);
    MT6826S_Unselect();

    if (status != HAL_OK) {
        g_mt6826s_ok = 0U;
        ++g_mt6826s_transfer_error_count;
        return g_mt6826s_last_angle;
    }

    /* During the two command bytes MISO is high impedance.  The following
     * bytes are registers 0x003 (ANGLE[14:7]), 0x004 (ANGLE[6:0],0),
     * 0x005 (STATUS[2:0]) and 0x006 (CRC8 over the preceding 24 bits). */
    uint8_t calculated_crc = MT6826S_CalculateCrc8(&rx_data[2], 3U);
    if (calculated_crc != rx_data[5]) {
        g_mt6826s_ok = 0U;
        ++g_mt6826s_crc_error_count;
        return g_mt6826s_last_angle;
    }

    g_mt6826s_status = rx_data[4] & 0x07U;
    if (g_mt6826s_status != 0U) {
        g_mt6826s_ok = 0U;
        return g_mt6826s_last_angle;
    }

    g_mt6826s_last_angle =
        ((((uint16_t)rx_data[2]) << 7) | (((uint16_t)rx_data[3]) >> 1)) &
        MT6826S_ANGLE_MAX_15BIT;
    g_mt6826s_ok = 1U;

    return g_mt6826s_last_angle;
}

uint8_t MT6826S_IsOk(void)
{
    return g_mt6826s_ok;
}

uint8_t MT6826S_GetStatus(void)
{
    return g_mt6826s_status;
}

uint32_t MT6826S_GetCrcErrorCount(void)
{
    return g_mt6826s_crc_error_count;
}

uint32_t MT6826S_GetTransferErrorCount(void)
{
    return g_mt6826s_transfer_error_count;
}
