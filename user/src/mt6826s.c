#include "mt6826s.h"
#include "spi.h"
#include "gpio.h"
#include "motor_encoder.h"

#define MT6826S_CS_GPIO_PORT GPIOA
#define MT6826S_CS_GPIO_PIN  GPIO_PIN_4

/* Burst-read command 1010b + 12-bit start address 0x003. */
#define MT6826S_BURST_COMMAND_HIGH  0xA0U
#define MT6826S_BURST_COMMAND_LOW   0x03U
#define MT6826S_BURST_FRAME_BYTES   6U
#define MT6826S_CRC8_POLYNOMIAL     0x07U

/* DMA owns these buffers until HAL_SPI_TxRxCpltCallback fires. */
static const uint8_t g_mt6826s_tx[MT6826S_BURST_FRAME_BYTES] = {
    MT6826S_BURST_COMMAND_HIGH, MT6826S_BURST_COMMAND_LOW,
    0x00U, 0x00U, 0x00U, 0x00U
};
static uint8_t g_mt6826s_rx[MT6826S_BURST_FRAME_BYTES];
static volatile uint8_t g_mt6826s_busy = 0U;
static volatile uint8_t g_mt6826s_ok = 0U;
static volatile uint16_t g_mt6826s_last_angle = 0U;
static volatile uint8_t g_mt6826s_status = 0U;
static volatile uint32_t g_mt6826s_crc_error_count = 0U;
static volatile uint32_t g_mt6826s_transfer_error_count = 0U;
static uint32_t g_mt6826s_start_ms = 0U;
static uint8_t g_mt6826s_timeout_reported = 0U;

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
    g_mt6826s_busy = 0U;
    g_mt6826s_timeout_reported = 0U;
}

void MT6826S_RequestReadDMA(void)
{
    if (g_mt6826s_busy != 0U) {
        /* A lost DMA completion must invalidate the angle instead of holding
         * the last valid reading indefinitely. Do not touch an active DMA. */
        if ((g_mt6826s_timeout_reported == 0U) &&
            ((uint32_t)(HAL_GetTick() - g_mt6826s_start_ms) > 2U)) {
            g_mt6826s_timeout_reported = 1U;
            g_mt6826s_ok = 0U;
            ++g_mt6826s_transfer_error_count;
        }
        return;
    }

    g_mt6826s_busy = 1U;
    g_mt6826s_start_ms = HAL_GetTick();
    g_mt6826s_timeout_reported = 0U;
    MT6826S_Select();
    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive_DMA(&hspi1,
                                    g_mt6826s_tx,
                                    g_mt6826s_rx,
                                    MT6826S_BURST_FRAME_BYTES);
    if (status != HAL_OK) {
        MT6826S_Unselect();
        g_mt6826s_ok = 0U;
        ++g_mt6826s_transfer_error_count;
        /* HAL may have started RX DMA before TX setup failed. Keep this
         * request latched so the ADC IRQ cannot reuse either DMA buffer. */
        g_mt6826s_timeout_reported = 1U;
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1) {
        return;
    }

    MT6826S_Unselect();

    /* During the two command bytes MISO is high impedance.  The following
     * bytes are registers 0x003 (ANGLE[14:7]), 0x004 (ANGLE[6:0],0),
     * 0x005 (STATUS[2:0]) and 0x006 (CRC8 over the preceding 24 bits). */
    uint8_t calculated_crc = MT6826S_CalculateCrc8(&g_mt6826s_rx[2], 3U);
    if (calculated_crc != g_mt6826s_rx[5]) {
        /* Drop this frame without publishing its angle or refreshing the
         * timestamp. The last CRC-valid sample remains usable only within
         * the existing 2 ms freshness deadline; repeated CRC loss goes stale. */
        ++g_mt6826s_crc_error_count;
        g_mt6826s_busy = 0U;
        return;
    }

    /*
     * STATUS[2:0] are warning flags, not a frame-valid indication.  The
     * datasheet still defines ANGLE and CRC for frames with warning bits set.
     * Rejecting every warning frame used to hold the previous angle for whole
     * rotor sectors (most visibly when the motor field asserted STATUS[1]),
     * which then produced a large artificial speed spike when the warning
     * cleared.  Keep publishing CRC-valid angles and report STATUS separately.
     */
    g_mt6826s_status = g_mt6826s_rx[4] & 0x07U;
    g_mt6826s_last_angle =
        ((((uint16_t)g_mt6826s_rx[2]) << 7) |
         (((uint16_t)g_mt6826s_rx[3]) >> 1)) &
        MT6826S_ANGLE_MAX_15BIT;
    Motor_Encoder_OnSample(g_mt6826s_last_angle);
    g_mt6826s_ok = 1U;
    g_mt6826s_busy = 0U;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1) {
        return;
    }
    MT6826S_Unselect();
    g_mt6826s_ok = 0U;
    ++g_mt6826s_transfer_error_count;
    /* The other DMA channel may still be active after an error. The motor
     * will stop on invalid encoder data; do not reuse these buffers. */
    g_mt6826s_timeout_reported = 1U;
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
