#include "as5600.h"
#include "i2c.h"

uint8_t as5600_rx_buffer[2] = {0};
volatile uint16_t as5600_current_angle = 0U;
volatile uint8_t as5600_i2c_error = 1U;

static volatile uint8_t g_as5600_has_sample = 0U;
static volatile uint8_t g_as5600_recovery_pending = 0U;
static volatile uint32_t g_as5600_last_update_ms = 0U;

/* 发起一次 DMA 读取，完成回调会自动继续下一次读取。 */
void AS5600_RequestRead_DMA(void)
{
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
        return;
    }

    if (HAL_I2C_Mem_Read_DMA(&hi2c1,
                             AS5600_Address,
                             RAW_ANGLE_HI,
                             I2C_MEMADD_SIZE_8BIT,
                             as5600_rx_buffer,
                             2U) != HAL_OK) {
        as5600_i2c_error = 1U;
        g_as5600_recovery_pending = 1U;
    }
}

uint16_t AS5600_ReadRawAngle(void)
{
    return as5600_current_angle;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) {
        return;
    }

    as5600_current_angle =
        (uint16_t)(((uint16_t)(as5600_rx_buffer[0] & 0x0FU) << 8U) |
                   (uint16_t)as5600_rx_buffer[1]);
    g_as5600_last_update_ms = HAL_GetTick();
    g_as5600_has_sample = 1U;
    g_as5600_recovery_pending = 0U;
    as5600_i2c_error = 0U;

    AS5600_RequestRead_DMA();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) {
        return;
    }

    /* 中断内只置故障标志，耗时的外设重启交给主循环。 */
    as5600_i2c_error = 1U;
    g_as5600_recovery_pending = 1U;
}

uint8_t AS5600_IsI2cOk(void)
{
    return (as5600_i2c_error == 0U) ? 1U : 0U;
}

void AS5600_BackgroundTask(void)
{
    static uint32_t last_attempt_ms = 0U;
    uint32_t now = HAL_GetTick();

    /* 即使 HAL 没有报错，数据长时间不更新也视为总线卡死。 */
    if ((g_as5600_recovery_pending == 0U) &&
        (((g_as5600_has_sample != 0U) &&
          ((uint32_t)(now - g_as5600_last_update_ms) > 20U)) ||
         ((g_as5600_has_sample == 0U) && (now > 100U)))) {
        as5600_i2c_error = 1U;
        g_as5600_recovery_pending = 1U;
    }

    if (g_as5600_recovery_pending == 0U) {
        return;
    }

    if ((uint32_t)(now - last_attempt_ms) < 20U) {
        return;
    }
    last_attempt_ms = now;

    g_as5600_recovery_pending = 0U;
    (void)HAL_I2C_DeInit(&hi2c1);
    MX_I2C1_Init();
    AS5600_RequestRead_DMA();
}

uint8_t AS5600_IsDataFresh(uint32_t max_age_ms)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t last_update;
    uint8_t has_sample;
    uint8_t has_error;

    /* 复制中断共享数据时保持快照一致。 */
    __disable_irq();
    last_update = g_as5600_last_update_ms;
    has_sample = g_as5600_has_sample;
    has_error = as5600_i2c_error;
    if (primask == 0U) {
        __enable_irq();
    }

    if ((has_sample == 0U) || (has_error != 0U)) {
        return 0U;
    }

    return ((uint32_t)(HAL_GetTick() - last_update) <= max_age_ms) ? 1U : 0U;
}
