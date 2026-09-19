#include "motor_encoder.h"
#include "board_profile.h"
#if BOARD_CONTROL_ENCODER_AS5600
#include "as5600.h"
#else
#include "mt6826s.h"
#endif
#include "stm32g4xx_hal.h"

#define MOTOR_ENCODER_STATUS_COMM_ERROR 0x80U

static volatile uint16_t g_encoder_angle = 0U;
static volatile uint32_t g_encoder_last_update_ms = 0U;
static volatile uint8_t g_encoder_has_sample = 0U;

void Motor_Encoder_Init(void)
{
    g_encoder_angle = 0U;
    g_encoder_last_update_ms = 0U;
    g_encoder_has_sample = 0U;
#if !BOARD_CONTROL_ENCODER_AS5600
    MT6826S_Init();
#endif
    Motor_Encoder_UpdateFast();
}

void Motor_Encoder_UpdateFast(void)
{
#if BOARD_SENSORED_CONTROL_ENABLE
#if BOARD_CONTROL_ENCODER_AS5600
    if (AS5600_IsDataFresh(2U) == 0U) {
        return;
    }
    /* Preserve the existing 15-bit control-angle ABI. */
    g_encoder_angle = (uint16_t)((AS5600_ReadRawAngle() & 0x0FFFU) << 3U);
#else
    uint16_t angle = MT6826S_ReadRawAngle15();
    if (MT6826S_IsOk() == 0U) {
        return;
    }
    g_encoder_angle = angle & MOTOR_ENCODER_COUNT_MASK_U16;
#endif
    g_encoder_last_update_ms = HAL_GetTick();
    g_encoder_has_sample = 1U;
#endif
}

uint16_t Motor_Encoder_GetRawAngle(void)
{
    return g_encoder_angle;
}

uint8_t Motor_Encoder_IsDataFresh(uint32_t max_age_ms)
{
#if BOARD_CONTROL_ENCODER_AS5600
    return ((g_encoder_has_sample != 0U) &&
            (AS5600_IsDataFresh(max_age_ms) != 0U)) ? 1U : 0U;
#else
    uint32_t last_update = g_encoder_last_update_ms;
    if ((g_encoder_has_sample == 0U) || (MT6826S_IsOk() == 0U)) {
        return 0U;
    }
    return ((uint32_t)(HAL_GetTick() - last_update) <= max_age_ms) ? 1U : 0U;
#endif
}

uint8_t Motor_Encoder_IsOk(void)
{
#if BOARD_CONTROL_ENCODER_AS5600
    return ((g_encoder_has_sample != 0U) &&
            (AS5600_IsI2cOk() != 0U)) ? 1U : 0U;
#else
    return ((g_encoder_has_sample != 0U) && (MT6826S_IsOk() != 0U)) ? 1U : 0U;
#endif
}

uint8_t Motor_Encoder_GetStatus(void)
{
#if BOARD_CONTROL_ENCODER_AS5600
    return (AS5600_IsI2cOk() != 0U) ? 0U : MOTOR_ENCODER_STATUS_COMM_ERROR;
#else
    return MT6826S_GetStatus();
#endif
}
