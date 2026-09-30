#include "motor_encoder.h"
#include "board_profile.h"
#include "mt6826s.h"
#include "stm32g4xx_hal.h"

static volatile uint16_t g_encoder_angle = 0U;
static volatile uint32_t g_encoder_last_update_ms = 0U;
static volatile uint8_t g_encoder_has_sample = 0U;

void Motor_Encoder_Init(void)
{
    g_encoder_angle = 0U;
    g_encoder_last_update_ms = 0U;
    g_encoder_has_sample = 0U;
    MT6826S_Init();
    /* ADC中断启动前先请求一帧，供首次FOC计算使用。 */
    Motor_Encoder_UpdateFast();
}

void Motor_Encoder_UpdateFast(void)
{
#if BOARD_SENSORED_CONTROL_ENABLE
    MT6826S_RequestReadDMA();
#endif
}

void Motor_Encoder_OnSample(uint16_t angle)
{
#if BOARD_SENSORED_CONTROL_ENABLE
    g_encoder_angle = angle & MOTOR_ENCODER_COUNT_MASK_U16;
    g_encoder_last_update_ms = HAL_GetTick();
    g_encoder_has_sample = 1U;
#else
    (void)angle;
#endif
}

uint16_t Motor_Encoder_GetRawAngle(void)
{
    return g_encoder_angle;
}

uint16_t Motor_Encoder_GetSampleAgeMs(void)
{
    uint32_t age = HAL_GetTick() - g_encoder_last_update_ms;
    return !g_encoder_has_sample || age > 65535U ? 65535U : (uint16_t)age;
}

uint8_t Motor_Encoder_IsDataFresh(uint32_t max_age_ms)
{
    uint32_t last_update = g_encoder_last_update_ms;
    if ((g_encoder_has_sample == 0U) || (MT6826S_IsOk() == 0U)) {
        return 0U;
    }
    return ((uint32_t)(HAL_GetTick() - last_update) <= max_age_ms) ? 1U : 0U;
}

uint8_t Motor_Encoder_IsOk(void)
{
    return ((g_encoder_has_sample != 0U) && (MT6826S_IsOk() != 0U)) ? 1U : 0U;
}
