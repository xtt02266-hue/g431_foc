#include "motor_angle_reference.h"
#include "board_profile.h"
#include <math.h>
#include <stddef.h>
#if BOARD_AS5600_REFERENCE_ENABLE
#include "as5600.h"
#endif

void Motor_AngleReference_Init(void)
{
#if BOARD_AS5600_REFERENCE_ENABLE
    AS5600_RequestRead_DMA();
#endif
}

void Motor_AngleReference_BackgroundTask(void)
{
#if BOARD_AS5600_REFERENCE_ENABLE
    AS5600_BackgroundTask();
#endif
}

uint8_t Motor_AngleReference_GetElectricalAngle(uint16_t pole_pairs,
                                               int8_t direction,
                                               float mechanical_offset_rad,
                                               float *electrical_angle_rad)
{
#if BOARD_AS5600_REFERENCE_ENABLE
    uint32_t primask;
    uint16_t raw;
    uint8_t fresh;
    float angle;
    const float two_pi = 6.28318530718f;
    if ((electrical_angle_rad == NULL) || (pole_pairs == 0U) ||
        ((direction != 1) && (direction != -1)) ||
        !isfinite(mechanical_offset_rad)) {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    fresh = AS5600_IsDataFresh(20U);
    raw = AS5600_ReadRawAngle();
    if (primask == 0U) {
        __enable_irq();
    }
    if (fresh == 0U) {
        return 0U;
    }
    angle = fmodf(((float)raw * (two_pi / 4096.0f) -
                    fmodf(mechanical_offset_rad, two_pi)) *
                   (float)pole_pairs * (float)direction, two_pi);
    *electrical_angle_rad = (angle < 0.0f) ? angle + two_pi : angle;
    return 1U;
#else
    (void)pole_pairs;
    (void)direction;
    (void)mechanical_offset_rad;
    (void)electrical_angle_rad;
    return 0U;
#endif
}
