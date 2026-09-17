#ifndef MOTOR_ANGLE_REFERENCE_H
#define MOTOR_ANGLE_REFERENCE_H

#include <stdint.h>

void Motor_AngleReference_Init(void);
void Motor_AngleReference_BackgroundTask(void);
/* Independent AS5600 calibration: offset is mechanical radians; direction +/-1.
 * Returns 0 on stale/unavailable data or invalid arguments; never controls FOC. */
uint8_t Motor_AngleReference_GetElectricalAngle(uint16_t pole_pairs,
                                               int8_t direction,
                                               float mechanical_offset_rad,
                                               float *electrical_angle_rad);

#endif
