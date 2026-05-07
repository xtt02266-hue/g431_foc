#ifndef AS5600_H
#define AS5600_H

#include "stm32g4xx.h"

#define RAW_Angle_Hi 0x0C
#define RAW_Angle_Lo 0x0D
#define RAW_ANGLE_HI RAW_Angle_Hi
#define RAW_ANGLE_LO RAW_Angle_Lo
#define AS5600_Address 0x6c
#define I2C_TIMEOUT 10000U

uint16_t AS5600_ReadRawAngle(void);

#endif
