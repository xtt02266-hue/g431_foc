#ifndef AS5600_H
#define AS5600_H

#include "stm32g4xx.h"

// AS5600 原始角度寄存器地址。
#define RAW_Angle_Hi 0x0C
#define RAW_Angle_Lo 0x0D
#define RAW_ANGLE_HI RAW_Angle_Hi
#define RAW_ANGLE_LO RAW_Angle_Lo
// 7 位 I2C 从机地址（未包含读写位）。
#define AS5600_Address 0x6c
// I2C 超时（ms）。
#define I2C_TIMEOUT 10000U

// 读取 AS5600 原始角度值（12 位）。
uint16_t AS5600_ReadRawAngle(void);

#endif
