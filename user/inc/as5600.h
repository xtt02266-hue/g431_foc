#ifndef AS5600_H
#define AS5600_H

#include "stm32g4xx.h"

// AS5600 原始角度寄存器地址。
#define RAW_Angle_Hi 0x0C
#define RAW_Angle_Lo 0x0D
#define RAW_ANGLE_HI RAW_Angle_Hi
#define RAW_ANGLE_LO RAW_Angle_Lo
// 7 位 I2C 从机地址（未包含读写位）。
#define AS5600_7BIT_ADDRESS 0x36
#define AS5600_Address      (AS5600_7BIT_ADDRESS << 1)
// I2C 超时（ms）。
#define I2C_TIMEOUT 10000U

extern uint8_t as5600_rx_buffer[2];

// 发起 DMA 无阻塞读取请求
void AS5600_RequestRead_DMA(void);

// 读取 AS5600 原始角度值缓存（12 位）。
uint16_t AS5600_ReadRawAngle(void);

// 检查 I2C 通信是否正常（1=正常, 0=故障）。
uint8_t AS5600_IsI2cOk(void);

#endif
