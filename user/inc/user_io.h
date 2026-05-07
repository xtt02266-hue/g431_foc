#ifndef USER_IO_H
#define USER_IO_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

// 启动电位器 ADC 的 DMA 采样。
HAL_StatusTypeDef UserIO_StartDma(void);

// 读取电位器原始 ADC 值。
uint16_t Pot_ReadRaw(void);

// 读取按键 1/2 原始电平。
uint8_t Button1_ReadLevel(void);
uint8_t Button2_ReadLevel(void);

// 控制 LED 灯。
void Led_Set(uint8_t on);
void Led_Toggle(void);

#endif
