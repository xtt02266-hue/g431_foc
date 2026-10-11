#ifndef USER_IO_H
#define USER_IO_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

// 供硬件初始化模块 DMA 配置使用
extern volatile uint16_t g_user_pot_raw;

// 启动电位器 ADC 的 DMA 采样。
HAL_StatusTypeDef user_io_start_dma(void);

// 读取电位器原始 ADC 值。
uint16_t pot_read_raw(void);

// 读取按键 1/2 原始电平。
uint8_t button1_read_level(void);
// 读取按键 2 原始引脚电平，不包含软件去抖。
uint8_t button2_read_level(void);

// 控制 LED 灯。
void led_set(uint8_t on);
// 翻转板载 LED 当前电平，供既有运行状态指示使用。
void led_toggle(void);

#endif
