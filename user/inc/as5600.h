#ifndef AS5600_H
#define AS5600_H

#include "stm32g4xx.h"

// AS5600 原始角度寄存器地址。
#define RAW_Angle_Hi 0x0C         // AS5600 原始角高字节寄存器地址，保留旧名称
#define RAW_Angle_Lo 0x0D         // AS5600 原始角低字节寄存器地址，保留旧名称
#define RAW_ANGLE_HI RAW_Angle_Hi // 原始角高字节寄存器的兼容别名
#define RAW_ANGLE_LO RAW_Angle_Lo // 原始角低字节寄存器的兼容别名
// 7 位 I2C 从机地址（未包含读写位）。
#define AS5600_7BIT_ADDRESS 0x36                       // AS5600 七位 I2C 从机地址
#define AS5600_Address      (AS5600_7BIT_ADDRESS << 1) // HAL I2C 接口使用左移一位的从机地址

extern uint8_t as5600_rx_buffer[2];

// 发起 DMA 无阻塞读取请求
void as5600_request_read_dma(void);

// 读取 AS5600 原始角度值缓存（12 位）。
uint16_t as5600_read_raw_angle(void);

// 检查 I2C 通信是否正常（1=正常, 0=故障）。
uint8_t as5600_is_i2c_ok(void);

/* 主循环后台任务：把可能阻塞的 I2C 恢复操作移出中断。 */
void as5600_background_task(void);

/* 最近 max_age_ms 内收到过有效角度时返回 1。 */
uint8_t as5600_is_data_fresh(uint32_t max_age_ms);

#endif
