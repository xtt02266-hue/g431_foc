#ifndef MOTOR_PUBLICDATA_H
#define MOTOR_PUBLICDATA_H

#include <stdint.h>

// 外部模块共享的数据结构。
typedef struct
{
    // 电位器 ADC 原始值（DMA 更新）。
    volatile uint16_t pot_raw;
} MotorPublicData;

// 全局共享数据实例。
extern MotorPublicData g_motor_publicdata;

#endif
