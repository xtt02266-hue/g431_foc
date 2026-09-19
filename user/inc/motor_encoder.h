#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <stdint.h>

/* 控制链统一使用15位角度域。启用有感控制的 sensorless 配置把
 * AS5600 的12位角左移3位；其他有感配置直接使用MT6826S的15位角。 */
#define MOTOR_ENCODER_COUNTS_PER_REV_U32 32768U
#define MOTOR_ENCODER_HALF_REV_U32       16384U
#define MOTOR_ENCODER_COUNT_MASK_U16     0x7FFFU
#define MOTOR_ENCODER_COUNTS_PER_REV_F   32768.0f
#define MOTOR_ENCODER_HALF_REV_F         16384.0f
#define MOTOR_ENCODER_RAD_PER_COUNT      (6.28318530718f / MOTOR_ENCODER_COUNTS_PER_REV_F)

void Motor_Encoder_Init(void);
/* 只允许20kHz ADC/FOC中断调用，刷新当前控制角度快照。 */
void Motor_Encoder_UpdateFast(void);
/* 其他模块只读缓存，不直接争用SPI。STM32上对齐的16位读写是原子的。 */
uint16_t Motor_Encoder_GetRawAngle(void);
uint8_t Motor_Encoder_IsDataFresh(uint32_t max_age_ms);
uint8_t Motor_Encoder_IsOk(void);
uint8_t Motor_Encoder_GetStatus(void);

#endif
