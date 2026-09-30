#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <stdint.h>

/* 当前实际控制角度源：MT6826S 15位SPI绝对式编码器。 */
#define MOTOR_ENCODER_COUNTS_PER_REV_U32 32768U
#define MOTOR_ENCODER_HALF_REV_U32       16384U
#define MOTOR_ENCODER_COUNT_MASK_U16     0x7FFFU
#define MOTOR_ENCODER_COUNTS_PER_REV_F   32768.0f
#define MOTOR_ENCODER_HALF_REV_F         16384.0f
#define MOTOR_ENCODER_RAD_PER_COUNT      (6.28318530718f / MOTOR_ENCODER_COUNTS_PER_REV_F)

void Motor_Encoder_Init(void);
/* 由20kHz ADC/FOC中断发起下一帧SPI DMA读取，不等待结果。 */
void Motor_Encoder_UpdateFast(void);
/* SPI DMA完成后发布CRC有效的角度快照。 */
void Motor_Encoder_OnSample(uint16_t angle);
/* 其他模块只读缓存，不直接争用SPI。STM32上对齐的16位读写是原子的。 */
uint16_t Motor_Encoder_GetRawAngle(void);
uint8_t Motor_Encoder_IsDataFresh(uint32_t max_age_ms);
uint8_t Motor_Encoder_IsOk(void);

uint16_t Motor_Encoder_GetSampleAgeMs(void);
#endif
