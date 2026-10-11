#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <stdint.h>

/* 当前实际控制角度源：MT6826S 15位SPI绝对式编码器。 */
#define MOTOR_ENCODER_COUNTS_PER_REV_U32 32768U   // MT6826S 机械一圈的 15 位计数范围
#define MOTOR_ENCODER_HALF_REV_U32       16384U   // 半圈计数，用于单圈差分跨零判断
#define MOTOR_ENCODER_COUNT_MASK_U16     0x7FFFU  // 有效机械角计数的低 15 位掩码
#define MOTOR_ENCODER_COUNTS_PER_REV_F   32768.0f // 浮点计算的一圈计数，与整数范围一致
#define MOTOR_ENCODER_HALF_REV_F         16384.0f // 浮点最短路径误差的半圈计数
#define MOTOR_ENCODER_RAD_PER_COUNT (6.28318530718f / MOTOR_ENCODER_COUNTS_PER_REV_F) // 机械角计数到 rad 的换算系数

// 清除角度缓存和有效性历史；等待第一帧有效 MT6826S 样本。
void motor_encoder_init(void);
/* 由20kHz ADC/FOC中断发起下一帧SPI DMA读取，不等待结果。 */
void motor_encoder_update_fast(void);
/* SPI DMA完成后发布CRC有效的角度快照。 */
void motor_encoder_on_sample(uint16_t angle);
/* 其他模块只读缓存，不直接争用SPI。STM32上对齐的16位读写是原子的。 */
uint16_t motor_encoder_get_raw_angle(void);
// 同时检查传感器状态和样本年龄；CRC 错误不刷新有效样本时间戳。
uint8_t motor_encoder_is_data_fresh(uint32_t max_age_ms);
// 读取底层编码器通信状态；不替代有效样本年龄检查。
uint8_t motor_encoder_is_ok(void);

// 返回最近有效样本年龄，单位 ms；无样本时沿用原哨兵值。
uint16_t motor_encoder_get_sample_age_ms(void);
#endif
