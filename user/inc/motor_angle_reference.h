#ifndef MOTOR_ANGLE_REFERENCE_H
#define MOTOR_ANGLE_REFERENCE_H

#include <stdint.h>

// 按板级配置启动 AS5600 参考角采样；有感配置下保持空操作。
void motor_angle_reference_init(void);
// 前台维护可选参考编码器，避免在控制中断中执行 I2C 恢复。
void motor_angle_reference_background_task(void);
/* 独立 AS5600 参考角：偏置单位机械 rad，方向为 ±1。
 * 数据过期、不可用或参数无效时返回 0；此接口不控制 FOC。 */
uint8_t motor_angle_reference_get_electrical_angle(uint16_t pole_pairs, int8_t direction, float mechanical_offset_rad,
                                                   float *electrical_angle_rad);

#endif
