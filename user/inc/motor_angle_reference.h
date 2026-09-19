#ifndef MOTOR_ANGLE_REFERENCE_H
#define MOTOR_ANGLE_REFERENCE_H

#include <stdint.h>

void Motor_AngleReference_Init(void);
void Motor_AngleReference_BackgroundTask(void);
/* sensorless混合配置下，从同一个AS5600样本计算对比用参考电角度。
 * offset为机械弧度，direction为+/-1；数据过期或参数非法时返回0。 */
uint8_t Motor_AngleReference_GetElectricalAngle(uint16_t pole_pairs,
                                               int8_t direction,
                                               float mechanical_offset_rad,
                                               float *electrical_angle_rad);

#endif
