#ifndef __MOTOR_SPEED_LOOP_H
#define __MOTOR_SPEED_LOOP_H

#include "main.h"
#include <stdint.h>
#include "pid.h"

/* 速度测算器结构体 */
typedef struct {
    uint16_t last_angle_raw; // 上一次编码器原始读数 (0~4095)
    float speed_rpm;         // 滤波后的转速 (RPM)
    float filter_alpha;      // 一阶低通滤波系数 (0~1)
    uint8_t initialized;     // 是否已初始化首个角度
} MotorSpeedEstimator;

/* 速度环结构体和变量声明 */
extern PID_Controller speed_pid;
extern MotorSpeedEstimator speed_est;

/* 速度环宏定义：将 PID 参数暴露在头文件中 */
#define MOTOR_SPEED_PID_KP          0.00008f
#define MOTOR_SPEED_PID_KI          0.000003f
#define MOTOR_SPEED_PID_KD          0.0f
#define MOTOR_SPEED_PID_OUT_MAX     2.0f    // 输出到电流环的最大 Iq_ref (A)
#define MOTOR_SPEED_PID_OUT_MIN     -2.0f   // 输出到电流环的最小 Iq_ref (A)

/* 函数声明 */
// PID 相关
void Motor_SpeedLoop_Init(void);
void Motor_SpeedLoop_SetTarget(float target_speed_rpm);
float Motor_SpeedLoop_Update(float current_speed_rpm);

// 速度测算相关
void Motor_SpeedEstimator_Init(float filter_alpha);
float Motor_SpeedEstimator_Update(uint16_t current_angle_raw, float dt_seconds);

// 弱磁控制 (Field Weakening)
float Motor_SpeedLoop_FieldWeakening(float current_rpm);


#endif /* __MOTOR_SPEED_LOOP_H */
