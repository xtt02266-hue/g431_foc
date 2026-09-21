#ifndef __MOTOR_SPEED_LOOP_H
#define __MOTOR_SPEED_LOOP_H

#include "main.h"
#include <stdint.h>
#include "pid.h"

/* 速度测算器结构体 */
typedef struct {
    uint16_t last_angle_raw; // 上一次15位编码器原始读数 (0~32767)
    float speed_rpm;         // 滤波后的转速 (RPM)
    float filter_alpha;      // 一阶低通滤波系数 (0~1)
    uint8_t initialized;     // 是否已初始化首个角度
} MotorSpeedEstimator;

/* 速度环结构体和变量声明 */
extern PID_Controller speed_pid;
extern MotorSpeedEstimator speed_est;

/*
 * 无外加负载、摩擦前馈配合下的保守速度环基线参数。
 * 输入误差单位为机械 RPM，输出单位为 q 轴电流 A。
 */
#define MOTOR_SPEED_PID_KP          0.0020f
#define MOTOR_SPEED_PID_KI          0.0010f
#define MOTOR_SPEED_PID_KD          0.0f
#define MOTOR_SPEED_PID_OUT_MAX     1.1f
#define MOTOR_SPEED_PID_OUT_MIN     -1.1f

/* 函数声明 */
// PID 相关
void Motor_SpeedLoop_Init(void);
void Motor_SpeedLoop_SetTarget(float target_speed_rpm);
float Motor_SpeedLoop_Update(float current_speed_rpm);
void Motor_SpeedLoop_Preload(float target_speed_rpm,
                             float current_speed_rpm,
                             float output_a);

// 速度测算相关
void Motor_SpeedEstimator_Init(float filter_alpha);
float Motor_SpeedEstimator_Update(uint16_t current_angle_raw, float dt_seconds);

/*
 * 每1 ms读取角度并发布速度；根据转速选择20/10/5 ms滚动位置窗，切换带迟滞。
 */
float Motor_SpeedEstimator_UpdateAdaptive(float published_speed_rpm);

// 弱磁控制 (Field Weakening)
float Motor_SpeedLoop_FieldWeakening(float current_rpm);


#endif /* __MOTOR_SPEED_LOOP_H */
