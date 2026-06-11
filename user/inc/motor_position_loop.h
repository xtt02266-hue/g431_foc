#ifndef MOTOR_POSITION_LOOP_H
#define MOTOR_POSITION_LOOP_H

#include "stdint.h"
#include "pid.h"

// 位置环相关的宏定义（根据实际情况调整）
#define MOTOR_POSITION_PID_KP       0.05f
#define MOTOR_POSITION_PID_KI       0.04f
#define MOTOR_POSITION_PID_KD       0.001f
#define MOTOR_POSITION_PID_OUT_MAX  200.0f  // 输出到速度环的最大速度 (rad/s 或 rpm)
#define MOTOR_POSITION_PID_OUT_MIN  -200.0f // 输出到速度环的最小速度

// 对外暴露的控制变量
extern PID_Controller g_pi_pos;

// 函数声明
void Motor_PositionLoop_Init(void);
float Motor_PositionLoop_Run(float target_position, float actual_position);

#endif // MOTOR_POSITION_LOOP_H

