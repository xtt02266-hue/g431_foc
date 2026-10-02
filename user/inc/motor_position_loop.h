#ifndef MOTOR_POSITION_LOOP_H
#define MOTOR_POSITION_LOOP_H

#include "stdint.h"
#include "pid.h"

/*
 * 无外加负载的位置环基线参数，恢复自惯性补偿引入前的 b7cc543。
 * 输入误差单位为MT6826S 15位count，输出单位为机械RPM。
 */
#define MOTOR_POSITION_PID_KP       1.2f
#define MOTOR_POSITION_PID_KI       0.64f
#define MOTOR_POSITION_PID_KD       0.025f
#define MOTOR_POSITION_PID_OUT_MAX  350.0f
#define MOTOR_POSITION_PID_OUT_MIN  -350.0f

// 对外暴露的控制变量
extern PID_Controller g_pi_pos;

// 函数声明
void Motor_PositionLoop_Init(void);
/* 清除位置 PID 和跨零展开状态；模式切换、停机或音乐结束时调用。 */
void Motor_PositionLoop_Reset(void);
float Motor_PositionLoop_Run(float target_position, float actual_position);

#endif // MOTOR_POSITION_LOOP_H
