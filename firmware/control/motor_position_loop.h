#ifndef MOTOR_POSITION_LOOP_H
#define MOTOR_POSITION_LOOP_H

#include <stdint.h>
#include "pid.h"

/*
 * 无外加负载的位置环基线参数，恢复自惯性补偿引入前的 b7cc543。
 * 输入误差单位为MT6826S 15位count，输出单位为机械RPM。
 */
#define MOTOR_POSITION_PID_KP      1.2f
#define MOTOR_POSITION_PID_KI      0.64f
#define MOTOR_POSITION_PID_KD      0.025f
#define MOTOR_POSITION_PID_OUT_MAX 350.0f
#define MOTOR_POSITION_PID_OUT_MIN -350.0f

// 对外暴露的控制变量
extern pid_controller_t g_pi_pos;

// 函数声明
void motor_position_loop_init(void);
/* 清除位置 PID 和跨零展开状态；模式切换、停机或音乐结束时调用。 */
void motor_position_loop_reset(void);
// 输入目标/反馈位置计数和当前 rpm，运行单圈最短路径位置 PID 并输出 rpm。
float motor_position_loop_run(float target_position, float actual_position);

#endif // MOTOR_POSITION_LOOP_H
