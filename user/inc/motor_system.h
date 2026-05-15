#ifndef MOTOR_SYSTEM_H
#define MOTOR_SYSTEM_H

#include <stdint.h>

// -----------------------------------------
// 全局软硬件核心参数配置区
// -----------------------------------------
// 当前电源供应母线电压 (修改此值，全局相关电压算法都会自动对齐)
#define SYSTEM_BUS_VOLTAGE    15.0f
// -----------------------------------------

// 电机运行状态。
typedef enum
{
    MOTOR_STATE_STOPPED = 0,
    MOTOR_STATE_ALIGN,
    MOTOR_STATE_RUN,
    MOTOR_STATE_FAULT
} MotorState;

// 电机系统关键变量。
typedef struct
{
    MotorState state;
} MotorSystem;

// 电机系统全局实例。
extern MotorSystem g_motor_system;

// 初始化电机系统。
void Motor_System_Init(void);
// 周期任务入口。
void Motor_System_Task(void);
// OLED上显示临时调试信息。
void Motor_ShowDebugInfo_OLED(void);

#endif
