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

// 电机实时运行数据
typedef struct
{
    float speed_rpm;       // 实时转速 (RPM)
    uint16_t pot_raw;      // 电位器原始数值 (0-4095)
} MotorRunData;

// 电机系统关键变量。
typedef struct
{
    MotorState state;
    MotorRunData run_data; // 实时的运行状态数据 (转速、电位器值等)
} MotorSystem;

#define MOTOR_SYSTEM_TASK_DT_SEC       0.001f
#define SPEED_EST_LOW_RPM_THRESHOLD    50.0f
#define SPEED_EST_MID_RPM_THRESHOLD    200.0f
#define SPEED_EST_HIGH_RPM_THRESHOLD   500.0f
#define SPEED_EST_LOW_PERIOD_TICKS     20U     // 50Hz，低速测速窗口更长，降低量化抖动
#define SPEED_EST_MID_PERIOD_TICKS     5U      // 200Hz
#define SPEED_EST_HIGH_PERIOD_TICKS    2U      // 500Hz
#define SPEED_EST_MAX_PERIOD_TICKS     1U      // 1000Hz，高速测速

// 电机系统全局实例。
extern MotorSystem g_motor_system;

// 初始化电机系统。
void Motor_System_Init(void);
// 周期任务入口。
void Motor_System_Task(void);
// OLED上显示临时调试信息。
void Motor_ShowDebugInfo_OLED(void);
void Motor_SimulateSpring_Task(void);

#endif
