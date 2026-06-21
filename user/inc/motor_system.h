#ifndef MOTOR_SYSTEM_H
#define MOTOR_SYSTEM_H

#include <stdint.h>
#include "motor_music.h"

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
    MOTOR_STATE_WAIT_PARAMETERS,
    MOTOR_STATE_IDENTIFYING,
    MOTOR_STATE_SENSORED_RUN,
    MOTOR_STATE_MUSIC,
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
MotorState Motor_System_GetState(void);

/* 系统级功能入口：负责检查参数、传感器和当前运行状态。 */
uint8_t Motor_System_StartControl(void);
void Motor_System_StopControl(void);
/* 传感器恢复后需主动清故障，防止电机突然自动恢复出力。 */
uint8_t Motor_System_ClearFault(void);
/* 主动辨识并保存；必须确认电机无负载且可以自由转动。 */
uint8_t Motor_System_IdentifyAndSave(void);
/* 播放有限次数或循环播放，正常有感位置环会在音乐期间暂停。 */
uint8_t Motor_System_PlaySong(const MotorMusicNote *song,
                              uint16_t note_count,
                              uint16_t play_count);
uint8_t Motor_System_PlaySongLoop(const MotorMusicNote *song,
                                  uint16_t note_count);
void Motor_System_StopMusic(void);
/* 当前仅启停并行无感观测器，不会替换 AS5600 换相角度。 */
uint8_t Motor_System_EnableSensorlessObserver(uint8_t enable);
// OLED上显示临时调试信息。
void Motor_ShowDebugInfo_OLED(void);
void Motor_SimulateSpring_Task(void);

#endif
