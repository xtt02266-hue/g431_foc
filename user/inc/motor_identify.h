#ifndef MOTOR_IDENTIFY_H
#define MOTOR_IDENTIFY_H

#include <stdint.h>

// 电机辨识状态机枚举。
typedef enum
{
    IDENTIFY_STATE_IDLE = 0,       // 空闲状态
    IDENTIFY_STATE_MEASURE_R,      // 测量相电阻 (R)
    IDENTIFY_STATE_MEASURE_L,      // 测量相电感 (L)
    IDENTIFY_STATE_UVW_AND_POLES,  // 识别 UVW 相序方向和极对数
    IDENTIFY_STATE_ALIGN,          // 强行对齐转子（获取电角度零点偏移）
    IDENTIFY_STATE_DONE,           // 辨识完成
    IDENTIFY_STATE_ERROR           // 辨识出错（如电流超限等）
} MotorIdentifyState;

// 辨识结果结构体。
typedef struct
{
    float resistance;         // 相电阻 (Ω)
    float inductance;         // 相电感 (H)
    uint16_t pole_pairs;      // 极对数
    float zero_angle_offset;  // 机械/电角度的零点对应偏置 (弧度)
    int8_t uvw_dir;            // UVW 相序方向: 1=正向, -1=反向
} MotorIdentifiedParams;

// 辨识结果全局变量（用于调试器直接观察）。
extern MotorIdentifiedParams g_identified_params;

// 启动辨识流程序列。
void Motor_Identify_Start(void);

// 获取当前辨识器运行状态。
MotorIdentifyState Motor_Identify_GetState(void);

// 获取已辨识出的参数结果。
MotorIdentifiedParams Motor_Identify_GetResult(void);

// Load a previously validated result without running identification motion.
void Motor_Identify_UseResult(const MotorIdentifiedParams *params);

// 辨识系统周期调度任务（主循环或定时器中调用）。
void Motor_Identify_Task(void);

void Motor_OpenLoop_Drive(float elec_angle, float amplitude);

#endif
