#ifndef STAGE5_TEST_SUPPORT_H
#define STAGE5_TEST_SUPPORT_H

#include <stdint.h>
#include "motor_parameters.h"

/* 主机验证桩的可控输入与可观测量。所有桩均以本结构作为唯一可观察出口，
 * 以便对 baseline 与当前实现做逐周期 A/B 比较。 */
typedef struct
{
    /* 可控输入 */
    MotorParametersStatus param_status;     /* 参数模块状态 */
    uint8_t  as5600_fresh;                  /* 编码器数据是否新鲜 */
    uint16_t as5600_angle;                  /* 编码器原始角度 */
    uint8_t  music_playing;                 /* 音乐是否播放中 */
    int      current_loop_enabled;          /* 电流环使能（由 Enable 桩更新） */
    int      svpwm_enabled;                 /* SVPWM 使能（由 Enable 桩更新） */

    /* 观测量/调用计数 */
    unsigned pid_reset_count;               /* PID_Reset 调用次数 */
    unsigned pos_reset_count;               /* Motor_PositionLoop_Reset 次数 */
    unsigned traj_clear_count;              /* Motor_Trajectory_Clear 次数 */
    unsigned ff_calc_count;                 /* Motor_Feedforward_Calculate 次数 */
    unsigned pos_run_count;                 /* Motor_PositionLoop_Run 次数 */
    unsigned music_task_count;              /* Motor_Music_Task1ms 次数 */

    /* 单周期内桩调用顺序追踪（用于识别 early return 路径） */
    char     event[64];
    unsigned evlen;
} TestState;

extern TestState g_test;

void Test_ResetAll(void);
void Test_ResetCounters(void);
void Test_Event(char c);

#endif
