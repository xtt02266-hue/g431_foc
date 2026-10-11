#ifndef MOTOR_IDENTIFY_H
#define MOTOR_IDENTIFY_H

#include <stdint.h>

// 电机辨识状态机枚举。
typedef enum {
    IDENTIFY_STATE_IDLE = 0,      // 空闲状态
    IDENTIFY_STATE_MEASURE_R,     // 装载标称相电阻 (R)
    IDENTIFY_STATE_MEASURE_L,     // 装载标称相电感 (L)
    IDENTIFY_STATE_UVW_AND_POLES, // 识别 UVW 相序方向和极对数
    IDENTIFY_STATE_ALIGN,         // 强行对齐转子（获取电角度零点偏移）
    IDENTIFY_STATE_DONE,          // 辨识完成
    IDENTIFY_STATE_ERROR          // 辨识出错（如电流超限等）
} motor_identify_state_t;

// 辨识结果结构体。
typedef struct {
    float resistance;        // 相电阻 (Ω)
    float inductance;        // 相电感 (H)
    uint16_t pole_pairs;     // 极对数
    float zero_angle_offset; // 机械/电角度的零点对应偏置 (弧度)
    int8_t uvw_dir;          // UVW 相序方向: 1=正向, -1=反向
} motor_identified_params_t;

// 辨识结果全局变量（用于调试器直接观察）。
extern motor_identified_params_t g_identified_params;

// 启动辨识流程序列。
void motor_identify_start(void);

// 获取当前辨识器运行状态。
uint8_t motor_identify_get_state(void);

// 获取已辨识出的参数结果。
motor_identified_params_t motor_identify_get_result(void);

// 加载已校验的辨识结果，不启动辨识运动。
void motor_identify_use_result(float resistance, float inductance, uint16_t pole_pairs, float zero_angle_offset,
                               int8_t uvw_dir);

// 辨识系统周期调度任务（主循环或定时器中调用）。
void motor_identify_task(void);

// 按电角度 rad 和原始幅值生成开环电压矢量；保留 SVPWM 调用与相位约定。
void motor_open_loop_drive(float elec_angle, float amplitude);

#endif
