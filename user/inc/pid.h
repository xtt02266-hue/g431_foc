#ifndef PID_H
#define PID_H

// PID 控制器通用结构体。
typedef struct
{
    float kp;           // 比例系数 (V/A)
    float ki;           // 积分系数 (V/(A·s))，内部自动乘以 dt
    float kd;           // 微分系数 (V/(A/s))
    float dt;           // 采样周期 (s)，如 20kHz → 0.00005f
    
    float target;       // 目标设定值 (Setpoint)
    float measure;      // 实际测量值 (Measurement)
    float output;       // PID 计算输出值
    
    float integral;     // 积分累加求和
    float prev_error;   // 上一次的误差记录，用于计算微分
    float prev_measure; // 上一次测量值（用于测量值微分，避免微分冲击）
    
    float out_max;      // 输出限幅上限
    float out_min;      // 输出限幅下限
} PID_Controller;

// 初始化 PID 参数。dt 为采样周期（秒），例如 20kHz → 0.00005f。
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, 
              float out_max, float out_min, float dt);

// 清除 PID 内部状态（如积分和历史误差）。
void PID_Reset(PID_Controller *pid);

// 执行一次 PID 计算，计算结果会存入 pid->output。
void PID_Calculate(PID_Controller *pid);

// 根据电机 R/L 参数自动计算电流环 PI 增益。
// resistance: 相电阻 (Ω), inductance: 相电感 (H)
// bandwidth_hz: 期望电流环带宽 (Hz)，典型值 500~2000
// dt: 采样周期 (s)
void PID_AutoTune_CurrentLoop(PID_Controller *pid, float resistance, float inductance,
                               float bandwidth_hz, float bus_voltage, float dt);

#endif
