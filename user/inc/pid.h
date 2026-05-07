#ifndef PID_H
#define PID_H

// PID 控制器通用结构体。
typedef struct
{
    float kp;           // 比例系数
    float ki;           // 积分系数（通常已包含采样周期 dt）
    float kd;           // 微分系数
    
    float target;       // 目标设定值 (Setpoint)
    float measure;      // 实际测量值 (Measurement)
    float output;       // PID 计算输出值
    
    float integral;     // 积分累加求和
    float prev_error;   // 上一次的误差记录，用于计算微分
    
    float out_max;      // 输出限幅上限
    float out_min;      // 输出限幅下限
} PID_Controller;

// 初始化 PID 参数。
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max, float out_min);

// 清除 PID 内部状态（如积分和历史误差）。
void PID_Reset(PID_Controller *pid);

// 执行一次 PID 计算，计算结果会存入 pid->output。
void PID_Calculate(PID_Controller *pid);

#endif
