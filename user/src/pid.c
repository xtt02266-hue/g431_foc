#include "pid.h"

// 初始化 PID 参数。
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max, float out_min)
{
    if (pid == 0) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    pid->out_max = out_max;
    pid->out_min = out_min;
    
    pid->target = 0.0f;
    pid->measure = 0.0f;
    pid->output = 0.0f;
    
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

// 积分清零操作。
void PID_Reset(PID_Controller *pid)
{
    if (pid == 0) return;

    pid->target = 0.0f;
    pid->measure = 0.0f;
    pid->output = 0.0f;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

// 执行一次 PID 计算，并将结果保存在 pid->output 中。
void PID_Calculate(PID_Controller *pid)
{
    if (pid == 0) return;

    // 1. 计算当前误差
    float error = pid->target - pid->measure;
    
    // 2. 比例项
    float p_term = pid->kp * error;
    
    // 3. 积分项（累加误差，并包含防积分饱和）
    pid->integral += pid->ki * error;
    
    // 简单的抗积分风饱和（Anti-windup）：将积分项直接限制在输出范围内
    if (pid->integral > pid->out_max)
    {
        pid->integral = pid->out_max;
    }
    else if (pid->integral < pid->out_min)
    {
        pid->integral = pid->out_min;
    }
    
    // 4. 微分项
    float derivative = error - pid->prev_error;
    float d_term = pid->kd * derivative;
    
    // 更新上一次误差
    pid->prev_error = error;
    
    // 5. 计算总输出
    float output = p_term + pid->integral + d_term;
    
    // 6. 输出限幅
    if (output > pid->out_max)
    {
        output = pid->out_max;
    }
    else if (output < pid->out_min)
    {
        output = pid->out_min;
    }
    
    pid->output = output;
}