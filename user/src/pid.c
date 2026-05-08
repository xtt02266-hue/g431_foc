#include "pid.h"
#include <math.h>
#include "stm32g4xx.h"                  // Device header

// 初始化 PID 参数。
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, 
              float out_max, float out_min, float dt)
{
    if (pid == 0) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = (dt > 0.0f) ? dt : 0.00005f;  // 默认 20kHz
    
    pid->out_max = out_max;
    pid->out_min = out_min;
    
    pid->target = 0.0f;
    pid->measure = 0.0f;
    pid->output = 0.0f;
    
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measure = 0.0f;
}

// 积分清零操作。
void PID_Reset(PID_Controller *pid)
{
    if (pid == 0) return;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measure = pid->measure;
    pid->output = 0.0f;
}

// 执行一次 PID 计算（含 dt 缩放 + 条件积分抗饱和）。
void PID_Calculate(PID_Controller *pid)
{
    if (pid == 0) return;
    if (pid->dt <= 0.0f) return;

    // 1. 计算当前误差
    float error = pid->target - pid->measure;
    
    // 2. 比例项：直接响应误差
    float p_term = pid->kp * error;
    
    // 3. 条件积分抗饱和（Conditional Integration）
    //    只有当输出未饱和，或积分正在将输出拉离饱和区时，才累加积分
    //    这防止积分在输出饱和后继续"刹车失灵"式地累积
    float tentative_output = p_term + pid->integral;
    uint8_t allow_integral = 1;

    if (tentative_output >= pid->out_max && error > 0.0f)
    {
        allow_integral = 0;  // 已到上限且误差仍为正，不累加
    }
    else if (tentative_output <= pid->out_min && error < 0.0f)
    {
        allow_integral = 0;  // 已到下限且误差仍为负，不累加
    }

    if (allow_integral)
    {
        pid->integral += pid->ki * error * pid->dt;
        // 积分项独立限幅（防御性编程）
        if (pid->integral > pid->out_max) pid->integral = pid->out_max;
        if (pid->integral < pid->out_min) pid->integral = pid->out_min;
    }
    
    // 4. 微分项：使用测量值微分（避免目标突变产生微分冲击）
    float derivative = (pid->prev_measure - pid->measure) / pid->dt;
    float d_term = pid->kd * derivative;
    
    // 更新历史值
    pid->prev_error = error;
    pid->prev_measure = pid->measure;
    
    // 5. 计算总输出
    float output = p_term + pid->integral + d_term;
    
    // 6. 输出限幅
    if (output > pid->out_max)  output = pid->out_max;
    if (output < pid->out_min)  output = pid->out_min;
    
    pid->output = output;
}

// 根据电机 R/L 参数自动计算电流环 PI 增益。
// 使用零极点对消法：PI 零点抵消电机极点 (s + R/L)
//   Kp = L * ωc        (ωc = 2π * bandwidth_hz)
//   Ki = R * ωc
// 输出限幅：SVPWM 线性区最大相电压 = Vbus/√3，留 5% 余量
void PID_AutoTune_CurrentLoop(PID_Controller *pid, float resistance, float inductance,
                               float bandwidth_hz, float bus_voltage, float dt)
{
    if (pid == 0) return;
    if (resistance <= 0.0f || inductance <= 0.0f) return;

    float wc = 2.0f * 3.1415926f * bandwidth_hz;  // 期望带宽 (rad/s)
    
    float kp = inductance * wc;                     // 比例增益 (V/A)
    float ki = resistance * wc*0.5f;                     // 积分增益 (V/(A·s))
    float kd = 0.1f;                                // 电流环一般不用微分
    
    // SVPWM 最大不失真相电压 = Vbus/√3 ≈ Vbus × 0.577
    float v_max = bus_voltage * 0.57735f * 0.95f;
    float out_max =  v_max;
    float out_min = -v_max;
    
    PID_Init(pid, kp, ki, kd, out_max, out_min, dt);
}