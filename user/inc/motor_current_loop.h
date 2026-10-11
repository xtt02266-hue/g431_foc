#ifndef MOTOR_CURRENT_LOOP_H
#define MOTOR_CURRENT_LOOP_H

#include <stdint.h>
#include "pid.h"

// ADC 参考电压（V）。
#ifndef MOTOR_CURRENT_VREF_VOLTS
#define MOTOR_CURRENT_VREF_VOLTS 3.3f
#endif

// 电流采样偏置电压（V）。
#ifndef MOTOR_CURRENT_BIAS_VOLTS
#define MOTOR_CURRENT_BIAS_VOLTS 1.65f
#endif

// 分流电阻值（Ω）。
#ifndef MOTOR_CURRENT_SHUNT_OHMS
#define MOTOR_CURRENT_SHUNT_OHMS 0.016f
#endif

// 放大器增益。
#ifndef MOTOR_CURRENT_GAIN
#define MOTOR_CURRENT_GAIN 20.0f
#endif

// ADC 最大计数值。
#ifndef MOTOR_CURRENT_ADC_MAX
#define MOTOR_CURRENT_ADC_MAX 4095.0f
#endif

// d 轴电流环 PID 默认参数（辨识完成后由 AutoTunePID 自动覆盖为最优值）。
// Kp 单位: V/A  (每安培误差输出电压)
// Ki 单位: V/(A·s) (内部自动乘 dt)
// Kd 单位: V/(A/s)
#ifndef MOTOR_CURRENT_PID_D_KP
#define MOTOR_CURRENT_PID_D_KP      1.0f
#define MOTOR_CURRENT_PID_D_KI      1000.0f
#define MOTOR_CURRENT_PID_D_KD      0.0f
#define MOTOR_CURRENT_PID_D_OUT_MAX 8.3f // 默认 d 轴 PID 电压输出上限 V；实际电压还由 SVPWM 按母线配置限幅
#define MOTOR_CURRENT_PID_D_OUT_MIN -8.3f
#endif

// q 轴电流环 PID 默认参数。
#ifndef MOTOR_CURRENT_PID_Q_KP
#define MOTOR_CURRENT_PID_Q_KP      1.0f
#define MOTOR_CURRENT_PID_Q_KI      1000.0f
#define MOTOR_CURRENT_PID_Q_KD      0.0f
#define MOTOR_CURRENT_PID_Q_OUT_MAX 8.3f
#define MOTOR_CURRENT_PID_Q_OUT_MIN -8.3f
#endif

// 电流采样参数。
typedef struct {
    float vref_volts;
    float bias_u_volts;
    float bias_w_volts;
    float shunt_ohms;
    float gain;
    float adc_max;

    // 电机本体辨识参数
    uint16_t pole_pairs;     // 极对数
    float zero_angle_offset; // 机械零点偏置(弧度)
    int8_t uvw_dir;          // 相序方向 (1 或 -1)
} motor_current_params_t;

// 电流采样数据。
typedef struct {
    uint16_t iu_raw;
    uint16_t iw_raw;
    float iu_a;
    float iw_a;
} motor_current_sample_t;

// Clarke 变换输出（α-β）。
typedef struct {
    float alpha;
    float beta;
} motor_clarke_frame_t;

// Park 变换输出（d-q）。
typedef struct {
    float d;
    float q;
} motor_park_frame_t;

// 电流环/FOC 全局状态。
typedef struct {
    motor_current_params_t params; // 固定硬件参数
    motor_current_sample_t sample; // ADC 采样结果
    motor_clarke_frame_t clarke;   // Clarke 变换结果 (α, β)
    motor_park_frame_t park;       // Park 变换结果 (d, q)

    pid_controller_t pi_d; // d 轴电流 PID 控制器
    pid_controller_t pi_q; // q 轴电流 PID 控制器

    float target_d; // d 轴目标电流 (A)
    float target_q; // q 轴目标电流 (A)

    // 闭环使能标志：0=闭环计算停止(PID复位并停止输出占空比)，1=执行全套电流闭环
    uint8_t closed_loop_enable;

    float sin_theta;            // 当前电角度的正弦值
    float cos_theta;            // 当前电角度的余弦值
    float electrical_angle_rad; // FOC实际使用的归一化电角度 [0, 2π)
} motor_current_loop_state_t;

// 暴露全局状态以供其他模块读取/调试，或作为对外统一接口。
extern motor_current_loop_state_t g_foc_state;

// 初始化电流环模块。
void motor_current_loop_init(void);
// 设置所有参数。
void motor_current_loop_set_params(float vref_volts, float bias_u_volts, float bias_w_volts, float shunt_ohms,
                                   float gain, float adc_max, uint16_t pole_pairs, float zero_angle_offset,
                                   int8_t uvw_dir);
// 设置电机辨识参数 (把 motor_identify 辨识出的参数灌入给 FOC)
void motor_current_loop_set_motor_identity_params(uint16_t pole_pairs, float zero_angle_offset, int8_t uvw_dir);
// FOC 闭环启停开关
void motor_current_loop_enable(uint8_t enable);
// 读取电流环使能标志；该值不代表实际 PWM 已产生输出。
uint8_t motor_current_loop_is_enabled(void);
// 根据辨识出的 R/L 自动整定电流环 PID（辨识完成后调用）
void motor_current_loop_auto_tune_pid(float resistance, float inductance, float bus_voltage);
// 按辨识 R/L 和指定带宽 Hz 配置电流 PI；保持原有阻尼和电压限幅。
void motor_current_loop_auto_tune_pid_with_bandwidth(float resistance, float inductance, float bus_voltage,
                                                     float bandwidth_hz);
// 获取当前参数。
motor_current_params_t motor_current_loop_get_params(void);
// 设置偏置电压。
void motor_current_loop_set_bias_volts(float bias_volts);
// 分别设置 U/W 两路零电流偏置电压。
void motor_current_loop_set_phase_bias_volts(float bias_u_volts, float bias_w_volts);
// 设置参考电压。
void motor_current_loop_set_vref_volts(float vref_volts);
// 设置放大器增益。
void motor_current_loop_set_gain(float gain);
// 设置分流电阻值。
void motor_current_loop_set_shunt_ohms(float shunt_ohms);
// 根据相电流（U/W）计算变换及控制。
void motor_current_loop_run(uint16_t iu_raw, uint16_t iw_raw);

// 获取最近一次采样。
motor_current_sample_t motor_current_loop_get_last_sample(void);

// Clarke 变换（两相电流输入，三相假设 iU + iV + iW = 0）。
motor_clarke_frame_t motor_current_loop_clarke(float iu_a, float iw_a);
// Park 变换（α-β 到 d-q）。
motor_park_frame_t motor_current_loop_park(motor_clarke_frame_t ab, float sin_theta, float cos_theta);
// 逆 Park 变换（d-q 到 α-β）。
motor_clarke_frame_t motor_current_loop_inv_park(motor_park_frame_t dq, float sin_theta, float cos_theta);

#endif
