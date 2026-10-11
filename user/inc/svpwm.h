#ifndef SVPWM_H
#define SVPWM_H

#include <stdint.h>

// SVPWM 输出状态。
typedef struct {
    float duty_a;    // A 相占空比 (0.0 ~ 1.0)
    float duty_b;    // B 相占空比 (0.0 ~ 1.0)
    float duty_c;    // C 相占空比 (0.0 ~ 1.0)
    float v_alpha;   // 当前输入的 α 轴电压 (V)
    float v_beta;    // 当前输入的 β 轴电压 (V)
    float v_bus;     // 母线电压 (V)
    float mod_index; // 当前调制比 (0.0 ~ 1.15)
    uint8_t enabled; // 输出使能: 0=禁止(50%零矢量), 1=使能
} svpwm_state_t;

// 全局 SVPWM 状态（供调试查看）。
extern svpwm_state_t g_svpwm;

// 初始化 SVPWM 模块（设置 TIM1 参数）。
void svpwm_init(void);

// 设置 α-β 轴目标电压并执行 SVPWM 输出。
// v_alpha, v_beta: 静止坐标系电压 (V)，来自逆 Park 变换。
// v_bus: 当前直流母线电压 (V)。
// 调用后自动更新 TIM1 CCR1/CCR2/CCR3。
void svpwm_set_voltage(float v_alpha, float v_beta, float v_bus);

// 使能原 PWM 输出路径；实际电压仍受 SetVoltage 和控制状态约束。
void svpwm_enable(void);
// 读取 PWM 使能状态，不代表实际端电压或电机运动。
uint8_t svpwm_is_enabled(void);

// 禁用 SVPWM 输出：三相均输出 50% 占空比（零电压矢量），电机无力。
void svpwm_disable(void);

#endif
