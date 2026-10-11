#ifndef MOTOR_SPEED_LOOP_H
#define MOTOR_SPEED_LOOP_H

#include "main.h"
#include <stdint.h>
#include "pid.h"
#include "motor_config.h"

/* 速度测算器结构体 */
typedef struct {
    uint16_t last_angle_raw; // 上一次15位编码器原始读数 (0~32767)
    float speed_rpm;         // 滤波后的转速 (RPM)
    float instant_speed_rpm; // 自适应窗口直接计算的未滤波转速 (RPM)
    float filter_alpha;      // 一阶低通滤波系数 (0~1)
    uint8_t initialized;     // 是否已初始化首个角度
} motor_speed_estimator_t;

/* 速度环结构体和变量声明 */
extern pid_controller_t speed_pid;
extern motor_speed_estimator_t speed_est;

/*
 * 无外加负载、摩擦前馈配合下的保守速度环基线参数。
 * 输入误差单位为机械 RPM，输出单位为 q 轴电流 A。
 */
#define MOTOR_SPEED_PID_KP      0.0007f
#define MOTOR_SPEED_PID_KI      0.0032f
#define MOTOR_SPEED_PID_KD      0.0f
#define MOTOR_SPEED_PID_OUT_MAX MOTOR_RATED_CURRENT_A
#define MOTOR_SPEED_PID_OUT_MIN (-MOTOR_RATED_CURRENT_A)

/* 函数声明 */
// PID 相关
void motor_speed_loop_init(void);
// 校验 Kp（A/rpm）与 Ki（A/(rpm·s)）；运行时保持输出连续，成功返回 1。
uint8_t motor_speed_loop_set_pi(float kp, float ki, uint8_t running);
// 设置速度环目标，单位 rpm；不自动启动电机。
void motor_speed_loop_set_target(float target_speed_rpm);
// 以 1 ms 周期计算速度 PID，输入 rpm，输出 Iq（A）。
float motor_speed_loop_update(float current_speed_rpm);

// 速度测算相关
void motor_speed_estimator_init(float filter_alpha);
// 用单圈角计数和 dt（s）更新估算 rpm；保留环绕及滤波规则。
float motor_speed_estimator_update(uint16_t current_angle_raw, float dt_seconds);

/*
 * 每1 ms读取角度并发布速度；根据转速选择20/10/5 ms滚动位置窗，切换带迟滞。
 */
float motor_speed_estimator_update_adaptive(float published_speed_rpm);
// 依据转速 rpm 与迟滞门限选择计数差分窗口，返回 1 ms 周期数。
uint8_t motor_speed_estimator_get_window_ticks(void);

// 弱磁控制 (Field Weakening)
float motor_speed_loop_field_weakening(float current_rpm);

#endif /* __MOTOR_SPEED_LOOP_H */
