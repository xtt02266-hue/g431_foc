#ifndef MOTOR_FEEDFORWARD_H
#define MOTOR_FEEDFORWARD_H

#include <stdint.h>

typedef struct {
    uint8_t enabled;
    float coulomb_iq_a;
    float viscous_iq_a_per_rpm;
    float smooth_speed_rpm;
    float max_iq_a;
} motor_friction_config_t;

// 初始化摩擦补偿配置，保持原默认参数与使能规则。
void motor_feedforward_friction_init(void);
// 检查库仑/黏性摩擦参数、平滑速度和 Iq 限幅；配置非法返回 0。
uint8_t motor_feedforward_is_friction_config_valid(const motor_friction_config_t *config);
// 接受有效摩擦配置；电流单位 A，速度单位 rpm。
void motor_feedforward_set_friction_config(uint8_t enabled, float coulomb_iq_a, float viscous_iq_a_per_rpm,
                                           float smooth_speed_rpm, float max_iq_a);
// 复制当前摩擦配置，供协议读取、保存或标定恢复。
void motor_feedforward_get_friction_config(motor_friction_config_t *config);
// 按当前配置把 rpm 转换为摩擦前馈 Iq（A），保留零速平滑和限幅。
float motor_feedforward_friction_compensation(float speed_rpm);
// 按指定配置计算库仑项与黏性项 Iq（A），不修改模块配置。
float motor_feedforward_friction_with_config(const motor_friction_config_t *config, float speed_rpm);
// 计算自由模式的小幅速度阻尼 Iq（A）；保持独立阻尼限幅。
float motor_feedforward_free_compensation(float speed_rpm);

#endif
