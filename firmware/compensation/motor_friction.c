#include "motor_friction.h"
#include "motor_config.h"
#include "motor_friction_storage.h"
#include <math.h>

/*
 * 低速摩擦模型：Iq = Ic*tanh(rpm/smooth) + Kv*rpm。
 * tanh使零速附近连续，Ic描述库仑摩擦，Kv描述随速度增长的粘性摩擦。
 * 参数由上位机在无外载、恒速正反转数据上拟合。
 */
/* 摩擦配置由停机配置命令更新，控制路径只读并计算 Iq（A）。
 * 同步由原调用路径负责，初始化、更新和限幅次序保持不变。
 */
static motor_friction_config_t g_friction_config;

// 按给定上下界裁剪标量；输入与边界使用相同物理单位。
static float motor_feedforward_clamp_float(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

// 检查库仑/黏性摩擦参数、平滑速度和 Iq 限幅；配置非法返回 0。
uint8_t motor_feedforward_is_friction_config_valid(const motor_friction_config_t *config)
{
    if ((config == 0) || (config->enabled > 1U) || !isfinite(config->coulomb_iq_a) ||
        !isfinite(config->viscous_iq_a_per_rpm) || !isfinite(config->smooth_speed_rpm) || !isfinite(config->max_iq_a)) {
        return 0U;
    }
    if ((config->coulomb_iq_a < 0.0f) || (config->coulomb_iq_a > 0.5f) || (config->viscous_iq_a_per_rpm < 0.0f) ||
        (config->viscous_iq_a_per_rpm > 0.01f) || (config->smooth_speed_rpm < 1.0f) ||
        (config->smooth_speed_rpm > 200.0f) || (config->max_iq_a < 0.0f) ||
        (config->max_iq_a > MOTOR_TORQUE_CURRENT_LIMIT_A) || ((config->enabled != 0U) && (config->max_iq_a <= 0.0f))) {
        return 0U;
    }
    return 1U;
}

// 初始化摩擦补偿配置，保持原默认参数与使能规则。
void motor_feedforward_friction_init(void)
{
    g_friction_config.enabled = 0U;
    g_friction_config.coulomb_iq_a = 0.02f;
    g_friction_config.viscous_iq_a_per_rpm = 0.0f;
    g_friction_config.smooth_speed_rpm = 20.0f;
    g_friction_config.max_iq_a = 0.03f;
    (void)motor_friction_storage_load(&g_friction_config);
}

// 接受有效摩擦配置；电流单位 A，速度单位 rpm。
void motor_feedforward_set_friction_config(uint8_t enabled, float coulomb_iq_a, float viscous_iq_a_per_rpm,
                                           float smooth_speed_rpm, float max_iq_a)
{
    const motor_friction_config_t config = {enabled, coulomb_iq_a, viscous_iq_a_per_rpm, smooth_speed_rpm, max_iq_a};

    if (motor_feedforward_is_friction_config_valid(&config) != 0U) {
        g_friction_config = config;
    }
}

// 复制当前摩擦配置，供协议读取、保存或标定恢复。
void motor_feedforward_get_friction_config(motor_friction_config_t *config)
{
    if (config != 0) {
        *config = g_friction_config;
    }
}

// 按当前配置把 rpm 转换为摩擦前馈 Iq（A），保留零速平滑和限幅。
float motor_feedforward_friction_compensation(float speed_rpm)
{
    return motor_feedforward_friction_with_config(&g_friction_config, speed_rpm);
}

// 按指定配置计算库仑项与黏性项 Iq（A），不修改模块配置。
float motor_feedforward_friction_with_config(const motor_friction_config_t *config, float speed_rpm)
{
    if (!motor_feedforward_is_friction_config_valid(config) || !config->enabled || !isfinite(speed_rpm)) {
        return 0.0f;
    }
    float iq =
        config->coulomb_iq_a * tanhf(speed_rpm / config->smooth_speed_rpm) + config->viscous_iq_a_per_rpm * speed_rpm;
    return motor_feedforward_clamp_float(iq, -config->max_iq_a, config->max_iq_a);
}

// 计算自由模式的小幅速度阻尼 Iq（A）；保持独立阻尼限幅。
float motor_feedforward_free_compensation(float speed_rpm)
{
    if ((g_friction_config.enabled == 0U) || !isfinite(speed_rpm)) {
        return 0.0f;
    }
    float damping_iq = MOTOR_FREE_FRICTION_DAMPING_A_PER_RAD_S * speed_rpm * (6.28318530718f / 60.0f);
    damping_iq = motor_feedforward_clamp_float(damping_iq, -MOTOR_FREE_FRICTION_DAMPING_MAX_A,
                                               MOTOR_FREE_FRICTION_DAMPING_MAX_A);
    return motor_feedforward_clamp_float(motor_feedforward_friction_compensation(speed_rpm) - damping_iq,
                                         -g_friction_config.max_iq_a, g_friction_config.max_iq_a);
}
