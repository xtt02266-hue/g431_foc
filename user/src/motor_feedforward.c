#include "motor_feedforward.h"
#include "motor_config.h"
#include <math.h>

/*
 * 低速摩擦模型：Iq = Ic*tanh(rpm/smooth) + Kv*rpm。
 * tanh使零速附近连续，Ic描述库仑摩擦，Kv描述随速度增长的粘性摩擦。
 * 参数由上位机在无外载、恒速正反转数据上拟合。
 */
static MotorFrictionConfig g_friction_config;

static float Motor_Feedforward_ClampFloat(float value,
                                          float min_value,
                                          float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

uint8_t Motor_Feedforward_IsFrictionConfigValid(
    const MotorFrictionConfig *config)
{
    if ((config == 0) || (config->enabled > 1U) ||
        !isfinite(config->coulomb_iq_a) ||
        !isfinite(config->viscous_iq_a_per_rpm) ||
        !isfinite(config->smooth_speed_rpm) ||
        !isfinite(config->max_iq_a)) return 0U;
    if ((config->coulomb_iq_a < 0.0f) ||
        (config->coulomb_iq_a > 0.5f) ||
        (config->viscous_iq_a_per_rpm < 0.0f) ||
        (config->viscous_iq_a_per_rpm > 0.01f) ||
        (config->smooth_speed_rpm < 1.0f) ||
        (config->smooth_speed_rpm > 200.0f) ||
        (config->max_iq_a < 0.0f) ||
        (config->max_iq_a > MOTOR_TORQUE_CURRENT_LIMIT_A) ||
        ((config->enabled != 0U) && (config->max_iq_a <= 0.0f))) return 0U;
    return 1U;
}

void Motor_Feedforward_FrictionInit(void)
{
    g_friction_config.enabled = 0U;
    g_friction_config.coulomb_iq_a = 0.02f;
    g_friction_config.viscous_iq_a_per_rpm = 0.0f;
    g_friction_config.smooth_speed_rpm = 20.0f;
    g_friction_config.max_iq_a = 0.03f;
}

void Motor_Feedforward_SetFrictionConfig(const MotorFrictionConfig *config)
{
    if (Motor_Feedforward_IsFrictionConfigValid(config) != 0U) {
        g_friction_config = *config;
    }
}

void Motor_Feedforward_GetFrictionConfig(MotorFrictionConfig *config)
{
    if (config != 0) *config = g_friction_config;
}

float Motor_Feedforward_FrictionCompensation(float speed_rpm)
{
    if ((g_friction_config.enabled == 0U) || !isfinite(speed_rpm)) {
        return 0.0f;
    }
    float iq = g_friction_config.coulomb_iq_a *
                   tanhf(speed_rpm / g_friction_config.smooth_speed_rpm) +
               g_friction_config.viscous_iq_a_per_rpm * speed_rpm;
    return Motor_Feedforward_ClampFloat(iq,
                                        -g_friction_config.max_iq_a,
                                        g_friction_config.max_iq_a);
}
