#include "motor_sensorless.h"
#include "main.h"
#include <math.h>
#include <stddef.h>

#define MOTOR_SENSORLESS_TWO_PI    6.283185307f // 无感电角度环绕的一圈 rad
#define MOTOR_SENSORLESS_RPM_SCALE 9.549296586f // 机械 rad/s 到 rpm 的换算系数 60/(2π)

typedef struct {
    motor_sensorless_config_t config;
    motor_sensorless_output_t output;

    float last_current_alpha;
    float last_current_beta;
    float filtered_bemf_alpha;
    float filtered_bemf_beta;
    float pll_integral_speed;

    uint16_t lock_counter;
    uint16_t loss_counter;
    uint8_t pll_counter;
    uint8_t current_initialized;
    uint8_t phase_initialized;
    uint8_t enabled;
    int8_t direction;
} motor_sensorless_internal_t;

/* 观测器历史由 Update 维护，配置入口沿用原临界区复位状态。
 * α/β 电流单位 A、反电势单位 V、PLL 速度为电气 rad/s；对外机械速度为 rpm。
 */
static motor_sensorless_internal_t g_sensorless = {0};

// 计算标量绝对值，沿用观测器原有非有限值传播方式。
static float motor_sensorless_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

// 按上下界限制观测器标量，输入与边界保持相同单位。
static float motor_sensorless_clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

// 将电角度 rad 环绕至 [0, 2π)，保持现有浮点运算次序。
static float motor_sensorless_wrap_angle(float angle)
{
    while (angle >= MOTOR_SENSORLESS_TWO_PI) {
        angle -= MOTOR_SENSORLESS_TWO_PI;
    }
    while (angle < 0.0f) {
        angle += MOTOR_SENSORLESS_TWO_PI;
    }
    return angle;
}

// 清除反电势滤波、PLL 和锁定历史，避免沿用上次观测结果。
static void motor_sensorless_reset_state(void)
{
    g_sensorless.output = (motor_sensorless_output_t){0};
    g_sensorless.output.status = g_sensorless.enabled ? MOTOR_SENSORLESS_SEARCHING : MOTOR_SENSORLESS_DISABLED;
    g_sensorless.last_current_alpha = 0.0f;
    g_sensorless.last_current_beta = 0.0f;
    g_sensorless.filtered_bemf_alpha = 0.0f;
    g_sensorless.filtered_bemf_beta = 0.0f;
    g_sensorless.pll_integral_speed = 0.0f;
    g_sensorless.lock_counter = 0U;
    g_sensorless.loss_counter = 0U;
    g_sensorless.pll_counter = 0U;
    g_sensorless.current_initialized = 0U;
    g_sensorless.phase_initialized = 0U;
}

// 输出默认观测器配置；采样周期单位 s，R/L 分别为 Ω/H。
void motor_sensorless_get_default_config(motor_sensorless_config_t *config)
{
    if (config == NULL) {
        return;
    }

    config->sample_time_sec = MOTOR_SENSORLESS_SAMPLE_TIME_SEC;
    config->resistance_ohm = 1.0f;
    config->inductance_h = 0.0006f;
    config->pole_pairs = 1U;
    config->bemf_filter_alpha = 0.05f;
    config->minimum_bemf_volts = 0.40f;
    config->pll_kp = 250.0f;
    config->pll_ki = 12000.0f;
    config->maximum_electrical_speed_rad_s = 8000.0f;
    config->lock_phase_error = 0.30f;
    config->lock_updates = 100U;
    config->loss_updates = 250U;
    config->pll_divider = 4U;
}

// 装入默认配置并复位观测器；沿用板级默认使能设置。
void motor_sensorless_init(void)
{
    motor_sensorless_get_default_config(&g_sensorless.config);
    g_sensorless.direction = 1;
    g_sensorless.enabled = MOTOR_SENSORLESS_DEFAULT_ENABLE;
    motor_sensorless_reset_state();
}

// 校验并采用观测器配置，保持原有范围检查和复位规则。
uint8_t motor_sensorless_configure(float sample_time_sec, float resistance_ohm, float inductance_h, uint16_t pole_pairs,
                                   float bemf_filter_alpha, float minimum_bemf_volts, float pll_kp, float pll_ki,
                                   float maximum_electrical_speed_rad_s, float lock_phase_error, uint16_t lock_updates,
                                   uint16_t loss_updates, uint8_t pll_divider)
{
    const motor_sensorless_config_t config = {sample_time_sec,
                                              resistance_ohm,
                                              inductance_h,
                                              pole_pairs,
                                              bemf_filter_alpha,
                                              minimum_bemf_volts,
                                              pll_kp,
                                              pll_ki,
                                              maximum_electrical_speed_rad_s,
                                              lock_phase_error,
                                              lock_updates,
                                              loss_updates,
                                              pll_divider};

    uint32_t primask;

    if (!(config.sample_time_sec > 0.0f) || !(config.resistance_ohm > 0.0f) || !(config.inductance_h > 0.0f) ||
        (config.pole_pairs == 0U) || !(config.bemf_filter_alpha > 0.0f) || (config.bemf_filter_alpha > 1.0f) ||
        !(config.minimum_bemf_volts > 0.0f) || !(config.pll_kp > 0.0f) || !(config.pll_ki > 0.0f) ||
        !(config.maximum_electrical_speed_rad_s > 0.0f) || !(config.lock_phase_error > 0.0f) ||
        (config.lock_updates == 0U) || (config.loss_updates == 0U) || (config.pll_divider == 0U)) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    g_sensorless.config = config;
    motor_sensorless_reset_state();
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

// 更新电机 R（Ω）、L（H）和极对数，用于反电势估计。
uint8_t motor_sensorless_configure_motor(float resistance_ohm, float inductance_h, uint16_t pole_pairs)
{
    motor_sensorless_config_t config = g_sensorless.config;

    config.resistance_ohm = resistance_ohm;
    config.inductance_h = inductance_h;
    config.pole_pairs = pole_pairs;
    return motor_sensorless_configure(
        config.sample_time_sec, config.resistance_ohm, config.inductance_h, config.pole_pairs, config.bemf_filter_alpha,
        config.minimum_bemf_volts, config.pll_kp, config.pll_ki, config.maximum_electrical_speed_rad_s,
        config.lock_phase_error, config.lock_updates, config.loss_updates, config.pll_divider);
}

// 更新观测器使能状态并按既有规则复位历史。
void motor_sensorless_enable(uint8_t enable)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_sensorless.enabled = (enable != 0U) ? 1U : 0U;
    motor_sensorless_reset_state();
    if (primask == 0U) {
        __enable_irq();
    }
}

// 读取观测器使能状态，不表示估计角度已锁定有效。
uint8_t motor_sensorless_is_enabled(void)
{
    return g_sensorless.enabled;
}

// 主动复位观测器历史和输出有效性。
void motor_sensorless_reset(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    motor_sensorless_reset_state();
    if (primask == 0U) {
        __enable_irq();
    }
}

// 设置电角度方向，保持原有 ±1 约定与无效输入处理。
void motor_sensorless_set_direction(int8_t direction)
{
    if (direction > 0) {
        g_sensorless.direction = 1;
    } else if (direction < 0) {
        g_sensorless.direction = -1;
    }
}

// 高频路径用 α/β 电流 A 与电压 V 更新反电势及 PLL；输出电角度 rad。
void motor_sensorless_update(float voltage_alpha, float voltage_beta, float current_alpha, float current_beta)
{
    motor_sensorless_config_t *config = &g_sensorless.config;
    motor_sensorless_output_t *output = &g_sensorless.output;
    float derivative_alpha;
    float derivative_beta;
    float raw_bemf_alpha;
    float raw_bemf_beta;
    float bemf_magnitude;
    float measured_alpha;
    float measured_beta;
    float predicted_alpha;
    float predicted_beta;
    float pll_dt;
    float speed_limit;
    float phase_error;

    if (g_sensorless.enabled == 0U) {
        return;
    }

    if (g_sensorless.current_initialized == 0U) {
        g_sensorless.last_current_alpha = current_alpha;
        g_sensorless.last_current_beta = current_beta;
        g_sensorless.current_initialized = 1U;
        return;
    }

    derivative_alpha = (current_alpha - g_sensorless.last_current_alpha) / config->sample_time_sec;
    derivative_beta = (current_beta - g_sensorless.last_current_beta) / config->sample_time_sec;
    g_sensorless.last_current_alpha = current_alpha;
    g_sensorless.last_current_beta = current_beta;

    raw_bemf_alpha = voltage_alpha - config->resistance_ohm * current_alpha - config->inductance_h * derivative_alpha;
    raw_bemf_beta = voltage_beta - config->resistance_ohm * current_beta - config->inductance_h * derivative_beta;

    g_sensorless.filtered_bemf_alpha += config->bemf_filter_alpha * (raw_bemf_alpha - g_sensorless.filtered_bemf_alpha);
    g_sensorless.filtered_bemf_beta += config->bemf_filter_alpha * (raw_bemf_beta - g_sensorless.filtered_bemf_beta);

    output->bemf_alpha_volts = g_sensorless.filtered_bemf_alpha;
    output->bemf_beta_volts = g_sensorless.filtered_bemf_beta;

    g_sensorless.pll_counter++;
    if (g_sensorless.pll_counter < config->pll_divider) {
        return;
    }
    g_sensorless.pll_counter = 0U;

    bemf_magnitude = sqrtf(g_sensorless.filtered_bemf_alpha * g_sensorless.filtered_bemf_alpha +
                           g_sensorless.filtered_bemf_beta * g_sensorless.filtered_bemf_beta);
    output->bemf_magnitude_volts = bemf_magnitude;
    pll_dt = config->sample_time_sec * (float)config->pll_divider;

    if (bemf_magnitude < config->minimum_bemf_volts) {
        g_sensorless.lock_counter = 0U;
        if (g_sensorless.loss_counter < UINT16_MAX) {
            g_sensorless.loss_counter++;
        }
        if (g_sensorless.loss_counter >= config->loss_updates) {
            output->valid = 0U;
            output->status = g_sensorless.phase_initialized ? MOTOR_SENSORLESS_LOST : MOTOR_SENSORLESS_SEARCHING;
        }
        output->electrical_angle_rad =
            motor_sensorless_wrap_angle(output->electrical_angle_rad + output->electrical_speed_rad_s * pll_dt);
        return;
    }

    g_sensorless.loss_counter = 0U;
    measured_alpha = g_sensorless.filtered_bemf_alpha / bemf_magnitude;
    measured_beta = g_sensorless.filtered_bemf_beta / bemf_magnitude;

    if (g_sensorless.phase_initialized == 0U) {
        output->electrical_angle_rad = motor_sensorless_wrap_angle(
            atan2f(-(float)g_sensorless.direction * measured_alpha, (float)g_sensorless.direction * measured_beta));
        g_sensorless.phase_initialized = 1U;
    }

    predicted_alpha = -(float)g_sensorless.direction * sinf(output->electrical_angle_rad);
    predicted_beta = (float)g_sensorless.direction * cosf(output->electrical_angle_rad);
    phase_error = predicted_alpha * measured_beta - predicted_beta * measured_alpha;
    output->pll_phase_error = phase_error;

    speed_limit = config->maximum_electrical_speed_rad_s;
    g_sensorless.pll_integral_speed += config->pll_ki * phase_error * pll_dt;
    if (g_sensorless.direction > 0) {
        g_sensorless.pll_integral_speed = motor_sensorless_clamp(g_sensorless.pll_integral_speed, 0.0f, speed_limit);
    } else {
        g_sensorless.pll_integral_speed = motor_sensorless_clamp(g_sensorless.pll_integral_speed, -speed_limit, 0.0f);
    }

    output->electrical_speed_rad_s = g_sensorless.pll_integral_speed + config->pll_kp * phase_error;
    if (g_sensorless.direction > 0) {
        output->electrical_speed_rad_s = motor_sensorless_clamp(output->electrical_speed_rad_s, 0.0f, speed_limit);
    } else {
        output->electrical_speed_rad_s = motor_sensorless_clamp(output->electrical_speed_rad_s, -speed_limit, 0.0f);
    }

    output->electrical_angle_rad =
        motor_sensorless_wrap_angle(output->electrical_angle_rad + output->electrical_speed_rad_s * pll_dt);
    output->mechanical_speed_rpm =
        output->electrical_speed_rad_s * MOTOR_SENSORLESS_RPM_SCALE / (float)config->pole_pairs;

    if (motor_sensorless_abs(phase_error) <= config->lock_phase_error) {
        if (g_sensorless.lock_counter < UINT16_MAX) {
            g_sensorless.lock_counter++;
        }
    } else {
        g_sensorless.lock_counter = 0U;
        output->valid = 0U;
        output->status = MOTOR_SENSORLESS_SEARCHING;
    }

    if (g_sensorless.lock_counter >= config->lock_updates) {
        output->valid = 1U;
        output->status = MOTOR_SENSORLESS_TRACKING;
    }
}

// 按既有接口返回最新估计结果；保持结构体返回 ABI 与有效性字段。
motor_sensorless_output_t motor_sensorless_get_output(void)
{
    motor_sensorless_output_t output;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    output = g_sensorless.output;
    if (primask == 0U) {
        __enable_irq();
    }
    return output;
}

// 读取最近估计电角度 rad；有效性由独立接口判断。
float motor_sensorless_get_electrical_angle(void)
{
    return motor_sensorless_get_output().electrical_angle_rad;
}

// 读取 PLL 推算的机械转速 rpm，不作为编码器实测值。
float motor_sensorless_get_mechanical_speed_rpm(void)
{
    return motor_sensorless_get_output().mechanical_speed_rpm;
}

// 查询观测器当前输出有效性。
uint8_t motor_sensorless_is_valid(void)
{
    return motor_sensorless_get_output().valid;
}

// 按锁定历史与失锁规则判断是否满足既有切换条件。
uint8_t motor_sensorless_is_ready_for_handover(void)
{
    motor_sensorless_output_t output = motor_sensorless_get_output();
    return ((output.valid != 0U) && (output.status == MOTOR_SENSORLESS_TRACKING)) ? 1U : 0U;
}
