#include "motor_sensorless.h"
#include "main.h"
#include <math.h>
#include <stddef.h>

#define MOTOR_SENSORLESS_TWO_PI            6.283185307f
#define MOTOR_SENSORLESS_RPM_SCALE          9.549296586f

typedef struct
{
    MotorSensorlessConfig config;
    MotorSensorlessOutput output;

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
} MotorSensorlessInternal;

static MotorSensorlessInternal g_sensorless = {0};

static float Motor_Sensorless_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Motor_Sensorless_Clamp(float value,
                                    float minimum,
                                    float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static float Motor_Sensorless_WrapAngle(float angle)
{
    while (angle >= MOTOR_SENSORLESS_TWO_PI) {
        angle -= MOTOR_SENSORLESS_TWO_PI;
    }
    while (angle < 0.0f) {
        angle += MOTOR_SENSORLESS_TWO_PI;
    }
    return angle;
}

static void Motor_Sensorless_ResetState(void)
{
    g_sensorless.output = (MotorSensorlessOutput){0};
    g_sensorless.output.status = g_sensorless.enabled
                                         ? MOTOR_SENSORLESS_SEARCHING
                                         : MOTOR_SENSORLESS_DISABLED;
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

void Motor_Sensorless_GetDefaultConfig(MotorSensorlessConfig *config)
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

void Motor_Sensorless_Init(void)
{
    Motor_Sensorless_GetDefaultConfig(&g_sensorless.config);
    g_sensorless.direction = 1;
    g_sensorless.enabled = MOTOR_SENSORLESS_DEFAULT_ENABLE;
    Motor_Sensorless_ResetState();
}

uint8_t Motor_Sensorless_Configure(const MotorSensorlessConfig *config)
{
    uint32_t primask;

    if ((config == NULL) ||
        !(config->sample_time_sec > 0.0f) ||
        !(config->resistance_ohm > 0.0f) ||
        !(config->inductance_h > 0.0f) ||
        (config->pole_pairs == 0U) ||
        !(config->bemf_filter_alpha > 0.0f) ||
        (config->bemf_filter_alpha > 1.0f) ||
        !(config->minimum_bemf_volts > 0.0f) ||
        !(config->pll_kp > 0.0f) ||
        !(config->pll_ki > 0.0f) ||
        !(config->maximum_electrical_speed_rad_s > 0.0f) ||
        !(config->lock_phase_error > 0.0f) ||
        (config->lock_updates == 0U) ||
        (config->loss_updates == 0U) ||
        (config->pll_divider == 0U)) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    g_sensorless.config = *config;
    Motor_Sensorless_ResetState();
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

uint8_t Motor_Sensorless_ConfigureMotor(float resistance_ohm,
                                        float inductance_h,
                                        uint16_t pole_pairs)
{
    MotorSensorlessConfig config = g_sensorless.config;

    config.resistance_ohm = resistance_ohm;
    config.inductance_h = inductance_h;
    config.pole_pairs = pole_pairs;
    return Motor_Sensorless_Configure(&config);
}

void Motor_Sensorless_Enable(uint8_t enable)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_sensorless.enabled = (enable != 0U) ? 1U : 0U;
    Motor_Sensorless_ResetState();
    if (primask == 0U) {
        __enable_irq();
    }
}

uint8_t Motor_Sensorless_IsEnabled(void)
{
    return g_sensorless.enabled;
}

void Motor_Sensorless_Reset(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    Motor_Sensorless_ResetState();
    if (primask == 0U) {
        __enable_irq();
    }
}

void Motor_Sensorless_SetDirection(int8_t direction)
{
    if (direction > 0) {
        g_sensorless.direction = 1;
    } else if (direction < 0) {
        g_sensorless.direction = -1;
    }
}

void Motor_Sensorless_Update(float voltage_alpha,
                             float voltage_beta,
                             float current_alpha,
                             float current_beta)
{
    MotorSensorlessConfig *config = &g_sensorless.config;
    MotorSensorlessOutput *output = &g_sensorless.output;
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

    derivative_alpha = (current_alpha - g_sensorless.last_current_alpha) /
                       config->sample_time_sec;
    derivative_beta = (current_beta - g_sensorless.last_current_beta) /
                      config->sample_time_sec;
    g_sensorless.last_current_alpha = current_alpha;
    g_sensorless.last_current_beta = current_beta;

    raw_bemf_alpha = voltage_alpha -
                     config->resistance_ohm * current_alpha -
                     config->inductance_h * derivative_alpha;
    raw_bemf_beta = voltage_beta -
                    config->resistance_ohm * current_beta -
                    config->inductance_h * derivative_beta;

    g_sensorless.filtered_bemf_alpha +=
        config->bemf_filter_alpha *
        (raw_bemf_alpha - g_sensorless.filtered_bemf_alpha);
    g_sensorless.filtered_bemf_beta +=
        config->bemf_filter_alpha *
        (raw_bemf_beta - g_sensorless.filtered_bemf_beta);

    output->bemf_alpha_volts = g_sensorless.filtered_bemf_alpha;
    output->bemf_beta_volts = g_sensorless.filtered_bemf_beta;

    g_sensorless.pll_counter++;
    if (g_sensorless.pll_counter < config->pll_divider) {
        return;
    }
    g_sensorless.pll_counter = 0U;

    bemf_magnitude = sqrtf(g_sensorless.filtered_bemf_alpha *
                           g_sensorless.filtered_bemf_alpha +
                           g_sensorless.filtered_bemf_beta *
                           g_sensorless.filtered_bemf_beta);
    output->bemf_magnitude_volts = bemf_magnitude;
    pll_dt = config->sample_time_sec * (float)config->pll_divider;

    if (bemf_magnitude < config->minimum_bemf_volts) {
        g_sensorless.lock_counter = 0U;
        if (g_sensorless.loss_counter < UINT16_MAX) {
            g_sensorless.loss_counter++;
        }
        if (g_sensorless.loss_counter >= config->loss_updates) {
            output->valid = 0U;
            output->status = g_sensorless.phase_initialized
                                 ? MOTOR_SENSORLESS_LOST
                                 : MOTOR_SENSORLESS_SEARCHING;
        }
        output->electrical_angle_rad = Motor_Sensorless_WrapAngle(
            output->electrical_angle_rad +
            output->electrical_speed_rad_s * pll_dt);
        return;
    }

    g_sensorless.loss_counter = 0U;
    measured_alpha = g_sensorless.filtered_bemf_alpha / bemf_magnitude;
    measured_beta = g_sensorless.filtered_bemf_beta / bemf_magnitude;

    if (g_sensorless.phase_initialized == 0U) {
        output->electrical_angle_rad = Motor_Sensorless_WrapAngle(
            atan2f(-(float)g_sensorless.direction * measured_alpha,
                   (float)g_sensorless.direction * measured_beta));
        g_sensorless.phase_initialized = 1U;
    }

    predicted_alpha = -(float)g_sensorless.direction *
                      sinf(output->electrical_angle_rad);
    predicted_beta = (float)g_sensorless.direction *
                     cosf(output->electrical_angle_rad);
    phase_error = predicted_alpha * measured_beta -
                  predicted_beta * measured_alpha;
    output->pll_phase_error = phase_error;

    speed_limit = config->maximum_electrical_speed_rad_s;
    g_sensorless.pll_integral_speed += config->pll_ki * phase_error * pll_dt;
    if (g_sensorless.direction > 0) {
        g_sensorless.pll_integral_speed = Motor_Sensorless_Clamp(
            g_sensorless.pll_integral_speed, 0.0f, speed_limit);
    } else {
        g_sensorless.pll_integral_speed = Motor_Sensorless_Clamp(
            g_sensorless.pll_integral_speed, -speed_limit, 0.0f);
    }

    output->electrical_speed_rad_s =
        g_sensorless.pll_integral_speed + config->pll_kp * phase_error;
    if (g_sensorless.direction > 0) {
        output->electrical_speed_rad_s = Motor_Sensorless_Clamp(
            output->electrical_speed_rad_s, 0.0f, speed_limit);
    } else {
        output->electrical_speed_rad_s = Motor_Sensorless_Clamp(
            output->electrical_speed_rad_s, -speed_limit, 0.0f);
    }

    output->electrical_angle_rad = Motor_Sensorless_WrapAngle(
        output->electrical_angle_rad +
        output->electrical_speed_rad_s * pll_dt);
    output->mechanical_speed_rpm =
        output->electrical_speed_rad_s * MOTOR_SENSORLESS_RPM_SCALE /
        (float)config->pole_pairs;

    if (Motor_Sensorless_Abs(phase_error) <= config->lock_phase_error) {
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

MotorSensorlessOutput Motor_Sensorless_GetOutput(void)
{
    MotorSensorlessOutput output;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    output = g_sensorless.output;
    if (primask == 0U) {
        __enable_irq();
    }
    return output;
}

float Motor_Sensorless_GetElectricalAngle(void)
{
    return Motor_Sensorless_GetOutput().electrical_angle_rad;
}

float Motor_Sensorless_GetMechanicalSpeedRpm(void)
{
    return Motor_Sensorless_GetOutput().mechanical_speed_rpm;
}

uint8_t Motor_Sensorless_IsValid(void)
{
    return Motor_Sensorless_GetOutput().valid;
}

uint8_t Motor_Sensorless_IsReadyForHandover(void)
{
    MotorSensorlessOutput output = Motor_Sensorless_GetOutput();
    return ((output.valid != 0U) &&
            (output.status == MOTOR_SENSORLESS_TRACKING)) ? 1U : 0U;
}
