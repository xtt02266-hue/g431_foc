#include "motor_sensorless.h"
#include <math.h>
#include <stdio.h>

static unsigned saw_tracking;
static unsigned saw_lost;
static unsigned saw_wrap;
static unsigned saw_recovery;
static unsigned saw_reverse_tracking;

static void emit(unsigned phase, unsigned step, float *last_angle)
{
    MotorSensorlessOutput out = Motor_Sensorless_GetOutput();
    if (out.status == MOTOR_SENSORLESS_TRACKING && out.valid != 0U) {
        saw_tracking = 1U;
        if (phase == 3U) saw_recovery = 1U;
        if (phase == 5U) saw_reverse_tracking = 1U;
    }
    if (out.status == MOTOR_SENSORLESS_LOST) saw_lost = 1U;
    if (fabsf(out.electrical_angle_rad - *last_angle) > 3.0f) saw_wrap = 1U;
    *last_angle = out.electrical_angle_rad;
    printf("%u,%u,%d,%u,%a,%a,%a,%a,%a,%a,%a\n",
           phase, step, (int)out.status, out.valid,
           out.electrical_angle_rad, out.electrical_speed_rad_s,
           out.mechanical_speed_rpm, out.bemf_alpha_volts,
           out.bemf_beta_volts, out.bemf_magnitude_volts,
           out.pll_phase_error);
}

static void rotating_signal(unsigned phase, unsigned count, int direction,
                            float *theta, float *last_angle)
{
    unsigned step;
    float previous_alpha = 0.0f;
    float previous_beta = 0.0f;
    for (step = 0U; step < count; ++step) {
        float current_alpha = 0.06f * sinf((float)step * 0.013f);
        float current_beta = 0.04f * cosf((float)step * 0.017f);
        float bemf_alpha;
        float bemf_beta;
        float voltage_alpha;
        float voltage_beta;
        *theta += (float)direction * 0.03f;
        bemf_alpha = -(float)direction * 3.0f * sinf(*theta);
        bemf_beta = (float)direction * 3.0f * cosf(*theta);
        voltage_alpha = bemf_alpha + current_alpha +
                        0.0006f * (current_alpha - previous_alpha) / 0.00005f;
        voltage_beta = bemf_beta + current_beta +
                       0.0006f * (current_beta - previous_beta) / 0.00005f;
        Motor_Sensorless_Update(voltage_alpha, voltage_beta,
                                current_alpha, current_beta);
        emit(phase, step, last_angle);
        previous_alpha = current_alpha;
        previous_beta = current_beta;
    }
}

int main(void)
{
    MotorSensorlessConfig config;
    float theta = 0.0f;
    float last_angle = 0.0f;
    unsigned step;

    Motor_Sensorless_Init();
    Motor_Sensorless_GetDefaultConfig(&config);
    /* Freeze the old model values so this replay checks refactoring only. */
    config.resistance_ohm = 1.0f;
    config.inductance_h = 0.0006f;
    config.pole_pairs = 1U;
    config.lock_updates = 8U;
    config.loss_updates = 8U;
    config.pll_divider = 2U;
    if (Motor_Sensorless_Configure(&config) == 0U) return 10;
    Motor_Sensorless_Enable(1U);
    Motor_Sensorless_Update(0.0f, 0.0f, 0.0f, 0.0f);
    emit(0U, 0U, &last_angle); /* First sample only seeds di/dt history. */

    rotating_signal(1U, 1200U, 1, &theta, &last_angle);
    for (step = 0U; step < 200U; ++step) {
        Motor_Sensorless_Update(0.0f, 0.0f, 0.0f, 0.0f);
        emit(2U, step, &last_angle);
    }
    rotating_signal(3U, 400U, 1, &theta, &last_angle);

    Motor_Sensorless_SetDirection(-1);
    Motor_Sensorless_Reset();
    theta = 0.0f;
    last_angle = 0.0f;
    Motor_Sensorless_Update(0.0f, 0.0f, 0.0f, 0.0f);
    emit(4U, 0U, &last_angle);
    rotating_signal(5U, 1200U, -1, &theta, &last_angle);

    fprintf(stderr, "coverage tracking=%u lost=%u wrap=%u recovery=%u reverse=%u\n",
            saw_tracking, saw_lost, saw_wrap, saw_recovery,
            saw_reverse_tracking);
    return (saw_tracking && saw_lost && saw_wrap && saw_recovery &&
            saw_reverse_tracking) ? 0 : 11;
}
