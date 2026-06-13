#include "motor_trajectory.h"
#include "motor_system.h"
#include <stdint.h>

#define MOTOR_TRAJ_MAX_VELOCITY_RPM       100.0f
#define MOTOR_TRAJ_MAX_ACCEL_RPM_S        1200.0f
#define MOTOR_TRAJ_MAX_JERK_RPM_S2        30000.0f
#define MOTOR_TRAJ_ARRIVE_COUNTS          2.0f
#define MOTOR_TRAJ_TARGET_FILTER_ALPHA    0.04f
#define MOTOR_TRAJ_TARGET_DEADBAND_COUNTS 1.0f
#define MOTOR_TRAJ_NATURAL_FREQ_HZ        7.0f
#define MOTOR_TRAJ_DAMPING_RATIO          1.1f
#define MOTOR_TRAJ_COUNTS_PER_REV         4096.0f
#define MOTOR_TRAJ_HALF_REV_COUNTS        2048.0f
#define MOTOR_TRAJ_RPM_TO_COUNTS_PER_SEC  (MOTOR_TRAJ_COUNTS_PER_REV / 60.0f)

static float g_traj_position = 0.0f;
static float g_traj_velocity = 0.0f;
static float g_traj_accel = 0.0f;
static float g_traj_target_position = 0.0f;
static uint8_t g_traj_initialized = 0U;

static float Motor_Trajectory_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Motor_Trajectory_ClampFloat(float value,
                                         float min_value,
                                         float max_value)
{
    if (value > max_value) {
        return max_value;
    } else if (value < min_value) {
        return min_value;
    } else {
        return value;
    }
}

static float Motor_Trajectory_WrapPosition(float position)
{
    while (position >= MOTOR_TRAJ_COUNTS_PER_REV) {
        position -= MOTOR_TRAJ_COUNTS_PER_REV;
    }

    while (position < 0.0f) {
        position += MOTOR_TRAJ_COUNTS_PER_REV;
    }

    return position;
}

static float Motor_Trajectory_ShortestPositionError(float target,
                                                    float actual)
{
    float error = target - actual;

    if (error > MOTOR_TRAJ_HALF_REV_COUNTS) {
        error -= MOTOR_TRAJ_COUNTS_PER_REV;
    } else if (error < -MOTOR_TRAJ_HALF_REV_COUNTS) {
        error += MOTOR_TRAJ_COUNTS_PER_REV;
    }

    return error;
}

static float Motor_Trajectory_MoveTowardFloat(float current,
                                              float target,
                                              float max_step)
{
    if (current < target) {
        current += max_step;
        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current -= max_step;
        if (current < target) {
            current = target;
        }
    }

    return current;
}

static float Motor_Trajectory_FilterTarget(float target_position)
{
    float target_error =
        Motor_Trajectory_ShortestPositionError(target_position,
                                               g_traj_target_position);

    if (Motor_Trajectory_AbsFloat(target_error) <=
        MOTOR_TRAJ_TARGET_DEADBAND_COUNTS) {
        target_error = 0.0f;
    }

    g_traj_target_position =
        Motor_Trajectory_WrapPosition(
            g_traj_target_position +
            target_error * MOTOR_TRAJ_TARGET_FILTER_ALPHA);

    return g_traj_target_position;
}

void Motor_Trajectory_Reset(float actual_position,
                            float actual_speed_rpm)
{
    g_traj_position =
        Motor_Trajectory_WrapPosition(actual_position);

    g_traj_target_position = g_traj_position;

    g_traj_velocity =
        Motor_Trajectory_ClampFloat(actual_speed_rpm,
                                    -MOTOR_TRAJ_MAX_VELOCITY_RPM,
                                    MOTOR_TRAJ_MAX_VELOCITY_RPM);

    g_traj_accel = 0.0f;
    g_traj_initialized = 1U;
}

void Motor_Trajectory_Clear(void)
{
    g_traj_position = 0.0f;
    g_traj_velocity = 0.0f;
    g_traj_accel = 0.0f;
    g_traj_target_position = 0.0f;
    g_traj_initialized = 0U;
}

void Motor_Trajectory_Step(float target_position,
                           float actual_position,
                           float actual_speed_rpm)
{
    (void)actual_speed_rpm;

    const float dt = MOTOR_SYSTEM_TASK_DT_SEC;
    const float two_pi = 6.2831853f;
    const float omega = two_pi * MOTOR_TRAJ_NATURAL_FREQ_HZ;
    const float max_accel_counts_s2 =
        MOTOR_TRAJ_MAX_ACCEL_RPM_S * MOTOR_TRAJ_RPM_TO_COUNTS_PER_SEC;
    const float max_jerk_rpm_step =
        MOTOR_TRAJ_MAX_JERK_RPM_S2 * dt;

    if (dt <= 0.0f) {
        g_traj_accel = 0.0f;
        return;
    }

    if (!g_traj_initialized) {
        Motor_Trajectory_Reset(actual_position,
                               actual_speed_rpm);
    }

    float filtered_target =
        Motor_Trajectory_FilterTarget(target_position);
    float position_error =
        Motor_Trajectory_ShortestPositionError(filtered_target,
                                               g_traj_position);
    float velocity_counts_s =
        g_traj_velocity * MOTOR_TRAJ_RPM_TO_COUNTS_PER_SEC;

    float accel_counts_s2 =
        omega * omega * position_error -
        2.0f * MOTOR_TRAJ_DAMPING_RATIO * omega * velocity_counts_s;

    accel_counts_s2 =
        Motor_Trajectory_ClampFloat(accel_counts_s2,
                                    -max_accel_counts_s2,
                                    max_accel_counts_s2);

    float accel_target_rpm_s =
        accel_counts_s2 / MOTOR_TRAJ_RPM_TO_COUNTS_PER_SEC;

    float accel_cmd_rpm_s =
        Motor_Trajectory_MoveTowardFloat(g_traj_accel,
                                         accel_target_rpm_s,
                                         max_jerk_rpm_step);

    float old_velocity = g_traj_velocity;
    float new_velocity =
        old_velocity + accel_cmd_rpm_s * dt;

    new_velocity =
        Motor_Trajectory_ClampFloat(new_velocity,
                                    -MOTOR_TRAJ_MAX_VELOCITY_RPM,
                                    MOTOR_TRAJ_MAX_VELOCITY_RPM);

    if (Motor_Trajectory_AbsFloat(position_error) <=
        MOTOR_TRAJ_ARRIVE_COUNTS &&
        Motor_Trajectory_AbsFloat(new_velocity) <= 0.5f) {
        g_traj_position = filtered_target;
        g_traj_velocity = 0.0f;
        g_traj_accel = 0.0f;
        return;
    }

    g_traj_accel =
        (new_velocity - old_velocity) / dt;
    g_traj_velocity = new_velocity;

    float average_velocity =
        0.5f * (old_velocity + new_velocity);

    g_traj_position =
        Motor_Trajectory_WrapPosition(
            g_traj_position +
            average_velocity *
            MOTOR_TRAJ_RPM_TO_COUNTS_PER_SEC *
            dt);
}

float Motor_Trajectory_GetPosition(void)
{
    return g_traj_position;
}

float Motor_Trajectory_GetVelocity(void)
{
    return g_traj_velocity;
}

float Motor_Trajectory_GetAccel(void)
{
    return g_traj_accel;
}
