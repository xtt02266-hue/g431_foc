#include "motor_trajectory.h"
#include "motor_system.h"
#include <stdint.h>

#define MOTOR_TRAJ_MAX_VELOCITY_RPM   80.0f
#define MOTOR_TRAJ_MAX_ACCEL_RPM_S    3000.0f
#define MOTOR_TRAJ_ARRIVE_COUNTS      3.0f
#define MOTOR_TRAJ_COUNTS_PER_REV     4096.0f
#define MOTOR_TRAJ_RPM_TO_COUNTS_PER_SEC \
    (MOTOR_TRAJ_COUNTS_PER_REV / 60.0f)

static float g_traj_position = 0.0f;
static float g_traj_velocity = 0.0f;
static float g_traj_accel = 0.0f;
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

    if (error > 2048.0f) {
        error -= MOTOR_TRAJ_COUNTS_PER_REV;
    } else if (error < -2048.0f) {
        error += MOTOR_TRAJ_COUNTS_PER_REV;
    }

    return error;
}

static float Motor_Trajectory_SignFloat(float value)
{
    if (value > 0.0f) {
        return 1.0f;
    } else if (value < 0.0f) {
        return -1.0f;
    } else {
        return 0.0f;
    }
}

static float Motor_Trajectory_BrakeDistanceCounts(float velocity_rpm)
{
    float velocity_counts_s =
        Motor_Trajectory_AbsFloat(velocity_rpm) *
        MOTOR_TRAJ_RPM_TO_COUNTS_PER_SEC;

    float accel_counts_s2 =
        MOTOR_TRAJ_MAX_ACCEL_RPM_S *
        MOTOR_TRAJ_RPM_TO_COUNTS_PER_SEC;

    if (accel_counts_s2 <= 0.0f) {
        return MOTOR_TRAJ_ARRIVE_COUNTS;
    }

    return (velocity_counts_s * velocity_counts_s) /
           (2.0f * accel_counts_s2) +
           MOTOR_TRAJ_ARRIVE_COUNTS;
}

void Motor_Trajectory_Reset(float actual_position,
                            float actual_speed_rpm)
{
    g_traj_position =
        Motor_Trajectory_WrapPosition(actual_position);

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
    g_traj_initialized = 0U;
}

void Motor_Trajectory_Step(float target_position,
                           float actual_position,
                           float actual_speed_rpm)
{
    const float dt = MOTOR_SYSTEM_TASK_DT_SEC;

    if (dt <= 0.0f) {
        g_traj_accel = 0.0f;
        return;
    }

    if (!g_traj_initialized) {
        Motor_Trajectory_Reset(actual_position,
                               actual_speed_rpm);
    }

    float remaining_error =
        Motor_Trajectory_ShortestPositionError(target_position,
                                               g_traj_position);

    float abs_error =
        Motor_Trajectory_AbsFloat(remaining_error);

    float direction =
        Motor_Trajectory_SignFloat(remaining_error);

    if (abs_error <= MOTOR_TRAJ_ARRIVE_COUNTS &&
        Motor_Trajectory_AbsFloat(g_traj_velocity) <= 0.5f) {
        g_traj_position =
            Motor_Trajectory_WrapPosition(target_position);

        g_traj_velocity = 0.0f;
        g_traj_accel = 0.0f;
        return;
    }

    float accel_cmd = 0.0f;
    uint8_t braking_to_stop = 0U;

    if (abs_error <= MOTOR_TRAJ_ARRIVE_COUNTS ||
        direction == 0.0f) {
        accel_cmd =
            -Motor_Trajectory_SignFloat(g_traj_velocity) *
            MOTOR_TRAJ_MAX_ACCEL_RPM_S;

        braking_to_stop = 1U;
    } else {
        float velocity_along_target =
            g_traj_velocity * direction;

        float brake_distance =
            Motor_Trajectory_BrakeDistanceCounts(g_traj_velocity);

        if (velocity_along_target < -0.5f) {
            accel_cmd =
                direction *
                MOTOR_TRAJ_MAX_ACCEL_RPM_S;
        } else if (abs_error <= brake_distance) {
            accel_cmd =
                -direction *
                MOTOR_TRAJ_MAX_ACCEL_RPM_S;

            braking_to_stop = 1U;
        } else if (velocity_along_target <
                   MOTOR_TRAJ_MAX_VELOCITY_RPM) {
            accel_cmd =
                direction *
                MOTOR_TRAJ_MAX_ACCEL_RPM_S;
        } else {
            accel_cmd = 0.0f;
        }
    }

    float old_velocity =
        g_traj_velocity;

    float new_velocity =
        old_velocity + accel_cmd * dt;

    new_velocity =
        Motor_Trajectory_ClampFloat(new_velocity,
                                    -MOTOR_TRAJ_MAX_VELOCITY_RPM,
                                    MOTOR_TRAJ_MAX_VELOCITY_RPM);

    if (braking_to_stop &&
        old_velocity != 0.0f &&
        old_velocity * new_velocity <= 0.0f) {
        new_velocity = 0.0f;
    }

    g_traj_accel =
        (new_velocity - old_velocity) / dt;

    g_traj_velocity =
        new_velocity;

    float average_velocity =
        0.5f * (old_velocity + new_velocity);

    float new_position =
        Motor_Trajectory_WrapPosition(
            g_traj_position +
            average_velocity *
            MOTOR_TRAJ_RPM_TO_COUNTS_PER_SEC *
            dt);

    float new_error =
        Motor_Trajectory_ShortestPositionError(target_position,
                                               new_position);

    if (braking_to_stop &&
        remaining_error != 0.0f &&
        remaining_error * new_error <= 0.0f) {
        new_position =
            Motor_Trajectory_WrapPosition(target_position);

        g_traj_velocity = 0.0f;
        g_traj_accel = 0.0f;
    }

    g_traj_position =
        new_position;
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
