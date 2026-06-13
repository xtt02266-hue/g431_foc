#include "motor_trajectory.h"
#include "motor_system.h"
#include <stdint.h>

/* 最大期望速度，单位 RPM。只用于限制惯性前馈规划速度，不直接作为速度环目标。 */
#define MOTOR_TRAJ_MAX_VELOCITY_RPM        210.0f

/* 最大期望加速度，单位 RPM/s。惯性前馈电流由这个加速度计算。 */
#define MOTOR_TRAJ_MAX_ACCEL_RPM_S         10000.0f

/* 最大 jerk，单位 RPM/s^2，用于避免惯性前馈电流突变。 */
#define MOTOR_TRAJ_MAX_JERK_RPM_S2         3000000.0f

/* 到达目标附近的死区，单位 counts。 */
#define MOTOR_TRAJ_ARRIVE_COUNTS           2.0f

/* 虚拟轨迹点相对实际位置的前视时间，单位 s。 */
#define MOTOR_TRAJ_LOOKAHEAD_SEC           0.04f

/* 速度命令跟随时间，单位 s。越小惯性前馈越快响应目标速度变化。 */
#define MOTOR_TRAJ_SPEED_MATCH_TIME_SEC    0.02f

/* 编码器一圈计数值。 */
#define MOTOR_TRAJ_COUNTS_PER_REV          4096.0f

/* 半圈计数值，用于环形最短路径判断。 */
#define MOTOR_TRAJ_HALF_REV_COUNTS         2048.0f

/* RPM 到 counts/s 的换算系数。 */
#define MOTOR_TRAJ_RPM_TO_COUNTS_PER_SEC   (MOTOR_TRAJ_COUNTS_PER_REV / 60.0f)

/*
 * 轨迹规划器状态。
 *
 * 这个模块只服务于惯性前馈，不参与位置环参考。
 *
 * g_traj_target_position:
 *   当前目标位置，单位 counts。
 * g_traj_position:
 *   虚拟轨迹点位置，单位 counts。
 *   它会跟着实际电机位置移动，并始终夹在实际位置和目标位置之间。
 * g_traj_velocity:
 *   按当前实际速度和规划加速度外推得到的虚拟速度，单位 RPM。
 * g_traj_accel:
 *   输出给惯性前馈使用的规划加速度，单位 RPM/s。
 */
static float g_traj_target_position = 0.0f;
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

    if (error > MOTOR_TRAJ_HALF_REV_COUNTS) {
        error -= MOTOR_TRAJ_COUNTS_PER_REV;
    } else if (error < -MOTOR_TRAJ_HALF_REV_COUNTS) {
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

void Motor_Trajectory_Reset(float actual_position,
                            float actual_speed_rpm)
{
    /*
     * 输入：
     *   actual_position  当前实测位置，单位 counts
     *   actual_speed_rpm 当前实测速度，单位 RPM
     * 输出：
     *   让虚拟轨迹点从当前实际位置开始，速度从当前实际速度开始。
     */
    g_traj_target_position = Motor_Trajectory_WrapPosition(actual_position);
    g_traj_position = g_traj_target_position;
    g_traj_velocity =
        Motor_Trajectory_ClampFloat(actual_speed_rpm,
                                    -MOTOR_TRAJ_MAX_VELOCITY_RPM,
                                    MOTOR_TRAJ_MAX_VELOCITY_RPM);
    g_traj_accel = 0.0f;
    g_traj_initialized = 1U;
}

void Motor_Trajectory_Clear(void)
{
    /*
     * 输入：无
     * 输出：清零惯性前馈轨迹规划状态。
     */
    g_traj_target_position = 0.0f;
    g_traj_position = 0.0f;
    g_traj_velocity = 0.0f;
    g_traj_accel = 0.0f;
    g_traj_initialized = 0U;
}

void Motor_Trajectory_Step(float target_position,
                           float actual_position,
                           float actual_speed_rpm,
                           float target_speed_rpm)
{
    /*
     * 输入：
     *   target_position  目标位置，单位 counts
     *   actual_position  当前实测位置，单位 counts
     *   actual_speed_rpm 当前实测速度，单位 RPM
     *   target_speed_rpm 位置环输出的目标机械速度，单位 RPM
     *
     * 输出：
     *   g_traj_position  虚拟轨迹点，单位 counts；始终位于实际位置和目标位置之间
     *   g_traj_velocity  虚拟速度，单位 RPM
     *   g_traj_accel     惯性前馈用规划加速度，单位 RPM/s
     *
     * 算法：
     *   1. 每次都基于“当前实际位置”和“目标位置”重新计算剩余距离。
     *   2. 位置环已经决定当前应该给多少目标速度，惯性前馈只估算
     *      “实际速度追到目标速度”需要的加速度。
     *   3. 如果实际速度已经超出目标速度，尤其目标速度变成反向时，
     *      target_speed_rpm - actual_speed_rpm 会直接产生反向减速补偿。
     *   4. 加速度经过限幅和 jerk 限制后输出给惯性前馈。
     *   5. 虚拟轨迹点由实际位置向目标方向前视一小段距离得到，
     *      因此它会跟着电机走，而不是独立跑到目标附近。
     */
    const float dt = MOTOR_SYSTEM_TASK_DT_SEC;
    const float max_jerk_step =
        MOTOR_TRAJ_MAX_JERK_RPM_S2 * dt;

    if (dt <= 0.0f) {
        g_traj_accel = 0.0f;
        return;
    }

    if (!g_traj_initialized) {
        Motor_Trajectory_Reset(actual_position, actual_speed_rpm);
    }

    g_traj_target_position =
        Motor_Trajectory_WrapPosition(target_position);

    float actual_wrapped =
        Motor_Trajectory_WrapPosition(actual_position);
    float position_error =
        Motor_Trajectory_ShortestPositionError(g_traj_target_position,
                                               actual_wrapped);
    float abs_error =
        Motor_Trajectory_AbsFloat(position_error);
    float direction =
        Motor_Trajectory_SignFloat(position_error);
    float speed_abs_rpm =
        Motor_Trajectory_AbsFloat(actual_speed_rpm);
    float speed_counts_s =
        speed_abs_rpm * MOTOR_TRAJ_RPM_TO_COUNTS_PER_SEC;
    float target_speed_limited =
        Motor_Trajectory_ClampFloat(target_speed_rpm,
                                    -MOTOR_TRAJ_MAX_VELOCITY_RPM,
                                    MOTOR_TRAJ_MAX_VELOCITY_RPM);

    float accel_target_rpm_s =
        (target_speed_limited - actual_speed_rpm) /
        MOTOR_TRAJ_SPEED_MATCH_TIME_SEC;
    accel_target_rpm_s =
        Motor_Trajectory_ClampFloat(accel_target_rpm_s,
                                    -MOTOR_TRAJ_MAX_ACCEL_RPM_S,
                                    MOTOR_TRAJ_MAX_ACCEL_RPM_S);

    if (abs_error <= MOTOR_TRAJ_ARRIVE_COUNTS &&
        speed_abs_rpm <= 0.5f &&
        Motor_Trajectory_AbsFloat(target_speed_limited) <= 0.5f) {
        accel_target_rpm_s = 0.0f;
    }

    g_traj_accel =
        Motor_Trajectory_MoveTowardFloat(g_traj_accel,
                                         accel_target_rpm_s,
                                         max_jerk_step);

    g_traj_velocity =
        Motor_Trajectory_ClampFloat(actual_speed_rpm +
                                    g_traj_accel * MOTOR_TRAJ_LOOKAHEAD_SEC,
                                    -MOTOR_TRAJ_MAX_VELOCITY_RPM,
                                    MOTOR_TRAJ_MAX_VELOCITY_RPM);

    /*
     * 虚拟轨迹点：
     *   从实际位置出发，沿目标方向前视一小段距离。
     *   前视距离由当前速度和规划加速度估算，并限制在剩余距离以内。
     */
    float lookahead_counts =
        speed_counts_s * MOTOR_TRAJ_LOOKAHEAD_SEC +
        0.5f *
        Motor_Trajectory_AbsFloat(g_traj_accel) *
        MOTOR_TRAJ_RPM_TO_COUNTS_PER_SEC *
        MOTOR_TRAJ_LOOKAHEAD_SEC *
        MOTOR_TRAJ_LOOKAHEAD_SEC;

    lookahead_counts =
        Motor_Trajectory_ClampFloat(lookahead_counts,
                                    0.0f,
                                    abs_error);

    g_traj_position =
        Motor_Trajectory_WrapPosition(actual_wrapped +
                                      direction * lookahead_counts);
}

float Motor_Trajectory_GetPosition(void)
{
    /* 输出：虚拟轨迹点位置，单位 counts。 */
    return g_traj_position;
}

float Motor_Trajectory_GetVelocity(void)
{
    /* 输出：虚拟轨迹点速度，单位 RPM。 */
    return g_traj_velocity;
}

float Motor_Trajectory_GetAccel(void)
{
    /* 输出：惯性前馈用规划加速度，单位 RPM/s。 */
    return g_traj_accel;
}

float Motor_Trajectory_GetFilteredTarget(void)
{
    /* 输出：当前目标位置，单位 counts。 */
    return g_traj_target_position;
}
