#include "motor_trajectory.h"
#include "motor_config.h"
#include <stdint.h>

/* 最大期望速度，单位 RPM。只用于限制惯性前馈规划速度，不直接作为速度环目标。 */
#define MOTOR_TRAJ_MAX_VELOCITY_RPM 220.0f

/* 最大期望加速度，单位 RPM/s。惯性前馈电流由这个加速度计算。 */
#define MOTOR_TRAJ_MAX_ACCEL_RPM_S 50000.0f

/* 最大 jerk，单位 RPM/s^2，用于避免惯性前馈电流突变。 */
#define MOTOR_TRAJ_MAX_JERK_RPM_S2 9000000.0f

/* 到达目标附近的死区，单位 counts。 */
#define MOTOR_TRAJ_ARRIVE_COUNTS 2.0f

/* 虚拟轨迹点相对实际位置的前视时间，单位 s。 */
#define MOTOR_TRAJ_LOOKAHEAD_SEC 0.04f

/* 速度命令跟随时间，单位 s。越小惯性前馈越快响应目标速度变化。 */
#define MOTOR_TRAJ_SPEED_MATCH_TIME_SEC 0.02f

/* 编码器一圈计数值。 */
#define MOTOR_TRAJ_COUNTS_PER_REV 32768.0f

/* 半圈计数值，用于环形最短路径判断。 */
#define MOTOR_TRAJ_HALF_REV_COUNTS 16384.0f

/* RPM 到 counts/s 的换算系数。 */
#define MOTOR_TRAJ_RPM_TO_COUNTS_PER_SEC (MOTOR_TRAJ_COUNTS_PER_REV / 60.0f)

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

// 计算轨迹标量绝对值，保留原有浮点比较行为。
static float motor_trajectory_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

// 按上下界限制轨迹标量，输入与上下界使用相同单位。
static float motor_trajectory_clamp_float(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    } else if (value < min_value) {
        return min_value;
    } else {
        return value;
    }
}

// 把位置计数环绕到单圈范围，不改变连续轨迹状态。
static float motor_trajectory_wrap_position(float position)
{
    while (position >= MOTOR_TRAJ_COUNTS_PER_REV) {
        position -= MOTOR_TRAJ_COUNTS_PER_REV;
    }

    while (position < 0.0f) {
        position += MOTOR_TRAJ_COUNTS_PER_REV;
    }

    return position;
}

// 按半圈边界计算目标到实际位置的最短计数误差。
static float motor_trajectory_shortest_position_error(float target, float actual)
{
    float error = target - actual;

    if (error > MOTOR_TRAJ_HALF_REV_COUNTS) {
        error -= MOTOR_TRAJ_COUNTS_PER_REV;
    } else if (error < -MOTOR_TRAJ_HALF_REV_COUNTS) {
        error += MOTOR_TRAJ_COUNTS_PER_REV;
    }

    return error;
}

// 返回轨迹方向符号，保留零值的原有约定。
static float motor_trajectory_sign_float(float value)
{
    if (value > 0.0f) {
        return 1.0f;
    } else if (value < 0.0f) {
        return -1.0f;
    } else {
        return 0.0f;
    }
}

// 以最大允许步长接近目标，避免速度/加速度更新越过目标。
static float motor_trajectory_move_toward_float(float current, float target, float max_step)
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

// 以当前位置计数初始化前馈轨迹和历史速度/加速度。
void motor_trajectory_reset(float actual_position, float actual_speed_rpm)
{
    /*
     * 输入：
     *   actual_position  当前实测位置，单位 counts
     *   actual_speed_rpm 当前实测速度，单位 RPM
     * 输出：
     *   让虚拟轨迹点从当前实际位置开始，速度从当前实际速度开始。
     */
    g_traj_target_position = motor_trajectory_wrap_position(actual_position);
    g_traj_position = g_traj_target_position;
    g_traj_velocity =
        motor_trajectory_clamp_float(actual_speed_rpm, -MOTOR_TRAJ_MAX_VELOCITY_RPM, MOTOR_TRAJ_MAX_VELOCITY_RPM);
    g_traj_accel = 0.0f;
    g_traj_initialized = 1U;
}

// 清除轨迹有效性与历史，供停机或控制模式切换使用。
void motor_trajectory_clear(void)
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

// 按 dt（s）、位置计数和 rpm 更新惯性前馈轨迹；不替代位置环参考。
void motor_trajectory_step(float target_position, float actual_position, float actual_speed_rpm, float target_speed_rpm)
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
    const float max_jerk_step = MOTOR_TRAJ_MAX_JERK_RPM_S2 * dt;

    if (dt <= 0.0f) {
        g_traj_accel = 0.0f;
        return;
    }

    if (!g_traj_initialized) {
        motor_trajectory_reset(actual_position, actual_speed_rpm);
    }

    g_traj_target_position = motor_trajectory_wrap_position(target_position);

    float actual_wrapped = motor_trajectory_wrap_position(actual_position);
    float position_error = motor_trajectory_shortest_position_error(g_traj_target_position, actual_wrapped);
    float abs_error = motor_trajectory_abs_float(position_error);
    float direction = motor_trajectory_sign_float(position_error);
    float speed_abs_rpm = motor_trajectory_abs_float(actual_speed_rpm);
    float speed_counts_s = speed_abs_rpm * MOTOR_TRAJ_RPM_TO_COUNTS_PER_SEC;
    float target_speed_limited =
        motor_trajectory_clamp_float(target_speed_rpm, -MOTOR_TRAJ_MAX_VELOCITY_RPM, MOTOR_TRAJ_MAX_VELOCITY_RPM);

    float accel_target_rpm_s = (target_speed_limited - actual_speed_rpm) / MOTOR_TRAJ_SPEED_MATCH_TIME_SEC;
    accel_target_rpm_s =
        motor_trajectory_clamp_float(accel_target_rpm_s, -MOTOR_TRAJ_MAX_ACCEL_RPM_S, MOTOR_TRAJ_MAX_ACCEL_RPM_S);

    if (abs_error <= MOTOR_TRAJ_ARRIVE_COUNTS && speed_abs_rpm <= 0.5f &&
        motor_trajectory_abs_float(target_speed_limited) <= 0.5f) {
        accel_target_rpm_s = 0.0f;
    }

    g_traj_accel = motor_trajectory_move_toward_float(g_traj_accel, accel_target_rpm_s, max_jerk_step);

    g_traj_velocity = motor_trajectory_clamp_float(actual_speed_rpm + g_traj_accel * MOTOR_TRAJ_LOOKAHEAD_SEC,
                                                   -MOTOR_TRAJ_MAX_VELOCITY_RPM, MOTOR_TRAJ_MAX_VELOCITY_RPM);

    /*
     * 虚拟轨迹点：
     *   从实际位置出发，沿目标方向前视一小段距离。
     *   前视距离由当前速度和规划加速度估算，并限制在剩余距离以内。
     */
    float lookahead_counts = speed_counts_s * MOTOR_TRAJ_LOOKAHEAD_SEC +
                             0.5f * motor_trajectory_abs_float(g_traj_accel) * MOTOR_TRAJ_RPM_TO_COUNTS_PER_SEC *
                                 MOTOR_TRAJ_LOOKAHEAD_SEC * MOTOR_TRAJ_LOOKAHEAD_SEC;

    lookahead_counts = motor_trajectory_clamp_float(lookahead_counts, 0.0f, abs_error);

    g_traj_position = motor_trajectory_wrap_position(actual_wrapped + direction * lookahead_counts);
}

// 读取虚拟轨迹位置，单位编码器计数。
float motor_trajectory_get_position(void)
{
    /* 输出：虚拟轨迹点位置，单位 counts。 */
    return g_traj_position;
}

// 读取规划速度，单位 rpm。
float motor_trajectory_get_velocity(void)
{
    /* 输出：虚拟轨迹点速度，单位 RPM。 */
    return g_traj_velocity;
}

// 读取规划加速度，单位 rpm/s，供惯性前馈使用。
float motor_trajectory_get_accel(void)
{
    /* 输出：惯性前馈用规划加速度，单位 RPM/s。 */
    return g_traj_accel;
}

// 读取模块保存的目标位置计数，保留原有接口含义。
float motor_trajectory_get_filtered_target(void)
{
    /* 输出：当前目标位置，单位 counts。 */
    return g_traj_target_position;
}
