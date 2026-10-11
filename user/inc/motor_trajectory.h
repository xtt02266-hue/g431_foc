#ifndef MOTOR_TRAJECTORY_H
#define MOTOR_TRAJECTORY_H

/*
 * 轨迹规划器接口说明
 *
 * 输入坐标系：
 *   位置：MT6826S 15位机械角度计数值，范围 [0, 32768)
 *   速度：机械转速，单位 RPM
 *
 * 输出坐标系：
 *   规划位置：MT6826S counts
 *   规划速度：RPM
 *   规划加速度：RPM/s
 */

/*
 * 输入：
 *   actual_position  当前实测机械位置，单位 counts
 *   actual_speed_rpm 当前实测机械速度，单位 RPM
 * 输出：
 *   将规划位置初始化到 actual_position，
 *   将规划速度初始化到 actual_speed_rpm，
 *   将规划加速度清零。
 */
void motor_trajectory_reset(float actual_position, float actual_speed_rpm);

/*
 * 输入：
 *   无
 * 输出：
 *   清零规划器内部状态，并标记为未初始化。
 */
void motor_trajectory_clear(void);

/*
 * 输入：
 *   target_position  外部目标机械位置，单位 counts
 *   actual_position  当前实测机械位置，单位 counts；每次用于计算前视轨迹点
 *   actual_speed_rpm 当前实测机械速度，单位 RPM
 *   target_speed_rpm 位置环输出的目标机械速度，单位 RPM
 * 输出：
 *   更新内部规划位置、规划速度、规划加速度。
 *   其中规划位置只用于调试观察，规划加速度用于惯性前馈补偿。
 */
void motor_trajectory_step(float target_position, float actual_position, float actual_speed_rpm,
                           float target_speed_rpm);

/*
 * 输出：
 *   当前规划机械位置，单位 counts
 */
float motor_trajectory_get_position(void);

/*
 * 输出：
 *   当前规划机械速度，单位 RPM
 */
float motor_trajectory_get_velocity(void);

/*
 * 输出：
 *   当前规划机械加速度，单位 RPM/s
 */
float motor_trajectory_get_accel(void);

/*
 * 输出：
 *   当前规划器使用的目标位置，单位 counts。
 *   当前版本不再对目标位置做低通滤波，因此该值等于最近一次输入目标。
 */
float motor_trajectory_get_filtered_target(void);

#endif
