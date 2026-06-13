#ifndef MOTOR_TRAJECTORY_H
#define MOTOR_TRAJECTORY_H

/*
 * 轨迹规划器接口说明
 *
 * 输入坐标系：
 *   位置：AS5600 机械角度计数值，范围 [0, 4096)
 *   速度：机械转速，单位 RPM
 *
 * 输出坐标系：
 *   规划位置：AS5600 counts
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
void Motor_Trajectory_Reset(float actual_position,
                            float actual_speed_rpm);

/*
 * 输入：
 *   无
 * 输出：
 *   清零规划器内部状态，并标记为未初始化。
 */
void Motor_Trajectory_Clear(void);

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
void Motor_Trajectory_Step(float target_position,
                           float actual_position,
                           float actual_speed_rpm,
                           float target_speed_rpm);

/*
 * 输出：
 *   当前规划机械位置，单位 counts
 */
float Motor_Trajectory_GetPosition(void);

/*
 * 输出：
 *   当前规划机械速度，单位 RPM
 */
float Motor_Trajectory_GetVelocity(void);

/*
 * 输出：
 *   当前规划机械加速度，单位 RPM/s
 */
float Motor_Trajectory_GetAccel(void);

/*
 * 输出：
 *   当前规划器使用的目标位置，单位 counts。
 *   当前版本不再对目标位置做低通滤波，因此该值等于最近一次输入目标。
 */
float Motor_Trajectory_GetFilteredTarget(void);

#endif
