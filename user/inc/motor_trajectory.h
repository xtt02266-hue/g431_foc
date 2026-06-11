#ifndef MOTOR_TRAJECTORY_H
#define MOTOR_TRAJECTORY_H

void Motor_Trajectory_Reset(float actual_position,
                            float actual_speed_rpm);
void Motor_Trajectory_Clear(void);
void Motor_Trajectory_Step(float target_position,
                           float actual_position,
                           float actual_speed_rpm);

float Motor_Trajectory_GetPosition(void);
float Motor_Trajectory_GetVelocity(void);
float Motor_Trajectory_GetAccel(void);

#endif
