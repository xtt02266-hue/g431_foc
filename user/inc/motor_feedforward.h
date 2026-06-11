#ifndef MOTOR_FEEDFORWARD_H
#define MOTOR_FEEDFORWARD_H

float Motor_Feedforward_FrictionIq(float speed_rpm);
float Motor_Feedforward_InertiaIq(float accel_rpm_s);
float Motor_Feedforward_ApplyIq(float speed_loop_iq,
                                float friction_iq,
                                float inertia_iq,
                                float min_iq,
                                float max_iq);

#endif
