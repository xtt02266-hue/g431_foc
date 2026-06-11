#include "motor_feedforward.h"

#define MOTOR_FRICTION_COMP_GAIN_A     0.04f
#define MOTOR_FRICTION_COMP_C_RPM      10.0f
#define MOTOR_FRICTION_COMP_LIMIT_A    0.035f

#define MOTOR_INERTIA_COMP_GAIN_A_PER_RPM_S 0.000025f
#define MOTOR_INERTIA_COMP_LIMIT_A     0.20f

static float Motor_Feedforward_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Motor_Feedforward_ClampFloat(float value,
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

float Motor_Feedforward_FrictionIq(float speed_rpm)
{
    float denominator =
        Motor_Feedforward_AbsFloat(speed_rpm) +
        MOTOR_FRICTION_COMP_C_RPM;

    float iq =
        MOTOR_FRICTION_COMP_GAIN_A *
        speed_rpm /
        denominator;

    return Motor_Feedforward_ClampFloat(iq,
                                        -MOTOR_FRICTION_COMP_LIMIT_A,
                                        MOTOR_FRICTION_COMP_LIMIT_A);
}

float Motor_Feedforward_InertiaIq(float accel_rpm_s)
{
    float iq =
        MOTOR_INERTIA_COMP_GAIN_A_PER_RPM_S *
        accel_rpm_s;

    return Motor_Feedforward_ClampFloat(iq,
                                        -MOTOR_INERTIA_COMP_LIMIT_A,
                                        MOTOR_INERTIA_COMP_LIMIT_A);
}

float Motor_Feedforward_ApplyIq(float speed_loop_iq,
                                float friction_iq,
                                float inertia_iq,
                                float min_iq,
                                float max_iq)
{
    float iq =
        speed_loop_iq +
        friction_iq +
        inertia_iq;

    return Motor_Feedforward_ClampFloat(iq,
                                        min_iq,
                                        max_iq);
}
