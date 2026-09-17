#ifndef MOTOR_FEEDFORWARD_H
#define MOTOR_FEEDFORWARD_H

#include <stdint.h>

typedef struct
{
    uint8_t enabled;
    float coulomb_iq_a;
    float viscous_iq_a_per_rpm;
    float smooth_speed_rpm;
    float max_iq_a;
} MotorFrictionConfig;

void Motor_Feedforward_FrictionInit(void);
uint8_t Motor_Feedforward_IsFrictionConfigValid(
    const MotorFrictionConfig *config);
void Motor_Feedforward_SetFrictionConfig(const MotorFrictionConfig *config);
void Motor_Feedforward_GetFrictionConfig(MotorFrictionConfig *config);
float Motor_Feedforward_FrictionCompensation(float speed_rpm);

#endif
