#ifndef MOTOR_FRICTION_STORAGE_H
#define MOTOR_FRICTION_STORAGE_H
#include "motor_feedforward.h"
uint8_t Motor_FrictionStorage_Load(MotorFrictionConfig *config);
uint8_t Motor_FrictionStorage_Save(const MotorFrictionConfig *config);
#endif
