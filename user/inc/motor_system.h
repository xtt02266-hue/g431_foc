#ifndef MOTOR_SYSTEM_H
#define MOTOR_SYSTEM_H

#include <stdint.h>

typedef enum
{
    MOTOR_STATE_STOPPED = 0,
    MOTOR_STATE_ALIGN,
    MOTOR_STATE_RUN,
    MOTOR_STATE_FAULT
} MotorState;

typedef struct
{
    MotorState state;
    uint16_t pot_raw;
    uint16_t target;
} MotorSystem;

extern MotorSystem g_motor_system;

void Motor_System_Init(void);
void Motor_System_Task(void);

#endif
