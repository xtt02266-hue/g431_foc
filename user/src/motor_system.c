#include "motor_system.h"
#include "motor_publicdata.h"

MotorSystem g_motor_system = {
    .state = MOTOR_STATE_STOPPED,
    .pot_raw = 0U,
    .target = 0U,
};

static uint16_t Motor_MapPotToTarget(uint16_t pot_raw)
{
    return pot_raw;
}

void Motor_System_Init(void)
{
    g_motor_system.state = MOTOR_STATE_STOPPED;
    g_motor_system.pot_raw = g_motor_publicdata.pot_raw;
    g_motor_system.target = Motor_MapPotToTarget(g_motor_system.pot_raw);
}

void Motor_System_Task(void)
{
    g_motor_system.pot_raw = g_motor_publicdata.pot_raw;
    g_motor_system.target = Motor_MapPotToTarget(g_motor_system.pot_raw);
}
