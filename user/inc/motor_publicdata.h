#ifndef MOTOR_PUBLICDATA_H
#define MOTOR_PUBLICDATA_H

#include <stdint.h>

typedef struct
{
    volatile uint16_t pot_raw;
} MotorPublicData;

extern MotorPublicData g_motor_publicdata;

#endif
