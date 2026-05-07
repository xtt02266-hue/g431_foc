#ifndef MOTOR_CURRENT_LOOP_H
#define MOTOR_CURRENT_LOOP_H

#include <stdint.h>

#ifndef MOTOR_CURRENT_VREF_VOLTS
#define MOTOR_CURRENT_VREF_VOLTS 3.3f
#endif

#ifndef MOTOR_CURRENT_BIAS_VOLTS
#define MOTOR_CURRENT_BIAS_VOLTS 1.65f
#endif

#ifndef MOTOR_CURRENT_SHUNT_OHMS
#define MOTOR_CURRENT_SHUNT_OHMS 0.01f
#endif

#ifndef MOTOR_CURRENT_GAIN
#define MOTOR_CURRENT_GAIN 20.0f
#endif

#ifndef MOTOR_CURRENT_ADC_MAX
#define MOTOR_CURRENT_ADC_MAX 4095.0f
#endif

typedef struct
{
    uint16_t iu_raw;
    uint16_t iw_raw;
    float iu_a;
    float iw_a;
} MotorCurrentSample;

void Motor_CurrentLoop_Init(void);
void Motor_CurrentLoop_SetBiasVolts(float bias_volts);
void Motor_CurrentLoop_SetVrefVolts(float vref_volts);
void Motor_CurrentLoop_SetGain(float gain);
void Motor_CurrentLoop_SetShuntOhms(float shunt_ohms);
MotorCurrentSample Motor_CurrentLoop_GetLastSample(void);

#endif
