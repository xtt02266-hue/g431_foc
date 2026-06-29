#ifndef MOTOR_PARAMETERS_H
#define MOTOR_PARAMETERS_H

#include <stdint.h>

typedef enum
{
    MOTOR_PARAMETERS_NO_DATA = 0,
    MOTOR_PARAMETERS_READY,
    MOTOR_PARAMETERS_IDENTIFYING,
    MOTOR_PARAMETERS_SAVE_PENDING,
    MOTOR_PARAMETERS_ERROR
} MotorParametersStatus;

/*
 * Load and validate saved parameters. If none are valid, schedule one
 * automatic identification attempt after the encoder becomes ready.
 */
void Motor_Parameters_Init(void);

/*
 * Public one-shot entry point. Call once when the motor is unloaded.
 * Identification runs asynchronously and the result is saved automatically.
 * Returns 1 when the request was accepted, otherwise 0.
 */
uint8_t Motor_Parameters_IdentifyAndSave(void);

/* Internal scheduler hooks. */
void Motor_Parameters_ControlTask1ms(void);
void Motor_Parameters_BackgroundTask(void);

uint8_t Motor_Parameters_IsReady(void);
uint8_t Motor_Parameters_IsBusy(void);
uint8_t Motor_Parameters_HasStoredData(void);
MotorParametersStatus Motor_Parameters_GetStatus(void);

#endif
