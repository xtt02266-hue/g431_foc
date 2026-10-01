#ifndef MOTOR_ENCODER_CALIBRATION_H
#define MOTOR_ENCODER_CALIBRATION_H
#include <stdint.h>

enum { ENC_CAL_IDLE, ENC_CAL_RAMP, ENC_CAL_SETTLE, ENC_CAL_CONFIGURE,
       ENC_CAL_RUNNING, ENC_CAL_SUCCESS, ENC_CAL_FAILED, ENC_CAL_ABORTED };
enum { ENC_CAL_OK, ENC_CAL_CANCELLED, ENC_CAL_CONTROL_LOST, ENC_CAL_SENSOR,
       ENC_CAL_SPEED, ENC_CAL_SPI, ENC_CAL_READBACK, ENC_CAL_CHIP_FAILED,
       ENC_CAL_TIMEOUT, ENC_CAL_UNCONFIRMED };
typedef struct {
    uint8_t phase, reason, chip_status, flags;
    uint32_t elapsed_ms, calibrated_counts;
    float measured_rpm;
} MotorEncoderCalStatus;
void Motor_EncoderCal_Begin(void);
void Motor_EncoderCal_BackgroundTask(void);
void Motor_EncoderCal_NotifyStop(void); /* IRQ safe; cleanup deferred */
uint8_t Motor_EncoderCal_IsActive(void);
uint8_t Motor_EncoderCal_RequiresPowerCycle(void);
void Motor_EncoderCal_GetStatus(MotorEncoderCalStatus *status);
#endif
