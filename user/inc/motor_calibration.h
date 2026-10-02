#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H
#include <stdint.h>

#define MOTOR_CAL_SCHEMA 1U
#define MOTOR_CAL_RING_SIZE 256U /* retain recently sent records for optional replay */
#define MOTOR_CAL_POINT_COUNT 512U
#define MOTOR_CAL_POINT_MAX_ERROR_COUNTS 2.0f
#define MOTOR_CAL_POINT_SETTLE_MS 1000U
#define MOTOR_CAL_POINT_CAPTURE_MS 500U
#define MOTOR_CAL_POINT_AVERAGE_MS 1000U
#define MOTOR_CAL_POINT_TIMEOUT_MS 30000U
#define MOTOR_CAL_REQUIRED_TURNS 14U /* two boundary guard turns; analyzer requires 12 */
enum { CAL_IDLE, CAL_SETTLE, CAL_CAPTURE, CAL_ZERO, CAL_DONE, CAL_FAILED, CAL_ABORTED };
enum { CAL_OK, CAL_UNSTABLE, CAL_STALL, CAL_SENSOR, CAL_OVERFLOW,
       CAL_CANCELLED, CAL_TIMEOUT, CAL_INSUFFICIENT, CAL_CONTROL_LOST };
enum { CAL_LIMIT = 1U, CAL_SLEW = 2U, CAL_BAD_SENSOR = 4U, CAL_CONTROL_GAP = 8U,
       CAL_POINT_AVERAGE = 16U }; /* fresh 1s fallback window, not a fault */

/* Wire schema 1, little-endian IEEE754, exactly 64 bytes. flags also encodes
 * point average bit4 (policy1), phase bits 8..10, speed index bits 11..12
 * and reverse direction bit13. Point schemas append a target float (68 bytes). */
typedef struct {
    uint32_t sequence, tick_ms;
    uint16_t angle, flags;
    float target_rpm, speed_rpm, window_rpm;
    float pi_iq, friction_iq, cogging_iq, applied_iq, actual_iq, actual_id, uq;
    uint8_t sensor_status, window_ticks;
    uint16_t sensor_age_ms;
    uint32_t crc_errors, transfer_errors;
} MotorCalSample;
typedef struct {
    uint32_t session, produced, dropped;
    uint8_t phase, reason, speed_index, reverse;
    uint16_t accepted_turns, rejected_turns;
    uint8_t passed_mask, mode, speed_reasons[3];
    float target_rpm;
} MotorCalStatus;

uint8_t Motor_Calibration_Begin(uint8_t mode);
uint8_t Motor_Calibration_BeginWithPoints(uint8_t mode, uint16_t count);
uint8_t Motor_Calibration_BeginWithOptions(uint8_t mode, uint16_t count, uint8_t repeats, uint8_t retry_until_good);
uint8_t Motor_Calibration_BeginWithPolicy(uint8_t mode, uint16_t count, uint8_t repeats, uint8_t retry_until_good, uint8_t policy);
uint8_t Motor_Calibration_PointPolicy(void);
uint8_t Motor_Calibration_DecodeStartPolicy(const uint8_t *body, uint16_t size, uint8_t *mode, uint16_t *count, uint8_t *repeats, uint8_t *retry_until_good, uint8_t *policy);
uint16_t Motor_Calibration_PointCount(void);
uint16_t Motor_Calibration_PointSchema(void);
uint8_t Motor_Calibration_PointRepeats(void);
uint8_t Motor_Calibration_PointRetryUntilGood(void);
uint8_t Motor_Calibration_DecodeStartRequest(const uint8_t *body, uint16_t size, uint8_t *mode, uint16_t *count);
uint8_t Motor_Calibration_DecodeStartOptions(const uint8_t *body, uint16_t size, uint8_t *mode, uint16_t *count, uint8_t *repeats, uint8_t *retry_until_good);
void Motor_Calibration_Abort(uint8_t reason);
uint8_t Motor_Calibration_IsActive(void);
float Motor_Calibration_Step(uint16_t angle, float speed, float applied_target,
                            uint16_t control_flags, uint8_t healthy);
void Motor_Calibration_GetStatus(MotorCalStatus *status);
uint16_t Motor_Calibration_SampleFlags(void);
void Motor_Calibration_Push(const MotorCalSample *sample);
uint8_t Motor_Calibration_Peek(MotorCalSample *samples, uint8_t maximum);
void Motor_Calibration_Consume(uint8_t count);
/* PublishControl runs with interrupts masked; Capture runs in the ADC ISR. */
void Motor_Calibration_PublishControl(const MotorCalSample *sample);
void Motor_Calibration_Capture(uint16_t angle, float reference, float iq, float id, float uq);
uint8_t Motor_Calibration_HasSamples(void);
uint8_t Motor_Calibration_IsPointMode(void);
uint8_t Motor_Calibration_PointAssist(void);
float Motor_Calibration_PointTarget(void);
void Motor_Calibration_StepPoint(uint16_t angle,float speed,uint16_t flags,uint8_t healthy);
void Motor_Calibration_PeekTargets(float *targets,uint8_t count);
uint8_t Motor_Calibration_Replay(uint32_t first,uint8_t count,MotorCalSample *samples,float *targets);
#endif
