#ifndef MOTOR_COGGING_H
#define MOTOR_COGGING_H

#include <stdint.h>

#define MOTOR_COGGING_TABLE_SIZE          512U
#define MOTOR_COGGING_CHUNK_MAX_POINTS    120U
#define MOTOR_COGGING_TABLE_SCALE_A       0.0001f
#define MOTOR_COGGING_TABLE_MAX_COUNTS    5000
#define MOTOR_COGGING_RAMP_TIME_MS         500U

typedef struct
{
    uint8_t enabled;
    float gain;
    float max_iq_a;
    float fade_start_rpm;
    float fade_end_rpm;
    uint16_t phase_offset_counts;
} MotorCoggingConfig;

void Motor_Cogging_Init(void);
uint8_t Motor_Cogging_IsConfigValid(const MotorCoggingConfig *config);
void Motor_Cogging_SetConfig(const MotorCoggingConfig *config);
void Motor_Cogging_GetConfig(MotorCoggingConfig *config);

uint8_t Motor_Cogging_TableBegin(uint16_t count, uint32_t expected_crc,
                                 uint16_t *transaction_id);
uint8_t Motor_Cogging_TableWriteChunk(uint16_t transaction_id,
                                      uint16_t offset, uint8_t count,
                                      const int16_t *values);
uint8_t Motor_Cogging_TableCommit(uint16_t transaction_id);
uint8_t Motor_Cogging_TableReadChunk(uint16_t offset, uint8_t count,
                                     int16_t *values);
uint8_t Motor_Cogging_SaveToFlash(void);

void Motor_Cogging_RuntimeStart(void);
void Motor_Cogging_RuntimeStop(void);
void Motor_Cogging_Task1ms(uint8_t applicable_mode_running);
float Motor_Cogging_Compensation(uint16_t mechanical_angle_counts,
                                 float mechanical_speed_rpm);

uint32_t Motor_Cogging_Crc32(const void *data, uint32_t length);
uint32_t Motor_Cogging_GetActiveTableCrc(void);
uint32_t Motor_Cogging_GetTableRevision(void);
float Motor_Cogging_GetEffectiveGain(void);
uint8_t Motor_Cogging_IsPersisted(void);

#endif
