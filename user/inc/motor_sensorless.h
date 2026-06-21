#ifndef MOTOR_SENSORLESS_H
#define MOTOR_SENSORLESS_H

#include <stdint.h>

#define MOTOR_SENSORLESS_SAMPLE_TIME_SEC       0.00005f
#define MOTOR_SENSORLESS_DEFAULT_ENABLE        0U

typedef enum
{
    MOTOR_SENSORLESS_DISABLED = 0,
    MOTOR_SENSORLESS_SEARCHING,
    MOTOR_SENSORLESS_TRACKING,
    MOTOR_SENSORLESS_LOST
} MotorSensorlessStatus;

typedef struct
{
    float sample_time_sec;
    float resistance_ohm;
    float inductance_h;
    uint16_t pole_pairs;

    float bemf_filter_alpha;
    float minimum_bemf_volts;
    float pll_kp;
    float pll_ki;
    float maximum_electrical_speed_rad_s;
    float lock_phase_error;

    uint16_t lock_updates;
    uint16_t loss_updates;
    uint8_t pll_divider;
} MotorSensorlessConfig;

typedef struct
{
    float electrical_angle_rad;
    float electrical_speed_rad_s;
    float mechanical_speed_rpm;
    float bemf_alpha_volts;
    float bemf_beta_volts;
    float bemf_magnitude_volts;
    float pll_phase_error;
    MotorSensorlessStatus status;
    uint8_t valid;
} MotorSensorlessOutput;

void Motor_Sensorless_Init(void);
void Motor_Sensorless_GetDefaultConfig(MotorSensorlessConfig *config);
uint8_t Motor_Sensorless_Configure(const MotorSensorlessConfig *config);
uint8_t Motor_Sensorless_ConfigureMotor(float resistance_ohm,
                                        float inductance_h,
                                        uint16_t pole_pairs);
void Motor_Sensorless_Enable(uint8_t enable);
uint8_t Motor_Sensorless_IsEnabled(void);
void Motor_Sensorless_Reset(void);

/* Direction is the expected electrical direction: +1 or -1. */
void Motor_Sensorless_SetDirection(int8_t direction);

/* Called from the 20 kHz current loop. The module is a no-op while disabled. */
void Motor_Sensorless_Update(float voltage_alpha,
                             float voltage_beta,
                             float current_alpha,
                             float current_beta);

MotorSensorlessOutput Motor_Sensorless_GetOutput(void);
float Motor_Sensorless_GetElectricalAngle(void);
float Motor_Sensorless_GetMechanicalSpeedRpm(void);
uint8_t Motor_Sensorless_IsValid(void);
uint8_t Motor_Sensorless_IsReadyForHandover(void);

#endif
