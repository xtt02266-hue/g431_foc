#ifndef MOTOR_MUSIC_H
#define MOTOR_MUSIC_H

#include <stdint.h>

/*
 * Set to 1 to play the built-in demo once after motor identification.
 * Keep it disabled by default so normal position control remains unchanged.
 */
#ifndef MOTOR_MUSIC_AUTOPLAY_DEMO
#define MOTOR_MUSIC_AUTOPLAY_DEMO          0
#endif

/* The FOC current loop is called from the 20 kHz injected ADC callback. */
#define MOTOR_MUSIC_SAMPLE_RATE_HZ         20000.0f
#define MOTOR_MUSIC_DEFAULT_AMPLITUDE_A    0.12f
#define MOTOR_MUSIC_MAX_AMPLITUDE_A        0.30f
#define MOTOR_MUSIC_MAX_FREQUENCY_HZ       4000.0f

/* 0 disables interpolation; 8 gives 256 interpolated steps per LUT segment. */
#define MOTOR_MUSIC_INTERPOLATION_BITS      8U

#define MOTOR_MUSIC_REST                    0.0f
#define MOTOR_NOTE_C4                     261.63f
#define MOTOR_NOTE_D4                     293.66f
#define MOTOR_NOTE_E4                     329.63f
#define MOTOR_NOTE_F4                     349.23f
#define MOTOR_NOTE_G4                     392.00f
#define MOTOR_NOTE_A4                     440.00f
#define MOTOR_NOTE_B4                     493.88f
#define MOTOR_NOTE_C5                     523.25f
#define MOTOR_NOTE_D5                     587.33f
#define MOTOR_NOTE_E5                     659.25f
#define MOTOR_NOTE_F5                     698.46f
#define MOTOR_NOTE_G5                     783.99f
#define MOTOR_NOTE_A5                     880.00f
#define MOTOR_NOTE_B5                     987.77f
#define MOTOR_NOTE_C6                    1046.50f

typedef struct
{
    float frequency_hz;
    uint16_t duration_ms;
} MotorMusicNote;

void Motor_Music_Init(void);
void Motor_Music_Start(const MotorMusicNote *song,
                       uint16_t note_count,
                       uint8_t repeat);
void Motor_Music_StartTimes(const MotorMusicNote *song,
                            uint16_t note_count,
                            uint16_t play_count);
void Motor_Music_StartDemo(void);
void Motor_Music_StartDemoTimes(uint16_t play_count);
void Motor_Music_Stop(void);
void Motor_Music_Task1ms(void);
void Motor_Music_SetAmplitude(float amplitude_a);
uint8_t Motor_Music_IsPlaying(void);

/* Called only by the 20 kHz current loop. */
float Motor_Music_ProcessIqTarget(float normal_iq_target);

#endif
