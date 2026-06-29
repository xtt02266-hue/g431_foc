#ifndef MOTOR_MUSIC_H
#define MOTOR_MUSIC_H

#include <stdint.h>

/*
 * Play the built-in demo once after the motor first enters closed-loop run.
 * Position control resumes automatically when the song finishes.
 */
#ifndef MOTOR_MUSIC_AUTOPLAY_DEMO
#define MOTOR_MUSIC_AUTOPLAY_DEMO          1
#endif

/* The FOC current loop is called from the 20 kHz injected ADC callback. */
#define MOTOR_MUSIC_SAMPLE_RATE_HZ         20000.0f
#define MOTOR_MUSIC_DEFAULT_AMPLITUDE_A    0.30f
#define MOTOR_MUSIC_MAX_AMPLITUDE_A        0.30f
#define MOTOR_MUSIC_MAX_FREQUENCY_HZ       4500.0f

/* 0 disables interpolation; 8 gives 256 interpolated steps per LUT segment. */
#define MOTOR_MUSIC_INTERPOLATION_BITS      8U

#define MOTOR_MUSIC_REST                    0.0f
#define MOTOR_NOTE_FS3                    185.00f
#define MOTOR_NOTE_GS3                    207.65f
#define MOTOR_NOTE_A3                     220.00f
#define MOTOR_NOTE_AS3                    233.08f
#define MOTOR_NOTE_B3                     246.94f
#define MOTOR_NOTE_C4                     261.63f
#define MOTOR_NOTE_CS4                    277.18f
#define MOTOR_NOTE_D4                     293.66f
#define MOTOR_NOTE_DS4                    311.13f
#define MOTOR_NOTE_E4                     329.63f
#define MOTOR_NOTE_F4                     349.23f
#define MOTOR_NOTE_FS4                    369.99f
#define MOTOR_NOTE_G4                     392.00f
#define MOTOR_NOTE_GS4                    415.30f
#define MOTOR_NOTE_A4                     440.00f
#define MOTOR_NOTE_AS4                    466.16f
#define MOTOR_NOTE_B4                     493.88f
#define MOTOR_NOTE_C5                     523.25f
#define MOTOR_NOTE_CS5                    554.37f
#define MOTOR_NOTE_D5                     587.33f
#define MOTOR_NOTE_DS5                    622.25f
#define MOTOR_NOTE_E5                     659.25f
#define MOTOR_NOTE_F5                     698.46f
#define MOTOR_NOTE_FS5                    739.99f
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
