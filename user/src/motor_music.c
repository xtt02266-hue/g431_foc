#include "motor_music.h"
#include "motor_songs.h"
#include "main.h"
#include <stddef.h>

#define MOTOR_MUSIC_SINE_TABLE_SIZE       32U
#define MOTOR_MUSIC_SINE_INDEX_SHIFT      27U
#define MOTOR_MUSIC_NOTE_GAP_MS            0U

#if MOTOR_MUSIC_INTERPOLATION_BITS > 16U
#error "MOTOR_MUSIC_INTERPOLATION_BITS must be between 0 and 16"
#endif

#define MOTOR_MUSIC_INTERPOLATION_SHIFT \
    (MOTOR_MUSIC_SINE_INDEX_SHIFT - MOTOR_MUSIC_INTERPOLATION_BITS)
#define MOTOR_MUSIC_INTERPOLATION_MASK \
    ((1UL << MOTOR_MUSIC_INTERPOLATION_BITS) - 1UL)
#define MOTOR_MUSIC_INTERPOLATION_SCALE \
    (1.0f / (float)(1UL << MOTOR_MUSIC_INTERPOLATION_BITS))

static const float g_sine_table[MOTOR_MUSIC_SINE_TABLE_SIZE] = {
     0.000000f,  0.195090f,  0.382683f,  0.555570f,
     0.707107f,  0.831470f,  0.923880f,  0.980785f,
     1.000000f,  0.980785f,  0.923880f,  0.831470f,
     0.707107f,  0.555570f,  0.382683f,  0.195090f,
     0.000000f, -0.195090f, -0.382683f, -0.555570f,
    -0.707107f, -0.831470f, -0.923880f, -0.980785f,
    -1.000000f, -0.980785f, -0.923880f, -0.831470f,
    -0.707107f, -0.555570f, -0.382683f, -0.195090f
};

static const MotorMusicNote *g_song = NULL;
static uint16_t g_note_count = 0U;
static uint16_t g_note_index = 0U;
static uint16_t g_note_remaining_ms = 0U;
static uint16_t g_gap_start_ms = 0U;
static uint16_t g_plays_remaining = 0U;
static uint8_t g_repeat_forever = 0U;

/* Shared by the 1 kHz sequencer and the 20 kHz current-loop interrupt. */
static volatile uint8_t g_playing = 0U;
static volatile uint8_t g_phase_reset_requested = 0U;
static volatile uint32_t g_phase_step = 0U;
static volatile float g_amplitude_a = MOTOR_MUSIC_DEFAULT_AMPLITUDE_A;
static uint32_t g_phase_accumulator = 0U;

static uint32_t Motor_Music_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Motor_Music_ExitCritical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static uint32_t Motor_Music_FrequencyToPhaseStep(float frequency_hz)
{
    const float phase_scale = 4294967296.0f / MOTOR_MUSIC_SAMPLE_RATE_HZ;

    if (!(frequency_hz > 0.0f)) {
        return 0U;
    }

    if (frequency_hz > MOTOR_MUSIC_MAX_FREQUENCY_HZ) {
        frequency_hz = MOTOR_MUSIC_MAX_FREQUENCY_HZ;
    }

    return (uint32_t)(frequency_hz * phase_scale);
}

static void Motor_Music_SetFrequency(float frequency_hz)
{
    g_phase_step = Motor_Music_FrequencyToPhaseStep(frequency_hz);
    g_phase_reset_requested = 1U;
}

static void Motor_Music_LoadNote(uint16_t note_index)
{
    uint16_t duration_ms = g_song[note_index].duration_ms;

    if (duration_ms == 0U) {
        duration_ms = 1U;
    }

    g_note_index = note_index;
    g_note_remaining_ms = duration_ms;
    g_gap_start_ms = (duration_ms > (2U * MOTOR_MUSIC_NOTE_GAP_MS))
                         ? MOTOR_MUSIC_NOTE_GAP_MS
                         : 0U;
    Motor_Music_SetFrequency(g_song[note_index].frequency_hz);
}

void Motor_Music_Init(void)
{
    Motor_Music_Stop();
    Motor_Music_SetAmplitude(MOTOR_MUSIC_DEFAULT_AMPLITUDE_A);
}

static void Motor_Music_StartInternal(const MotorMusicNote *song,
                                      uint16_t note_count,
                                      uint8_t repeat_forever,
                                      uint16_t play_count)
{
    uint32_t primask;

    if ((song == NULL) || (note_count == 0U)) {
        Motor_Music_Stop();
        return;
    }

    primask = Motor_Music_EnterCritical();
    g_song = song;
    g_note_count = note_count;
    g_repeat_forever = repeat_forever;
    g_plays_remaining = play_count;
    Motor_Music_LoadNote(0U);
    g_playing = 1U;
    Motor_Music_ExitCritical(primask);
}

void Motor_Music_Start(const MotorMusicNote *song,
                       uint16_t note_count,
                       uint8_t repeat)
{
    Motor_Music_StartInternal(song,
                              note_count,
                              (repeat != 0U) ? 1U : 0U,
                              1U);
}

void Motor_Music_StartTimes(const MotorMusicNote *song,
                            uint16_t note_count,
                            uint16_t play_count)
{
    if (play_count == 0U) {
        Motor_Music_Stop();
        return;
    }

    Motor_Music_StartInternal(song, note_count, 0U, play_count);
}

void Motor_Music_StartDemo(void)
{
    Motor_Music_Start(g_motor_song_haruhikage_intro,
                      g_motor_song_haruhikage_intro_count,
                      0U);
}

void Motor_Music_StartDemoTimes(uint16_t play_count)
{
    Motor_Music_StartTimes(
        g_motor_song_haruhikage_intro,
        g_motor_song_haruhikage_intro_count,
        play_count);
}

void Motor_Music_Stop(void)
{
    uint32_t primask = Motor_Music_EnterCritical();

    g_playing = 0U;
    g_phase_step = 0U;
    g_phase_reset_requested = 1U;
    g_song = NULL;
    g_note_count = 0U;
    g_note_index = 0U;
    g_note_remaining_ms = 0U;
    g_gap_start_ms = 0U;
    g_plays_remaining = 0U;
    g_repeat_forever = 0U;

    Motor_Music_ExitCritical(primask);
}

void Motor_Music_Task1ms(void)
{
    if (g_playing == 0U) {
        return;
    }

    if (g_note_remaining_ms > 0U) {
        g_note_remaining_ms--;
    }

    if (g_note_remaining_ms == 0U) {
        uint16_t next_note = (uint16_t)(g_note_index + 1U);

        if (next_note >= g_note_count) {
            if (g_repeat_forever == 0U) {
                if (g_plays_remaining <= 1U) {
                    Motor_Music_Stop();
                    return;
                }
                g_plays_remaining--;
            }
            next_note = 0U;
        }

        Motor_Music_LoadNote(next_note);
    } else if ((g_gap_start_ms != 0U) &&
               (g_note_remaining_ms == g_gap_start_ms)) {
        Motor_Music_SetFrequency(MOTOR_MUSIC_REST);
    }
}

void Motor_Music_SetAmplitude(float amplitude_a)
{
    if (amplitude_a < 0.0f) {
        amplitude_a = 0.0f;
    } else if (amplitude_a > MOTOR_MUSIC_MAX_AMPLITUDE_A) {
        amplitude_a = MOTOR_MUSIC_MAX_AMPLITUDE_A;
    }

    g_amplitude_a = amplitude_a;
}

uint8_t Motor_Music_IsPlaying(void)
{
    return g_playing;
}

float Motor_Music_ProcessIqTarget(float normal_iq_target)
{
    uint32_t phase_step;
    uint32_t phase;
    uint32_t table_index;
    uint32_t next_index;
    uint32_t interpolation_raw;
    float interpolation;
    float sample;

    if (g_playing == 0U) {
        return normal_iq_target;
    }

    if (g_phase_reset_requested != 0U) {
        g_phase_accumulator = 0U;
        g_phase_reset_requested = 0U;
    }

    phase_step = g_phase_step;
    if (phase_step == 0U) {
        return 0.0f;
    }

    phase = g_phase_accumulator;
    table_index = phase >> MOTOR_MUSIC_SINE_INDEX_SHIFT;
    next_index = (table_index + 1U) & (MOTOR_MUSIC_SINE_TABLE_SIZE - 1U);
    interpolation_raw = (phase >> MOTOR_MUSIC_INTERPOLATION_SHIFT) &
                        MOTOR_MUSIC_INTERPOLATION_MASK;
    interpolation = (float)interpolation_raw * MOTOR_MUSIC_INTERPOLATION_SCALE;
    sample = g_sine_table[table_index] +
             (g_sine_table[next_index] - g_sine_table[table_index]) *
             interpolation;
    g_phase_accumulator = phase + phase_step;

    return g_amplitude_a * sample;
}
