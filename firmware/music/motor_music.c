#include "motor_music.h"
#include "motor_songs.h"
#include "main.h"
#include <stddef.h>

#define MOTOR_MUSIC_SINE_TABLE_SIZE  32U // DDS 正弦表一圈的采样点数
#define MOTOR_MUSIC_SINE_INDEX_SHIFT 27U // 32 位相位累加器到 32 点表索引的移位量
#define MOTOR_MUSIC_NOTE_GAP_MS      0U  // 当前音符间休止时长，单位 ms；0 表示不插入间隔

#if MOTOR_MUSIC_INTERPOLATION_BITS > 16U
#error "MOTOR_MUSIC_INTERPOLATION_BITS must be between 0 and 16"
#endif

#define MOTOR_MUSIC_INTERPOLATION_SHIFT                                                                                \
    (MOTOR_MUSIC_SINE_INDEX_SHIFT - MOTOR_MUSIC_INTERPOLATION_BITS)                    // DDS 相位到插值小数位的移位量
#define MOTOR_MUSIC_INTERPOLATION_MASK ((1UL << MOTOR_MUSIC_INTERPOLATION_BITS) - 1UL) // 表间插值小数的有效位掩码
#define MOTOR_MUSIC_INTERPOLATION_SCALE                                                                                \
    (1.0f / (float)(1UL << MOTOR_MUSIC_INTERPOLATION_BITS)) // 插值小数归一化到 [0,1) 的系数

/* 音符指针和时长由 1 kHz 序列器维护，曲目内存由调用方保持有效。
 * 播放、相位步长和幅值与 20 kHz 电流 ISR 共用；DDS 累加器由高频路径维护。
 * 多字段发布沿用现有临界区，音符时长单位 ms，幅值单位 A。
 */
static const float g_sine_table[MOTOR_MUSIC_SINE_TABLE_SIZE] = {
    0.000000f,  0.195090f,  0.382683f,  0.555570f,  0.707107f,  0.831470f,  0.923880f,  0.980785f,
    1.000000f,  0.980785f,  0.923880f,  0.831470f,  0.707107f,  0.555570f,  0.382683f,  0.195090f,
    0.000000f,  -0.195090f, -0.382683f, -0.555570f, -0.707107f, -0.831470f, -0.923880f, -0.980785f,
    -1.000000f, -0.980785f, -0.923880f, -0.831470f, -0.707107f, -0.555570f, -0.382683f, -0.195090f};

static const motor_music_note_t *g_song = NULL;
static uint16_t g_note_count = 0U;
static uint16_t g_note_index = 0U;
static uint16_t g_note_remaining_ms = 0U;
static uint16_t g_gap_start_ms = 0U;
static uint16_t g_plays_remaining = 0U;
static uint8_t g_repeat_forever = 0U;

/* 由 1 kHz 音符序列器与 20 kHz 电流 ISR 共用，沿用原发布方式。 */
static volatile uint8_t g_playing = 0U;
static volatile uint8_t g_phase_reset_requested = 0U;
static volatile uint32_t g_phase_step = 0U;
static volatile float g_amplitude_a = MOTOR_MUSIC_DEFAULT_AMPLITUDE_A;
static uint32_t g_phase_accumulator = 0U;

// 保存原 PRIMASK 后屏蔽中断，保护音符序列器与电流 ISR 的共享状态。
static uint32_t motor_music_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

// 按进入前的 PRIMASK 恢复中断，避免打开调用方原本屏蔽的中断。
static void motor_music_exit_critical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

// 把频率 Hz 转为 32 位 DDS 相位步长；非正频率输出休止符，保留最高频率限幅。
static uint32_t motor_music_frequency_to_phase_step(float frequency_hz)
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

// 发布新 DDS 步长并请求高频电流路径复位相位。
static void motor_music_set_frequency(float frequency_hz)
{
    g_phase_step = motor_music_frequency_to_phase_step(frequency_hz);
    g_phase_reset_requested = 1U;
}

// 装入指定音符的频率 Hz 和时长 ms；零时长按原规则处理为 1 ms。
static void motor_music_load_note(uint16_t note_index)
{
    uint16_t duration_ms = g_song[note_index].duration_ms;

    if (duration_ms == 0U) {
        duration_ms = 1U;
    }

    g_note_index = note_index;
    g_note_remaining_ms = duration_ms;
    g_gap_start_ms = (duration_ms > (2U * MOTOR_MUSIC_NOTE_GAP_MS)) ? MOTOR_MUSIC_NOTE_GAP_MS : 0U;
    motor_music_set_frequency(g_song[note_index].frequency_hz);
}

// 复位音乐输出并恢复默认幅值 A；不自动启动曲目。
void motor_music_init(void)
{
    motor_music_stop();
    motor_music_set_amplitude(MOTOR_MUSIC_DEFAULT_AMPLITUDE_A);
}

// 在临界区内发布曲目缓冲区与播放次数；调用方须保持曲目内存有效。
static void motor_music_start_internal(const motor_music_note_t *song, uint16_t note_count, uint8_t repeat_forever,
                                       uint16_t play_count)
{
    uint32_t primask;

    if ((song == NULL) || (note_count == 0U)) {
        motor_music_stop();
        return;
    }

    primask = motor_music_enter_critical();
    g_song = song;
    g_note_count = note_count;
    g_repeat_forever = repeat_forever;
    g_plays_remaining = play_count;
    motor_music_load_note(0U);
    g_playing = 1U;
    motor_music_exit_critical(primask);
}

// 按曲目 ID 和 repeat 选项播放；ID 由 motor_songs.h 定义，非零 repeat 循环。
void motor_music_start(uint8_t song_id, uint8_t repeat)
{
    uint16_t note_count;
    const motor_music_note_t *song = motor_songs_get_score(song_id, &note_count);

    motor_music_start_internal(song, note_count, (repeat != 0U) ? 1U : 0U, 1U);
}

// 按曲目 ID 指定有限播放次数；次数为零时停止，音符时长单位 ms。
void motor_music_start_times(uint8_t song_id, uint16_t play_count)
{
    uint16_t note_count;
    const motor_music_note_t *song = motor_songs_get_score(song_id, &note_count);

    if (play_count == 0U) {
        motor_music_stop();
        return;
    }

    motor_music_start_internal(song, note_count, 0U, play_count);
}

// 播放一遍内置示范曲目，使用现有频率和时长数据。
void motor_music_start_demo(void)
{
    motor_music_start(MOTOR_SONG_DEMO, 0U);
}

// 按指定次数播放内置曲目；次数为零时沿用停止语义。
void motor_music_start_demo_times(uint16_t play_count)
{
    motor_music_start_times(MOTOR_SONG_DEMO, play_count);
}

// 清除播放请求和 DDS 步长，安排高频路径清零相位。
void motor_music_stop(void)
{
    uint32_t primask = motor_music_enter_critical();

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

    motor_music_exit_critical(primask);
}

// 1 ms 序列器推进音符时长、间隔和播放次数；保持与高频 DDS 的发布关系。
void motor_music_task1ms(void)
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
                    motor_music_stop();
                    return;
                }
                g_plays_remaining--;
            }
            next_note = 0U;
        }

        motor_music_load_note(next_note);
    } else if ((g_gap_start_ms != 0U) && (g_note_remaining_ms == g_gap_start_ms)) {
        motor_music_set_frequency(MOTOR_MUSIC_REST);
    }
}

// 设置音乐 Iq 幅值 A，保留原有非正值处理和最大幅值裁剪。
void motor_music_set_amplitude(float amplitude_a)
{
    if (amplitude_a < 0.0f) {
        amplitude_a = 0.0f;
    } else if (amplitude_a > MOTOR_MUSIC_MAX_AMPLITUDE_A) {
        amplitude_a = MOTOR_MUSIC_MAX_AMPLITUDE_A;
    }

    g_amplitude_a = amplitude_a;
}

// 读取播放请求标志；不代表绕组已实际发声。
uint8_t motor_music_is_playing(void)
{
    return g_playing;
}

// 20 kHz 电流路径用 DDS 查表插值生成 Iq（A）；停播透传正常控制目标，休止符输出零。
float motor_music_process_iq_target(float normal_iq_target)
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
    interpolation_raw = (phase >> MOTOR_MUSIC_INTERPOLATION_SHIFT) & MOTOR_MUSIC_INTERPOLATION_MASK;
    interpolation = (float)interpolation_raw * MOTOR_MUSIC_INTERPOLATION_SCALE;
    sample = g_sine_table[table_index] + (g_sine_table[next_index] - g_sine_table[table_index]) * interpolation;
    g_phase_accumulator = phase + phase_step;

    return g_amplitude_a * sample;
}
