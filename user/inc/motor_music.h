#ifndef MOTOR_MUSIC_H
#define MOTOR_MUSIC_H

#include <stdint.h>

/*
 * 上电自动播放开关：0 为关闭，避免进入闭环后音乐抢占正常力矩输出。
 * 改为 1 时，首次进入有感运行会自动播放一次内置歌曲。
 * 此开关只控制自动触发，音乐合成、曲谱和手动播放接口仍然保留。
 */
#ifndef MOTOR_MUSIC_AUTOPLAY_DEMO
#define MOTOR_MUSIC_AUTOPLAY_DEMO 0
#endif

/* 音乐 DDS 按电流注入 ADC 回调的 20 kHz 频率计算相位步长。 */
#define MOTOR_MUSIC_SAMPLE_RATE_HZ      20000.0f
#define MOTOR_MUSIC_DEFAULT_AMPLITUDE_A 0.30f
#define MOTOR_MUSIC_MAX_AMPLITUDE_A     0.30f
#define MOTOR_MUSIC_MAX_FREQUENCY_HZ    4500.0f

/* 0 关闭插值；8 表示每个查表区间使用 256 个插值小数步。 */
#define MOTOR_MUSIC_INTERPOLATION_BITS 8U

#define MOTOR_MUSIC_REST 0.0f
#define MOTOR_NOTE_FS3   185.00f  // 音符 FS3，单位 Hz
#define MOTOR_NOTE_GS3   207.65f  // 音符 GS3，单位 Hz
#define MOTOR_NOTE_A3    220.00f  // 音符 A3，单位 Hz
#define MOTOR_NOTE_AS3   233.08f  // 音符 AS3，单位 Hz
#define MOTOR_NOTE_B3    246.94f  // 音符 B3，单位 Hz
#define MOTOR_NOTE_C4    261.63f  // 音符 C4，单位 Hz
#define MOTOR_NOTE_CS4   277.18f  // 音符 CS4，单位 Hz
#define MOTOR_NOTE_D4    293.66f  // 音符 D4，单位 Hz
#define MOTOR_NOTE_DS4   311.13f  // 音符 DS4，单位 Hz
#define MOTOR_NOTE_E4    329.63f  // 音符 E4，单位 Hz
#define MOTOR_NOTE_F4    349.23f  // 音符 F4，单位 Hz
#define MOTOR_NOTE_FS4   369.99f  // 音符 FS4，单位 Hz
#define MOTOR_NOTE_G4    392.00f  // 音符 G4，单位 Hz
#define MOTOR_NOTE_GS4   415.30f  // 音符 GS4，单位 Hz
#define MOTOR_NOTE_A4    440.00f  // 音符 A4，单位 Hz
#define MOTOR_NOTE_AS4   466.16f  // 音符 AS4，单位 Hz
#define MOTOR_NOTE_B4    493.88f  // 音符 B4，单位 Hz
#define MOTOR_NOTE_C5    523.25f  // 音符 C5，单位 Hz
#define MOTOR_NOTE_CS5   554.37f  // 音符 CS5，单位 Hz
#define MOTOR_NOTE_D5    587.33f  // 音符 D5，单位 Hz
#define MOTOR_NOTE_DS5   622.25f  // 音符 DS5，单位 Hz
#define MOTOR_NOTE_E5    659.25f  // 音符 E5，单位 Hz
#define MOTOR_NOTE_F5    698.46f  // 音符 F5，单位 Hz
#define MOTOR_NOTE_FS5   739.99f  // 音符 FS5，单位 Hz
#define MOTOR_NOTE_G5    783.99f  // 音符 G5，单位 Hz
#define MOTOR_NOTE_A5    880.00f  // 音符 A5，单位 Hz
#define MOTOR_NOTE_B5    987.77f  // 音符 B5，单位 Hz
#define MOTOR_NOTE_C6    1046.50f // 音符 C6，单位 Hz

typedef struct {
    float frequency_hz;
    uint16_t duration_ms;
} motor_music_note_t;

// 复位音乐输出并恢复默认幅值 A；不自动启动曲目。
void motor_music_init(void);
// 按曲目 ID 和 repeat 选项播放；ID 由 motor_songs.h 定义，非零 repeat 循环。
void motor_music_start(uint8_t song_id, uint8_t repeat);
// 按曲目 ID 指定有限播放次数；次数为零时停止，音符时长单位 ms。
void motor_music_start_times(uint8_t song_id, uint16_t play_count);
// 播放一遍内置示范曲目，使用现有频率和时长数据。
void motor_music_start_demo(void);
// 按指定次数播放内置曲目；次数为零时沿用停止语义。
void motor_music_start_demo_times(uint16_t play_count);
// 清除播放请求和 DDS 步长，安排高频路径清零相位。
void motor_music_stop(void);
// 1 ms 序列器推进音符时长、间隔和播放次数；保持与高频 DDS 的发布关系。
void motor_music_task1ms(void);
// 设置音乐 Iq 幅值 A，保留原有非正值处理和最大幅值裁剪。
void motor_music_set_amplitude(float amplitude_a);
// 读取播放请求标志；不代表绕组已实际发声。
uint8_t motor_music_is_playing(void);

/* Called only by the 20 kHz current loop. */
float motor_music_process_iq_target(float normal_iq_target);

#endif
