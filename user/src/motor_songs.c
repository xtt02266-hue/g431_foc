#include "motor_songs.h"

/* 50 ms-grid monophonic transcription of the first 15.5 seconds. */
const motor_music_note_t g_motor_song_haruhikage_intro[] = {
    {MOTOR_MUSIC_REST, 500U}, {MOTOR_NOTE_FS4, 100U}, {MOTOR_NOTE_AS4, 50U},  {MOTOR_NOTE_DS5, 750U},
    {MOTOR_NOTE_FS5, 50U},    {MOTOR_NOTE_CS5, 250U}, {MOTOR_NOTE_FS5, 50U},  {MOTOR_NOTE_B4, 550U},
    {MOTOR_NOTE_FS5, 50U},    {MOTOR_NOTE_CS5, 300U}, {MOTOR_NOTE_FS5, 50U},  {MOTOR_NOTE_DS5, 350U},
    {MOTOR_NOTE_E5, 200U},    {MOTOR_NOTE_FS5, 50U},  {MOTOR_NOTE_DS5, 250U}, {MOTOR_NOTE_FS5, 50U},
    {MOTOR_NOTE_CS5, 950U},   {MOTOR_NOTE_DS5, 550U}, {MOTOR_NOTE_FS5, 50U},  {MOTOR_NOTE_CS5, 250U},
    {MOTOR_NOTE_FS5, 50U},    {MOTOR_NOTE_B4, 450U},  {MOTOR_NOTE_DS5, 100U}, {MOTOR_NOTE_FS5, 50U},
    {MOTOR_NOTE_CS5, 250U},   {MOTOR_NOTE_DS5, 350U}, {MOTOR_NOTE_CS5, 50U},  {MOTOR_NOTE_B4, 50U},
    {MOTOR_NOTE_FS5, 50U},    {MOTOR_NOTE_E5, 200U},  {MOTOR_NOTE_DS5, 300U}, {MOTOR_NOTE_CS5, 900U},
    {MOTOR_NOTE_E5, 50U},     {MOTOR_NOTE_DS5, 500U}, {MOTOR_NOTE_FS5, 100U}, {MOTOR_NOTE_CS5, 300U},
    {MOTOR_NOTE_B4, 400U},    {MOTOR_NOTE_CS5, 500U}, {MOTOR_NOTE_FS5, 50U},  {MOTOR_NOTE_DS5, 400U},
    {MOTOR_NOTE_CS5, 50U},    {MOTOR_NOTE_E5, 150U},  {MOTOR_NOTE_DS5, 250U}, {MOTOR_NOTE_FS5, 50U},
    {MOTOR_NOTE_CS5, 500U},   {MOTOR_NOTE_FS5, 150U}, {MOTOR_NOTE_CS5, 300U}, {MOTOR_NOTE_DS5, 500U},
    {MOTOR_NOTE_E5, 50U},     {MOTOR_NOTE_CS5, 400U}, {MOTOR_NOTE_B4, 350U},  {MOTOR_NOTE_CS5, 500U},
    {MOTOR_NOTE_DS5, 450U},   {MOTOR_NOTE_FS5, 100U}, {MOTOR_NOTE_E5, 100U},  {MOTOR_NOTE_DS5, 250U},
    {MOTOR_NOTE_CS5, 750U},   {MOTOR_NOTE_FS5, 100U},
};

const uint16_t g_motor_song_haruhikage_intro_count =
    (uint16_t)(sizeof(g_motor_song_haruhikage_intro) / sizeof(g_motor_song_haruhikage_intro[0]));

/* 自定义曲目描述由前台独占；播放模块保存自己的活动描述，不受后续登记影响。
 * 仅保存地址和点数，曲目音符仍由调用方拥有，不引入动态分配或复制。
 */
static const motor_music_note_t *s_custom_score;
static uint16_t s_custom_note_count;

// 登记外部曲目数据；不调用 PWM、电流环或音乐控制接口。
void motor_songs_register_score(const motor_music_note_t *song, uint16_t note_count)
{
    s_custom_score = song;
    s_custom_note_count = note_count;
}

// 根据标量 ID 读取只读曲目描述；未知 ID 沿用无效缓冲区语义。
const motor_music_note_t *motor_songs_get_score(uint8_t song_id, uint16_t *note_count)
{
    if (song_id == MOTOR_SONG_DEMO) {
        *note_count = g_motor_song_haruhikage_intro_count;
        return g_motor_song_haruhikage_intro;
    }
    if (song_id == MOTOR_SONG_CUSTOM) {
        *note_count = s_custom_note_count;
        return s_custom_score;
    }
    *note_count = 0U;
    return 0;
}
