#ifndef MOTOR_SONGS_H
#define MOTOR_SONGS_H

#include "motor_music.h"
#include <stdint.h>

extern const motor_music_note_t g_motor_song_haruhikage_intro[];
extern const uint16_t g_motor_song_haruhikage_intro_count;

// 曲目 ID：内置曲目为 0，调用方注册的任意曲目为 1。
#define MOTOR_SONG_DEMO   0U
#define MOTOR_SONG_CUSTOM 1U

/* 曲目数据边界：仅登记只读缓冲区，不启动或改变正在播放的曲目。
 * 任意长度曲目不复制；前台调用，内存须保持有效直至所有播放结束。
 * NULL/零长度也可登记，以保留原播放入口对无效曲目的拒绝语义。
 */
void motor_songs_register_score(const motor_music_note_t *song, uint16_t note_count);
// 读取曲目数据；note_count 必须非空。未知 ID 返回 NULL、点数 0；前台独占登记。
const motor_music_note_t *motor_songs_get_score(uint8_t song_id, uint16_t *note_count);

#endif
