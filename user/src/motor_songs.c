#include "motor_songs.h"

const MotorMusicNote g_motor_song_twinkle[] = {
    {MOTOR_NOTE_C4, 350U}, {MOTOR_NOTE_C4, 350U},
    {MOTOR_NOTE_G4, 350U}, {MOTOR_NOTE_G4, 350U},
    {MOTOR_NOTE_A4, 350U}, {MOTOR_NOTE_A4, 350U},
    {MOTOR_NOTE_G4, 700U}, {MOTOR_MUSIC_REST, 120U},
    {MOTOR_NOTE_F4, 350U}, {MOTOR_NOTE_F4, 350U},
    {MOTOR_NOTE_E4, 350U}, {MOTOR_NOTE_E4, 350U},
    {MOTOR_NOTE_D4, 350U}, {MOTOR_NOTE_D4, 350U},
    {MOTOR_NOTE_C4, 700U}
};

const uint16_t g_motor_song_twinkle_count =
    (uint16_t)(sizeof(g_motor_song_twinkle) /
               sizeof(g_motor_song_twinkle[0]));
