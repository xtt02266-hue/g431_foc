#ifndef BOARD_PROFILE_H
#define BOARD_PROFILE_H

#ifndef FOC_PROFILE_SENSORLESS
#define FOC_PROFILE_SENSORLESS 0
#endif

/*
 * The sensorless profile is a hybrid validation profile. The host can select
 * AS5600 startup or an open-loop forced startup followed by sensorless
 * handover in Speed mode. MT6826S is not used by this profile.
 */
#define BOARD_SENSORED_CONTROL_ENABLE 1

/* 有感控制启用且选择 sensorless 混合调试配置时，默认控制角和
 * 遥测参考角来自 AS5600；其他有感配置使用 MT6826S。 */
#define BOARD_CONTROL_ENCODER_AS5600 \
    (BOARD_SENSORED_CONTROL_ENABLE && FOC_PROFILE_SENSORLESS)
#define BOARD_AS5600_REFERENCE_ENABLE BOARD_CONTROL_ENCODER_AS5600

#endif
