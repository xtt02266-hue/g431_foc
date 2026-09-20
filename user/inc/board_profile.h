#ifndef BOARD_PROFILE_H
#define BOARD_PROFILE_H

#ifndef FOC_PROFILE_SENSORLESS
#define FOC_PROFILE_SENSORLESS 0
#endif
#ifndef COMM_BACKEND_USB
#define COMM_BACKEND_USB 0
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
#define BOARD_COMM_USB_CDC COMM_BACKEND_USB

/* 新版 USB 板状态灯接 PB5；旧版 UART 板继续使用 PB6。 */
#define BOARD_STATUS_LED_GPIO_PORT GPIOB
#if BOARD_COMM_USB_CDC
#define BOARD_STATUS_LED_GPIO_PIN GPIO_PIN_5
#else
#define BOARD_STATUS_LED_GPIO_PIN GPIO_PIN_6
#endif

#endif
