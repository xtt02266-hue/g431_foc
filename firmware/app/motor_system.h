#ifndef MOTOR_SYSTEM_H
#define MOTOR_SYSTEM_H

#include "motor_types.h"
#include "motor_music.h"
#include "motor_config.h"
/* 保留既有调用方的源代码兼容入口；协议和业务代码可直接包含职责对应的头文件。 */
#include "motor_control.h"
#include "motor_host.h"

// 初始化电机系统。
void motor_system_init(void);

// 周期任务入口。
void motor_system_task(void);

// 以 uint8_t 读取系统运行状态，数值对应 MOTOR_STATE_*。
uint8_t motor_system_get_state(void);
#endif
