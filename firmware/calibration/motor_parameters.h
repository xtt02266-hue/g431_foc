#ifndef MOTOR_PARAMETERS_H
#define MOTOR_PARAMETERS_H

#include <stdint.h>

typedef enum {
    MOTOR_PARAMETERS_NO_DATA = 0,
    MOTOR_PARAMETERS_READY,
    MOTOR_PARAMETERS_IDENTIFYING,
    MOTOR_PARAMETERS_SAVE_PENDING,
    MOTOR_PARAMETERS_ERROR
} motor_parameters_status_t;

/*
 * Load and validate saved parameters. If none are valid, schedule one
 * automatic identification attempt after the encoder becomes ready.
 */
void motor_parameters_init(void);

/*
 * Public one-shot entry point. Call once when the motor is unloaded.
 * Identification runs asynchronously and the result is saved automatically.
 * Returns 1 when the request was accepted, otherwise 0.
 */
uint8_t motor_parameters_identify_and_save(void);

// 取消待触发、正在辨识或待保存会话；恢复此前有效参数，保持停机。
void motor_parameters_cancel_identification(void);

/* Internal scheduler hooks. */
void motor_parameters_control_task1ms(void);
// 前台处理辨识完成后的参数保存与状态发布。
void motor_parameters_background_task(void);

// 查询控制所需电机参数是否已有效装入。
uint8_t motor_parameters_is_ready(void);
// 查询辨识或保存流程是否占用参数模块，供控制命令门控。
uint8_t motor_parameters_is_busy(void);
// 读取是否存在校验通过的已保存参数记录。
uint8_t motor_parameters_has_stored_data(void);
// 读取参数加载、辨识或保存状态码；保持现有枚举数值。
uint8_t motor_parameters_get_status(void);

#endif
