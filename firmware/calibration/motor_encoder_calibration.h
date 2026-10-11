#ifndef MOTOR_ENCODER_CALIBRATION_H
#define MOTOR_ENCODER_CALIBRATION_H
#include <stdint.h>

enum {
    ENC_CAL_IDLE,
    ENC_CAL_RAMP,
    ENC_CAL_SETTLE,
    ENC_CAL_CONFIGURE,
    ENC_CAL_RUNNING,
    ENC_CAL_SUCCESS,
    ENC_CAL_FAILED,
    ENC_CAL_ABORTED
};

enum {
    ENC_CAL_OK,
    ENC_CAL_CANCELLED,
    ENC_CAL_CONTROL_LOST,
    ENC_CAL_SENSOR,
    ENC_CAL_SPEED,
    ENC_CAL_SPI,
    ENC_CAL_READBACK,
    ENC_CAL_CHIP_FAILED,
    ENC_CAL_TIMEOUT,
    ENC_CAL_UNCONFIRMED
};

typedef struct {
    uint8_t phase, reason, chip_status, flags;
    uint32_t elapsed_ms, calibrated_counts;
    float measured_rpm;
} motor_encoder_cal_status_t;

// 在保护原中断状态的临界区内初始化自校准步骤和错误计数基线。
void motor_encoder_cal_begin(void);
// 前台推进自校准和 SPI 清理；单次寄存器事务最长 1 ms，忙时跳过。
void motor_encoder_cal_background_task(void);
// 停机路径标记校准取消并安排前台清理；保留已触发芯片校准的上电要求。
void motor_encoder_cal_notify_stop(void); /* 可由停机路径调用，寄存器清理由前台延后处理 */
uint8_t motor_encoder_cal_is_active(void);
// 读取触发自校准后的重新上电要求；结束不自动清除此标志。
uint8_t motor_encoder_cal_requires_power_cycle(void);
// 复制校准阶段、原因、累计计数和测得 rpm；调用方须提供有效输出对象。
void motor_encoder_cal_get_status(motor_encoder_cal_status_t *status);
#endif
