#include "motor_angle_reference.h"
#include "board_profile.h"
#include <math.h>
#include <stddef.h>
#if BOARD_AS5600_REFERENCE_ENABLE
#include "as5600.h"
#endif

// 按板级配置启动 AS5600 参考角采样；有感配置下保持空操作。
void motor_angle_reference_init(void)
{
#if BOARD_AS5600_REFERENCE_ENABLE
    as5600_request_read_dma();
#endif
}

// 前台维护可选参考编码器，避免在控制中断中执行 I2C 恢复。
void motor_angle_reference_background_task(void)
{
#if BOARD_AS5600_REFERENCE_ENABLE
    as5600_background_task();
#endif
}

// 将参考机械角转换为电角度 rad；偏置单位 rad，方向为 ±1，成功返回 1。
uint8_t motor_angle_reference_get_electrical_angle(uint16_t pole_pairs, int8_t direction, float mechanical_offset_rad,
                                                   float *electrical_angle_rad)
{
#if BOARD_AS5600_REFERENCE_ENABLE
    uint32_t primask;
    uint16_t raw;
    uint8_t fresh;
    float angle;
    const float two_pi = 6.28318530718f;
    if ((electrical_angle_rad == NULL) || (pole_pairs == 0U) || ((direction != 1) && (direction != -1)) ||
        !isfinite(mechanical_offset_rad)) {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    fresh = as5600_is_data_fresh(20U);
    raw = as5600_read_raw_angle();
    if (primask == 0U) {
        __enable_irq();
    }
    if (fresh == 0U) {
        return 0U;
    }
    angle = fmodf(((float)raw * (two_pi / 4096.0f) - fmodf(mechanical_offset_rad, two_pi)) * (float)pole_pairs *
                      (float)direction,
                  two_pi);
    *electrical_angle_rad = (angle < 0.0f) ? angle + two_pi : angle;
    return 1U;
#else
    (void)pole_pairs;
    (void)direction;
    (void)mechanical_offset_rad;
    (void)electrical_angle_rad;
    return 0U;
#endif
}
