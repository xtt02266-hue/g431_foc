#include "motor_encoder.h"
#include "board_profile.h"
#include "mt6826s.h"
#include "stm32g4xx_hal.h"

/* 角度、时间戳和有效标志由 SPI DMA 回调发布，控制路径与前台只读。
 * 沿用现有逐字段发布顺序；volatile 不意味着多字段读取具有快照一致性。
 */
static volatile uint16_t g_encoder_angle = 0U;
static volatile uint32_t g_encoder_last_update_ms = 0U;
static volatile uint8_t g_encoder_has_sample = 0U;

// 清除角度缓存和有效性历史；等待第一帧有效 MT6826S 样本。
void motor_encoder_init(void)
{
    g_encoder_angle = 0U;
    g_encoder_last_update_ms = 0U;
    g_encoder_has_sample = 0U;
    mt6826s_init();
    /* ADC中断启动前先请求一帧，供首次FOC计算使用。 */
    motor_encoder_update_fast();
}

// 快速路径请求角度 DMA，忙时沿用原有跳过规则而不阻塞。
void motor_encoder_update_fast(void)
{
#if BOARD_SENSORED_CONTROL_ENABLE
    mt6826s_request_read_dma();
#endif
}

// DMA 完成路径发布通过校验的 15 位角计数及 HAL 时间戳。
void motor_encoder_on_sample(uint16_t angle)
{
#if BOARD_SENSORED_CONTROL_ENABLE
    g_encoder_angle = angle & MOTOR_ENCODER_COUNT_MASK_U16;
    g_encoder_last_update_ms = HAL_GetTick();
    g_encoder_has_sample = 1U;
#else
    (void)angle;
#endif
}

// 只读最近发布的机械角计数，范围 0～32767；新鲜度由独立接口检查。
uint16_t motor_encoder_get_raw_angle(void)
{
    return g_encoder_angle;
}

// 返回最近有效样本年龄，单位 ms；无样本时沿用原哨兵值。
uint16_t motor_encoder_get_sample_age_ms(void)
{
    uint32_t age = HAL_GetTick() - g_encoder_last_update_ms;
    return !g_encoder_has_sample || age > 65535U ? 65535U : (uint16_t)age;
}

// 同时检查传感器状态和样本年龄；CRC 错误不刷新有效样本时间戳。
uint8_t motor_encoder_is_data_fresh(uint32_t max_age_ms)
{
    uint32_t last_update = g_encoder_last_update_ms;
    if ((g_encoder_has_sample == 0U) || (mt6826s_is_ok() == 0U)) {
        return 0U;
    }
    return ((uint32_t)(HAL_GetTick() - last_update) <= max_age_ms) ? 1U : 0U;
}

// 同时检查已发布样本标志和底层通信状态；不替代样本年龄检查。
uint8_t motor_encoder_is_ok(void)
{
    return ((g_encoder_has_sample != 0U) && (mt6826s_is_ok() != 0U)) ? 1U : 0U;
}
