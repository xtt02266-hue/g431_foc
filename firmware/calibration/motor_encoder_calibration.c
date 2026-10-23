#include "motor_encoder_calibration.h"
#include "motor_system.h"
#include "motor_encoder.h"
#include "mt6826s.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <string.h>

/* 沿用 MT6826S Rev1.1 第 8.6.4/5、10.2 节对应的芯片自校准流程。
 * 当前代码仅修改运行时 AUTO_CAL_FREQ 字段并保留其他位，不发送整片 EEPROM 编程命令。 */
#define FREQ_REG   0x00EU // 自校准频率配置寄存器，保持其他位
#define KEY_REG    0x155U // 自校准触发/清理键值寄存器
#define STATUS_REG 0x113U // 自校准状态寄存器，状态取高两位
/* 前台推进芯片自校准和 SPI 寄存器步骤，停机路径可同时请求取消。
 * 共享阶段和触发标志沿用 volatile 与既有 PRIMASK 临界区；清理和超时保持原流程。
 * *_started/finished_at/last_poll 为 HAL 毫秒时间戳，travel/counts 为累计角计数。
 */
static volatile uint8_t phase, reason, reboot_required, cleanup;
static uint8_t step, original_freq, freq_known, freq_written;
static volatile uint8_t trigger_attempted;
static uint8_t chip_status, saw_running, settle_started;
static uint32_t started, stage_started, settled_since;
static volatile uint32_t operation_started, finished_at;
static uint32_t crc_start, transfer_start, last_poll, counts;
static uint16_t last_angle;
static int32_t travel;
static float measured_rpm;

// 查询自校准阶段或清理是否仍在执行，供启动和停机门控使用。
uint8_t motor_encoder_cal_is_active(void)
{
    return (phase >= ENC_CAL_RAMP && phase <= ENC_CAL_RUNNING) || cleanup;
}

// 读取触发自校准后的重新上电要求；结束不自动清除此标志。
uint8_t motor_encoder_cal_requires_power_cycle(void)
{
    return reboot_required;
}

// 在保护原中断状态的临界区内初始化自校准步骤和错误计数基线。
void motor_encoder_cal_begin(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    reason = ENC_CAL_OK;
    step = chip_status = saw_running = settle_started = freq_known = freq_written = 0U;
    trigger_attempted = cleanup = reboot_required = 0U;
    started = stage_started = HAL_GetTick();
    counts = 0U;
    travel = 0;
    crc_start = mt6826s_get_crc_error_count();
    transfer_start = mt6826s_get_transfer_error_count();
    phase = ENC_CAL_RAMP;
    if (!mask) {
        __enable_irq();
    }
}

// 停机路径标记校准取消并安排前台清理；保留已触发芯片校准的上电要求。
void motor_encoder_cal_notify_stop(void)
{
    if (phase >= ENC_CAL_RAMP && phase <= ENC_CAL_RUNNING) {
        phase = ENC_CAL_ABORTED;
        reason = ENC_CAL_CANCELLED;
        finished_at = HAL_GetTick();
        reboot_required = trigger_attempted;
        cleanup = 1U;
        operation_started = HAL_GetTick();
    }
}

// 锁存自校准结果并停止输出；根据芯片状态安排清理或暂停后续角度读取。
static void finish(uint8_t result_phase, uint8_t result_reason)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (phase < ENC_CAL_RAMP || phase > ENC_CAL_RUNNING) {
        if (!mask) {
            __enable_irq();
        }
        return;
    }
    phase = result_phase;
    reason = result_reason;
    finished_at = HAL_GetTick();
    reboot_required = trigger_attempted;
    cleanup = (result_phase == ENC_CAL_SUCCESS || chip_status == 3U) ? 0U : 1U;
    operation_started = HAL_GetTick();
    if (result_phase == ENC_CAL_SUCCESS || chip_status == 3U) {
        mt6826s_suspend_angle_reads();
    }
    motor_system_stop_control();
    if (!mask) {
        __enable_irq();
    }
}

/* 仅由前台调用；单次 SPI 寄存器事务最长 1 ms。
 * 角度 DMA 忙时跳过本次事务，避免在 1 kHz/20 kHz 控制中断中等待。 */
// 撤销触发键或恢复原频率配置；清理结束后按原规则暂停角度读取。
static void step_cleanup(uint32_t now)
{
    uint8_t r = 1U;
    if (trigger_attempted) {
        r = mt6826s_write_register(KEY_REG, 0U);
    } else if (freq_known && freq_written) {
        r = mt6826s_write_register(FREQ_REG, original_freq);
    }
    if (r || (uint32_t)(now - operation_started) > 100U) {
        cleanup = 0U;
        if (reboot_required) {
            mt6826s_suspend_angle_reads();
        }
    }
}

// 推进编码器自校准阶段，保留事务、保护阈值及阶段转换时机。
static void step_ramp(uint32_t now, const motor_control_snapshot_t *s)
{
    if (s->state == MOTOR_STATE_SENSORED_RUN && s->target_speed_rpm >= 249.9f) {
        phase = ENC_CAL_SETTLE;
        stage_started = now;
    }
}

// 推进编码器自校准阶段，保留事务、保护阈值及阶段转换时机。
static void step_settle(uint32_t now)
{
    if (fabsf(fabsf(measured_rpm) - 250.0f) <= 25.0f) {
        if (!settle_started) {
            settled_since = now;
            settle_started = 1U;
        }
        if ((uint32_t)(now - settled_since) >= 1000U) {
            phase = ENC_CAL_CONFIGURE;
            operation_started = now;
        }
    } else {
        settle_started = 0U;
    }
    if ((uint32_t)(now - stage_started) > 5000U) {
        finish(ENC_CAL_FAILED, ENC_CAL_SPEED);
    }
}

// 推进编码器自校准阶段，保留事务、保护阈值及阶段转换时机。
static void step_configure(uint32_t now)
{
    uint8_t r = 0U, value = 0U;
    switch (step) {
    case 0:
        r = mt6826s_read_register(FREQ_REG, &original_freq);
        if (r == 1U) {
            freq_known = 1U;
        }
        break;
    case 1:
        freq_written = 1U; /* 写入超时仍可能已到达芯片，保留恢复记录 */
        r = mt6826s_write_register(FREQ_REG, (original_freq & 0x8FU) | 0x40U);
        break;
    case 2:
        r = mt6826s_read_register(FREQ_REG, &value);
        if (r == 1U && value != ((original_freq & 0x8FU) | 0x40U)) {
            finish(ENC_CAL_FAILED, ENC_CAL_READBACK);
            return;
        }
        break;
    case 3:
        r = mt6826s_write_register(KEY_REG, 0U);
        break;
    default:
        /* 在可能结果不确定的写入前锁存重新上电要求。 */
        trigger_attempted = 1U;
        r = mt6826s_write_register(KEY_REG, 0x5EU);
        if (r == 0U) {
            trigger_attempted = 0U; /* 忙状态表示此次尚未发送字节 */
        }
        if (r == 1U && phase == ENC_CAL_CONFIGURE) {
            phase = ENC_CAL_RUNNING;
            stage_started = now;
            last_poll = now;
            last_angle = motor_encoder_get_raw_angle();
        }
        break;
    }
    if (phase != ENC_CAL_CONFIGURE && phase != ENC_CAL_RUNNING) {
        return;
    }
    if (r == 2U) {
        finish(ENC_CAL_FAILED, ENC_CAL_SPI);
        return;
    }
    if (r == 1U) {
        ++step;
        operation_started = now;
    }
    if ((uint32_t)(now - operation_started) > 100U) {
        finish(ENC_CAL_FAILED, ENC_CAL_SPI);
    }
}

// 推进编码器自校准阶段，保留事务、保护阈值及阶段转换时机。
static void step_running(uint32_t now)
{
    uint16_t angle = motor_encoder_get_raw_angle();
    int32_t delta = (int32_t)angle - last_angle;
    if (delta > 16384) {
        delta -= 32768;
    }
    if (delta < -16384) {
        delta += 32768;
    }
    travel += delta;
    last_angle = angle;
    counts = (uint32_t)(travel < 0 ? -travel : travel);
    if (!isfinite(measured_rpm) || fabsf(measured_rpm) < 200.0f || fabsf(measured_rpm) >= 400.0f) {
        finish(ENC_CAL_FAILED, ENC_CAL_SPEED);
        return;
    }
    if ((uint32_t)(now - last_poll) < 100U) {
        return;
    }
    uint8_t value = 0U, r = mt6826s_read_register(STATUS_REG, &value);
    if (r == 2U) {
        finish(ENC_CAL_FAILED, ENC_CAL_SPI);
        return;
    }
    if (r == 0U) {
        return;
    }
    last_poll = now;
    chip_status = (value >> 6U) & 3U;
    if (phase != ENC_CAL_RUNNING) {
        return;
    }
    if (chip_status == 1U) {
        saw_running = 1U;
    }
    if (chip_status == 2U) {
        finish(ENC_CAL_FAILED, ENC_CAL_CHIP_FAILED);
    } else if (chip_status == 3U) {
        finish(saw_running && counts > 18UL * 32768UL ? ENC_CAL_SUCCESS : ENC_CAL_FAILED,
               saw_running && counts > 18UL * 32768UL ? ENC_CAL_OK : ENC_CAL_UNCONFIRMED);
    } else if ((uint32_t)(now - stage_started) > 30000U) {
        finish(ENC_CAL_FAILED, ENC_CAL_TIMEOUT);
    }
}

void motor_encoder_cal_background_task(void)
{
    uint32_t now = HAL_GetTick();
    if (cleanup) {
        step_cleanup(now);
        return;
    }

    if (!motor_encoder_cal_is_active()) {
        return;
    }
    motor_control_snapshot_t s;
    motor_system_get_control_snapshot(&s);
    measured_rpm = g_motor_system.run_data.speed_rpm;
    if (s.owner != MOTOR_OWNER_HOST || !s.run_requested) {
        finish(ENC_CAL_ABORTED, ENC_CAL_CONTROL_LOST);
        return;
    }
    if (s.state == MOTOR_STATE_FAULT || !motor_encoder_is_data_fresh(2U) || mt6826s_get_status() != 0U ||
        mt6826s_get_crc_error_count() != crc_start || mt6826s_get_transfer_error_count() != transfer_start) {
        finish(ENC_CAL_FAILED, ENC_CAL_SENSOR);
        return;
    }
    if ((uint32_t)(now - started) > 35000U) {
        finish(ENC_CAL_FAILED, ENC_CAL_TIMEOUT);
        return;
    }
    switch (phase) {
    case ENC_CAL_RAMP:
        step_ramp(now, &s);
        break;
    case ENC_CAL_SETTLE:
        step_settle(now);
        break;
    case ENC_CAL_CONFIGURE:
        step_configure(now);
        break;
    case ENC_CAL_RUNNING:
        step_running(now);
        break;
    default:
        break;
    }
}


// 复制校准阶段、原因、累计计数和测得 rpm；调用方须提供有效输出对象。
void motor_encoder_cal_get_status(motor_encoder_cal_status_t *s)
{
    memset(s, 0, sizeof(*s));
    s->phase = phase;
    s->reason = reason;
    s->chip_status = chip_status;
    s->flags = (motor_encoder_cal_is_active() ? 1U : 0U) | (reboot_required ? 2U : 0U) |
               (phase != ENC_CAL_IDLE ? 4U : 0U) | (saw_running ? 8U : 0U);
    s->elapsed_ms = phase == ENC_CAL_IDLE
                        ? 0U
                        : ((phase >= ENC_CAL_RAMP && phase <= ENC_CAL_RUNNING) ? HAL_GetTick() : finished_at) - started;
    s->calibrated_counts = counts;
    s->measured_rpm = measured_rpm;
}
