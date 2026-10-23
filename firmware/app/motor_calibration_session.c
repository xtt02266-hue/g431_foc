#include "motor_host.h"
#include "motor_control.h"
#include "motor_context.h"
#include "motor_position_loop.h"
#include "motor_encoder.h"
#include "motor_identify.h"
#include "motor_music.h"
#include "motor_parameters.h"
#include "motor_calibration.h"
#include "mt6826s.h"
#include "motor_encoder_calibration.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stddef.h>

/* 标定开始时在原临界区保存配置，统一停机后恢复；电流单位 A，速度斜率 rpm/s。 */
static motor_cogging_config_t g_cal_saved_cogging;
static float g_cal_saved_limit;
static float g_cal_saved_speed_slew;
static motor_control_mode_t g_cal_saved_mode;
static motor_input_source_t g_cal_saved_source;
static float g_cal_saved_pos_max, g_cal_saved_pos_min;

/* 调用方已屏蔽中断；输出停止后再恢复标定前配置。 */
void motor_system_restore_calibration(void)
{
    if (!g_cal_saved_valid) {
        return;
    }
    motor_cogging_set_config(g_cal_saved_cogging.enabled, g_cal_saved_cogging.gain, g_cal_saved_cogging.max_iq_a,
                             g_cal_saved_cogging.fade_start_rpm, g_cal_saved_cogging.fade_end_rpm,
                             g_cal_saved_cogging.phase_offset_counts);
    motor_feedforward_set_friction_config(g_cal_saved_friction.enabled, g_cal_saved_friction.coulomb_iq_a,
                                          g_cal_saved_friction.viscous_iq_a_per_rpm,
                                          g_cal_saved_friction.smooth_speed_rpm, g_cal_saved_friction.max_iq_a);
    g_host_iq_limit_a = g_cal_saved_limit;
    g_host_speed_slew_rpm_per_s = g_cal_saved_speed_slew;
    g_control_mode = g_cal_saved_mode;
    g_input_source = g_cal_saved_source;
    g_pi_pos.out_max = g_cal_saved_pos_max;
    g_pi_pos.out_min = g_cal_saved_pos_min;
    g_cal_saved_valid = 0U;
    ++g_config_revision;
}

// 依原顺序检查控制权、模式、参数和补偿配置，返回 MOTOR_CMD_*。
static uint8_t calibration_session_validate(uint8_t mode)
{
    if (motor_encoder_cal_is_active()) {
        return MOTOR_CMD_BUSY;
    }
    if (motor_encoder_cal_requires_power_cycle()) {
        return MOTOR_CMD_NOT_READY;
    }
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (mode > 5U) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    if (motor_calibration_is_active() || motor_calibration_has_samples() || motor_music_is_playing() ||
        motor_parameters_is_busy()) {
        return MOTOR_CMD_BUSY;
    }
    int8_t direction = motor_identify_get_result().uvw_dir;
    if ((direction != 1 && direction != -1) || !motor_parameters_is_ready() || !motor_encoder_is_data_fresh(2U)) {
        return MOTOR_CMD_NOT_READY;
    }
    motor_cogging_config_t verification;
    motor_cogging_get_config(&verification);
    if ((mode == 2U || mode == 3U) &&
        (!verification.enabled || verification.gain <= 0.0f || verification.gain > 1.0f ||
         verification.max_iq_a <= 0.0f || verification.max_iq_a > .010001f || !motor_cogging_get_table_revision())) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    return MOTOR_CMD_OK;
}

// 在已屏蔽中断的启动路径保存配置，供统一停机恢复。
static void calibration_session_save_config(void)
{
    motor_cogging_get_config(&g_cal_saved_cogging);
    motor_feedforward_get_friction_config(&g_cal_saved_friction);
    g_cal_saved_limit = g_host_iq_limit_a;
    g_cal_saved_speed_slew = g_host_speed_slew_rpm_per_s;
    g_cal_saved_mode = g_control_mode;
    g_cal_saved_source = g_input_source;
    g_cal_saved_pos_max = g_pi_pos.out_max;
    g_cal_saved_pos_min = g_pi_pos.out_min;
    g_cal_saved_valid = 1U;
}

// 设置标定控制模式、补偿和保护限幅；调用方持有原临界区。
static void calibration_session_configure(uint8_t mode)
{
    motor_cogging_config_t disabled = g_cal_saved_cogging;
    if (mode != 3U) {
        disabled.enabled = 0U;
    }
    motor_cogging_set_config(disabled.enabled, disabled.gain, disabled.max_iq_a, disabled.fade_start_rpm,
                             disabled.fade_end_rpm, disabled.phase_offset_counts);
    if (mode == 4U || mode == 5U) {
        motor_friction_config_t friction_disabled = g_cal_saved_friction;
        friction_disabled.enabled = 0U;
        motor_feedforward_set_friction_config(friction_disabled.enabled, friction_disabled.coulomb_iq_a,
                                              friction_disabled.viscous_iq_a_per_rpm,
                                              friction_disabled.smooth_speed_rpm, friction_disabled.max_iq_a);
    }
    g_control_mode = mode == 5U ? MOTOR_CONTROL_POSITION : MOTOR_CONTROL_SPEED;
    g_input_source = MOTOR_INPUT_HOST;
    g_host_iq_limit_a = fminf(g_host_iq_limit_a, .20f);
    motor_system_reset_outer_loops();
    if (mode == 5U) {
        g_pi_pos.out_max = 30.0f;
        g_pi_pos.out_min = -30.0f;
    }
    motor_runtime_begin_calibration();
    g_host_speed_slew_rpm_per_s = 30.0f;
    g_host_speed_target_rpm = g_host_speed_applied_rpm = 0.0f;
    g_host_iq_applied_a = 0.0f;
    g_host_last_heartbeat_ms = HAL_GetTick();
    motor_cogging_runtime_start();
    g_run_requested = 1U;
    ++g_config_revision;
}

// 检查启动条件后在原临界区建立会话、保存配置并请求运行。
uint8_t motor_system_host_start_calibration(uint8_t mode, uint16_t point_count, uint8_t repeats,
                                            uint8_t retry_until_good, uint8_t policy)
{
    uint8_t result = calibration_session_validate(mode);
    if (result != MOTOR_CMD_OK) {
        return result;
    }
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!motor_calibration_begin_with_policy(mode, point_count, repeats, retry_until_good, policy)) {
        if (!mask) {
            __enable_irq();
        }
        return MOTOR_CMD_BUSY;
    }
    calibration_session_save_config();
    calibration_session_configure(mode);
    if (!mask) {
        __enable_irq();
    }
    return MOTOR_CMD_OK;
}
