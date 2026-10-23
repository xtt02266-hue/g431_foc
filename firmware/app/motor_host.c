#include "motor_host.h"
#include "motor_control.h"
#include "motor_context.h"
#include "motor_current_loop.h"
#include "motor_speed_loop.h"
#include "motor_encoder.h"
#include "motor_identify.h"
#include "motor_music.h"
#include "motor_parameters.h"
#include "motor_calibration.h"
#include "motor_friction_storage.h"
#include "motor_encoder_calibration.h"
#include "mt6826s.h"
#include <math.h>
#include <stddef.h>

/* 前台仅在命令成功时更新应答缓存；ISR 停机清目标时不覆盖已接受值，单位 A/rpm。 */
static float s_last_accepted_iq_a;
static float s_last_accepted_speed_rpm;
static uint8_t motor_system_is_mode_source_valid(uint8_t mode, uint8_t source);

// 读取当前给定来源；不切换控制权或运行状态。
uint8_t motor_system_get_input_source(void)
{
    return g_input_source;
}

// 读取当前本地/主机控制权，供协议令牌校验使用。
uint8_t motor_system_get_control_owner(void)
{
    return g_control_owner;
}

// 检查控制模式与给定来源的合法组合，沿用原有模式范围。
static uint8_t motor_system_is_mode_source_valid(uint8_t mode, uint8_t source)
{
    if (mode == MOTOR_CONTROL_POSITION) {
        return source == MOTOR_INPUT_POT;
    }
    if (mode == MOTOR_CONTROL_TORQUE) {
        return (source == MOTOR_INPUT_POT) || (source == MOTOR_INPUT_HOST);
    }
    if (mode == MOTOR_CONTROL_SPEED) {
        return source == MOTOR_INPUT_HOST;
    }
    return ((mode >= MOTOR_CONTROL_FREE) && (mode <= MOTOR_CONTROL_LIMIT) && (source == MOTOR_INPUT_INTERNAL));
}

// 在允许状态下交接主机控制权并初始化心跳和限幅；返回 MOTOR_CMD_*。
uint8_t motor_system_claim_host(void)
{
    if ((g_motor_system.state == MOTOR_STATE_IDENTIFYING) || (g_motor_system.state == MOTOR_STATE_MUSIC)) {
        return MOTOR_CMD_BUSY;
    }
    if (g_motor_system.state == MOTOR_STATE_FAULT) {
        return MOTOR_CMD_FAULT_ACTIVE;
    }
    if ((g_motor_system.state != MOTOR_STATE_STOPPED) && (g_motor_system.state != MOTOR_STATE_WAIT_PARAMETERS)) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (g_control_owner == MOTOR_OWNER_HOST) {
        return MOTOR_CMD_BUSY;
    }
    g_control_owner = MOTOR_OWNER_HOST;
    g_host_iq_limit_a = MOTOR_HOST_DEFAULT_IQ_LIMIT_A;
    g_host_iq_slew_a_per_s = MOTOR_HOST_IQ_SLEW_A_PER_S;
    g_host_last_heartbeat_ms = HAL_GetTick();
    return MOTOR_CMD_OK;
}

// 先停机再恢复本地力矩/电位器控制权，清除主机给定历史。
void motor_system_release_host(void)
{
    motor_system_stop_control();
    g_control_owner = MOTOR_OWNER_LOCAL;
    g_control_mode = MOTOR_CONTROL_TORQUE;
    g_input_source = MOTOR_INPUT_POT;
    g_host_iq_limit_a = MOTOR_HOST_DEFAULT_IQ_LIMIT_A;
    g_torque_current_a = 0.0f;
    g_host_iq_applied_a = 0.0f;
    g_host_iq_slew_a_per_s = MOTOR_HOST_IQ_SLEW_A_PER_S;
    g_host_speed_target_rpm = 0.0f;
    g_host_speed_applied_rpm = 0.0f;
    ++g_config_revision;
}

// 主机拥有控制权时更新 HAL 毫秒心跳，不改变本地控制状态。
void motor_system_host_heartbeat(void)
{
    if (g_control_owner == MOTOR_OWNER_HOST) {
        g_host_last_heartbeat_ms = HAL_GetTick();
    }
}

// 检查主机控制权、参数和方向后请求运行，并清除上次主机目标。
uint8_t motor_system_host_start(void)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state == MOTOR_STATE_FAULT) {
        return MOTOR_CMD_FAULT_ACTIVE;
    }
    if (motor_parameters_is_busy() != 0U) {
        return MOTOR_CMD_BUSY;
    }
    // 重复 START 不重新捕获触觉中心或清除正在运行的电流历史。
    if (g_run_requested != 0U) {
        return MOTOR_CMD_OK;
    }
    if ((g_control_mode >= MOTOR_CONTROL_FREE) && (g_control_mode <= MOTOR_CONTROL_SPEED)) {
        int8_t direction = motor_identify_get_result().uvw_dir;
        if ((direction != 1) && (direction != -1)) {
            return MOTOR_CMD_NOT_READY;
        }
    }
    // 短临界区内完成启动历史，避免 TIM2 在运行请求与历史初始化之间执行。
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (motor_system_start_control() == 0U) {
        if (primask == 0U) {
            __enable_irq();
        }
        return MOTOR_CMD_NOT_READY;
    }
    g_torque_current_a = 0.0f;
    g_host_iq_applied_a = 0.0f;
    g_host_speed_target_rpm = 0.0f;
    g_host_speed_applied_rpm = 0.0f;
    motor_cogging_runtime_start();
    // 触觉中心交由首次控制周期捕获，使用该周期的最新连续机械角。
    motor_haptic_reset();
    g_host_last_heartbeat_ms = HAL_GetTick();
    if (primask == 0U) {
        __enable_irq();
    }
    return MOTOR_CMD_OK;
}

// 接受诊断强拖请求：先固定相位对齐，再按原斜率升至 250 rpm。
uint8_t motor_system_host_start_force_drag250(void)
{
    if (motor_encoder_cal_is_active()) {
        return MOTOR_CMD_BUSY;
    }
    if (motor_encoder_cal_requires_power_cycle()) {
        return MOTOR_CMD_NOT_READY;
    }
    uint32_t primask;

    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state == MOTOR_STATE_FAULT) {
        return MOTOR_CMD_FAULT_ACTIVE;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (motor_parameters_is_busy() != 0U) {
        return MOTOR_CMD_BUSY;
    }
    if ((motor_parameters_is_ready() == 0U) || (motor_encoder_is_data_fresh(MOTOR_ENCODER_MAX_SAMPLE_AGE_MS) == 0U)) {
        return MOTOR_CMD_NOT_READY;
    }

    /* 使用固定相位先把转子吸到已知位置，再缓升至 250 rpm 旋转磁场。
     * 这里故意不采用辨识出的零点、极对数和方向：GB4310 已知为 14 极对，
     * 这样强拖结果不会被待排查的编码器换相角或辨识参数污染。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    motor_system_reset_outer_loops();
    g_force_drag_phase_rad = 0.0f;
    g_force_drag_align_ticks = MOTOR_FORCE_DRAG_ALIGN_MS;
    g_force_drag_active = 1U;
    g_run_requested = 1U;
    g_host_speed_target_rpm = MOTOR_FORCE_DRAG_SPEED_RPM;
    g_host_speed_applied_rpm = 0.0f;
    g_torque_current_a = 0.0f;
    if (primask == 0U) {
        __enable_irq();
    }
    g_host_last_heartbeat_ms = HAL_GetTick();
    return MOTOR_CMD_OK;
}

// 停机状态下先保存禁用齿槽配置，再请求强拖并启动编码器自校准。
uint8_t motor_system_host_start_encoder_calibration(void)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (motor_encoder_cal_is_active() || motor_calibration_is_active() || motor_calibration_has_samples() ||
        motor_parameters_is_busy()) {
        return MOTOR_CMD_BUSY;
    }
    if (motor_encoder_cal_requires_power_cycle()) {
        return MOTOR_CMD_NOT_READY;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (!motor_parameters_is_ready() || !motor_encoder_is_data_fresh(2U) || mt6826s_get_status() != 0U) {
        return MOTOR_CMD_NOT_READY;
    }
    /* 保留表及 CRC，在修改传感器映射前先保存禁用配置，
     * 避免重新上电时旧映射对应的补偿表自动使能。 */
    motor_cogging_config_t cogging;
    motor_cogging_get_config(&cogging);
    cogging.enabled = 0U;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    motor_cogging_set_config(cogging.enabled, cogging.gain, cogging.max_iq_a, cogging.fade_start_rpm,
                             cogging.fade_end_rpm, cogging.phase_offset_counts);
    ++g_config_revision;
    uint8_t saved = motor_cogging_save_to_flash();
    if (!mask) {
        __enable_irq();
    }
    if (!saved) {
        return MOTOR_CMD_NOT_READY;
    }
    motor_command_result_t r = motor_system_host_start_force_drag250();
    if (r == MOTOR_CMD_OK) {
        motor_encoder_cal_begin();
    }
    return r;
}

// 主机停机时校验并切换模式/来源；保持错误码和外环复位规则。
uint8_t motor_system_host_set_mode(uint8_t mode, uint8_t source)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (motor_system_is_mode_source_valid(mode, source) == 0U) {
        return MOTOR_CMD_INVALID_COMBINATION;
    }
    if ((g_control_mode != mode) || (g_input_source != source)) {
        g_control_mode = (motor_control_mode_t)mode;
        g_input_source = (motor_input_source_t)source;
        g_torque_current_a = 0.0f;
        g_host_iq_applied_a = 0.0f;
        g_host_speed_target_rpm = 0.0f;
        g_host_speed_applied_rpm = 0.0f;
        motor_system_reset_outer_loops();
        ++g_config_revision;
    }
    return MOTOR_CMD_OK;
}

// 主机力矩运行时设置 Iq（A）及斜率 A/s；返回 MOTOR_CMD_*。
uint8_t motor_system_host_set_iq(float iq_a, float slew_a_per_s)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if ((g_control_mode != MOTOR_CONTROL_TORQUE) || (g_input_source != MOTOR_INPUT_HOST)) {
        return MOTOR_CMD_INVALID_COMBINATION;
    }
    if (g_motor_system.state != MOTOR_STATE_SENSORED_RUN) {
        return MOTOR_CMD_NOT_RUNNING;
    }
    if (!isfinite(iq_a) || !isfinite(slew_a_per_s) || (slew_a_per_s < 0.0f) ||
        (slew_a_per_s > MOTOR_HOST_IQ_SLEW_MAX_A_PER_S)) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    if (iq_a > g_host_iq_limit_a) {
        iq_a = g_host_iq_limit_a;
    }
    if (iq_a < -g_host_iq_limit_a) {
        iq_a = -g_host_iq_limit_a;
    }
    g_torque_current_a = iq_a;
    g_host_iq_slew_a_per_s = slew_a_per_s;
    s_last_accepted_iq_a = iq_a;
    return MOTOR_CMD_OK;
}

// 读取最近成功命令接受的 Iq，不读取可能被 ISR 清零的运行目标。
float motor_system_get_last_accepted_iq(void)
{
    return s_last_accepted_iq_a;
}

// 主机停机时设置 Iq 限幅 A；保留原有数值边界与配置修订号。
uint8_t motor_system_host_set_iq_limit(float limit_a)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (!isfinite(limit_a) || (limit_a < 0.02f) || (limit_a > MOTOR_TORQUE_CURRENT_LIMIT_A)) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    g_host_iq_limit_a = limit_a;
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

// 主机停机时校验触觉参数；刚度单位 A/rad、阻尼 A/(rad/s)、限位角度 deg。
uint8_t motor_system_host_set_haptic_params(float spring_k_a_per_rad, float damping_b_a_per_rad_s,
                                            float detent_k_a_per_rad, float limit_k_a_per_rad,
                                            float limit_half_range_deg, uint16_t detent_count)
{
    const motor_haptic_params_t p = {spring_k_a_per_rad, damping_b_a_per_rad_s, detent_k_a_per_rad,
                                     limit_k_a_per_rad,  limit_half_range_deg,  detent_count};

    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (!isfinite(p.spring_k_a_per_rad) || !isfinite(p.damping_b_a_per_rad_s) || !isfinite(p.detent_k_a_per_rad) ||
        !isfinite(p.limit_k_a_per_rad) || !isfinite(p.limit_half_range_deg) || (p.spring_k_a_per_rad < 0.0f) ||
        (p.spring_k_a_per_rad > 5.0f) || (p.damping_b_a_per_rad_s < 0.0f) || (p.damping_b_a_per_rad_s > 1.0f) ||
        (p.detent_k_a_per_rad < 0.0f) || (p.detent_k_a_per_rad > 5.0f) || (p.limit_k_a_per_rad < 0.0f) ||
        (p.limit_k_a_per_rad > 5.0f) || (p.limit_half_range_deg < 5.0f) || (p.limit_half_range_deg > 180.0f) ||
        (p.detent_count < 1U) || (p.detent_count > 128U)) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    g_haptic_params = p;
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

// 复制当前触觉配置供协议读取；空指针沿用原有跳过行为。
void motor_system_get_haptic_params(motor_haptic_params_t *p)
{
    if (p != NULL) {
        *p = g_haptic_params;
    }
}

// 主机停机时校验并采用摩擦配置；不触发 Flash 保存。
uint8_t motor_system_host_set_friction_config(uint8_t enabled, float coulomb_iq_a, float viscous_iq_a_per_rpm,
                                              float smooth_speed_rpm, float max_iq_a)
{
    const motor_friction_config_t p = {enabled, coulomb_iq_a, viscous_iq_a_per_rpm, smooth_speed_rpm, max_iq_a};

    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (motor_feedforward_is_friction_config_valid(&p) == 0U) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    motor_feedforward_set_friction_config(p.enabled, p.coulomb_iq_a, p.viscous_iq_a_per_rpm, p.smooth_speed_rpm,
                                          p.max_iq_a);
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

// 读取摩擦配置，保持下层复制接口与空指针处理规则。
void motor_system_get_friction_config(motor_friction_config_t *p)
{
    motor_feedforward_get_friction_config(p);
}

// 主机停机且采集空闲时保存摩擦配置；保持错误返回与更新次序。
uint8_t motor_system_host_save_friction_config(uint8_t enabled, float coulomb_iq_a, float viscous_iq_a_per_rpm,
                                               float smooth_speed_rpm, float max_iq_a)
{
    const motor_friction_config_t p = {enabled, coulomb_iq_a, viscous_iq_a_per_rpm, smooth_speed_rpm, max_iq_a};

    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (motor_calibration_is_active() || motor_calibration_has_samples() || motor_parameters_is_busy()) {
        return MOTOR_CMD_BUSY;
    }
    if (!motor_feedforward_is_friction_config_valid(&p)) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    if (!motor_friction_storage_save(&p)) {
        return MOTOR_CMD_NOT_READY;
    }
    motor_feedforward_set_friction_config(p.enabled, p.coulomb_iq_a, p.viscous_iq_a_per_rpm, p.smooth_speed_rpm,
                                          p.max_iq_a);
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

// 主机停机时校验并采用齿槽配置，保持现有电流/增益边界。
uint8_t motor_system_host_set_cogging_config(uint8_t enabled, float gain, float max_iq_a, float fade_start_rpm,
                                             float fade_end_rpm, uint16_t phase_offset_counts)
{
    const motor_cogging_config_t p = {enabled, gain, max_iq_a, fade_start_rpm, fade_end_rpm, phase_offset_counts};

    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (motor_cogging_is_config_valid(&p) == 0U) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    motor_cogging_set_config(p.enabled, p.gain, p.max_iq_a, p.fade_start_rpm, p.fade_end_rpm, p.phase_offset_counts);
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

// 主机停机时开始固定容量表事务；输出事务号，错误返回 MOTOR_CMD_*。
uint8_t motor_system_host_begin_cogging_table(uint16_t count, uint32_t expected_crc)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (count != MOTOR_COGGING_TABLE_SIZE) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    if (motor_cogging_table_begin(count, expected_crc) == 0U) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    return MOTOR_CMD_OK;
}

// 校验主机控制权和停机状态后写入带长度的表片段；不提交活动表。
uint8_t motor_system_host_write_cogging_chunk(uint16_t transaction_id, uint16_t offset, uint8_t count,
                                              const int16_t *values)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    return motor_cogging_table_write_chunk(transaction_id, offset, count, values) ? MOTOR_CMD_OK
                                                                                  : MOTOR_CMD_INVALID_VALUE;
}

// 主机停机时提交收齐且 CRC 正确的表，并更新配置修订号。
uint8_t motor_system_host_commit_cogging_table(uint16_t transaction_id)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (motor_cogging_table_commit(transaction_id) == 0U) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    ++g_config_revision;
    return MOTOR_CMD_OK;
}

// 主机取得控制权后读取活动表片段；保持允许读取的状态规则。
uint8_t motor_system_host_read_cogging_chunk(uint16_t offset, uint8_t count, int16_t *values)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    return motor_cogging_table_read_chunk(offset, count, values) ? MOTOR_CMD_OK : MOTOR_CMD_INVALID_VALUE;
}

// 主机停机时保存活动齿槽表；返回保存结果对应的既有状态码。
uint8_t motor_system_host_save_cogging(void)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (g_motor_system.state != MOTOR_STATE_STOPPED) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    return motor_cogging_save_to_flash() ? MOTOR_CMD_OK : MOTOR_CMD_NOT_READY;
}

// 复制当前齿槽补偿配置用于诊断与协议应答。
void motor_system_get_cogging_config(motor_cogging_config_t *p)
{
    motor_cogging_get_config(p);
}

// 主机速度运行时设置 rpm 及 rpm/s 斜率；零速/换向按原规则复位速度积分。
uint8_t motor_system_host_set_speed(float speed_rpm, float slew_rpm_per_s)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if ((g_control_mode != MOTOR_CONTROL_SPEED) || (g_input_source != MOTOR_INPUT_HOST)) {
        return MOTOR_CMD_INVALID_COMBINATION;
    }
    if (g_motor_system.state != MOTOR_STATE_SENSORED_RUN) {
        return MOTOR_CMD_NOT_RUNNING;
    }
    if (!isfinite(speed_rpm) || !isfinite(slew_rpm_per_s) || (slew_rpm_per_s < 0.0f) ||
        (slew_rpm_per_s > MOTOR_HOST_SPEED_SLEW_MAX_RPM_PER_S)) {
        return MOTOR_CMD_INVALID_VALUE;
    }
    if (speed_rpm > MOTOR_HOST_SPEED_MAX_RPM) {
        speed_rpm = MOTOR_HOST_SPEED_MAX_RPM;
    }
    if (speed_rpm < -MOTOR_HOST_SPEED_MAX_RPM) {
        speed_rpm = -MOTOR_HOST_SPEED_MAX_RPM;
    }
    if ((speed_rpm == 0.0f) || ((g_host_speed_target_rpm * speed_rpm) < 0.0f)) {
        /* 停止或换向时清除旧方向积分，避免静摩擦挣脱后的反向冲击。 */
        pid_reset(&speed_pid);
    }
    g_host_speed_target_rpm = speed_rpm;
    g_host_speed_slew_rpm_per_s = slew_rpm_per_s;
    s_last_accepted_speed_rpm = speed_rpm;
    return MOTOR_CMD_OK;
}

// 读取最近成功命令接受的 rpm，保留协议应答与该命令的对应关系。
float motor_system_get_last_accepted_speed(void)
{
    return s_last_accepted_speed_rpm;
}

// 更新速度 PI；运行时保持输出连续性，停机时按原规则复位积分。
uint8_t motor_system_host_set_speed_pi(float kp, float ki)
{
    if (g_control_owner != MOTOR_OWNER_HOST) {
        return MOTOR_CMD_NOT_OWNER;
    }
    if (motor_calibration_is_active()) {
        return MOTOR_CMD_BUSY;
    }
    uint8_t running = g_motor_system.state == MOTOR_STATE_SENSORED_RUN;
    if (g_motor_system.state != MOTOR_STATE_STOPPED && !running) {
        return MOTOR_CMD_MUST_STOP_FIRST;
    }
    if (running && (g_control_mode != MOTOR_CONTROL_SPEED || g_input_source != MOTOR_INPUT_HOST)) {
        return MOTOR_CMD_INVALID_COMBINATION;
    }
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint8_t valid = motor_speed_loop_set_pi(kp, ki, running);
    if (valid) {
        ++g_config_revision;
    }
    if (!mask) {
        __enable_irq();
    }
    return valid ? MOTOR_CMD_OK : MOTOR_CMD_INVALID_VALUE;
}

// 按原读取顺序复制状态与观测量；保留既有同步方式和字段布局。
void motor_system_get_control_snapshot(motor_control_snapshot_t *s)
{
    if (s == NULL) {
        return;
    }
    s->state = g_motor_system.state;
    s->mode = g_control_mode;
    s->source = g_input_source;
    s->owner = g_control_owner;
    s->run_requested = g_run_requested;
    s->iq_limit_a = g_host_iq_limit_a;
    s->continuous_angle_rad = g_continuous_angle_rad;
    // 64 位运行位置与中心不能被 TIM2 拆开读取；临界区内只复制整数差。
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    int64_t relative_counts = motor_haptic_get_relative_counts();
    if (mask == 0U) {
        __enable_irq();
    }
    s->relative_center_angle_rad = ((g_control_mode >= MOTOR_CONTROL_SPRING) && (g_control_mode <= MOTOR_CONTROL_LIMIT))
                                       ? (float)relative_counts * MOTOR_ENCODER_RAD_PER_COUNT
                                       : 0.0f;
    s->target_speed_rpm = g_host_speed_applied_rpm;
    s->speed_loop_iq_a = g_debug_speed_loop_iq_a;
    s->position_target_counts = g_debug_position_target_counts;
    s->position_actual_counts = g_debug_position_actual_counts;
    s->position_error_counts = g_debug_position_error_counts;
    s->position_target_speed_rpm = g_debug_position_target_speed_rpm;
    s->friction_iq_a = g_debug_friction_iq_a;
    s->cogging_iq_a = g_debug_cogging_iq_a;
    s->cogging_effective_gain = motor_cogging_get_effective_gain();
    s->cogging_table_revision = motor_cogging_get_table_revision();
    s->cogging_table_crc = motor_cogging_get_active_table_crc();
    s->config_revision = g_config_revision;
}
