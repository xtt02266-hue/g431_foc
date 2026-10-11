#include "motor_context.h"

/* 保持原静态初始化：共享变量只有一个定义，初始化复位顺序由 motor_control_init 管理。 */
motor_system_t g_motor_system = {
    .state = MOTOR_STATE_WAIT_PARAMETERS,
};
volatile uint8_t g_run_requested = 0U;
volatile uint8_t g_fault_latched = 0U;
volatile motor_control_mode_t g_control_mode = MOTOR_DEFAULT_CONTROL_MODE;
volatile float g_torque_current_a = 0.0f;
volatile motor_input_source_t g_input_source = MOTOR_INPUT_POT;
volatile motor_control_owner_t g_control_owner = MOTOR_OWNER_LOCAL;
volatile uint32_t g_host_last_heartbeat_ms = 0U;
volatile uint32_t g_config_revision = 0U;
// 主机启动强拖，TIM2 更新电角度 rad 与剩余对齐周期数；停机统一清除。
volatile uint8_t g_force_drag_active = 0U;
float g_force_drag_phase_rad = 0.0f;
uint16_t g_force_drag_align_ticks = 0U;
volatile float g_host_iq_limit_a = MOTOR_HOST_DEFAULT_IQ_LIMIT_A;
float g_host_iq_applied_a = 0.0f;
float g_host_iq_slew_a_per_s = MOTOR_HOST_IQ_SLEW_A_PER_S;
float g_host_speed_target_rpm = 0.0f;
float g_host_speed_applied_rpm = 0.0f;
float g_host_speed_slew_rpm_per_s = MOTOR_HOST_SPEED_SLEW_RPM_PER_S;
// 控制周期发布的观测量：电流 A、位置计数、角度 rad、目标速度 rpm；前台沿用原读取顺序。
float g_debug_speed_loop_iq_a = 0.0f;
float g_debug_position_target_counts = 0.0f;
float g_debug_position_actual_counts = 0.0f;
float g_debug_position_error_counts = 0.0f;
float g_debug_position_target_speed_rpm = 0.0f;
float g_debug_friction_iq_a = 0.0f;
float g_debug_cogging_iq_a = 0.0f;
float g_continuous_angle_rad = 0.0f;
float g_haptic_center_rad = 0.0f;
uint8_t g_angle_initialized = 0U;
motor_haptic_params_t g_haptic_params = {0.03f, 0.002f, 0.05f, 0.05f, 90.0f, 24U};
// 标定会话与控制周期共享的恢复标记、摩擦配置和传感器错误计数；保持原临界区。
uint8_t g_cal_saved_valid;
motor_friction_config_t g_cal_saved_friction;
uint16_t g_cal_control_flags;
uint32_t g_cal_last_crc, g_cal_last_transfer;
uint8_t g_pot_filter_initialized = 0U;
