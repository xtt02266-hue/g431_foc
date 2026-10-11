#ifndef MOTOR_CONTEXT_H
#define MOTOR_CONTEXT_H

#include "motor_types.h"
#include "motor_friction.h"
#include "motor_cogging.h"
#include "motor_config.h"

/* 仅应用内部可见：前台命令与 TIM2 共用的控制状态。
 * 单次拆分保留原变量类型、初始化值与临界区；各模块不得自行增加异步写入。
 * 生命周期负责运行/故障，主机模块负责权限和给定，控制周期发布观测，标定保存恢复配置。
 */
#define MOTOR_ENCODER_MAX_SAMPLE_AGE_MS  2U      // 控制路径允许的有效编码器样本最大年龄，单位 ms
#define MOTOR_CURRENT_LOOP_CONTROL_BW_HZ 75.0f   // 正常控制时电流 PI 调谐带宽，单位 Hz
#define MOTOR_CURRENT_LOOP_MUSIC_BW_HZ   2000.0f // 音乐模式电流 PI 调谐带宽，单位 Hz

// 电位器低通滤波系数 (一阶 EMA, 1kHz 更新率)
// α 越小滤波越强、响应越慢; α=1.0 则无滤波
// α=0.10f → 截止频率 ~16Hz, 适合人手旋转电位器
// α=0.05f → 截止频率 ~8Hz,  更平滑但略有滞后感
#define POT_LPF_ALPHA 0.10f

extern motor_system_t g_motor_system;
extern volatile uint8_t g_run_requested;
extern volatile uint8_t g_fault_latched;
extern volatile motor_control_mode_t g_control_mode;
extern volatile float g_torque_current_a;
extern volatile motor_input_source_t g_input_source;
extern volatile motor_control_owner_t g_control_owner;
extern volatile uint32_t g_host_last_heartbeat_ms;
extern volatile uint32_t g_config_revision;
extern volatile uint8_t g_force_drag_active;
extern float g_force_drag_phase_rad;
extern uint16_t g_force_drag_align_ticks;
extern volatile float g_host_iq_limit_a;
extern float g_host_iq_applied_a;
extern float g_host_iq_slew_a_per_s;
extern float g_host_speed_target_rpm;
extern float g_host_speed_applied_rpm;
extern float g_host_speed_slew_rpm_per_s;
extern float g_debug_speed_loop_iq_a;
extern float g_debug_position_target_counts;
extern float g_debug_position_actual_counts;
extern float g_debug_position_error_counts;
extern float g_debug_position_target_speed_rpm;
extern float g_debug_friction_iq_a;
extern float g_debug_cogging_iq_a;
extern float g_continuous_angle_rad;
extern float g_haptic_center_rad;
extern uint8_t g_angle_initialized;
extern motor_haptic_params_t g_haptic_params;
extern uint8_t g_cal_saved_valid;
extern motor_friction_config_t g_cal_saved_friction;
extern uint16_t g_cal_control_flags;
extern uint32_t g_cal_last_crc, g_cal_last_transfer;
extern uint8_t g_pot_filter_initialized;

/* 应用内部的状态更新和模式计算接口；不得由协议直接绕过命令校验调用。 */
void motor_system_restore_calibration(void);
float motor_abs_float(float value);
void motor_control_init(void);
float motor_system_torque_to_iq(float torque_nm);
void motor_system_reset_control_history(void);
void motor_system_reset_outer_loops(void);
void motor_system_tune_current_loop_bandwidth(float bandwidth_hz);
void motor_system_force_safe_stop(void);
void motor_control_update_state(void);
float motor_system_update_pot_target(void);
void motor_system_run_torque_mode(void);
void motor_system_run_position_mode(float target_pos, float current_rpm);
float motor_system_slew_host_iq(float target_iq, float slew_a_per_s);
void motor_system_run_speed_mode(float mechanical_speed_rpm, int8_t identified_direction,
                                 uint16_t mechanical_angle_counts);
void motor_control_task(void);
// 计算触觉模式的机械模型与补偿，并发布限幅后的 q/d 给定。
void motor_haptic_task(int8_t identified_direction, uint16_t angle_counts);
#endif
