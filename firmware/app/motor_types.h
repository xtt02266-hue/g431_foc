#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <stdint.h>

// 电机运行状态。
typedef enum {
    MOTOR_STATE_STOPPED = 0,
    MOTOR_STATE_WAIT_PARAMETERS,
    MOTOR_STATE_IDENTIFYING,
    MOTOR_STATE_SENSORED_RUN,
    MOTOR_STATE_MUSIC,
    MOTOR_STATE_FAULT
} motor_state_t;

/* 控制模式决定 Iq 由谁生成，与运行/停机/故障状态分开管理。
 * 音乐播放期间暂停所选模式；保留位置模式供后续主动切换使用。
 */
typedef enum {
    MOTOR_CONTROL_POSITION = 0, // 电位器给位置，经位置环和速度环生成 Iq
    MOTOR_CONTROL_TORQUE,
    MOTOR_CONTROL_FREE,
    MOTOR_CONTROL_DAMPING,
    MOTOR_CONTROL_SPRING,
    MOTOR_CONTROL_DETENT,
    MOTOR_CONTROL_LIMIT,
    MOTOR_CONTROL_SPEED
} motor_control_mode_t;

typedef enum {
    MOTOR_INPUT_POT = 0,
    MOTOR_INPUT_HOST,
    MOTOR_INPUT_INTERNAL
} motor_input_source_t;

typedef enum {
    MOTOR_OWNER_LOCAL = 0,
    MOTOR_OWNER_HOST
} motor_control_owner_t;

typedef enum {
    MOTOR_CMD_OK = 0,                  // 已接受
    MOTOR_CMD_INVALID_LENGTH = 1,      // 协议载荷长度错误
    MOTOR_CMD_INVALID_VALUE = 2,       // 参数数值或表数据非法
    MOTOR_CMD_UNSUPPORTED = 3,         // 不支持的命令
    MOTOR_CMD_NOT_OWNER = 4,           // 主机尚未取得控制权
    MOTOR_CMD_MUST_STOP_FIRST = 5,     // 需要先停机
    MOTOR_CMD_NOT_READY = 6,           // 参数/传感器/持久化未就绪
    MOTOR_CMD_FAULT_ACTIVE = 7,        // 故障仍锁存
    MOTOR_CMD_BUSY = 8,                // 辨识/标定/事务占用
    MOTOR_CMD_NOT_RUNNING = 9,         // 所需闭环尚未运行
    MOTOR_CMD_INVALID_COMBINATION = 10 // 模式和给定来源不匹配
} motor_command_result_t;

typedef struct {
    float spring_k_a_per_rad;       // 弹簧基础刚度 A/rad，较大偏转按触觉曲线渐硬
    float damping_b_a_per_rad_s;    // 基础阻尼 A/(rad/s)，三种位置触觉另按刚度增加阻尼
    float detent_k_a_per_rad;       // 档心基础刚度 A/rad，档缘渐硬并使用换档迟滞
    float limit_k_a_per_rad;        // 软墙基础刚度 A/rad，越界后渐硬
    float limit_half_range_deg;    // 启动中心两侧的可转范围，单位 deg
    uint16_t detent_count;         // 每机械圈档数
} motor_haptic_params_t;

typedef struct {
    motor_state_t state;
    motor_control_mode_t mode;
    motor_input_source_t source;
    motor_control_owner_t owner;
    uint8_t run_requested;
    float iq_limit_a;
    float continuous_angle_rad;
    float relative_center_angle_rad;
    float target_speed_rpm;
    float speed_loop_iq_a;
    float position_target_counts;
    float position_actual_counts;
    float position_error_counts;
    float position_target_speed_rpm;
    float friction_iq_a;
    float cogging_iq_a;
    float cogging_effective_gain;
    uint32_t cogging_table_revision;
    uint32_t cogging_table_crc;
    uint32_t config_revision;
} motor_control_snapshot_t;

#define MOTOR_DEFAULT_CONTROL_MODE MOTOR_CONTROL_TORQUE // 初始化后默认绕过位置/速度环

// 电机实时运行数据
typedef struct {
    float speed_rpm;        // 实时转速 (RPM)
    uint16_t pot_raw;       // 电位器原始数值 (0-4095)
    float target_torque_nm; // 基础命令对应的目标电磁力矩（不含补偿），非实测轴端力矩
} motor_run_data_t;

// 电机系统关键变量。
typedef struct {
    motor_state_t state;
    motor_run_data_t run_data; // 实时的运行状态数据 (转速、电位器值等)
} motor_system_t;

// 电机系统全局实例。
extern motor_system_t g_motor_system;

#endif
