#ifndef MOTOR_SYSTEM_H
#define MOTOR_SYSTEM_H

#include <stdint.h>
#include "motor_music.h"
#include "motor_config.h"
#include "motor_feedforward.h"
#include "motor_cogging.h"

/* 控制命令使用基础标量；状态码仍为 MOTOR_CMD_* 的原数值。
 * 模式原始数值通过范围/组合校验后才写入内部枚举。
 * 已接受的数值和事务号通过独立标量读取接口获取，失败时不更新记录。
 * 配置读取和诊断快照保留结构体输出，沿用原同步方式，不承诺新增原子性。
 * 表片段是带点数的协议数据边界，保留缓冲区接口。
 */

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
    float spring_k_a_per_rad;
    float damping_b_a_per_rad_s;
    float detent_k_a_per_rad;
    float limit_k_a_per_rad;
    float limit_half_range_deg;
    uint16_t detent_count;
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
    float target_torque_nm; // 限流后的目标电磁力矩，非实测轴端力矩
} motor_run_data_t;

// 电机系统关键变量。
typedef struct {
    motor_state_t state;
    motor_run_data_t run_data; // 实时的运行状态数据 (转速、电位器值等)
} motor_system_t;

// 电机系统全局实例。
extern motor_system_t g_motor_system;

// 初始化电机系统。
void motor_system_init(void);
// 周期任务入口。
void motor_system_task(void);
// 以 uint8_t 读取系统运行状态，数值对应 MOTOR_STATE_*。
uint8_t motor_system_get_state(void);

/* 系统级功能入口：负责检查参数、传感器和当前运行状态。 */
uint8_t motor_system_start_control(void);
// 在恢复原中断状态的临界区内撤销运行、归零输出并恢复标定前配置。
void motor_system_stop_control(void);
/* 选择既有本地控制模式；仅在模式改变时清零 Iq 和外环历史，不自动启动。
 * 返回 1 表示模式有效并已接受，0 表示数值或当前状态不允许；重复设置相同模式不清零。
 */
uint8_t motor_system_set_control_mode(uint8_t mode);
/* 查询所选模式，供业务逻辑/调试使用；返回值不表示电机已使能或正在输出。 */
uint8_t motor_system_get_control_mode(void);
// 读取当前给定来源；不切换控制权或运行状态。
uint8_t motor_system_get_input_source(void);
// 读取当前本地/主机控制权，供协议令牌校验使用。
uint8_t motor_system_get_control_owner(void);
// 在允许状态下交接主机控制权并初始化心跳和限幅；返回 MOTOR_CMD_*。
uint8_t motor_system_claim_host(void);
// 先停机再恢复本地力矩/电位器控制权，清除主机给定历史。
void motor_system_release_host(void);
// 主机拥有控制权时更新 HAL 毫秒心跳，不改变本地控制状态。
void motor_system_host_heartbeat(void);
// 检查主机控制权、参数和方向后请求运行，并清除上次主机目标。
uint8_t motor_system_host_start(void);
/* 诊断模式：先对齐转子，再将开环磁场渐升至 250 rpm。 */
uint8_t motor_system_host_start_force_drag250(void);
// 停机状态下先保存禁用齿槽配置，再请求强拖并启动编码器自校准。
uint8_t motor_system_host_start_encoder_calibration(void);
// 主机停机时校验并切换模式/来源；保持错误码和外环复位规则。
uint8_t motor_system_host_set_mode(uint8_t mode, uint8_t source);
// 主机力矩运行时设置 Iq（A）及斜率 A/s；返回 MOTOR_CMD_*。
uint8_t motor_system_host_set_iq(float iq_a, float slew_a_per_s);
// 读取上一次成功命令接受的 Iq（A）；前台命令独占更新，停机 ISR 不覆盖。
float motor_system_get_last_accepted_iq(void);
// 主机停机时设置 Iq 限幅 A；保留原有数值边界与配置修订号。
uint8_t motor_system_host_set_iq_limit(float limit_a);
// 主机停机时校验触觉参数；刚度单位 A/rad、阻尼 A/(rad/s)、限位角度 deg。
uint8_t motor_system_host_set_haptic_params(float spring_k_a_per_rad, float damping_b_a_per_rad_s,
                                            float detent_k_a_per_rad, float limit_k_a_per_rad,
                                            float limit_half_range_deg, uint16_t detent_count);
// 主机停机时校验并采用摩擦配置；不触发 Flash 保存。
uint8_t motor_system_host_set_friction_config(uint8_t enabled, float coulomb_iq_a, float viscous_iq_a_per_rpm,
                                              float smooth_speed_rpm, float max_iq_a);
// 更新速度 PI；运行时保持输出连续性，停机时按原规则复位积分。
uint8_t motor_system_host_set_speed_pi(float kp, float ki);
// 主机停机且采集空闲时保存摩擦配置；保持错误返回与更新次序。
uint8_t motor_system_host_save_friction_config(uint8_t enabled, float coulomb_iq_a, float viscous_iq_a_per_rpm,
                                               float smooth_speed_rpm, float max_iq_a);
// 主机停机时校验并采用齿槽配置，保持现有电流/增益边界。
uint8_t motor_system_host_set_cogging_config(uint8_t enabled, float gain, float max_iq_a, float fade_start_rpm,
                                             float fade_end_rpm, uint16_t phase_offset_counts);
// 主机停机时开始固定容量表事务；成功后从 motor_cogging_get_transaction_id() 读取事务号。
uint8_t motor_system_host_begin_cogging_table(uint16_t count, uint32_t expected_crc);
// 校验主机控制权和停机状态后写入带长度的表片段；不提交活动表。
uint8_t motor_system_host_write_cogging_chunk(uint16_t transaction_id, uint16_t offset, uint8_t count,
                                              const int16_t *values);
// 主机停机时提交收齐且 CRC 正确的表，并更新配置修订号。
uint8_t motor_system_host_commit_cogging_table(uint16_t transaction_id);
// 主机取得控制权后读取活动表片段；保持允许读取的状态规则。
uint8_t motor_system_host_read_cogging_chunk(uint16_t offset, uint8_t count, int16_t *values);
// 主机停机时保存活动齿槽表；返回保存结果对应的既有状态码。
uint8_t motor_system_host_save_cogging(void);
// 主机速度运行时设置 rpm 及 rpm/s 斜率；零速/换向按原规则复位速度积分。
uint8_t motor_system_host_set_speed(float speed_rpm, float slew_rpm_per_s);
// 读取上一次成功命令接受的 rpm；前台命令独占更新，供原协议应答使用。
float motor_system_get_last_accepted_speed(void);
// 复制当前触觉配置供协议读取；空指针沿用原有跳过行为。
void motor_system_get_haptic_params(motor_haptic_params_t *params);
// 读取摩擦配置，保持下层复制接口与空指针处理规则。
void motor_system_get_friction_config(motor_friction_config_t *config);
// 复制当前齿槽补偿配置用于诊断与协议应答。
void motor_system_get_cogging_config(motor_cogging_config_t *config);
// 按原读取顺序复制状态与观测量；保留既有同步方式和字段布局。
void motor_system_get_control_snapshot(motor_control_snapshot_t *snapshot);
// 检查主机控制权和启动条件，保存原配置后启动指定采集；返回 MOTOR_CMD_*。
uint8_t motor_system_host_start_calibration(uint8_t mode, uint16_t point_count, uint8_t repeats,
                                            uint8_t retry_until_good, uint8_t policy);
// 通过统一停机路径取消标定，保持输出撤销与配置恢复次序。
void motor_system_abort_calibration(void);
/* 手动给定仅在 HOST 来源且 TORQUE 正常运行时接受；POT 来源返回 0。
 * 直接给定电磁方向 Iq (A)，Id=0。
 * 有感运行且已选择 TORQUE 时可设置，限幅至 +/- MOTOR_TORQUE_CURRENT_LIMIT_A。
 * 非有限值被拒绝并清零目标。手动模式停机/故障/切换后需重新给定。
 * 电位器模式在恢复运行后自动采用当前旋钮位置。
 * 返回 1 表示接受（超限值会被限幅），0 表示拒绝并清零存储的手动目标。
 */
uint8_t motor_system_set_torque_current(float iq_a);
/* 手动力矩入口，参数单位 N·m：用 Iq=T/Kt 转换后交给电流给定接口检查。
 * Kt 未配置或输入非有限时清零并返回 0；正负沿用 Iq 电磁方向。
 * 返回值及运行条件与 SetTorqueCurrent 相同，POT 来源不接受手动给定。
 */
uint8_t motor_system_set_torque_nm(float torque_nm);
/* 传感器恢复后需主动清故障，防止电机突然自动恢复出力。 */
uint8_t motor_system_clear_fault(void);
/* 主动辨识并保存；必须确认电机无负载且可以自由转动。 */
uint8_t motor_system_identify_and_save(void);
/* 按 motor_songs.h 的曲目 ID 播放有限次数；外部曲目先登记，音乐期间暂停正常控制。 */
uint8_t motor_system_play_song(uint8_t song_id, uint16_t play_count);
// 请求循环播放曲目，沿用音乐期间暂停正常闭环控制的规则。
uint8_t motor_system_play_song_loop(uint8_t song_id);
// 撤销音乐播放请求；系统状态仍由原定时更新路径处理。
void motor_system_stop_music(void);
#endif
