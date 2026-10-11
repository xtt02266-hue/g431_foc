#ifndef MOTOR_HOST_H
#define MOTOR_HOST_H

#include "motor_types.h"
#include "motor_friction.h"
#include "motor_cogging.h"

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
#endif
