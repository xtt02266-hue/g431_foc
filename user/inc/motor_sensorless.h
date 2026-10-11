#ifndef MOTOR_SENSORLESS_H
#define MOTOR_SENSORLESS_H

#include <stdint.h>

#define MOTOR_SENSORLESS_SAMPLE_TIME_SEC 0.00005f // 观测器默认采样周期，单位 s
#define MOTOR_SENSORLESS_DEFAULT_ENABLE  0U       // 观测器默认关闭，保持当前启动条件

typedef enum {
    MOTOR_SENSORLESS_DISABLED = 0,
    MOTOR_SENSORLESS_SEARCHING,
    MOTOR_SENSORLESS_TRACKING,
    MOTOR_SENSORLESS_LOST
} motor_sensorless_status_t;

typedef struct {
    float sample_time_sec;
    float resistance_ohm;
    float inductance_h;
    uint16_t pole_pairs;

    float bemf_filter_alpha;
    float minimum_bemf_volts;
    float pll_kp;
    float pll_ki;
    float maximum_electrical_speed_rad_s;
    float lock_phase_error;

    uint16_t lock_updates;
    uint16_t loss_updates;
    uint8_t pll_divider;
} motor_sensorless_config_t;

typedef struct {
    float electrical_angle_rad;
    float electrical_speed_rad_s;
    float mechanical_speed_rpm;
    float bemf_alpha_volts;
    float bemf_beta_volts;
    float bemf_magnitude_volts;
    float pll_phase_error;
    motor_sensorless_status_t status;
    uint8_t valid;
} motor_sensorless_output_t;

// 装入默认配置并复位观测器；沿用板级默认使能设置。
void motor_sensorless_init(void);
// 输出默认观测器配置；采样周期单位 s，R/L 分别为 Ω/H。
void motor_sensorless_get_default_config(motor_sensorless_config_t *config);
// 校验并采用观测器配置，保持原有范围检查和复位规则。
uint8_t motor_sensorless_configure(float sample_time_sec, float resistance_ohm, float inductance_h, uint16_t pole_pairs,
                                   float bemf_filter_alpha, float minimum_bemf_volts, float pll_kp, float pll_ki,
                                   float maximum_electrical_speed_rad_s, float lock_phase_error, uint16_t lock_updates,
                                   uint16_t loss_updates, uint8_t pll_divider);
// 更新电机 R（Ω）、L（H）和极对数，用于反电势估计。
uint8_t motor_sensorless_configure_motor(float resistance_ohm, float inductance_h, uint16_t pole_pairs);
// 更新观测器使能状态并按既有规则复位历史。
void motor_sensorless_enable(uint8_t enable);
// 读取观测器使能状态，不表示估计角度已锁定有效。
uint8_t motor_sensorless_is_enabled(void);
// 主动复位观测器历史和输出有效性。
void motor_sensorless_reset(void);

/* 方向参数为预期电角度方向：+1 或 -1。 */
void motor_sensorless_set_direction(int8_t direction);

/* 观测器设计由 20 kHz 电流路径调用；禁用时沿用空操作语义。 */
void motor_sensorless_update(float voltage_alpha, float voltage_beta, float current_alpha, float current_beta);

// 按既有接口返回最新估计结果；保持结构体返回 ABI 与有效性字段。
motor_sensorless_output_t motor_sensorless_get_output(void);
// 读取最近估计电角度 rad；有效性由独立接口判断。
float motor_sensorless_get_electrical_angle(void);
// 读取 PLL 推算的机械转速 rpm，不作为编码器实测值。
float motor_sensorless_get_mechanical_speed_rpm(void);
// 查询观测器当前输出有效性。
uint8_t motor_sensorless_is_valid(void);
// 按锁定历史与失锁规则判断是否满足既有切换条件。
uint8_t motor_sensorless_is_ready_for_handover(void);

#endif
