#ifndef MOTOR_COGGING_H
#define MOTOR_COGGING_H

#include <stdint.h>

#define MOTOR_COGGING_TABLE_SIZE       512U    // 一圈齿槽补偿表点数；匹配 15 位角计数插值
#define MOTOR_COGGING_CHUNK_MAX_POINTS 120U    // 单次协议传输点数上限，单位点
#define MOTOR_COGGING_TABLE_SCALE_A    0.0001f // 表中有符号计数到 Iq 的缩放系数，单位 A/计数
#define MOTOR_COGGING_TABLE_MAX_COUNTS 5000    // 单点原始表计数幅值上限
#define MOTOR_COGGING_RAMP_TIME_MS     500U    // 运行时补偿增益由零渐入的时长，单位 ms

typedef struct {
    uint8_t enabled;
    float gain;
    float max_iq_a;
    float fade_start_rpm;
    float fade_end_rpm;
    uint16_t phase_offset_counts;
} motor_cogging_config_t;

// 初始化活动/暂存表和事务状态，再按原规则加载 Flash 记录。
void motor_cogging_init(void);
// 检查增益、电流限幅、速度衰减区间和编码器相位偏移；无效配置返回 0。
uint8_t motor_cogging_is_config_valid(const motor_cogging_config_t *config);
// 接受有效配置并更新补偿参数；电流单位 A，速度单位 rpm，相位单位计数。
void motor_cogging_set_config(uint8_t enabled, float gain, float max_iq_a, float fade_start_rpm, float fade_end_rpm,
                              uint16_t phase_offset_counts);
// 复制当前配置供诊断或协议读取；不改变活动表。
void motor_cogging_get_config(motor_cogging_config_t *config);

// 开始完整表上传事务；点数必须匹配固定表容量，失败返回 0。
uint8_t motor_cogging_table_begin(uint16_t count, uint32_t expected_crc);
// 读取最近成功分配的事务号；前台开始事务时更新，超时任务不覆盖。
uint16_t motor_cogging_get_transaction_id(void);
// 按事务号、偏移和点数写暂存表；记录收齐位图，活动表仍可读取。
uint8_t motor_cogging_table_write_chunk(uint16_t transaction_id, uint16_t offset, uint8_t count, const int16_t *values);
// 检查事务、收齐位图和 CRC 后交换活动/暂存表，保持原有提交边界。
uint8_t motor_cogging_table_commit(uint16_t transaction_id);
// 按偏移复制活动表的固定容量片段；输出计数乘缩放系数后为 A。
uint8_t motor_cogging_table_read_chunk(uint16_t offset, uint8_t count, int16_t *values);
// 擦写专属 Flash 页并回读校验；调用方负责停机条件与中断保护。
uint8_t motor_cogging_save_to_flash(void);

// 复位运行补偿增益渐入历史，避免沿用上次运行的渐入位置。
void motor_cogging_runtime_start(void);
// 停机时清零有效补偿增益及渐入计时。
void motor_cogging_runtime_stop(void);
// 1 ms 更新补偿渐入和上传事务超时；不改变上传点数或电流边界。
void motor_cogging_task1ms(uint8_t applicable_mode_running);
// 按机械角计数插值表值，叠加速度衰减与限幅后输出电磁方向 Iq（A）。
float motor_cogging_compensation(uint16_t mechanical_angle_counts, float mechanical_speed_rpm);

// 按反射 CRC-32 多项式计算表或记录校验值；输入是明确长度的字节范围。
uint32_t motor_cogging_crc32(const void *data, uint32_t length);
// 读取已提交活动表 CRC；不触发重新计算。
uint32_t motor_cogging_get_active_table_crc(void);
// 读取活动表修订号，供状态上报和版本一致性检查。
uint32_t motor_cogging_get_table_revision(void);
// 读取已应用渐入后的补偿增益；不等同于配置增益。
float motor_cogging_get_effective_gain(void);
// 查询当前活动表是否已按原规则保存到 Flash。
uint8_t motor_cogging_is_persisted(void);

#endif
