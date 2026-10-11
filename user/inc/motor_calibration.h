#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H
#include <stdint.h>

#define MOTOR_CAL_SCHEMA                 1U     // 固定标定样本协议版本，保持现有 64 字节布局
#define MOTOR_CAL_RING_SIZE              256U   /* 保留近期已发送记录，支持明确范围的重放 */
#define MOTOR_CAL_POINT_COUNT            512U   // 默认逐点网格数量；协议仍接受旧版 128/256 点请求
#define MOTOR_CAL_POINT_MAX_ERROR_COUNTS 2.0f   // 逐点位置质量门限，单位编码器计数
#define MOTOR_CAL_POINT_SETTLE_MS        1000U  // 逐点稳定等待时长，单位 ms
#define MOTOR_CAL_POINT_CAPTURE_MS       500U   // 普通逐点采集窗口，单位 ms
#define MOTOR_CAL_POINT_AVERAGE_MS       1000U  // 平均采集策略窗口，单位 ms
#define MOTOR_CAL_POINT_TIMEOUT_MS       30000U // 单点质量等待超时，单位 ms
#define MOTOR_CAL_REQUIRED_TURNS         14U    /* 包含两圈边界保护；上位机分析要求至少 12 圈 */

enum {
    CAL_IDLE,
    CAL_SETTLE,
    CAL_CAPTURE,
    CAL_ZERO,
    CAL_DONE,
    CAL_FAILED,
    CAL_ABORTED
};

enum {
    CAL_OK,
    CAL_UNSTABLE,
    CAL_STALL,
    CAL_SENSOR,
    CAL_OVERFLOW,
    CAL_CANCELLED,
    CAL_TIMEOUT,
    CAL_INSUFFICIENT,
    CAL_CONTROL_LOST
};

enum {
    CAL_LIMIT = 1U,
    CAL_SLEW = 2U,
    CAL_BAD_SENSOR = 4U,
    CAL_CONTROL_GAP = 8U,
    CAL_POINT_AVERAGE = 16U
}; /* 标记新的 1 s 平均采集窗口，不作为故障位 */

/* 样本协议版本 1：小端 IEEE 754，固定 64 字节。flags 中 bit4 为平均采集（策略 1），
 * bits8..10 为阶段、bits11..12 为速度档、bit13 为反向。
 * 逐点协议追加目标 float，单条共 68 字节；字段顺序和布局保持不变。 */
typedef struct {
    uint32_t sequence, tick_ms;
    uint16_t angle, flags;
    float target_rpm, speed_rpm, window_rpm;
    float pi_iq, friction_iq, cogging_iq, applied_iq, actual_iq, actual_id, uq;
    uint8_t sensor_status, window_ticks;
    uint16_t sensor_age_ms;
    uint32_t crc_errors, transfer_errors;
} motor_cal_sample_t;

typedef struct {
    uint32_t session, produced, dropped;
    uint8_t phase, reason, speed_index, reverse;
    uint16_t accepted_turns, rejected_turns;
    uint8_t passed_mask, mode, speed_reasons[3];
    float target_rpm;
} motor_cal_status_t;

// 使用默认网格启动指定标定模式；返回 1 表示请求已接受。
uint8_t motor_calibration_begin(uint8_t mode);
// 指定网格点数启动标定，沿用三次重复且不持续重试的默认选项。
uint8_t motor_calibration_begin_with_points(uint8_t mode, uint16_t count);
// 按网格、重复次数和重试选项启动；质量策略采用兼容默认值。
uint8_t motor_calibration_begin_with_options(uint8_t mode, uint16_t count, uint8_t repeats, uint8_t retry_until_good);
// 校验启动选项并复位采集状态；已有标定或待发送记录时拒绝请求。
uint8_t motor_calibration_begin_with_policy(uint8_t mode, uint16_t count, uint8_t repeats, uint8_t retry_until_good,
                                            uint8_t policy);
// 读取逐点质量判定策略，供上位机按协商版本解释结果。
uint8_t motor_calibration_point_policy(void);
// 解析带长度的标定启动字节；可选策略字节不改变旧请求默认行为。
uint8_t motor_calibration_decode_start_policy(const uint8_t *body, uint16_t size, uint8_t *mode, uint16_t *count,
                                              uint8_t *repeats, uint8_t *retry_until_good, uint8_t *policy);
// 读取本次逐点采集的网格点数；不改变采集状态。
uint16_t motor_calibration_point_count(void);
// 按 128/256/512 点网格返回兼容的目标数据协议版本。
uint16_t motor_calibration_point_schema(void);
// 读取当前逐点采集的重复次数。
uint8_t motor_calibration_point_repeats(void);
// 读取质量不合格时持续重试的选项。
uint8_t motor_calibration_point_retry_until_good(void);
// 解析旧版启动字节及可选网格大小；返回 1 表示字段合法。
uint8_t motor_calibration_decode_start_request(const uint8_t *body, uint16_t size, uint8_t *mode, uint16_t *count);
// 解析带长度的启动字节缓冲区；保留旧请求的默认重复次数和重试策略。
uint8_t motor_calibration_decode_start_options(const uint8_t *body, uint16_t size, uint8_t *mode, uint16_t *count,
                                               uint8_t *repeats, uint8_t *retry_until_good);
// 终止活动标定并保存原因码；待发送数据按原队列规则保留。
void motor_calibration_abort(uint8_t reason);
// 查询是否处于稳定、采集或归零阶段；结束后仍可能留有待发送记录。
uint8_t motor_calibration_is_active(void);
// 1 ms 标定状态机入口；速度单位 rpm、角度单位编码器计数，返回下一周期速度目标。
float motor_calibration_step(uint16_t angle, float speed, float applied_target, uint16_t control_flags,
                             uint8_t healthy);
// 复制当前标定状态用于协议上报；调用方须提供有效输出对象。
void motor_calibration_get_status(motor_cal_status_t *status);
// 生成当前速度阶段和方向标志，供样本与质量分析配对。
uint16_t motor_calibration_sample_flags(void);
// 采样路径向固定容量环形队列发布记录；队满按原规则中止并记录溢出。
void motor_calibration_push(const motor_cal_sample_t *sample);
// 协议前台只读复制待发送记录，不推进消费位置；返回实际复制条数。
uint8_t motor_calibration_peek(motor_cal_sample_t *samples, uint8_t maximum);
// 前台在发送成功后推进环形队列消费位置，保留原子发布顺序。
void motor_calibration_consume(uint8_t count);
/* PublishControl 由原控制路径屏蔽中断后调用；Capture 在 ADC ISR 内执行。 */
void motor_calibration_publish_control(const motor_cal_sample_t *sample);
// 电流采样路径记录本周期角计数、Iq/Id（A）与 Uq（V），并累计逐点质量统计。
void motor_calibration_capture(uint16_t angle, float reference, float iq, float id, float uq);
// 查询队列是否仍有待发送记录，用于发送重试和启动门控。
uint8_t motor_calibration_has_samples(void);
// 查询当前是否为逐点位置采集模式。
uint8_t motor_calibration_is_point_mode(void);
// 读取逐点控制辅助标志，供位置环保持现有辅助逻辑。
uint8_t motor_calibration_point_assist(void);
// 读取当前逐点目标位置，单位编码器计数。
float motor_calibration_point_target(void);
// 1 ms 逐点状态机入口；保持等待、稳定、质量重试和目标跨零处理次序。
void motor_calibration_step_point(uint16_t angle, float speed, uint16_t flags, uint8_t healthy);
// 按样本队列相同索引读取配对目标，供带目标位置的协议数据帧使用。
void motor_calibration_peek_targets(float *targets, uint8_t count);
// 按样本序号复制保留的记录和目标计数，用于明确范围的协议重放。
uint8_t motor_calibration_replay(uint32_t first, uint8_t count, motor_cal_sample_t *samples, float *targets);
#endif
