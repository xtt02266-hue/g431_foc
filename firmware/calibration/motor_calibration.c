#include "motor_calibration.h"
#include <math.h>
#include <string.h>
#include <stdatomic.h>

_Static_assert(sizeof(motor_cal_sample_t) == 64U, "calibration wire size");
/* 1 ms 控制路径维护标定状态并发布 control，ADC 电流路径配对后向 ring 写入。
 * 协议前台消费 ring；head/tail 按原 C11 原子操作发布，序号用于检测漏控制周期。
 * *_ticks 是 1 ms 周期数；角度/进度为编码器计数，速度误差为 rpm，电流统计为 A。
 */
static motor_cal_sample_t ring[MOTOR_CAL_RING_SIZE];
static float ring_targets[MOTOR_CAL_RING_SIZE], point_target, control_point_target;
static uint16_t point_index, point_count = MOTOR_CAL_POINT_COUNT;
static uint8_t point_repeats = 3U, point_retry_until_good;
static uint8_t point_policy, point_failed_attempts;
static uint8_t point_assist;
static atomic_uint head, tail;
static volatile motor_cal_status_t state;
static const float speeds[3] = {60.0f, 80.0f, 100.0f};
static uint32_t elapsed, stable_ticks, zero_ticks, stall_ticks, turn_ticks;
static int32_t progress, stable_progress, turn_progress;
static uint16_t previous_angle;
static uint8_t angle_valid, turn_bad, advance_speed;
static float error_sum, error_sq, error_peak;
static uint32_t friction_window_ticks;
static float friction_error_sum, friction_error_sq;
static uint8_t friction_window_bad;
static motor_cal_sample_t control;
static volatile uint32_t control_sequence, captured_sequence;
static uint32_t point_samples, point_first_crc, point_first_transfer;
static float point_iq_mean, point_iq_m2, point_actual_mean, point_abs_error_mean;
static uint8_t point_sample_bad;

// 清除当前采集窗口的均值、方差与坏样本标志；保留重试次数。
static void reset_point_capture(void)
{
    point_samples = 0U;
    point_iq_mean = point_iq_m2 = point_actual_mean = point_abs_error_mean = 0.0f;
    point_sample_bad = 0U;
}

// 按已协商策略和失败次数判断是否进入平均采集阶段。
static uint8_t point_averaging(void)
{
    return point_policy && point_failed_attempts >= 2U;
}

// 选择当前质量策略的采集窗口，单位 ms。
static uint32_t point_capture_ms(void)
{
    return point_averaging() ? MOTOR_CAL_POINT_AVERAGE_MS : MOTOR_CAL_POINT_CAPTURE_MS;
}

// 将本点退回稳定等待阶段并统计拒绝次数，避免输出不合格记录。
static void reject_point_capture(void)
{
    if (point_failed_attempts < 2U) {
        ++point_failed_attempts;
    }
    state.phase = CAL_SETTLE;
    stable_ticks = 0;
    reset_point_capture();
    ++state.rejected_turns;
}

// 根据有效样本数、Iq 均值/方差和位置误差判断窗口质量；电流单位 A。
static uint8_t point_capture_valid(void)
{
    float tolerance = fmaxf(.003f, fabsf(point_iq_mean) * .25f);
    float variation = point_policy ? fmaxf(.006f, fabsf(point_iq_mean) * .50f) : tolerance;
    return point_samples >= point_capture_ms() && !point_sample_bad &&
           (!point_averaging() || point_abs_error_mean <= MOTOR_CAL_POINT_MAX_ERROR_COUNTS) &&
           (point_averaging() || point_iq_m2 / (float)point_samples <= variation * variation) &&
           fabsf(point_actual_mean - point_iq_mean) <= tolerance;
}

// 1 ms 控制路径发布给定及质量标志，供电流采样路径配对；序号用于检测漏周期。
void motor_calibration_publish_control(const motor_cal_sample_t *sample)
{
    control = *sample;
    ++control_sequence;
    control_point_target = point_target;
}

// 电流采样路径记录本周期角计数、Iq/Id（A）与 Uq（V），并累计逐点质量统计。
void motor_calibration_capture(uint16_t angle, float reference, float iq, float id, float uq)
{
    if (!motor_calibration_is_active() || control_sequence == captured_sequence) {
        return;
    }
    motor_cal_sample_t sample = control;
    if (control_sequence - captured_sequence != 1U) {
        sample.flags |= CAL_CONTROL_GAP;
    }
    captured_sequence = control_sequence;
    sample.angle = angle;
    sample.applied_iq = reference;
    sample.actual_iq = iq;
    sample.actual_id = id;
    sample.uq = uq;
    if (state.mode == 5U && state.phase == CAL_CAPTURE) {
        float error = control_point_target - (float)angle;
        if (error > 16384.0f) {
            error -= 32768.0f;
        }
        if (error < -16384.0f) {
            error += 32768.0f;
        }
        if ((!point_averaging() && fabsf(error) > MOTOR_CAL_POINT_MAX_ERROR_COUNTS) || !isfinite(sample.speed_rpm) ||
            fabsf(sample.speed_rpm) > 1.0f || !isfinite(reference) || !isfinite(iq) || (sample.flags & 15U) ||
            sample.sensor_status || sample.sensor_age_ms > 2U || fabsf(sample.friction_iq) > 1e-6f ||
            fabsf(sample.cogging_iq) > 1e-6f) {
            point_sample_bad = 1U;
        }
        if (!point_samples) {
            point_first_crc = sample.crc_errors;
            point_first_transfer = sample.transfer_errors;
        } else if (sample.crc_errors != point_first_crc || sample.transfer_errors != point_first_transfer) {
            point_sample_bad = 1U;
        }
        ++point_samples;
        point_abs_error_mean += (fabsf(error) - point_abs_error_mean) / (float)point_samples;
        if (isfinite(reference) && isfinite(iq)) {
            float delta = reference - point_iq_mean;
            point_iq_mean += delta / (float)point_samples;
            point_iq_m2 += delta * (reference - point_iq_mean);
            point_actual_mean += (iq - point_actual_mean) / (float)point_samples;
        }
    }
    motor_calibration_push(&sample);
}

// 查询是否处于稳定、采集或归零阶段；结束后仍可能留有待发送记录。
uint8_t motor_calibration_is_active(void)
{
    return state.phase >= CAL_SETTLE && state.phase <= CAL_ZERO;
}

// 复位单圈计数和速度误差统计，不改变当前速度档位。
static void reset_turn(void)
{
    turn_ticks = 0U;
    turn_progress = 0;
    turn_bad = 0U;
    error_sum = error_sq = error_peak = 0.0f;
}

// 切换标定阶段并清除该阶段的计时、进度与误差历史。
static void enter(uint8_t phase)
{
    state.phase = phase;
    elapsed = stable_ticks = zero_ticks = stall_ticks = 0U;
    progress = stable_progress = 0;
    reset_turn();
    friction_window_ticks = 0U;
    friction_error_sum = friction_error_sq = 0.0f;
    friction_window_bad = 0U;
    if (phase == CAL_SETTLE) {
        state.accepted_turns = state.rejected_turns = 0U;
    }
    state.target_rpm = phase == CAL_ZERO ? 0.0f : speeds[state.speed_index] * (state.reverse ? -1.0f : 1.0f);
}

// 读取本次逐点采集的网格点数；不改变采集状态。
uint16_t motor_calibration_point_count(void)
{
    return point_count;
}

// 按 128/256/512 点网格返回兼容的目标数据协议版本。
uint16_t motor_calibration_point_schema(void)
{
    return point_count == 512U ? 4U : point_count == 256U ? 3U : 2U;
}

// 读取当前逐点采集的重复次数。
uint8_t motor_calibration_point_repeats(void)
{
    return point_repeats;
}

// 读取质量不合格时持续重试的选项。
uint8_t motor_calibration_point_retry_until_good(void)
{
    return point_retry_until_good;
}

// 读取逐点质量判定策略，供上位机按协商版本解释结果。
uint8_t motor_calibration_point_policy(void)
{
    return point_policy;
}

/* 旧请求沿用 128/256 点网格；协商能力 bit20 的客户端可请求 512 点。 */
uint8_t motor_calibration_decode_start_request(const uint8_t *body, uint16_t size, uint8_t *mode, uint16_t *count)
{
    if (!body || !mode || !count || (size != 1U && size != 3U) || body[0] > 5U) {
        return 0U;
    }
    uint16_t n = 128U;
    if (size == 3U) {
        n = (uint16_t)(body[1] | ((uint16_t)body[2] << 8U));
        if (body[0] != 5U || (n != 256U && n != 512U)) {
            return 0U;
        }
    }
    *mode = body[0];
    *count = n;
    return 1U;
}

// 解析带长度的启动字节缓冲区；保留旧请求的默认重复次数和重试策略。
uint8_t motor_calibration_decode_start_options(const uint8_t *body, uint16_t size, uint8_t *mode, uint16_t *count,
                                               uint8_t *repeats, uint8_t *retry_until_good)
{
    if (!body || !mode || !count || !repeats || !retry_until_good) {
        return 0U;
    }
    if (size == 5U) {
        uint16_t n = (uint16_t)(body[1] | ((uint16_t)body[2] << 8U));
        if (body[0] != 5U || (n != 256U && n != 512U) || (body[3] != 1U && body[3] != 3U) || body[4] > 1U) {
            return 0U;
        }
        *mode = 5U;
        *count = n;
        *repeats = body[3];
        *retry_until_good = body[4];
        return 1U;
    }
    *repeats = 3U;
    *retry_until_good = 0U;
    return motor_calibration_decode_start_request(body, size, mode, count);
}

// 使用默认网格启动指定标定模式；返回 1 表示请求已接受。
uint8_t motor_calibration_begin(uint8_t mode)
{
    return motor_calibration_begin_with_points(mode, MOTOR_CAL_POINT_COUNT);
}

// 指定网格点数启动标定，沿用三次重复且不持续重试的默认选项。
uint8_t motor_calibration_begin_with_points(uint8_t mode, uint16_t count)
{
    return motor_calibration_begin_with_options(mode, count, 3U, 0U);
}

// 按网格、重复次数和重试选项启动；质量策略采用兼容默认值。
uint8_t motor_calibration_begin_with_options(uint8_t mode, uint16_t count, uint8_t repeats, uint8_t retry_until_good)
{
    return motor_calibration_begin_with_policy(mode, count, repeats, retry_until_good, 0U);
}

/* 第六个可选字节由 HELLO bit23 协商；旧请求沿用原质量判定。 */
uint8_t motor_calibration_decode_start_policy(const uint8_t *body, uint16_t size, uint8_t *mode, uint16_t *count,
                                              uint8_t *repeats, uint8_t *retry_until_good, uint8_t *policy)
{
    if (!body || !policy) {
        return 0U;
    }
    *policy = 0U;
    if (size == 6U) {
        if (body[5] != 1U ||
            !motor_calibration_decode_start_options(body, 5U, mode, count, repeats, retry_until_good)) {
            return 0U;
        }
        *policy = 1U;
        return 1U;
    }
    return motor_calibration_decode_start_options(body, size, mode, count, repeats, retry_until_good);
}

// 校验启动选项并复位采集状态；已有标定或待发送记录时拒绝请求。
uint8_t motor_calibration_begin_with_policy(uint8_t mode, uint16_t count, uint8_t repeats, uint8_t retry_until_good,
                                            uint8_t policy)
{
    if (policy > 1U || (policy && mode != 5U)) {
        return 0U;
    }
    if ((repeats != 1U && repeats != 3U) || retry_until_good > 1U ||
        (mode != 5U && (repeats != 3U || retry_until_good))) {
        return 0U;
    }
    if (mode == 5U && count != 128U && count != 256U && count != 512U) {
        return 0U;
    }
    if (mode > 5U || motor_calibration_is_active() || motor_calibration_has_samples()) {
        return 0U;
    }
    point_policy = policy;
    point_failed_attempts = 0U;
    if (mode == 5U) {
        point_count = count;
        point_repeats = repeats;
        point_retry_until_good = retry_until_good;
    }
    uint32_t session = state.session + 1U;
    motor_cal_status_t empty = {0};
    state = empty;
    state.session = session ? session : 1U;
    state.mode = mode;
    angle_valid = advance_speed = 0U;
    captured_sequence = control_sequence;
    atomic_store(&head, 0U);
    atomic_store(&tail, 0U);
    enter(CAL_SETTLE);
    if (mode == 5U) {
        reset_point_capture();
        point_index = 0;
        point_assist = 1;
        point_target = (float)(32768U - 32768U / point_count);
        enter(CAL_ZERO);
        state.target_rpm = 0;
    }
    return 1U;
}

// 终止活动标定并保存原因码；待发送数据按原队列规则保留。
void motor_calibration_abort(uint8_t reason)
{
    if (!motor_calibration_is_active()) {
        return;
    }
    state.reason = reason;
    state.phase = reason == CAL_CANCELLED ? CAL_ABORTED : CAL_FAILED;
    state.target_rpm = 0.0f;
}

// 处理当前速度档采集失败，按既有阶段推进规则保留失败原因。
static void fail_speed(uint8_t reason)
{
    state.speed_reasons[state.speed_index] = reason;
    advance_speed = 1U;
    enter(CAL_ZERO);
}

// 检查丢样与输入有效性；失败原因和优先级沿用原状态机。
static uint8_t calibration_sample_valid(float speed, float target, uint8_t healthy)
{
    if (state.dropped) {
        motor_calibration_abort(CAL_OVERFLOW);
        return 0U;
    }
    if (!healthy || !isfinite(speed) || !isfinite(target)) {
        motor_calibration_abort(CAL_SENSOR);
        return 0U;
    }
    return 1U;
}

// 展开单圈角度并按当前采集方向归一化，返回编码器计数增量。
static int32_t calibration_angle_delta(uint16_t angle)
{
    int32_t delta = angle_valid ? (int32_t)angle - previous_angle : 0;
    previous_angle = angle;
    angle_valid = 1U;
    if (delta > 16384) {
        delta -= 32768;
    }
    if (delta < -16384) {
        delta += 32768;
    }
    if (state.reverse) {
        delta = -delta;
    }
    return delta;
}

// 等待零速后推进方向或速度档，返回当前目标 rpm。
static float calibration_step_zero(float speed, float applied_target)
{
    if (fabsf(speed) < 2.0f && fabsf(applied_target) < 0.1f) {
        ++zero_ticks;
    } else {
        zero_ticks = 0U;
    }
    if (elapsed > 10000U) {
        motor_calibration_abort(CAL_TIMEOUT);
    } else if (zero_ticks >= 500U) {
        if (advance_speed || state.reverse) {
            if (!advance_speed) {
                state.passed_mask |= (uint8_t)(1U << state.speed_index);
            }
            state.reverse = 0U;
            advance_speed = 0U;
            ++state.speed_index;
            if (state.speed_index >= 3U) {
                uint8_t n = 0U;
                for (uint8_t b = 0; b < 3; ++b) {
                    n += (state.passed_mask >> b) & 1U;
                }
                uint8_t required = state.mode == 4U ? 3U : 2U;
                state.phase = n >= required ? CAL_DONE : CAL_FAILED;
                state.reason = n >= required ? CAL_OK : CAL_INSUFFICIENT;
                state.speed_index = 2U;
            } else {
                enter(CAL_SETTLE);
            }
        } else {
            state.reverse = 1U;
            enter(CAL_SETTLE);
        }
    }
    return state.target_rpm;
}

// 累积运动进度并检查停转；返回 1 表示当前速度档已失败。
static uint8_t calibration_check_stall(int32_t delta, float speed, uint8_t commanded)
{
    if (delta > 0) {
        progress += delta;
    }
    if (commanded && speed * (state.reverse ? -1.0f : 1.0f) < 5.0f) {
        ++stall_ticks;
    } else {
        stall_ticks = 0U;
    }
    if (stall_ticks >= 50U) {
        fail_speed(CAL_STALL);
        return 1U;
    }
    return 0U;
}

// 按摩擦/齿槽各自的稳定窗口决定进入采集或失败，不改变阈值。
static void calibration_step_settle(int32_t delta, float speed, float target, float error, uint16_t flags, uint8_t commanded)
{
    if (state.mode == 4U) {
        ++friction_window_ticks;
        friction_error_sum += error;
        friction_error_sq += error * error;
        if (!commanded || (flags & (uint16_t)~CAL_SLEW) ||
            speed * (state.reverse ? -1.0f : 1.0f) < fabsf(target) * .5f) {
            friction_window_bad = 1U;
        }
        /* 直流摩擦采集按平均运动误差判断稳定性；
         * 保留独立窗口门限，不套用齿槽采集的逐周期误差门限。 */
        if (friction_window_ticks >= 1000U) {
            uint8_t good = !friction_window_bad && fabsf(friction_error_sum / 1000.0f) <= fabsf(target) * .05f &&
                           sqrtf(friction_error_sq / 1000.0f) <= fabsf(target) * .15f;
            stable_ticks = good ? stable_ticks + 1000U : 0U;
            friction_window_ticks = 0U;
            friction_error_sum = friction_error_sq = 0.0f;
            friction_window_bad = 0U;
        }
    } else if (commanded && fabsf(error) <= fmaxf(1.0f, fabsf(target) * .03f) && !flags) {
        ++stable_ticks;
        if (delta > 0) {
            stable_progress += delta;
        }
    } else {
        stable_ticks = 0U;
        stable_progress = 0;
    }
    /* 5 seconds and 5 complete revolutions in total, plus a continuous
     * 2-second stable window. No samples from startup qualify as turns. */
    if (elapsed >= 5000U && progress >= 5 * 32768 && ((stable_ticks >= 2000U) || state.mode == 0U)) {
        enter(CAL_CAPTURE);
    } else if (elapsed >= 20000U) {
        fail_speed(CAL_UNSTABLE);
    }
}

// 统计每圈误差及质量，推进有效圈数并保留超时检查。
static void calibration_step_capture(int32_t delta, float speed, float target, float error, uint16_t flags)
{
    ++turn_ticks;
    turn_progress += delta;
    error_sum += error;
    error_sq += error * error;
    if (fabsf(error) > error_peak) {
        error_peak = fabsf(error);
    }
    uint16_t reject_flags = state.mode == 4U ? (flags & (uint16_t)~CAL_SLEW) : flags;
    if (reject_flags || delta < -16 || speed * (state.reverse ? -1.0f : 1.0f) < 5.0f) {
        turn_bad = 1U;
    }
    if (turn_progress >= 32768) {
        uint8_t friction = state.mode == 4U;
        uint8_t good = !turn_bad &&
                       fabsf(error_sum / turn_ticks) <= fmaxf(.5f, fabsf(target) * (friction ? .05f : .02f)) &&
                       sqrtf(error_sq / turn_ticks) <= fmaxf(.5f, fabsf(target) * (friction ? .15f : .03f)) &&
                       error_peak <= fmaxf(1.0f, fabsf(target) * (friction ? .40f : .08f));
        if (good || state.mode == 0U) {
            ++state.accepted_turns;
        } else {
            ++state.rejected_turns;
        }
        reset_turn();
        if (state.accepted_turns >= (friction ? 8U : MOTOR_CAL_REQUIRED_TURNS)) {
            enter(CAL_ZERO);
        }
    }
    if (elapsed >= 90000U) {
        fail_speed(CAL_UNSTABLE);
    }
}

// 1 ms 旋转标定调度；按归零、稳定和采集阶段返回下一周期速度目标 rpm。
float motor_calibration_step(uint16_t angle, float speed, float applied_target, uint16_t flags, uint8_t healthy)
{
    if (!motor_calibration_is_active() || !calibration_sample_valid(speed, applied_target, healthy)) {
        return 0.0f;
    }
    ++elapsed;
    int32_t delta = calibration_angle_delta(angle);
    if (state.phase == CAL_ZERO) {
        return calibration_step_zero(speed, applied_target);
    }
    float target = state.target_rpm, error = speed - target;
    uint8_t commanded = fabsf(applied_target - target) < 0.5f;
    if (calibration_check_stall(delta, speed, commanded)) {
        return 0.0f;
    }
    if (state.phase == CAL_SETTLE) {
        calibration_step_settle(delta, speed, target, error, flags, commanded);
    } else {
        calibration_step_capture(delta, speed, target, error, flags);
    }
    return state.target_rpm;
}

// 复制当前标定状态用于协议上报；调用方须提供有效输出对象。
void motor_calibration_get_status(motor_cal_status_t *status)
{
    *status = state;
}

// 生成当前速度阶段和方向标志，供样本与质量分析配对。
uint16_t motor_calibration_sample_flags(void)
{
    return (uint16_t)((state.phase << 8U) | (state.speed_index << 11U) | (state.reverse << 13U) |
                      ((state.mode == 5U && state.phase == CAL_CAPTURE && point_averaging()) ? CAL_POINT_AVERAGE : 0U));
}

// 采样路径向固定容量环形队列发布记录；队满按原规则中止并记录溢出。
void motor_calibration_push(const motor_cal_sample_t *sample)
{
    if (!motor_calibration_is_active()) {
        return;
    }
    uint32_t h = atomic_load_explicit(&head, memory_order_relaxed);
    uint32_t t = atomic_load_explicit(&tail, memory_order_acquire);
    ++state.produced;
    if (h - t >= MOTOR_CAL_RING_SIZE) {
        ++state.dropped;
        return;
    }
    ring[h % MOTOR_CAL_RING_SIZE] = *sample;
    ring[h % MOTOR_CAL_RING_SIZE].sequence = state.produced;
    ring_targets[h % MOTOR_CAL_RING_SIZE] = control_point_target;
    atomic_store_explicit(&head, h + 1U, memory_order_release);
}

// 协议前台只读复制待发送记录，不推进消费位置；返回实际复制条数。
uint8_t motor_calibration_peek(motor_cal_sample_t *samples, uint8_t maximum)
{
    uint32_t t = atomic_load_explicit(&tail, memory_order_relaxed);
    uint32_t n = atomic_load_explicit(&head, memory_order_acquire) - t;
    if (n > maximum) {
        n = maximum;
    }
    for (uint32_t i = 0; i < n; ++i) {
        samples[i] = ring[(t + i) % MOTOR_CAL_RING_SIZE];
    }
    return (uint8_t)n;
}

// 前台在发送成功后推进环形队列消费位置，保留原子发布顺序。
void motor_calibration_consume(uint8_t count)
{
    atomic_fetch_add_explicit(&tail, count, memory_order_release);
}

// 查询队列是否仍有待发送记录，用于发送重试和启动门控。
uint8_t motor_calibration_has_samples(void)
{
    return atomic_load(&head) != atomic_load(&tail);
}

// 查询当前是否为逐点位置采集模式。
uint8_t motor_calibration_is_point_mode(void)
{
    return state.mode == 5U;
}

// 读取逐点控制辅助标志，供位置环保持现有辅助逻辑。
uint8_t motor_calibration_point_assist(void)
{
    return state.mode == 5U && motor_calibration_is_active() && point_assist;
}

// 读取当前逐点目标位置，单位编码器计数。
float motor_calibration_point_target(void)
{
    return point_target;
}

// 按样本队列相同索引读取配对目标，供带目标位置的协议数据帧使用。
void motor_calibration_peek_targets(float *targets, uint8_t count)
{
    uint32_t t = atomic_load_explicit(&tail, memory_order_relaxed);
    for (uint8_t i = 0; i < count; i++) {
        targets[i] = ring_targets[(t + i) % MOTOR_CAL_RING_SIZE];
    }
}

/* 调用方屏蔽中断：已消费记录在被环形队列覆盖前仍可重放。 */
uint8_t motor_calibration_replay(uint32_t first, uint8_t count, motor_cal_sample_t *samples, float *targets)
{
    if (!first || !count || count > 3U || state.dropped || first > state.produced ||
        (uint32_t)(count - 1U) > state.produced - first) {
        return 0U;
    }
    for (uint8_t i = 0; i < count; i++) {
        uint32_t index = (first + i - 1U) % MOTOR_CAL_RING_SIZE;
        if (ring[index].sequence != first + i) {
            return 0U;
        }
        samples[i] = ring[index];
        targets[i] = ring_targets[index];
    }
    return count;
}

// 计算最短机械位置误差，单位为编码器计数。
static float calibration_point_error(uint16_t angle)
{
    float error = point_target - angle;
    if (error > 16384) {
        error -= 32768;
    }
    if (error < -16384) {
        error += 32768;
    }
    return error;
}

// 当前点通过后推进点位、方向和轮次；返回 1 表示会话完成。
static uint8_t calibration_advance_point(void)
{
    ++state.accepted_turns;
    point_failed_attempts = 0U;
    if (++point_index >= point_count) {
        if (state.reverse) {
            state.passed_mask |= (uint8_t)(1U << state.speed_index);
            if (++state.speed_index >= point_repeats) {
                state.speed_index = (uint8_t)(point_repeats - 1U);
                state.phase = CAL_DONE;
                state.target_rpm = 0;
                return 1U;
            }
            state.reverse = 0;
        } else {
            state.reverse = 1;
        }
        point_index = 0;
        state.accepted_turns = 0;
        state.phase = CAL_ZERO;
        point_target = state.reverse ? 0.0f : (float)(32768U - 32768U / point_count);
    } else {
        state.phase = CAL_SETTLE;
        point_target = (float)(state.reverse ? point_count - 1U - point_index : point_index) *
                       (float)(32768U / point_count);
    }
    stable_ticks = elapsed = 0;
    point_assist = 1;
    return 0U;
}

// 处理逐点采集质量及重试；返回 1 表示本周期应立即结束。
static uint8_t calibration_capture_point(float error, float speed, uint16_t flags)
{
    uint8_t capture_stable =
        fabsf(speed) <= 1.0f && !flags && (point_averaging() || fabsf(error) <= MOTOR_CAL_POINT_MAX_ERROR_COUNTS);
    if (!capture_stable) {
        reject_point_capture();
    } else if (++stable_ticks >= point_capture_ms()) {
        if (!point_capture_valid()) {
            reject_point_capture();
            if (!point_retry_until_good && elapsed >= MOTOR_CAL_POINT_TIMEOUT_MS) {
                motor_calibration_abort(CAL_UNSTABLE);
            }
            return 1U; /* 重新完成连续稳定窗口后重试当前目标 */
        }
        if (calibration_advance_point()) {
            return 1U;
        }
    }
    return 0U;
}

// 辅助撤销后完成连续稳定窗口，再开始采样。
static void calibration_settle_point(uint8_t stable)
{
    stable_ticks = stable ? stable_ticks + 1 : 0;
    if (stable_ticks >= MOTOR_CAL_POINT_SETTLE_MS) {
        if (state.phase == CAL_ZERO) {
            point_target = state.reverse ? (float)(32768U - 32768U / point_count) : 0.0f;
            state.phase = CAL_SETTLE;
            elapsed = 0;
            point_assist = 1;
        } else {
            reset_point_capture();
            state.phase = CAL_CAPTURE;
        }
        stable_ticks = 0;
    }
}

// 1 ms 逐点调度；辅助退出、采集重试和超时的提前返回保持原语义。
void motor_calibration_step_point(uint16_t angle, float speed, uint16_t flags, uint8_t healthy)
{
    if (!motor_calibration_is_active() || state.mode != 5U) {
        return;
    }
    if (!calibration_sample_valid(speed, 0.0f, healthy)) {
        return;
    }
    float error = calibration_point_error(angle);
    ++elapsed;
    uint8_t stable = fabsf(error) <= MOTOR_CAL_POINT_MAX_ERROR_COUNTS && fabsf(speed) <= 1.0f && !flags;
    if (point_assist) {
        if (stable) {
            point_assist = 0;
            stable_ticks = 0;
        }
        if (!point_retry_until_good && elapsed >= MOTOR_CAL_POINT_TIMEOUT_MS) {
            motor_calibration_abort(CAL_UNSTABLE);
        }
        return; /* 辅助撤销后才开始计算稳定窗口 */
    }
    if (state.phase == CAL_CAPTURE) {
        if (calibration_capture_point(error, speed, flags)) {
            return;
        }
    } else {
        calibration_settle_point(stable);
    }

    if (state.phase != CAL_CAPTURE && (fabsf(error) > 32.0f || fabsf(speed) > 2.0f)) {
        point_assist = 1;
        stable_ticks = 0;
    }
    if (!point_retry_until_good && elapsed >= MOTOR_CAL_POINT_TIMEOUT_MS) {
        motor_calibration_abort(CAL_UNSTABLE);
    }
}
