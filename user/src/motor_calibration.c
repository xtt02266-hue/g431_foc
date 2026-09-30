#include "motor_calibration.h"
#include <math.h>
#include <string.h>
#include <stdatomic.h>

_Static_assert(sizeof(MotorCalSample) == 64U, "calibration wire size");
static MotorCalSample ring[MOTOR_CAL_RING_SIZE];
static float ring_targets[MOTOR_CAL_RING_SIZE], point_target, control_point_target;
static uint8_t point_index;
static uint8_t point_assist;
static atomic_uint head, tail;
static volatile MotorCalStatus state;
static const float speeds[3] = {60.0f, 80.0f, 100.0f};
static uint32_t elapsed, stable_ticks, zero_ticks, stall_ticks, turn_ticks;
static int32_t progress, stable_progress, turn_progress;
static uint16_t previous_angle;
static uint8_t angle_valid, turn_bad, advance_speed;
static float error_sum, error_sq, error_peak;
static uint32_t friction_window_ticks;
static float friction_error_sum, friction_error_sq;
static uint8_t friction_window_bad;
static MotorCalSample control;
static volatile uint32_t control_sequence, captured_sequence;

void Motor_Calibration_PublishControl(const MotorCalSample *sample) {
    control = *sample; ++control_sequence;
    control_point_target=point_target;
}
void Motor_Calibration_Capture(uint16_t angle, float reference, float iq, float id, float uq) {
    if (!Motor_Calibration_IsActive() || control_sequence == captured_sequence) return;
    MotorCalSample sample = control;
    if (control_sequence - captured_sequence != 1U) sample.flags |= CAL_CONTROL_GAP;
    captured_sequence = control_sequence;
    sample.angle = angle; sample.applied_iq = reference;
    sample.actual_iq = iq; sample.actual_id = id; sample.uq = uq;
    Motor_Calibration_Push(&sample);
}

uint8_t Motor_Calibration_IsActive(void) {
    return state.phase >= CAL_SETTLE && state.phase <= CAL_ZERO;
}
static void reset_turn(void) {
    turn_ticks = 0U; turn_progress = 0; turn_bad = 0U;
    error_sum = error_sq = error_peak = 0.0f;
}
static void enter(uint8_t phase) {
    state.phase = phase; elapsed = stable_ticks = zero_ticks = stall_ticks = 0U;
    progress = stable_progress = 0; reset_turn();
    friction_window_ticks=0U; friction_error_sum=friction_error_sq=0.0f; friction_window_bad=0U;
    if (phase == CAL_SETTLE) state.accepted_turns = state.rejected_turns = 0U;
    state.target_rpm = phase == CAL_ZERO ? 0.0f :
        speeds[state.speed_index] * (state.reverse ? -1.0f : 1.0f);
}
uint8_t Motor_Calibration_Begin(uint8_t mode) {
    if (mode > 5U || Motor_Calibration_IsActive() || Motor_Calibration_HasSamples()) return 0U;
    uint32_t session = state.session + 1U;
    MotorCalStatus empty = {0}; state = empty; state.session = session ? session : 1U;
    state.mode = mode; angle_valid = advance_speed = 0U;
    captured_sequence = control_sequence;
    atomic_store(&head, 0U); atomic_store(&tail, 0U); enter(CAL_SETTLE);
    if (mode==5U) { point_index=0;point_assist=1;point_target=32512.0f;enter(CAL_ZERO);state.target_rpm=0; }
    return 1U;
}
void Motor_Calibration_Abort(uint8_t reason) {
    if (!Motor_Calibration_IsActive()) return;
    state.reason = reason;
    state.phase = reason == CAL_CANCELLED ? CAL_ABORTED : CAL_FAILED;
    state.target_rpm = 0.0f;
}
static void fail_speed(uint8_t reason) {
    state.speed_reasons[state.speed_index] = reason;
    advance_speed = 1U; enter(CAL_ZERO);
}
float Motor_Calibration_Step(uint16_t angle, float speed, float applied_target,
                            uint16_t flags, uint8_t healthy) {
    if (!Motor_Calibration_IsActive()) return 0.0f;
    if (state.dropped) { Motor_Calibration_Abort(CAL_OVERFLOW); return 0.0f; }
    if (!healthy || !isfinite(speed) || !isfinite(applied_target)) {
        Motor_Calibration_Abort(CAL_SENSOR); return 0.0f;
    }
    ++elapsed;
    int32_t delta = angle_valid ? (int32_t)angle - previous_angle : 0;
    previous_angle = angle; angle_valid = 1U;
    if (delta > 16384) delta -= 32768;
    if (delta < -16384) delta += 32768;
    if (state.reverse) delta = -delta;
    if (state.phase == CAL_ZERO) {
        if (fabsf(speed) < 2.0f && fabsf(applied_target) < 0.1f) ++zero_ticks;
        else zero_ticks = 0U;
        if (elapsed > 10000U) Motor_Calibration_Abort(CAL_TIMEOUT);
        else if (zero_ticks >= 500U) {
            if (advance_speed || state.reverse) {
                if (!advance_speed) state.passed_mask |= (uint8_t)(1U << state.speed_index);
                state.reverse = 0U; advance_speed = 0U; ++state.speed_index;
                if (state.speed_index >= 3U) {
                    uint8_t n = 0U;
                    for (uint8_t b = 0; b < 3; ++b) n += (state.passed_mask >> b) & 1U;
                    uint8_t required = state.mode == 4U ? 3U : 2U;
                    state.phase = n >= required ? CAL_DONE : CAL_FAILED;
                    state.reason = n >= required ? CAL_OK : CAL_INSUFFICIENT;
                    state.speed_index = 2U;
                } else enter(CAL_SETTLE);
            } else { state.reverse = 1U; enter(CAL_SETTLE); }
        }
        return state.target_rpm;
    }
    float target = state.target_rpm, error = speed - target;
    if (delta > 0) progress += delta;
    uint8_t commanded = fabsf(applied_target - target) < 0.5f;
    if (commanded && speed * (state.reverse ? -1.0f : 1.0f) < 5.0f) ++stall_ticks;
    else stall_ticks = 0U;
    if (stall_ticks >= 50U) { fail_speed(CAL_STALL); return 0.0f; }
    if (state.phase == CAL_SETTLE) {
        if (state.mode == 4U) {
            ++friction_window_ticks; friction_error_sum+=error; friction_error_sq+=error*error;
            if (!commanded || (flags & (uint16_t)~CAL_SLEW) || speed*(state.reverse?-1.0f:1.0f)<fabsf(target)*.5f)
                friction_window_bad=1U;
            /* DC friction needs bounded average motion, not every instantaneous
             * point inside the much tighter cogging calibration tolerance. */
            if (friction_window_ticks>=1000U) {
                uint8_t good=!friction_window_bad &&
                    fabsf(friction_error_sum/1000.0f)<=fabsf(target)*.05f &&
                    sqrtf(friction_error_sq/1000.0f)<=fabsf(target)*.15f;
                stable_ticks=good?stable_ticks+1000U:0U;
                friction_window_ticks=0U;friction_error_sum=friction_error_sq=0.0f;friction_window_bad=0U;
            }
        } else if (commanded && fabsf(error) <= fmaxf(1.0f, fabsf(target) * .03f) && !flags) {
            ++stable_ticks; if (delta > 0) stable_progress += delta;
        } else { stable_ticks = 0U; stable_progress = 0; }
        /* 5 seconds and 5 complete revolutions in total, plus a continuous
         * 2-second stable window. No samples from startup qualify as turns. */
        if (elapsed >= 5000U && progress >= 5 * 32768 &&
            ((stable_ticks >= 2000U) || state.mode == 0U)) enter(CAL_CAPTURE);
        else if (elapsed >= 20000U) fail_speed(CAL_UNSTABLE);
    } else {
        ++turn_ticks; turn_progress += delta;
        error_sum += error; error_sq += error * error;
        if (fabsf(error) > error_peak) error_peak = fabsf(error);
        uint16_t reject_flags = state.mode == 4U ? (flags & (uint16_t)~CAL_SLEW) : flags;
        if (reject_flags || delta < -16 || speed * (state.reverse ? -1.0f : 1.0f) < 5.0f) turn_bad = 1U;
        if (turn_progress >= 32768) {
            uint8_t friction = state.mode == 4U;
            uint8_t good = !turn_bad &&
                fabsf(error_sum / turn_ticks) <= fmaxf(.5f, fabsf(target) * (friction?.05f:.02f)) &&
                sqrtf(error_sq / turn_ticks) <= fmaxf(.5f, fabsf(target) * (friction?.15f:.03f)) &&
                error_peak <= fmaxf(1.0f, fabsf(target) * (friction?.40f:.08f));
            if (good || state.mode == 0U) ++state.accepted_turns;
            else ++state.rejected_turns;
            reset_turn();
            if (state.accepted_turns >= (friction ? 8U : MOTOR_CAL_REQUIRED_TURNS)) enter(CAL_ZERO);
        }
        if (elapsed >= 90000U) fail_speed(CAL_UNSTABLE);
    }
    return state.target_rpm;
}
void Motor_Calibration_GetStatus(MotorCalStatus *status) { *status = state; }
uint16_t Motor_Calibration_SampleFlags(void) {
    return (uint16_t)((state.phase << 8U) | (state.speed_index << 11U) | (state.reverse << 13U));
}
void Motor_Calibration_Push(const MotorCalSample *sample) {
    if (!Motor_Calibration_IsActive()) return;
    uint32_t h = atomic_load_explicit(&head, memory_order_relaxed);
    uint32_t t = atomic_load_explicit(&tail, memory_order_acquire);
    ++state.produced;
    if (h - t >= MOTOR_CAL_RING_SIZE) { ++state.dropped; return; }
    ring[h % MOTOR_CAL_RING_SIZE] = *sample;
    ring[h % MOTOR_CAL_RING_SIZE].sequence = state.produced;
    ring_targets[h % MOTOR_CAL_RING_SIZE]=control_point_target;
    atomic_store_explicit(&head, h + 1U, memory_order_release);
}
uint8_t Motor_Calibration_Peek(MotorCalSample *samples, uint8_t maximum) {
    uint32_t t = atomic_load_explicit(&tail, memory_order_relaxed);
    uint32_t n = atomic_load_explicit(&head, memory_order_acquire) - t;
    if (n > maximum) n = maximum;
    for (uint32_t i = 0; i < n; ++i) samples[i] = ring[(t + i) % MOTOR_CAL_RING_SIZE];
    return (uint8_t)n;
}
void Motor_Calibration_Consume(uint8_t count) { atomic_fetch_add_explicit(&tail, count, memory_order_release); }
uint8_t Motor_Calibration_HasSamples(void) { return atomic_load(&head) != atomic_load(&tail); }

uint8_t Motor_Calibration_IsPointMode(void) { return state.mode==5U; }
uint8_t Motor_Calibration_PointAssist(void) { return state.mode==5U && Motor_Calibration_IsActive() && point_assist; }
float Motor_Calibration_PointTarget(void) { return point_target; }
void Motor_Calibration_PeekTargets(float *targets,uint8_t count) {
    uint32_t t=atomic_load_explicit(&tail,memory_order_relaxed);
    for (uint8_t i=0;i<count;i++) targets[i]=ring_targets[(t+i)%MOTOR_CAL_RING_SIZE];
}
/* Caller masks interrupts: a consumed record remains valid until overwritten. */
uint8_t Motor_Calibration_Replay(uint32_t first,uint8_t count,MotorCalSample *samples,float *targets) {
    if (!first || !count || count>3U || state.dropped || first>state.produced ||
        (uint32_t)(count-1U)>state.produced-first) return 0U;
    for (uint8_t i=0;i<count;i++) {
        uint32_t index=(first+i-1U)%MOTOR_CAL_RING_SIZE;
        if (ring[index].sequence!=first+i) return 0U;
        samples[i]=ring[index];targets[i]=ring_targets[index];
    }
    return count;
}
void Motor_Calibration_StepPoint(uint16_t angle,float speed,uint16_t flags,uint8_t healthy) {
    if (!Motor_Calibration_IsActive() || state.mode!=5U) return;
    if (state.dropped) { Motor_Calibration_Abort(CAL_OVERFLOW);return; }
    if (!healthy || !isfinite(speed)) { Motor_Calibration_Abort(CAL_SENSOR);return; }
    float error=point_target-angle;
    if (error>16384) error-=32768;
    if (error<-16384) error+=32768;
    ++elapsed;
    uint8_t stable=fabsf(error)<=16.0f && fabsf(speed)<=1.0f && !flags;
    if (point_assist) {
        if (stable) { point_assist=0;stable_ticks=0; }
        if (elapsed>=10000U) Motor_Calibration_Abort(CAL_UNSTABLE);
        return; /* stability timer starts only after assist has been removed */
    }
    if (state.phase==CAL_CAPTURE) {
        if (!stable) { state.phase=CAL_SETTLE;stable_ticks=0;++state.rejected_turns; }
        else if (++stable_ticks>=MOTOR_CAL_POINT_CAPTURE_MS) {
            ++state.accepted_turns;
            if (++point_index>=128U) {
                if (state.reverse) {
                    state.passed_mask|=(uint8_t)(1U<<state.speed_index);
                    if (++state.speed_index>=3U) {
                        state.speed_index=2;state.phase=CAL_DONE;state.target_rpm=0;return;
                    }
                    state.reverse=0;
                } else state.reverse=1;
                point_index=0;state.accepted_turns=0;
                state.phase=CAL_ZERO;point_target=state.reverse?0.0f:32512.0f;
            } else {
                state.phase=CAL_SETTLE;
                point_target=(float)(state.reverse?127-point_index:point_index)*256.0f;
            }
            stable_ticks=elapsed=0;point_assist=1;
        }
    } else {
        stable_ticks=stable?stable_ticks+1:0;
        if (stable_ticks>=MOTOR_CAL_POINT_SETTLE_MS) {
            if (state.phase==CAL_ZERO) {
                point_target=state.reverse?32512.0f:0.0f;state.phase=CAL_SETTLE;elapsed=0;point_assist=1;
            } else state.phase=CAL_CAPTURE;
            stable_ticks=0;
        }
    }
    if (state.phase!=CAL_CAPTURE && (fabsf(error)>32.0f || fabsf(speed)>2.0f)) {
        point_assist=1;stable_ticks=0;
    }
    if (elapsed>=10000U) Motor_Calibration_Abort(CAL_UNSTABLE);
}
