#include "motor_control.h"
#include "motor_context.h"
#include "motor_current_loop.h"
#include "motor_speed_loop.h"
#include "motor_position_loop.h"
#include "motor_encoder.h"
#include "user_io.h"
#include "motor_identify.h"
#include "motor_calibration.h"
#include <math.h>
#include <stddef.h>

// TIM2 发布反向映射后的 12 位 ADC 滤波计数；前台 VOFA 只读，不在读取时重新采样。
static float g_pot_target_filtered = 0.0f;
static uint8_t g_pot_filter_initialized = 0U;
static float motor_system_pot_torque_nm(float filtered_pot);
static float motor_system_clamp_host_iq(float iq);

// 生命周期只请求重新装入首个采样，滤波历史仍由本模块持有。
void motor_system_reset_pot_filter(void)
{
    g_pot_filter_initialized = 0U;
}

/* 为什么需要：FOC 电流环接收安培，上层使用 N·m，必须统一换算并限制电流。
 * 参数：torque_nm 为目标电磁力矩（N·m），符号沿用 Iq 方向。
 * 返回：按 Iq=T/Kt 换算并限幅的电流（A）；参数或 Kt 无效时返回 0。
 * 先与 Kt*电流上限比较，再做除法，避免极小 Kt 或极大输入导致除法溢出。
 * 此函数只计算，不修改运行状态；电位器和手动力矩入口共用相同换算规则。
 */
float motor_system_torque_to_iq(float torque_nm)
{
    const float kt = MOTOR_TORQUE_CONSTANT_NM_PER_A;
    if (!isfinite(kt) || (kt <= 0.0f) || !isfinite(torque_nm)) {
        return 0.0f;
    }
    float max_torque_nm = kt * MOTOR_TORQUE_CURRENT_LIMIT_A;
    if (torque_nm >= max_torque_nm) {
        return MOTOR_TORQUE_CURRENT_LIMIT_A;
    }
    if (torque_nm <= -max_torque_nm) {
        return -MOTOR_TORQUE_CURRENT_LIMIT_A;
    }
    return torque_nm / kt;
}

#if MOTOR_TORQUE_USE_POT
/* 为什么需要：将单路电位器变成可正反向调节的力矩旋钮，并抑制中点抖动。
 * 参数：filtered_pot 是经过4095-raw反向映射和低通滤波的12位ADC计数。
 * 返回：中点死区内为 0；死区外线性映射至 +/- MOTOR_TORQUE_POT_MAX_NM。
 * 减去死区后再归一化，使刚离开死区时从零连续增加，不会突然跳到一个非零力矩。
 * 原始 ADC 越小，反向映射后的给定越正；该符号不承诺实际轴端顺/逆时针方向。
 * 只生成力矩目标，电流限幅由后续 TorqueToIq 负责。
 */
static float motor_system_pot_torque_nm(float filtered_pot)
{
    float offset = filtered_pot - MOTOR_TORQUE_POT_CENTER;
    float magnitude = motor_abs_float(offset) - MOTOR_TORQUE_POT_DEADBAND;
    if (magnitude <= 0.0f) {
        return 0.0f;
    }
    float normalized = magnitude / (MOTOR_TORQUE_POT_CENTER - MOTOR_TORQUE_POT_DEADBAND);
    if (normalized > 1.0f) {
        normalized = 1.0f;
    }
    return ((offset > 0.0f) ? normalized : -normalized) * MOTOR_TORQUE_POT_MAX_NM;
}
#endif

/* 为什么需要：电位器采样、反向映射与滤波是力矩/位置模式共用的公共输入。
 * 功能：读取 ADC，做 4095-raw 反向映射，首次直接装入并做一阶 EMA 滤波。
 * 参数：无；返回映射到15位编码器范围的目标位置计数(0~32767)。
 * 副作用：更新 run_data.pot_raw、g_pot_target_filtered 与电位器初始化标记。
 * 调用上下文：每次 1ms 任务开始时由 Task 调用一次。
 */
float motor_system_update_pot_target(void)
{
    // ========== 步骤 2: 读取电位器给定 ==========
    // ADC 原始值 → 反向映射 (4095-raw) → 低通滤波。
    // 力矩模式将该计数映射成 N·m；保留的位置模式将该计数作为位置目标。
    g_motor_system.run_data.pot_raw = pot_read_raw();

    float target_pos_raw = 4095.0f - (float)g_motor_system.run_data.pot_raw;
    /* 首次直接装入采样，避免中点电位器从滤波初值 0 爬升造成反向力矩。 */
    if (g_pot_filter_initialized == 0U) {
        g_pot_target_filtered = target_pos_raw;
        g_pot_filter_initialized = 1U;
    }
    // 一阶 EMA 低通滤波: filtered += α * (raw - filtered)
    g_pot_target_filtered += POT_LPF_ALPHA * (target_pos_raw - g_pot_target_filtered);
    return g_pot_target_filtered * (MOTOR_ENCODER_COUNTS_PER_REV_F / 4096.0f);
}

/* 为什么需要：力矩模式给定换算独立成函数，使 Task 只表达模式调度。
 * 功能：从电位器或主机生成基础 Iq，再叠加补偿、限制总电流及主机输出斜率。
 * 参数：mechanical_speed_rpm 为实际机械 rpm，identified_direction 为换相方向 ±1，
 *       mechanical_angle_counts 为机械编码器计数；仅在 SENSORED_RUN/TORQUE 调用。
 * 副作用：更新 g_torque_current_a、g_foc_state.target_q/target_d，
 *         并可能更新 run_data.target_torque_nm。
 */
void motor_system_run_torque_mode(float mechanical_speed_rpm, int8_t identified_direction,
                                 uint16_t mechanical_angle_counts)
{
    if (g_input_source == MOTOR_INPUT_POT) {
        g_torque_current_a = motor_system_torque_to_iq(motor_system_pot_torque_nm(g_pot_target_filtered));
        g_host_iq_applied_a = g_torque_current_a;
    } else {
        float max_step = g_host_iq_slew_a_per_s * MOTOR_SYSTEM_TASK_DT_SEC;
        if (g_host_iq_slew_a_per_s <= 0.0f) {
            g_host_iq_applied_a = g_torque_current_a;
        } else if (g_torque_current_a > g_host_iq_applied_a + max_step) {
            g_host_iq_applied_a += max_step;
        } else if (g_torque_current_a < g_host_iq_applied_a - max_step) {
            g_host_iq_applied_a -= max_step;
        } else {
            g_host_iq_applied_a = g_torque_current_a;
        }
    }
    // 基础命令沿用电磁方向；仅摩擦模型需要从机械方向转换。
    g_debug_friction_iq_a =
        motor_feedforward_friction_compensation(mechanical_speed_rpm) * (float)identified_direction;
    g_debug_cogging_iq_a = motor_cogging_compensation(mechanical_angle_counts, mechanical_speed_rpm);
    // 电位器给定与补偿合计仍服从原旋钮满量程电流，不能借补偿扩大输出边界。
    float iq_limit = (g_input_source == MOTOR_INPUT_POT)
                         ? fminf(MOTOR_TORQUE_POT_MAX_CURRENT_A, MOTOR_TORQUE_CURRENT_LIMIT_A)
                         : g_host_iq_limit_a;
    float iq = g_host_iq_applied_a + g_debug_friction_iq_a + g_debug_cogging_iq_a;
    if (!isfinite(iq)) {
        iq = 0.0f;
    }
    iq = fmaxf(-iq_limit, fminf(iq_limit, iq));

    /* 补偿不能绕过主机斜率；上一拍总输出由现有目标保存，停机时统一清零。
     * g_host_iq_applied_a 仍只保存基础命令，避免下一周期把补偿重复累加。
     */
    if ((g_input_source != MOTOR_INPUT_POT) && (g_host_iq_slew_a_per_s > 0.0f)) {
        float max_step = g_host_iq_slew_a_per_s * MOTOR_SYSTEM_TASK_DT_SEC;
        iq = fmaxf(g_foc_state.target_q - max_step, fminf(g_foc_state.target_q + max_step, iq));
    }
    g_foc_state.target_q = fmaxf(-iq_limit, fminf(iq_limit, iq));
    g_foc_state.target_d = 0.0f;
    if (isfinite(MOTOR_TORQUE_CONSTANT_NM_PER_A) && (MOTOR_TORQUE_CONSTANT_NM_PER_A > 0.0f)) {
        // 继续报告基础命令的估算力矩；补偿电流由独立诊断字段报告，不是轴端实测力矩。
        g_motor_system.run_data.target_torque_nm = g_host_iq_applied_a * MOTOR_TORQUE_CONSTANT_NM_PER_A;
    }
}

/* 为什么需要：位置/速度级联计算独立成函数，使 Task 只表达模式调度。
 * 功能：用目标位置与实际位置跑位置环和速度环，直接生成 Iq。
 * 参数：target_pos 为电位器目标位置计数；current_rpm 为按电磁正方向统一后的转速。
 * 调用上下文：状态为 SENSORED_RUN 且未选择力矩模式时，由 Task 调用。
 * 副作用：更新位置/速度 PID 和 g_foc_state.target_q。
 */
void motor_system_run_position_mode(float target_pos, float current_rpm)
{
    float actual_pos = (float)motor_encoder_get_raw_angle();
    float position_error = target_pos - actual_pos;
    if (position_error > MOTOR_ENCODER_HALF_REV_F) {
        position_error -= MOTOR_ENCODER_COUNTS_PER_REV_F;
    } else if (position_error < -MOTOR_ENCODER_HALF_REV_F) {
        position_error += MOTOR_ENCODER_COUNTS_PER_REV_F;
    }
    g_debug_position_target_counts = target_pos;
    g_debug_position_actual_counts = actual_pos;
    g_debug_position_error_counts = position_error;

    float iq_max = g_cal_saved_valid && motor_calibration_is_point_mode() ? g_host_iq_limit_a : MOTOR_SPEED_PID_OUT_MAX;
    float iq_min =
        g_cal_saved_valid && motor_calibration_is_point_mode() ? -g_host_iq_limit_a : MOTOR_SPEED_PID_OUT_MIN;

    // 位置误差生成目标机械速度，也作为摩擦模型的输入。
    float target_mech_rpm = motor_position_loop_run(target_pos, actual_pos);
    g_debug_position_target_speed_rpm = target_mech_rpm;

    // 将机械期望转速乘以 uvw_dir 统一符号后给到速度环，防止正反馈。
    // target_signed_rpm 是按电磁转矩正方向统一符号的机械 RPM，没有乘极对数。
    float uvw_dir = (float)motor_identify_get_result().uvw_dir;
    float target_signed_rpm = target_mech_rpm * uvw_dir;

    /* 逐点标定仅在接近目标时使用已保存的摩擦辅助。
     * 辅助关闭并重新完成稳定窗口后才开始正式采样。 */
    float friction_iq;
    if (g_cal_saved_valid && motor_calibration_is_point_mode()) {
        friction_iq = motor_calibration_point_assist()
                          ? motor_feedforward_friction_with_config(&g_cal_saved_friction, target_mech_rpm) * uvw_dir
                          : 0.0f;
    } else {
        friction_iq = motor_feedforward_friction_compensation(target_mech_rpm) * uvw_dir;
    }
    friction_iq = fmaxf(iq_min, fminf(iq_max, friction_iq));
    g_debug_friction_iq_a = friction_iq;
    // 点位标定保留原始采样及摩擦辅助流程，不引入齿槽补偿污染待测模型。
    g_debug_cogging_iq_a = (g_cal_saved_valid && motor_calibration_is_point_mode())
                               ? 0.0f
                               : motor_cogging_compensation((uint16_t)actual_pos, g_motor_system.run_data.speed_rpm);
    float feedforward_iq = friction_iq + g_debug_cogging_iq_a;
    speed_pid.out_max = iq_max - feedforward_iq;
    speed_pid.out_min = iq_min - feedforward_iq;

    // 位置环输出的目标速度直接传给速度环。
    motor_speed_loop_set_target(target_signed_rpm);

    // 前馈占用的余量反馈给速度PI抗饱和；总电流仍服从原位置模式限幅。
    g_debug_speed_loop_iq_a = motor_speed_loop_update(current_rpm);
    g_foc_state.target_q = fmaxf(iq_min, fminf(iq_max, g_debug_speed_loop_iq_a + feedforward_iq));
    g_foc_state.target_d = 0.0f;
}

// 将主机 Iq（A）夹在当前正负电流限幅内。
static float motor_system_clamp_host_iq(float iq)
{
    if (!isfinite(iq)) {
        return 0.0f;
    }
    if (iq > g_host_iq_limit_a) {
        return g_host_iq_limit_a;
    }
    if (iq < -g_host_iq_limit_a) {
        return -g_host_iq_limit_a;
    }
    return iq;
}

// 以 1 ms 步长和 A/s 斜率更新已应用 Iq；斜率为零时保持上次值。
float motor_system_slew_host_iq(float target_iq, float slew_a_per_s)
{
    float max_step = slew_a_per_s * MOTOR_SYSTEM_TASK_DT_SEC;
    if (target_iq > g_host_iq_applied_a + max_step) {
        g_host_iq_applied_a += max_step;
    } else if (target_iq < g_host_iq_applied_a - max_step) {
        g_host_iq_applied_a -= max_step;
    } else {
        g_host_iq_applied_a = target_iq;
    }
    return g_host_iq_applied_a;
}

// 1 ms 速度模式计算速度 PI 和摩擦/齿槽前馈 Iq（A），保留叠加、限幅及发布次序。
void motor_system_run_speed_mode(float mechanical_speed_rpm, int8_t identified_direction,
                                 uint16_t mechanical_angle_counts)
{
    const float max_step = g_host_speed_slew_rpm_per_s * MOTOR_SYSTEM_TASK_DT_SEC;
    if (g_host_speed_slew_rpm_per_s <= 0.0f) {
        g_host_speed_applied_rpm = g_host_speed_target_rpm;
    } else if (g_host_speed_target_rpm > g_host_speed_applied_rpm + max_step) {
        g_host_speed_applied_rpm += max_step;
    } else if (g_host_speed_target_rpm < g_host_speed_applied_rpm - max_step) {
        g_host_speed_applied_rpm -= max_step;
    } else {
        g_host_speed_applied_rpm = g_host_speed_target_rpm;
    }

    /* SPEED按目标方向提前给摩擦前馈。实际速度为零时也能提供起步电流，
     * 避免只能等待积分累积后突然挣脱。FREE仍使用实际速度，防止静止自驱。 */
    g_debug_friction_iq_a =
        motor_feedforward_friction_compensation(g_host_speed_applied_rpm) * (float)identified_direction;
    g_debug_cogging_iq_a = motor_cogging_compensation(mechanical_angle_counts, mechanical_speed_rpm);
    float feedforward_iq = g_debug_friction_iq_a + g_debug_cogging_iq_a;

    /* 将前馈占用的电流余量反馈给PI抗饱和，防止总Iq已经限幅时积分仍增长。 */
    speed_pid.out_max = g_host_iq_limit_a - feedforward_iq;
    speed_pid.out_min = -g_host_iq_limit_a - feedforward_iq;
    motor_speed_loop_set_target(g_host_speed_applied_rpm * (float)identified_direction);
    g_debug_speed_loop_iq_a = motor_speed_loop_update(mechanical_speed_rpm * (float)identified_direction);
    float iq = motor_system_clamp_host_iq(g_debug_speed_loop_iq_a + feedforward_iq);
    g_cal_control_flags = (fabsf(iq) >= g_host_iq_limit_a - 1e-6f) ? CAL_LIMIT : 0U;
    g_foc_state.target_q = motor_system_slew_host_iq(iq, MOTOR_SPEED_IQ_SLEW_A_PER_S);
    if (fabsf(g_foc_state.target_q - iq) > 1e-6f) {
        g_cal_control_flags |= CAL_SLEW;
    }
    g_foc_state.target_d = 0.0f;
}

/* 仅供前台 VOFA 观测：读取不关中断、不做 I/O、不调用算法。 */
float motor_system_get_debug_pot_target(void)
{
    return g_pot_target_filtered;
}
