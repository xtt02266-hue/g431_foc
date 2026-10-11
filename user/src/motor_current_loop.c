#include "motor_current_loop.h"
#include "motor_config.h"
#include "board_profile.h"
#include "adc.h"
#include "svpwm.h"
#include "motor_encoder.h"
#include "motor_music.h"
#include "motor_calibration.h"
#include <math.h>

// 一阶低通滤波：滤除 PWM 开关噪声和 ADC 采样毛刺
// alpha 越大滤波越弱（响应越快），越小滤波越强（越平滑）
// 本配置采用 alpha=0.25；滤波按每次电流采样更新，保持原控制周期
#define ADC_FILTER_ALPHA 0.25f

// 电流环/FOC 全局运行状态与参数。
motor_current_loop_state_t g_foc_state = {0};

// 将 ADC 原始值转换为相电流（安培）。
static float motor_current_loop_raw_to_current(uint16_t raw, float bias_volts)
{
    float v_in = ((float)raw * g_foc_state.params.vref_volts) / g_foc_state.params.adc_max;
    float v_shunt = v_in - bias_volts;
    return v_shunt / (g_foc_state.params.shunt_ohms * g_foc_state.params.gain);
}

// ADC 电流路径完成 U/W 相电流换算、Clarke/Park、dq PID 与 SVPWM；输入为 ADC 计数。
void motor_current_loop_run(uint16_t iu_raw, uint16_t iw_raw)
{
    // 1. 将 ADC 原始值转换为实际相电流（安培）
    float iu_a = motor_current_loop_raw_to_current(iu_raw, g_foc_state.params.bias_u_volts);
    float iw_a = motor_current_loop_raw_to_current(iw_raw, g_foc_state.params.bias_w_volts);

    // 记录采样值（用于调试或显示）
    g_foc_state.sample.iu_raw = iu_raw;
    g_foc_state.sample.iw_raw = iw_raw;
    g_foc_state.sample.iu_a = -iu_a;
    g_foc_state.sample.iw_a = iw_a;
    // 以下为整个 FOC 电流环的调用骨架
    // 2. 读取 SPI DMA 最近发布的 MT6826S 15 位机械角缓存；请求 DMA 不代表本次中断已获新样本。
    uint16_t raw_mech_angle = motor_encoder_get_raw_angle();

    float mech_angle = (float)raw_mech_angle * MOTOR_ENCODER_RAD_PER_COUNT;

    // 减去在辨识阶段标定好、并传进来的绝对机械零点偏置
    float mech_offset = mech_angle - g_foc_state.params.zero_angle_offset;

    // 计算电气角度：电角度 = 机械角度偏差 * 极对数 * 相序方向
    // 安全防护：极对数不能为 0（未辨识或辨识失败时兜底）
    uint16_t pp = g_foc_state.params.pole_pairs;
    if (pp == 0) {
        pp = 1;
    }
    float elec_angle = mech_offset * (float)pp * (float)g_foc_state.params.uvw_dir;

    // 将电角度限制在 0 ~ 2π 之间 (这步对某些三角函数硬件加速库不仅防止溢出，还能加速)
    // 替换为高效的边界限制逻辑
    while (elec_angle >= 6.2831853f) {
        elec_angle -= 6.2831853f;
    }

    while (elec_angle < 0.0f) {
        elec_angle += 6.2831853f;
    }
    g_foc_state.electrical_angle_rad = elec_angle;

    // 供后续 Park 坐标变换使用的正余弦值
    g_foc_state.sin_theta = sinf(elec_angle);
    g_foc_state.cos_theta = cosf(elec_angle);

    // 3. Clarke 变换：将三相相电流（实际上只需两相，假设三相和为0）转换至两相静止坐标系 (Alpha-Beta)
    g_foc_state.clarke = motor_current_loop_clarke(g_foc_state.sample.iu_a, g_foc_state.sample.iw_a);

    // 4. Park 变换：将静止坐标系转化为同步旋转坐标系 (D-Q)
    g_foc_state.park = motor_current_loop_park(g_foc_state.clarke, g_foc_state.sin_theta, g_foc_state.cos_theta);

    // 5. 将 Park 算出来的实际 D-Q 轴电流传给 PID 作为测量值
    g_foc_state.pi_d.measure = g_foc_state.park.d;
    g_foc_state.pi_q.measure = g_foc_state.park.q;

    // 将外部期望的 D-Q 轴目标电流传给 PID (比如上层位置环/速度环计算出来的 target，通常 D轴目标为0)
    g_foc_state.pi_d.target = g_foc_state.target_d;
    g_foc_state.pi_q.target = motor_music_process_iq_target(g_foc_state.target_q);
    motor_calibration_capture(raw_mech_angle, g_foc_state.pi_q.target, g_foc_state.park.q, g_foc_state.park.d,
                              g_foc_state.pi_q.output);

    if (g_foc_state.closed_loop_enable) {
        // 6. 执行 PID 模块的计算，会自动将结果刷新到 g_foc_state.pi_d.output 和 pi_q.output 中
        pid_calculate(&g_foc_state.pi_d);
        pid_calculate(&g_foc_state.pi_q);

        // 7. 读取 PID 调整算出的新电压指令 (Vd, Vq)
        motor_park_frame_t v_dq;
        v_dq.d = g_foc_state.pi_d.output;
        v_dq.q = g_foc_state.pi_q.output;

        // 8. 逆 Park 变换：将新的电压指令转为静止坐标系 (V_alpha, V_beta)
        motor_clarke_frame_t v_ab;
        v_ab = motor_current_loop_inv_park(v_dq, g_foc_state.sin_theta, g_foc_state.cos_theta);

        // 9. SVPWM 生成：将 V_alpha/V_beta 转换为三相占空比，写入 TIM1 CCR
        svpwm_set_voltage(v_ab.alpha, v_ab.beta, SYSTEM_BUS_VOLTAGE); // 第三个参数：母线电压(V)
    } else {
        // 闭环未使能时，停止执行PID，防止积分跑飞，并不对外输出任何SVPWM修改。
        // 这时可以安全执行辨识流程（开环控制直接操作定时器CCR寄存器而不会与闭环打架）
    }
}

// 初始化电流环参数与缓存。
void motor_current_loop_init(void)
{
    g_foc_state.params.vref_volts = MOTOR_CURRENT_VREF_VOLTS;
    g_foc_state.params.bias_u_volts = MOTOR_CURRENT_BIAS_VOLTS;
    g_foc_state.params.bias_w_volts = MOTOR_CURRENT_BIAS_VOLTS;
    g_foc_state.params.shunt_ohms = MOTOR_CURRENT_SHUNT_OHMS;
    g_foc_state.params.gain = MOTOR_CURRENT_GAIN;
    g_foc_state.params.adc_max = MOTOR_CURRENT_ADC_MAX;

    // 提供默认安全的未标定参数（为了防爆，极对数默认1）
    g_foc_state.params.pole_pairs = 1;
    g_foc_state.params.zero_angle_offset = 0.0f;
    g_foc_state.params.uvw_dir = 1;

    g_foc_state.sample.iu_raw = 0U;
    g_foc_state.sample.iw_raw = 0U;
    g_foc_state.sample.iu_a = 0.0f;
    g_foc_state.sample.iw_a = 0.0f;

    // 初始化 PID 参数（默认保守值，辨识完成后自动整定为最优值）
    // dt = 1/20000 = 50us (20kHz PWM 触发 ADC 采样频率)
    const float pid_dt = 0.00005f;
    pid_init(&g_foc_state.pi_d, MOTOR_CURRENT_PID_D_KP, MOTOR_CURRENT_PID_D_KI, MOTOR_CURRENT_PID_D_KD,
             MOTOR_CURRENT_PID_D_OUT_MAX, MOTOR_CURRENT_PID_D_OUT_MIN, pid_dt);

    pid_init(&g_foc_state.pi_q, MOTOR_CURRENT_PID_Q_KP, MOTOR_CURRENT_PID_Q_KI, MOTOR_CURRENT_PID_Q_KD,
             MOTOR_CURRENT_PID_Q_OUT_MAX, MOTOR_CURRENT_PID_Q_OUT_MIN, pid_dt);

    g_foc_state.closed_loop_enable = 0; // 默认不上电闭环，等待辨识完成
}

// 设置所有参数。
void motor_current_loop_set_params(float vref_volts, float bias_u_volts, float bias_w_volts, float shunt_ohms,
                                   float gain, float adc_max, uint16_t pole_pairs, float zero_angle_offset,
                                   int8_t uvw_dir)
{
    const motor_current_params_t params = {vref_volts, bias_u_volts, bias_w_volts,      shunt_ohms, gain,
                                           adc_max,    pole_pairs,   zero_angle_offset, uvw_dir};

    g_foc_state.params = params;
}

// 专门接收并刷新电机辨识后的机械参数
void motor_current_loop_set_motor_identity_params(uint16_t pole_pairs, float zero_angle_offset, int8_t uvw_dir)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_foc_state.params.pole_pairs = pole_pairs;
    g_foc_state.params.zero_angle_offset = zero_angle_offset;
    g_foc_state.params.uvw_dir = uvw_dir;
    if (primask == 0U) {
        __enable_irq();
    }
}

// FOC 闭环启停开关
void motor_current_loop_enable(uint8_t enable)
{
#if !BOARD_SENSORED_CONTROL_ENABLE
    enable = 0U;
#endif
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_foc_state.closed_loop_enable = (enable != 0U) ? 1U : 0U;
    // 无论启用还是停用，都重置 PID 积分，确保从零开始
    pid_reset(&g_foc_state.pi_d);
    pid_reset(&g_foc_state.pi_q);
    if (primask == 0U) {
        __enable_irq();
    }
}

// 读取电流环使能标志；该值不代表实际 PWM 已产生输出。
uint8_t motor_current_loop_is_enabled(void)
{
    return g_foc_state.closed_loop_enable;
}

// 根据辨识出的电机 R/L 自动计算最优电流环 PI 参数。
// 应在辨识完成后调用，使 d/q 轴电流快速跟随目标。
//
// 调参指南：
//   CURRENT_LOOP_BW_HZ  — 电流环带宽 (Hz)，越大响应越快但越容易振荡
//                        云台电机推荐 200~500，高速电机 500~2000
//   KI_DAMPING          — 积分阻尼系数 (0.5~1.0)，<1.0 可减少超调
#define CURRENT_LOOP_BW_HZ 200.0f // 电流环带宽 (Hz)
#define KI_DAMPING         0.8f   // 积分阻尼 (0.3=柔和, 0.6=较快, 1.0=理论值)
#define KP_DAMPING         1.0f   // 电流 PI 比例系数相对 R/L 理论调谐值的缩放

// 按辨识 R/L 和指定带宽 Hz 配置电流 PI；保持原有阻尼和电压限幅。
void motor_current_loop_auto_tune_pid_with_bandwidth(float resistance, float inductance, float bus_voltage,
                                                     float bandwidth_hz)
{
    uint32_t primask;

    if (resistance <= 0.0f || inductance <= 0.0f) {
        return;
    }
    if (bandwidth_hz < 50.0f) {
        bandwidth_hz = 50.0f;
    } else if (bandwidth_hz > 2000.0f) {
        bandwidth_hz = 2000.0f;
    }

    float dt = 0.00005f; // 20kHz 采样
    float wc = 2.0f * 3.1415926f * bandwidth_hz;

    float kp = inductance * wc * KP_DAMPING; // 比例 (V/A)
    float ki = resistance * wc * KI_DAMPING; // 积分 (V/(A·s))，阻尼抑制超调
    float kd = 0.0f;

    // SVPWM 线性调制区最大相电压幅值 = Vbus / √3
    // 留 5% 余量防止进入过调制导致削波 → PID 积分饱和
    float v_max = bus_voltage * 0.57735f * 0.95f; // Vbus/√3 × 0.95
    float out_max = v_max;
    float out_min = -v_max;

    primask = __get_PRIMASK();
    __disable_irq();
    pid_init(&g_foc_state.pi_d, kp, ki, kd, out_max, out_min, dt);
    pid_init(&g_foc_state.pi_q, kp, ki, kd, out_max, out_min, dt);
    if (primask == 0U) {
        __enable_irq();
    }
}

// 获取当前参数。
void motor_current_loop_auto_tune_pid(float resistance, float inductance, float bus_voltage)
{
    motor_current_loop_auto_tune_pid_with_bandwidth(resistance, inductance, bus_voltage, CURRENT_LOOP_BW_HZ);
}

// 读取当前电流换算参数对象；保留既有只读接口与对象生命周期。
motor_current_params_t motor_current_loop_get_params(void)
{
    motor_current_params_t params;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    params = g_foc_state.params;
    if (primask == 0U) {
        __enable_irq();
    }

    return params;
}

// 设置电流采样偏置电压。
void motor_current_loop_set_bias_volts(float bias_volts)
{
    motor_current_loop_set_phase_bias_volts(bias_volts, bias_volts);
}

// 分别设置 U/W 相采样零偏，单位 V，用于 ADC 原始值到电流的换算。
void motor_current_loop_set_phase_bias_volts(float bias_u_volts, float bias_w_volts)
{
    g_foc_state.params.bias_u_volts = bias_u_volts;
    g_foc_state.params.bias_w_volts = bias_w_volts;
}

// 设置 ADC 参考电压。
void motor_current_loop_set_vref_volts(float vref_volts)
{
    g_foc_state.params.vref_volts = vref_volts;
}

// 设置电流放大器增益。
void motor_current_loop_set_gain(float gain)
{
    g_foc_state.params.gain = gain;
}

// 设置采样电阻值。
void motor_current_loop_set_shunt_ohms(float shunt_ohms)
{
    g_foc_state.params.shunt_ohms = shunt_ohms;
}

// 获取最近一次采样结果（临界区保护）。
motor_current_sample_t motor_current_loop_get_last_sample(void)
{
    motor_current_sample_t sample;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    sample = g_foc_state.sample;
    if (primask == 0U) {
        __enable_irq();
    }

    return sample;
}

// Clarke 变换：两相电流（U/W）到 α-β（假设 iU + iV + iW = 0）。
motor_clarke_frame_t motor_current_loop_clarke(float iu_a, float iw_a)
{
    motor_clarke_frame_t ab;
    float iv_a = -(iu_a + iw_a);

    ab.alpha = iu_a;
    ab.beta = (iu_a + 2.0f * iv_a) * (0.57735026919f); // 1/sqrt(3)

    return ab;
}

// Park 变换：α-β 到 d-q。
motor_park_frame_t motor_current_loop_park(motor_clarke_frame_t ab, float sin_theta, float cos_theta)
{
    motor_park_frame_t dq;

    dq.d = ab.alpha * cos_theta + ab.beta * sin_theta;
    dq.q = -ab.alpha * sin_theta + ab.beta * cos_theta;

    return dq;
}

// 逆 Park 变换：d-q 到 α-β。
motor_clarke_frame_t motor_current_loop_inv_park(motor_park_frame_t dq, float sin_theta, float cos_theta)
{
    motor_clarke_frame_t ab;

    ab.alpha = dq.d * cos_theta - dq.q * sin_theta;
    ab.beta = dq.d * sin_theta + dq.q * cos_theta;

    return ab;
}

// ADC 注入通道转换完成回调。
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) {
        return;
    }

    // 读取注入通道 U/W 相电流原始值。
    uint16_t iu_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint16_t iw_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

    static float iu_filtered = 0.0f;
    static float iw_filtered = 0.0f;
    static uint8_t filter_inited = 0;

    if (!filter_inited) {
        // 首次采样直接装入，避免从 0 爬坡
        iu_filtered = (float)iu_raw;
        iw_filtered = (float)iw_raw;
        filter_inited = 1;
    } else {
        iu_filtered += ADC_FILTER_ALPHA * ((float)iu_raw - iu_filtered);
        iw_filtered += ADC_FILTER_ALPHA * ((float)iw_raw - iw_filtered);
    }

    motor_current_loop_run((uint16_t)iu_filtered, (uint16_t)iw_filtered);

    /* 使用上一帧完整的角度计算FOC，再异步采集下一帧。 */
    motor_encoder_update_fast();
}
