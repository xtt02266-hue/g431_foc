#include "motor_current_loop.h"
#include "adc.h"

// 电流环/FOC 全局运行状态与参数。
MotorCurrentLoopState g_foc_state = {0};

// 将 ADC 原始值转换为相电流（安培）。
static float Motor_CurrentLoop_RawToCurrent(uint16_t raw)
{
    float v_in = ((float)raw * g_foc_state.params.vref_volts) / g_foc_state.params.adc_max;
    float v_shunt = v_in - g_foc_state.params.bias_volts;
    return v_shunt / (g_foc_state.params.shunt_ohms * g_foc_state.params.gain);
}

// 更新电流采样并为后续控制环节预留入口。
void Motor_CurrentLoop_Run(uint16_t iu_raw, uint16_t iw_raw)
{
    // 1. 将 ADC 原始值转换为实际相电流（安培）
    float iu_a = Motor_CurrentLoop_RawToCurrent(iu_raw);
    float iw_a = Motor_CurrentLoop_RawToCurrent(iw_raw);

    // 记录采样值（用于调试或显示）
    g_foc_state.sample.iu_raw = iu_raw;
    g_foc_state.sample.iw_raw = iw_raw;
    g_foc_state.sample.iu_a = iu_a;
    g_foc_state.sample.iw_a = iw_a;

    // ----------------------------------------------------
    // 以下为整个 FOC 电流环的调用骨架
    // ----------------------------------------------------

    // 2. 获取当前电角度  (注意：原先的硬编码已替换为真实传递的参数)
    // 从 AS5600 缓存中极速获取 12 位原始机械角度 (0 ~ 4095)
    uint16_t raw_mech_angle = AS5600_ReadRawAngle();
    
    // 将 AS5600 的计数转换成真实机械弧度 (0 ~ 2π)
    float mech_angle = (float)raw_mech_angle * (6.2831853f / 4096.0f);
    
    // 减去在辨识阶段标定好、并传进来的绝对机械零点偏置
    float mech_offset = mech_angle - g_foc_state.params.zero_angle_offset;
    
    // 计算电气角度：电角度 = 机械角度偏差 * 极对数 * 相序方向
    float elec_angle = mech_offset * (float)g_foc_state.params.pole_pairs * (float)g_foc_state.params.uvw_dir;
    
    // 将电角度限制在 0 ~ 2π 之间 (这步对某些三角函数硬件加速库不仅防止溢出，还能加速)
    elec_angle = fmodf(elec_angle, 6.2831853f);
    if (elec_angle < 0.0f) {
        elec_angle += 6.2831853f;
    }
    
    // 供后续 Park 坐标变换使用的正余弦值
    g_foc_state.sin_theta = sinf(elec_angle);
    g_foc_state.cos_theta = cosf(elec_angle);

    // 3. Clarke 变换：将三相相电流（实际上只需两相，假设三相和为0）转换至两相静止坐标系 (Alpha-Beta)
    g_foc_state.clarke = Motor_CurrentLoop_Clarke(iu_a, iw_a);

    // 4. Park 变换：将静止坐标系转化为同步旋转坐标系 (D-Q)
    g_foc_state.park = Motor_CurrentLoop_Park(g_foc_state.clarke, g_foc_state.sin_theta, g_foc_state.cos_theta);

    // 5. 将 Park 算出来的实际 D-Q 轴电流传给 PID 作为测量值
    g_foc_state.pi_d.measure = g_foc_state.park.d;
    g_foc_state.pi_q.measure = g_foc_state.park.q;

    // 将外部期望的 D-Q 轴目标电流传给 PID (比如上层位置环/速度环计算出来的 target，通常 D轴目标为0)
    g_foc_state.pi_d.target = g_foc_state.target_d;
    g_foc_state.pi_q.target = g_foc_state.target_q;

    if (g_foc_state.closed_loop_enable)
    {
        // 6. 执行 PID 模块的计算，会自动将结果刷新到 g_foc_state.pi_d.output 和 pi_q.output 中
        PID_Calculate(&g_foc_state.pi_d);
        PID_Calculate(&g_foc_state.pi_q);
        
        // 7. 读取 PID 调整算出的新电压指令 (Vd, Vq)
        MotorParkFrame v_dq;
        v_dq.d = g_foc_state.pi_d.output;
        v_dq.q = g_foc_state.pi_q.output;

        // 8. 逆 Park 变换：将新的电压指令转为静止坐标系 (V_alpha, V_beta)
        MotorClarkeFrame v_ab;
        v_ab = Motor_CurrentLoop_InvPark(v_dq, g_foc_state.sin_theta, g_foc_state.cos_theta);

        // 9. SVPWM 生成：将计算出的 V_alpha/V_beta 生成占空比控制定时器（TODO：待实现）
        // Motor_CurrentLoop_SVPWM(v_ab.alpha, v_ab.beta);
    } 
    else 
    {
        // 闭环未使能时，停止执行PID，防止积分跑飞，并不对外输出任何SVPWM修改。
        // 这时可以安全执行辨识流程（开环控制直接操作定时器CCR寄存器而不会与闭环打架）
    }
}

// 初始化电流环参数与缓存。
void Motor_CurrentLoop_Init(void)
{
    g_foc_state.params.vref_volts = MOTOR_CURRENT_VREF_VOLTS;
    g_foc_state.params.bias_volts = MOTOR_CURRENT_BIAS_VOLTS;
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
    
    // 初始化 PID 参数（使用头文件定义的宏变量，便于集中管理）
    PID_Init(&g_foc_state.pi_d, 
             MOTOR_CURRENT_PID_D_KP, MOTOR_CURRENT_PID_D_KI, MOTOR_CURRENT_PID_D_KD, 
             MOTOR_CURRENT_PID_D_OUT_MAX, MOTOR_CURRENT_PID_D_OUT_MIN);

    PID_Init(&g_foc_state.pi_q, 
             MOTOR_CURRENT_PID_Q_KP, MOTOR_CURRENT_PID_Q_KI, MOTOR_CURRENT_PID_Q_KD, 
             MOTOR_CURRENT_PID_Q_OUT_MAX, MOTOR_CURRENT_PID_Q_OUT_MIN);
             
    g_foc_state.closed_loop_enable = 0; // 默认不上电闭环，等待辨识完成
}

// 设置所有参数。
void Motor_CurrentLoop_SetParams(MotorCurrentParams params)
{
    g_foc_state.params = params;
}

// 专门接收并刷新电机辨识后的机械参数
void Motor_CurrentLoop_SetMotorIdentityParams(uint16_t pole_pairs, float zero_angle_offset, int8_t uvw_dir)
{
    g_foc_state.params.pole_pairs = pole_pairs;
    g_foc_state.params.zero_angle_offset = zero_angle_offset;
    g_foc_state.params.uvw_dir = uvw_dir;
}

// FOC 闭环启停开关
void Motor_CurrentLoop_Enable(uint8_t enable)
{
    g_foc_state.closed_loop_enable = enable;
    if (!enable) {
        // 如果关闭闭环，必须立刻清空PID积分器，否则如果被外部外力转动，PID会积分饱和
        PID_Reset(&g_foc_state.pi_d);
        PID_Reset(&g_foc_state.pi_q);
    }
}

// 获取当前参数。
MotorCurrentParams Motor_CurrentLoop_GetParams(void)
{
    MotorCurrentParams params;

    __disable_irq();
    params = g_foc_state.params;
    __enable_irq();

    return params;
}

// 设置电流采样偏置电压。
void Motor_CurrentLoop_SetBiasVolts(float bias_volts)
{
    g_foc_state.params.bias_volts = bias_volts;
}

// 设置 ADC 参考电压。
void Motor_CurrentLoop_SetVrefVolts(float vref_volts)
{
    g_foc_state.params.vref_volts = vref_volts;
}

// 设置电流放大器增益。
void Motor_CurrentLoop_SetGain(float gain)
{
    g_foc_state.params.gain = gain;
}

// 设置采样电阻值。
void Motor_CurrentLoop_SetShuntOhms(float shunt_ohms)
{
    g_foc_state.params.shunt_ohms = shunt_ohms;
}

// 获取最近一次采样结果（临界区保护）。
MotorCurrentSample Motor_CurrentLoop_GetLastSample(void)
{
    MotorCurrentSample sample;

    __disable_irq();
    sample = g_foc_state.sample;
    __enable_irq();

    return sample;
}

// Clarke 变换：两相电流（U/W）到 α-β（假设 iU + iV + iW = 0）。
MotorClarkeFrame Motor_CurrentLoop_Clarke(float iu_a, float iw_a)
{
    MotorClarkeFrame ab;
    float iv_a = -(iu_a + iw_a);

    ab.alpha = iu_a;
    ab.beta = (iu_a + 2.0f * iv_a) * (0.57735026919f); // 1/sqrt(3)

    return ab;
}

// Park 变换：α-β 到 d-q。
MotorParkFrame Motor_CurrentLoop_Park(MotorClarkeFrame ab, float sin_theta, float cos_theta)
{
    MotorParkFrame dq;

    dq.d = ab.alpha * cos_theta + ab.beta * sin_theta;
    dq.q = -ab.alpha * sin_theta + ab.beta * cos_theta;

    return dq;
}

// 逆 Park 变换：d-q 到 α-β。
MotorClarkeFrame Motor_CurrentLoop_InvPark(MotorParkFrame dq, float sin_theta, float cos_theta)
{
    MotorClarkeFrame ab;

    ab.alpha = dq.d * cos_theta - dq.q * sin_theta;
    ab.beta = dq.d * sin_theta + dq.q * cos_theta;

    return ab;
}

// ADC 注入通道转换完成回调。
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1)
    {
        return;
    }

    // 读取注入通道 U/W 相电流原始值。
    uint16_t iu_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint16_t iw_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

    Motor_CurrentLoop_Run(iu_raw, iw_raw);
}
