#include "svpwm.h"
#include "motor_system.h"
#include "tim.h"
#include <math.h>
#include <stddef.h>
// SVPWM 全局状态实例。
SVPWM_State g_svpwm = {0};

// TIM1 自动重装载值（中心对齐模式）。
#define SVPWM_ARR       (4249U)

// 死区补偿（占 ARR 的比例，用于防止上下管直通）。
// 实际死区由 TIM1 BreakDeadTime 硬件管理，这里仅做占空比边沿留白。
#define SVPWM_DEADBAND  (0.02f)

// 有效占空比范围 [min_duty, max_duty]，留出死区余量。
#define SVPWM_MIN_DUTY  (SVPWM_DEADBAND)
#define SVPWM_MAX_DUTY  (1.0f - SVPWM_DEADBAND)

// 快速限幅宏。
static inline float clamp(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

// 初始化 SVPWM 模块。
void SVPWM_Init(void)
{
    g_svpwm.duty_a = 0.5f;
    g_svpwm.duty_b = 0.5f;
    g_svpwm.duty_c = 0.5f;
    g_svpwm.v_alpha = 0.0f;
    g_svpwm.v_beta = 0.0f;
    g_svpwm.v_bus   = SYSTEM_BUS_VOLTAGE;   // 默认 SYSTEM_BUS_VOLTAGE 母线
    g_svpwm.mod_index = 0.0f;
    g_svpwm.enabled = 0;

    // 初始化 PWM 输出为 50% 零矢量（上电安全状态）
    uint32_t half_arr = SVPWM_ARR / 2U;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, half_arr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, half_arr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, half_arr);
}

// ---------------------------------------------------------
// SVPWM 核心算法（共模注入法 / Min-Max 法）
// 数学上等价于经典七段式 SVPWM，代码量更小、执行更快。
// ---------------------------------------------------------
void SVPWM_SetVoltage(float v_alpha, float v_beta, float v_bus)
{
    // 记录输入（供外部调试查看）
    g_svpwm.v_alpha = v_alpha;
    g_svpwm.v_beta  = v_beta;
    g_svpwm.v_bus   = (v_bus > 0.1f) ? v_bus : SYSTEM_BUS_VOLTAGE;

    if (!g_svpwm.enabled)
    {
        // 未使能时不更新 CCR，保持上次状态
        return;
    }

    // 1. 逆 Clarke 变换：α-β → 三相电压 (Va, Vb, Vc)
    //    Va = Vα
    //    Vb = -0.5*Vα + (√3/2)*Vβ
    //    Vc = -0.5*Vα - (√3/2)*Vβ
    const float sqrt3_2 = 0.86602540378f;  // √3 / 2
    float va = v_alpha;
    float vb = -0.5f * v_alpha + sqrt3_2 * v_beta;
    float vc = -0.5f * v_alpha - sqrt3_2 * v_beta;

    // 2. 共模注入法（Min-Max）：计算零序分量并注入
    //    找到三相中的最大值和最小值
    float v_max = va;
    if (vb > v_max) v_max = vb;
    if (vc > v_max) v_max = vc;

    float v_min = va;
    if (vb < v_min) v_min = vb;
    if (vc < v_min) v_min = vc;

    // 共模分量 = -(max+min)/2，注入后等价于 SVPWM 的七段式调制
    float v_com = -0.5f * (v_max + v_min);

    va += v_com;
    vb += v_com;
    vc += v_com;

    // 3. 归一化：将相电压映射到 [0, 1] 占空比
    //    母线电压 Vbus 对应满调制幅值
    //    占空比 = (Vphase / Vbus) + 0.5
    float v_norm = 0.5f / g_svpwm.v_bus;  // 缩放因子
    float duty_a = va * v_norm + 0.5f;
    float duty_b = vb * v_norm + 0.5f;
    float duty_c = vc * v_norm + 0.5f;

    // 4. 过调制处理：如果占空比超出 [0, 1]，等比例缩放
    float d_max = duty_a;
    if (duty_b > d_max) d_max = duty_b;
    if (duty_c > d_max) d_max = duty_c;

    float d_min = duty_a;
    if (duty_b < d_min) d_min = duty_b;
    if (duty_c < d_min) d_min = duty_c;

    float range = d_max - d_min;
    if (range > 1.0f)
    {
        // 超出线性调制范围，等比例压缩
        float scale = 1.0f / range;
        float mid = 0.5f * (d_max + d_min);
        duty_a = mid + (duty_a - mid) * scale;
        duty_b = mid + (duty_b - mid) * scale;
        duty_c = mid + (duty_c - mid) * scale;
    }
    else
    {
        // 整体偏移确保不超出 [0, 1]
        if (d_max > 1.0f)
        {
            float offset = d_max - 1.0f;
            duty_a -= offset;
            duty_b -= offset;
            duty_c -= offset;
        }
        else if (d_min < 0.0f)
        {
            float offset = -d_min;
            duty_a += offset;
            duty_b += offset;
            duty_c += offset;
        }
    }

    // 5. 死区安全边界限制
    duty_a = clamp(duty_a, SVPWM_MIN_DUTY, SVPWM_MAX_DUTY);
    duty_b = clamp(duty_b, SVPWM_MIN_DUTY, SVPWM_MAX_DUTY);
    duty_c = clamp(duty_c, SVPWM_MIN_DUTY, SVPWM_MAX_DUTY);

    // 6. 保存占空比（供调试）
    g_svpwm.duty_a = duty_a;
    g_svpwm.duty_b = duty_b;
    g_svpwm.duty_c = duty_c;

    // 7. 计算调制比（供调试）
    float v_mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    g_svpwm.mod_index = v_mag / (g_svpwm.v_bus * 0.577350269f); // Vbus/√3

    // 8. 写入 TIM1 比较寄存器（更新三相占空比）
    uint32_t ccr_a = (uint32_t)(duty_a * (float)SVPWM_ARR + 0.5f);
    uint32_t ccr_b = (uint32_t)(duty_b * (float)SVPWM_ARR + 0.5f);
    uint32_t ccr_c = (uint32_t)(duty_c * (float)SVPWM_ARR + 0.5f);

    // 边界钳位防止硬件错误
    if (ccr_a > SVPWM_ARR) ccr_a = SVPWM_ARR;
    if (ccr_b > SVPWM_ARR) ccr_b = SVPWM_ARR;
    if (ccr_c > SVPWM_ARR) ccr_c = SVPWM_ARR;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_c);
}

void SVPWM_Enable(void)
{
    g_svpwm.enabled = 1U;
}

uint8_t SVPWM_IsEnabled(void)
{
    return g_svpwm.enabled;
}

// 禁用 SVPWM 输出：三相 50% 占空比（零矢量）。
void SVPWM_Disable(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_svpwm.enabled = 0U;

    uint32_t half_arr = SVPWM_ARR / 2U;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, half_arr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, half_arr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, half_arr);

    g_svpwm.duty_a = 0.5f;
    g_svpwm.duty_b = 0.5f;
    g_svpwm.duty_c = 0.5f;
    g_svpwm.v_alpha = 0.0f;
    g_svpwm.v_beta = 0.0f;
    g_svpwm.mod_index = 0.0f;
    if (primask == 0U) {
        __enable_irq();
    }
}
