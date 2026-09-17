/*
 * 无感 FOC 转子位置/速度观测器。
 *
 * 原理：
 *   1. 由电机电压方程反推反电动势（BEMF）：
 *          e_alpha = v_alpha - R * i_alpha - L * di_alpha/dt
 *          e_beta  = v_beta  - R * i_beta  - L * di_beta/dt
 *      其中 di/dt 用相邻两次电流采样做差分近似。
 *   2. 低通滤波得到平滑的 BEMF 矢量。
 *   3. 用 PLL（锁相环）跟踪 BEMF 矢量的相角，输出电角度与电角速度。
 *      相位误差为预测矢量与实测矢量的叉积，PI 调节后修正角度/速度。
 *   4. 通过锁相误差与 BEMF 幅值判断“锁定/丢失”状态，供上层判断是否可切换闭环。
 *
 * 所有 Update 由 20 kHz 电流环中断调用；配置/使能等接口做了中断保护，可与中断并发。
 */

#include "motor_sensorless.h"
#include "main.h"
#include <math.h>
#include <stddef.h>

#define MOTOR_SENSORLESS_TWO_PI            6.283185307f   /* 2π，用于角度归一化 */
#define MOTOR_SENSORLESS_RPM_SCALE          9.549296586f  /* rad/s -> rpm：60/(2π) */

/* 观测器内部运行状态（仅本文件可见）。 */
typedef struct
{
    MotorSensorlessConfig config;   /* 当前使用的配置参数 */
    MotorSensorlessOutput output;   /* 对外发布的观测结果 */

    float last_current_alpha;       /* 上一拍 alpha 电流，用于差分求 di/dt */
    float last_current_beta;        /* 上一拍 beta 电流 */
    float filtered_bemf_alpha;      /* 滤波后的 alpha 轴反电动势 (V) */
    float filtered_bemf_beta;       /* 滤波后的 beta 轴反电动势 (V) */
    float pll_integral_speed;       /* PLL 积分项累积的转速估计 (rad/s) */

    uint16_t lock_counter;          /* 相位误差持续在阈值内的次数，达 lock_updates 判定锁定 */
    uint16_t loss_counter;          /* BEMF 幅值持续偏低的次数，达 loss_updates 判定丢失 */
    uint8_t pll_counter;            /* PLL 分频计数，每 pll_divider 拍运行一次 PLL */
    uint8_t current_initialized;    /* 是否已记录上一拍电流（首拍只做初始化） */
    uint8_t phase_initialized;      /* 是否已用 BEMF 矢量完成初始相位捕获 */
    uint8_t enabled;                /* 模块使能标志，0 时 Update 直接返回 */
    int8_t direction;               /* 期望的电气转向：+1 或 -1 */
} MotorSensorlessInternal;

static MotorSensorlessInternal g_sensorless = {0};

/* 取绝对值。 */
static float Motor_Sensorless_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* 将 value 限制在 [minimum, maximum] 区间内。 */
static float Motor_Sensorless_Clamp(float value,
                                    float minimum,
                                    float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

/* 把角度归一化到 [0, 2π)。 */
static float Motor_Sensorless_WrapAngle(float angle)
{
    while (angle >= MOTOR_SENSORLESS_TWO_PI) {
        angle -= MOTOR_SENSORLESS_TWO_PI;
    }
    while (angle < 0.0f) {
        angle += MOTOR_SENSORLESS_TWO_PI;
    }
    return angle;
}

/* 复位观测器的所有运行时状态（保留 config/enabled/direction）。 */
static void Motor_Sensorless_ResetState(void)
{
    g_sensorless.output = (MotorSensorlessOutput){0};
    g_sensorless.output.status = g_sensorless.enabled
                                         ? MOTOR_SENSORLESS_SEARCHING
                                         : MOTOR_SENSORLESS_DISABLED;
    g_sensorless.last_current_alpha = 0.0f;
    g_sensorless.last_current_beta = 0.0f;
    g_sensorless.filtered_bemf_alpha = 0.0f;
    g_sensorless.filtered_bemf_beta = 0.0f;
    g_sensorless.pll_integral_speed = 0.0f;
    g_sensorless.lock_counter = 0U;
    g_sensorless.loss_counter = 0U;
    g_sensorless.pll_counter = 0U;
    g_sensorless.current_initialized = 0U;
    g_sensorless.phase_initialized = 0U;
}

/* 填充一组安全可用的默认配置（默认按 20 kHz 采样、云台电机参数估算）。 */
void Motor_Sensorless_GetDefaultConfig(MotorSensorlessConfig *config)
{
    if (config == NULL) {
        return;
    }

    config->sample_time_sec = MOTOR_SENSORLESS_SAMPLE_TIME_SEC; /* 50us -> 20kHz */
    config->resistance_ohm = 1.0f;              /* 相电阻默认值 (Ω) */
    config->inductance_h = 0.0006f;             /* 相电感默认值 (H) */
    config->pole_pairs = 1U;                    /* 极对数默认值 */
    config->bemf_filter_alpha = 0.05f;          /* BEMF 一阶低通系数，越小越平滑但滞后越大 */
    config->minimum_bemf_volts = 0.40f;         /* 判定“有反电动势”的最低幅值 (V) */
    config->pll_kp = 250.0f;                    /* PLL 比例增益 */
    config->pll_ki = 12000.0f;                  /* PLL 积分增益 */
    config->maximum_electrical_speed_rad_s = 8000.0f; /* 电角速度限幅 (rad/s) */
    config->lock_phase_error = 0.30f;           /* 判定锁定的相位误差阈值 (rad) */
    config->lock_updates = 100U;                /* 连续多少次相位误差合格才判定锁定 */
    config->loss_updates = 250U;                /* 连续多少次 BEMF 偏低才判定丢失 */
    config->pll_divider = 4U;                   /* PLL 降速运行分频，实际 5 kHz */
}

/* 用默认配置初始化模块。 */
void Motor_Sensorless_Init(void)
{
    Motor_Sensorless_GetDefaultConfig(&g_sensorless.config);
    g_sensorless.direction = 1;
    g_sensorless.enabled = MOTOR_SENSORLESS_DEFAULT_ENABLE;
    Motor_Sensorless_ResetState();
}

/*
 * 应用新的配置；参数非法时返回 0 且不改变现有配置。
 * 成功返回 1，并复位观测器状态。写入过程关中断，避免与电流环 Update 竞争。
 */
uint8_t Motor_Sensorless_Configure(const MotorSensorlessConfig *config)
{
    uint32_t primask;

    /* 逐项校验：时间、电机参数、滤波、PLL 及各类计数值必须为正且合法 */
    if ((config == NULL) ||
        !(config->sample_time_sec > 0.0f) ||
        !(config->resistance_ohm > 0.0f) ||
        !(config->inductance_h > 0.0f) ||
        (config->pole_pairs == 0U) ||
        !(config->bemf_filter_alpha > 0.0f) ||
        (config->bemf_filter_alpha > 1.0f) ||
        !(config->minimum_bemf_volts > 0.0f) ||
        !(config->pll_kp > 0.0f) ||
        !(config->pll_ki > 0.0f) ||
        !(config->maximum_electrical_speed_rad_s > 0.0f) ||
        !(config->lock_phase_error > 0.0f) ||
        (config->lock_updates == 0U) ||
        (config->loss_updates == 0U) ||
        (config->pll_divider == 0U)) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    g_sensorless.config = *config;
    Motor_Sensorless_ResetState();
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

/* 只更新电机本体参数（R/L/极对数），其余保持当前配置。 */
uint8_t Motor_Sensorless_ConfigureMotor(float resistance_ohm,
                                        float inductance_h,
                                        uint16_t pole_pairs)
{
    MotorSensorlessConfig config = g_sensorless.config;

    config.resistance_ohm = resistance_ohm;
    config.inductance_h = inductance_h;
    config.pole_pairs = pole_pairs;
    return Motor_Sensorless_Configure(&config);
}

/* 使能/失能观测器。切换时复位状态；关中断保证原子性。 */
void Motor_Sensorless_Enable(uint8_t enable)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_sensorless.enabled = (enable != 0U) ? 1U : 0U;
    Motor_Sensorless_ResetState();
    if (primask == 0U) {
        __enable_irq();
    }
}

/* 返回当前使能状态。 */
uint8_t Motor_Sensorless_IsEnabled(void)
{
    return g_sensorless.enabled;
}

/* 复位观测器状态（不影响配置与使能）。 */
void Motor_Sensorless_Reset(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    Motor_Sensorless_ResetState();
    if (primask == 0U) {
        __enable_irq();
    }
}

/* 设置期望电气转向，仅接受正/负方向，0 被忽略。 */
void Motor_Sensorless_SetDirection(int8_t direction)
{
    if (direction > 0) {
        g_sensorless.direction = 1;
    } else if (direction < 0) {
        g_sensorless.direction = -1;
    }
}

/*
 * 观测器主更新，由 20 kHz 电流环调用。
 * 输入为 alpha/beta 轴电压与电流；内部完成 BEMF 估算、滤波及 PLL 跟踪。
 */
void Motor_Sensorless_Update(float voltage_alpha,
                             float voltage_beta,
                             float current_alpha,
                             float current_beta)
{
    MotorSensorlessConfig *config = &g_sensorless.config;
    MotorSensorlessOutput *output = &g_sensorless.output;
    float derivative_alpha;
    float derivative_beta;
    float raw_bemf_alpha;
    float raw_bemf_beta;
    float bemf_magnitude;
    float measured_alpha;
    float measured_beta;
    float predicted_alpha;
    float predicted_beta;
    float pll_dt;
    float speed_limit;
    float phase_error;

    if (g_sensorless.enabled == 0U) {
        return; /* 未使能时不做任何观测 */
    }

    /* 首拍没有历史电流，仅记录基准，无法进行差分 */
    if (g_sensorless.current_initialized == 0U) {
        g_sensorless.last_current_alpha = current_alpha;
        g_sensorless.last_current_beta = current_beta;
        g_sensorless.current_initialized = 1U;
        return;
    }

    /* 用相邻采样差分近似电流导数 di/dt */
    derivative_alpha = (current_alpha - g_sensorless.last_current_alpha) /
                       config->sample_time_sec;
    derivative_beta = (current_beta - g_sensorless.last_current_beta) /
                      config->sample_time_sec;
    g_sensorless.last_current_alpha = current_alpha;
    g_sensorless.last_current_beta = current_beta;

    /* 电机电压方程反推反电动势：e = v - R*i - L*di/dt */
    raw_bemf_alpha = voltage_alpha -
                     config->resistance_ohm * current_alpha -
                     config->inductance_h * derivative_alpha;
    raw_bemf_beta = voltage_beta -
                    config->resistance_ohm * current_beta -
                    config->inductance_h * derivative_beta;

    /* 一阶低通滤波，抑制差分与 PWM 引入的高频噪声 */
    g_sensorless.filtered_bemf_alpha +=
        config->bemf_filter_alpha *
        (raw_bemf_alpha - g_sensorless.filtered_bemf_alpha);
    g_sensorless.filtered_bemf_beta +=
        config->bemf_filter_alpha *
        (raw_bemf_beta - g_sensorless.filtered_bemf_beta);

    /* 即使本拍不跑 PLL，也先把最新的 BEMF 发布出去 */
    output->bemf_alpha_volts = g_sensorless.filtered_bemf_alpha;
    output->bemf_beta_volts = g_sensorless.filtered_bemf_beta;

    /* PLL 降速运行：每 pll_divider 拍执行一次，降低计算量 */
    g_sensorless.pll_counter++;
    if (g_sensorless.pll_counter < config->pll_divider) {
        return;
    }
    g_sensorless.pll_counter = 0U;

    /* BEMF 矢量幅值，用于判断转子是否已转起来（幅值太小则无法辨向） */
    bemf_magnitude = sqrtf(g_sensorless.filtered_bemf_alpha *
                           g_sensorless.filtered_bemf_alpha +
                           g_sensorless.filtered_bemf_beta *
                           g_sensorless.filtered_bemf_beta);
    output->bemf_magnitude_volts = bemf_magnitude;
    pll_dt = config->sample_time_sec * (float)config->pll_divider; /* PLL 实际步长 */

    /* BEMF 过小：认为转子未转动或观测失效，按当前速度做角度外推并累计丢失计数 */
    if (bemf_magnitude < config->minimum_bemf_volts) {
        g_sensorless.lock_counter = 0U;
        if (g_sensorless.loss_counter < UINT16_MAX) {
            g_sensorless.loss_counter++;
        }
        if (g_sensorless.loss_counter >= config->loss_updates) {
            output->valid = 0U;
            /* 曾捕获过相位才算“丢失”，否则仍处于搜索阶段 */
            output->status = g_sensorless.phase_initialized
                                 ? MOTOR_SENSORLESS_LOST
                                 : MOTOR_SENSORLESS_SEARCHING;
        }
        output->electrical_angle_rad = Motor_Sensorless_WrapAngle(
            output->electrical_angle_rad +
            output->electrical_speed_rad_s * pll_dt);
        return;
    }

    g_sensorless.loss_counter = 0U;
    /* 归一化为单位矢量，仅保留方向信息 */
    measured_alpha = g_sensorless.filtered_bemf_alpha / bemf_magnitude;
    measured_beta = g_sensorless.filtered_bemf_beta / bemf_magnitude;

    /* 首次获得有效 BEMF 时，用其矢量方向直接捕获初始电角度，加速 PLL 收敛 */
    if (g_sensorless.phase_initialized == 0U) {
        output->electrical_angle_rad = Motor_Sensorless_WrapAngle(
            atan2f(-(float)g_sensorless.direction * measured_alpha,
                   (float)g_sensorless.direction * measured_beta));
        g_sensorless.phase_initialized = 1U;
    }

    /* 由当前估计角度构造预测 BEMF 方向，与实测方向做叉积得到相位误差 */
    predicted_alpha = -(float)g_sensorless.direction *
                      sinf(output->electrical_angle_rad);
    predicted_beta = (float)g_sensorless.direction *
                     cosf(output->electrical_angle_rad);
    phase_error = predicted_alpha * measured_beta -
                  predicted_beta * measured_alpha;
    output->pll_phase_error = phase_error;

    /* PLL 积分支路：累积转速。按方向限幅，禁止反向积分 */
    speed_limit = config->maximum_electrical_speed_rad_s;
    g_sensorless.pll_integral_speed += config->pll_ki * phase_error * pll_dt;
    if (g_sensorless.direction > 0) {
        g_sensorless.pll_integral_speed = Motor_Sensorless_Clamp(
            g_sensorless.pll_integral_speed, 0.0f, speed_limit);
    } else {
        g_sensorless.pll_integral_speed = Motor_Sensorless_Clamp(
            g_sensorless.pll_integral_speed, -speed_limit, 0.0f);
    }

    /* PLL 比例支路 + 积分支路合成转速估计，并再次限幅 */
    output->electrical_speed_rad_s =
        g_sensorless.pll_integral_speed + config->pll_kp * phase_error;
    if (g_sensorless.direction > 0) {
        output->electrical_speed_rad_s = Motor_Sensorless_Clamp(
            output->electrical_speed_rad_s, 0.0f, speed_limit);
    } else {
        output->electrical_speed_rad_s = Motor_Sensorless_Clamp(
            output->electrical_speed_rad_s, -speed_limit, 0.0f);
    }

    /* 用转速积分更新电角度，并换算机械转速 (rpm) */
    output->electrical_angle_rad = Motor_Sensorless_WrapAngle(
        output->electrical_angle_rad +
        output->electrical_speed_rad_s * pll_dt);
    output->mechanical_speed_rpm =
        output->electrical_speed_rad_s * MOTOR_SENSORLESS_RPM_SCALE /
        (float)config->pole_pairs;

    /* 锁相判据：相位误差持续足够小才认为 PLL 已锁定 */
    if (Motor_Sensorless_Abs(phase_error) <= config->lock_phase_error) {
        if (g_sensorless.lock_counter < UINT16_MAX) {
            g_sensorless.lock_counter++;
        }
    } else {
        g_sensorless.lock_counter = 0U;
        output->valid = 0U;
        output->status = MOTOR_SENSORLESS_SEARCHING;
    }

    /* 连续 lock_updates 次满足误差要求，输出可用并进入跟踪状态 */
    if (g_sensorless.lock_counter >= config->lock_updates) {
        output->valid = 1U;
        output->status = MOTOR_SENSORLESS_TRACKING;
    }
}

/* 原子读取一份输出快照，避免读到中断更新到一半的数据。 */
MotorSensorlessOutput Motor_Sensorless_GetOutput(void)
{
    MotorSensorlessOutput output;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    output = g_sensorless.output;
    if (primask == 0U) {
        __enable_irq();
    }
    return output;
}

/* 获取电角度 (rad)。 */
float Motor_Sensorless_GetElectricalAngle(void)
{
    return Motor_Sensorless_GetOutput().electrical_angle_rad;
}

/* 获取机械转速 (rpm)。 */
float Motor_Sensorless_GetMechanicalSpeedRpm(void)
{
    return Motor_Sensorless_GetOutput().mechanical_speed_rpm;
}

/* 输出是否有效（BEMF 足够且 PLL 已锁定）。 */
uint8_t Motor_Sensorless_IsValid(void)
{
    return Motor_Sensorless_GetOutput().valid;
}

/* 是否可交接给无感闭环：输出有效且处于跟踪状态。 */
uint8_t Motor_Sensorless_IsReadyForHandover(void)
{
    MotorSensorlessOutput output = Motor_Sensorless_GetOutput();
    return ((output.valid != 0U) &&
            (output.status == MOTOR_SENSORLESS_TRACKING)) ? 1U : 0U;
}
