#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// -----------------------------------------
// 全局软硬件核心参数配置区
// -----------------------------------------
// 电机型号：3508（减速器已拆除）；下列输出轴规格换算为电机轴参数使用。
// 当前仍使用固定 15V 电源；电机额定 24V 不等于实际母线电压。
// 更换供电电压时须同步修改，供电压降不会被软件自动补偿。
#define SYSTEM_BUS_VOLTAGE    15.0f
#define MOTOR_OUTPUT_SPEED_CONSTANT_RPM_PER_V 24.48f
#define MOTOR_GEAR_RATIO                    (3591.0f / 187.0f)
#define MOTOR_ROTOR_SPEED_CONSTANT_RPM_PER_V \
    (MOTOR_OUTPUT_SPEED_CONSTANT_RPM_PER_V * MOTOR_GEAR_RATIO)
#define MOTOR_THEORETICAL_MAX_RPM \
    (SYSTEM_BUS_VOLTAGE * MOTOR_ROTOR_SPEED_CONSTANT_RPM_PER_V)
#define MOTOR_NOMINAL_PHASE_RESISTANCE_OHM 0.194f
#define MOTOR_NOMINAL_PHASE_INDUCTANCE_H   0.000097f
#define MOTOR_NOMINAL_POLE_PAIRS           7U
#define MOTOR_NOMINAL_UVW_DIRECTION        1
// -----------------------------------------

#define MOTOR_TORQUE_CURRENT_LIMIT_A   1.5f // 力矩给定最终换算出的 Iq 绝对值上限，单位 A
#define MOTOR_HOST_DEFAULT_IQ_LIMIT_A  1.00f // 上位机接管时的默认运行限流；仍受 1.5 A 硬上限约束
#define MOTOR_HOST_HEARTBEAT_TIMEOUT_MS 500U
#define MOTOR_IQ_SLEW_A_PER_S          1.0f
#define MOTOR_HOST_SPEED_MAX_RPM       MOTOR_THEORETICAL_MAX_RPM
#define MOTOR_HOST_SPEED_SLEW_RPM_PER_S 1000.0f
/* Speed闭环需要及时从驱动切换到制动；Free触觉仍使用上面的柔和斜率。 */
#define MOTOR_SPEED_IQ_SLEW_A_PER_S    5.0f
/* 无感强拖启动：缩短低速找位阶段；250 rpm 交接门槛保持不变。
 * 无编码器时仍可能有一次转子定向动作，不能保证任意初始角度零抖动。 */
#define MOTOR_SENSORLESS_STARTUP_ALIGN_MS          250U
#define MOTOR_SENSORLESS_STARTUP_RAMP_MS          4000U
#define MOTOR_SENSORLESS_STARTUP_LOCK_TIMEOUT_MS  2500U
#define MOTOR_SENSORLESS_LOCK_STABLE_MS             100U
#define MOTOR_SENSORLESS_HANDOVER_BLEND_MS           100U
#define MOTOR_SENSORLESS_STARTUP_CURRENT_A        0.40f
#define MOTOR_SENSORLESS_HANDOVER_DEFAULT_RPM     250.0f
#define MOTOR_SENSORLESS_HANDOVER_MIN_RPM         100.0f
#define MOTOR_SENSORLESS_HANDOVER_MAX_RPM         600.0f
#define MOTOR_PARAMETERS_AUTO_IDENTIFY 0
/* 数据表的 0.3 N.m/A 是减速器输出轴指标；拆除减速器后换算到电机轴。 */
#define MOTOR_OUTPUT_TORQUE_CONSTANT_NM_PER_A 0.3f
#define MOTOR_TORQUE_CONSTANT_NM_PER_A \
    (MOTOR_OUTPUT_TORQUE_CONSTANT_NM_PER_A / MOTOR_GEAR_RATIO)
/* 20 kHz ADC 电流采样一阶低通系数；恢复到 0.25 以减小高速相位滞后。 */
#define MOTOR_CURRENT_ADC_FILTER_ALPHA 0.25f
/* 保留旧宏供现有代码兼容；运行时由 MotorInputSource 决定 POT/HOST。 */
#define MOTOR_TORQUE_USE_POT           1
#define MOTOR_TORQUE_POT_CENTER        2047.5f // 12 位 ADC 的中点，对应零力矩
#define MOTOR_TORQUE_POT_DEADBAND      80.0f   // 中点两侧各 80 个计数置零，减少噪声引起的出力
/* 电位器两端先按 N·m 生成目标力矩，再由 Iq=T/Kt 换算进电流环。
 * 当前调试范围限制为 +/-0.2 A 对应的电磁力矩；修改电机 Kt 时会自动同步。
 Iq	估算扭矩
0.05 A	0.00078 N·m
0.10 A	0.00156 N·m
0.20 A	0.00312 N·m
0.50 A	0.00781 N·m
1.00 A	0.01562 N·m
 */
#define MOTOR_TORQUE_POT_MAX_CURRENT_A 0.2f // 约 ±3.12 mN·m（电机轴）


#define MOTOR_TORQUE_POT_MAX_NM        \
    (MOTOR_TORQUE_POT_MAX_CURRENT_A * MOTOR_TORQUE_CONSTANT_NM_PER_A)

#define MOTOR_SYSTEM_TASK_DT_SEC       0.001f
/*
 * 编码器抽象层的15位角度速度估算始终每1 ms发布；只调整滚动位置窗长度。
 * 窗口切换使用迟滞，避免临界转速附近来回切换。
 */
#define SPEED_EST_WINDOW_TICKS         20U
#define SPEED_EST_MID_WINDOW_TICKS     10U
#define SPEED_EST_HIGH_WINDOW_TICKS     5U
#define SPEED_EST_LOW_TO_MID_RPM       35.0f
#define SPEED_EST_MID_TO_LOW_RPM       25.0f
#define SPEED_EST_MID_TO_HIGH_RPM      85.0f
#define SPEED_EST_HIGH_TO_MID_RPM      70.0f

#endif
