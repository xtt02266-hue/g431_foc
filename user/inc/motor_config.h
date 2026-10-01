#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// -----------------------------------------
// 全局软硬件核心参数配置区
// -----------------------------------------
// 实际使用固定 15V 电源，硬件未采样母线电压，因此算法使用此配置值。
// 更换供电电压时须同步修改，供电压降不会被软件自动补偿。
#define SYSTEM_BUS_VOLTAGE    15.0f
// -----------------------------------------

/* GB4310 电机规格。数据表给出的电阻/电感是相间值；
 * 以星形绕组换算为 FOC 单相模型值时各除以 2。
 * 额定 24 V 是电机规格，不代替上面的控制板实际 15 V 母线配置。
 */
#define MOTOR_MODEL_NAME                         "GB4310"
#define MOTOR_RATED_VOLTAGE_V                    24.0f
#define MOTOR_RATED_CURRENT_A                     0.9f
#define MOTOR_PEAK_CURRENT_A                      1.9f
#define MOTOR_RATED_TORQUE_NM                     0.2f
#define MOTOR_PEAK_TORQUE_NM                      0.49f
#define MOTOR_RATED_SPEED_RPM                   504.0f
#define MOTOR_PEAK_SPEED_RPM                   1028.0f
#define MOTOR_SPEED_CONSTANT_RPM_PER_V            43.0f
#define MOTOR_SUPPLY_LIMITED_SPEED_RPM            (SYSTEM_BUS_VOLTAGE * MOTOR_SPEED_CONSTANT_RPM_PER_V)
#define MOTOR_TORQUE_CONSTANT_NM_PER_A             0.23f
#define MOTOR_INTERPHASE_RESISTANCE_OHM           10.32f
#define MOTOR_INTERPHASE_INDUCTANCE_H              0.00476f
#define MOTOR_NOMINAL_PHASE_RESISTANCE_OHM        (MOTOR_INTERPHASE_RESISTANCE_OHM * 0.5f)
#define MOTOR_NOMINAL_PHASE_INDUCTANCE_H           (MOTOR_INTERPHASE_INDUCTANCE_H * 0.5f)
#define MOTOR_POLE_PAIRS                           14U
#define MOTOR_ROTOR_INERTIA_KG_M2                   0.0000161f

#define MOTOR_TORQUE_CURRENT_LIMIT_A   MOTOR_PEAK_CURRENT_A
#define MOTOR_HOST_DEFAULT_IQ_LIMIT_A  MOTOR_RATED_CURRENT_A
#define MOTOR_HOST_HEARTBEAT_TIMEOUT_MS 500U
#define MOTOR_IQ_SLEW_A_PER_S          1.0f
#define MOTOR_HOST_IQ_SLEW_A_PER_S     0.0f
#define MOTOR_HOST_IQ_SLEW_MAX_A_PER_S 1000.0f
#define MOTOR_HOST_SPEED_MAX_RPM       MOTOR_SUPPLY_LIMITED_SPEED_RPM
#define MOTOR_HOST_SPEED_SLEW_RPM_PER_S 0.0f
#define MOTOR_HOST_SPEED_SLEW_MAX_RPM_PER_S 100000.0f
/* 诊断用开环强拖：旋转磁场升至机械 250 rpm，不经过位置/速度/电流闭环。
 * amplitude 是 Motor_OpenLoop_Drive 的 0..500 调制度，不是电流给定。
 */
#define MOTOR_FORCE_DRAG_SPEED_RPM       250.0f
#define MOTOR_FORCE_DRAG_POLE_PAIRS      MOTOR_POLE_PAIRS
#define MOTOR_FORCE_DRAG_ALIGN_AMPLITUDE 100.0f
#define MOTOR_FORCE_DRAG_RUN_AMPLITUDE   280.0f
#define MOTOR_FORCE_DRAG_ALIGN_MS        300U
#define MOTOR_FORCE_DRAG_RAMP_RPM_PER_S  250.0f
/* Speed闭环需要及时从驱动切换到制动；Free触觉仍使用上面的柔和斜率。 */
#define MOTOR_SPEED_IQ_SLEW_A_PER_S    5.0f

/* FREE 模式摩擦补偿的轻微反向速度阻尼，最多 20 mA。 */
#define MOTOR_FREE_FRICTION_DAMPING_A_PER_RAD_S 0.0001f
#define MOTOR_FREE_FRICTION_DAMPING_MAX_A       0.01f
#define MOTOR_PARAMETERS_AUTO_IDENTIFY 0
/* 保留旧宏供现有代码兼容；运行时由 MotorInputSource 决定 POT/HOST。 */
#define MOTOR_TORQUE_USE_POT           1
#define MOTOR_TORQUE_POT_CENTER        2047.5f // 12 位 ADC 的中点，对应零力矩
#define MOTOR_TORQUE_POT_DEADBAND      80.0f   // 中点两侧各 80 个计数置零，减少噪声引起的出力
/* 电位器两端限制为 +/-0.2 A，对应 GB4310 约 +/-0.046 N·m。 */
#define MOTOR_TORQUE_POT_MAX_CURRENT_A 0.2f


#define MOTOR_TORQUE_POT_MAX_NM        \
    (MOTOR_TORQUE_POT_MAX_CURRENT_A * MOTOR_TORQUE_CONSTANT_NM_PER_A)

#define MOTOR_SYSTEM_TASK_DT_SEC       0.001f
/*
 * MT6826S 15位SPI角度的速度估算始终每1 ms发布；只调整滚动位置窗长度。
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
