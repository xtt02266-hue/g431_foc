#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// -----------------------------------------
// 全局软硬件核心参数配置区
// -----------------------------------------
// 实际使用固定 15V 电源，硬件未采样母线电压，因此算法使用此配置值。
// 更换供电电压时须同步修改，供电压降不会被软件自动补偿。
#define SYSTEM_BUS_VOLTAGE    15.0f
// -----------------------------------------

#define MOTOR_TORQUE_CURRENT_LIMIT_A   1.5f // 力矩给定最终换算出的 Iq 绝对值上限，单位 A
#define MOTOR_HOST_DEFAULT_IQ_LIMIT_A  1.10f
#define MOTOR_HOST_HEARTBEAT_TIMEOUT_MS 500U
#define MOTOR_IQ_SLEW_A_PER_S          1.0f
#define MOTOR_HOST_SPEED_MAX_RPM       2000.0f
#define MOTOR_HOST_SPEED_SLEW_RPM_PER_S 0.0f
#define MOTOR_HOST_SPEED_SLEW_MAX_RPM_PER_S 100000.0f
/* 诊断用开环强拖：旋转磁场固定为机械 200 rpm，不经过位置/速度/电流闭环。
 * amplitude 是 Motor_OpenLoop_Drive 的 0..500 调制度，100 约为 20% 最大相电压。
 */
#define MOTOR_FORCE_DRAG_SPEED_RPM       200.0f
#define MOTOR_FORCE_DRAG_POLE_PAIRS      7U
#define MOTOR_FORCE_DRAG_AMPLITUDE       100.0f
#define MOTOR_FORCE_DRAG_ALIGN_MS        300U
/* Speed闭环需要及时从驱动切换到制动；Free触觉仍使用上面的柔和斜率。 */
#define MOTOR_SPEED_IQ_SLEW_A_PER_S    5.0f
#define MOTOR_PARAMETERS_AUTO_IDENTIFY 0
/* BM3514H 商品页：291 KV。按常用 FOC 近似 Kt=8.27/KV 估算。
 * 单位 N·m/A，Iq 峰值定义；非实测标定，现有 R/L 辨识不测 Kt。
 * 参考 https://docs.odriverobotics.com/v/latest/manual/control.html
 * 设为 0 表示未配置，电位器力矩输出为零。
 */
#define MOTOR_TORQUE_CONSTANT_NM_PER_A  (8.27f / 291.0f)
/* 保留旧宏供现有代码兼容；运行时由 MotorInputSource 决定 POT/HOST。 */
#define MOTOR_TORQUE_USE_POT           1
#define MOTOR_TORQUE_POT_CENTER        2047.5f // 12 位 ADC 的中点，对应零力矩
#define MOTOR_TORQUE_POT_DEADBAND      80.0f   // 中点两侧各 80 个计数置零，减少噪声引起的出力
/* 电位器两端先按 N·m 生成目标力矩，再由 Iq=T/Kt 换算进电流环。
 * 当前调试范围限制为 +/-0.2 A 对应的电磁力矩；修改电机 Kt 时会自动同步。
 Iq	估算扭矩
0.05 A	0.00142 N·m
0.10 A	0.00284 N·m
0.20 A	0.00568 N·m
0.50 A	0.01421 N·m
1.00 A	0.02842 N·m
 */
#define MOTOR_TORQUE_POT_MAX_CURRENT_A 0.2f//±5.68 mN·m


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
