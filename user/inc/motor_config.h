#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H
// 全局软硬件核心参数配置区
// 实际使用固定 15V 电源，硬件未采样母线电压，因此算法使用此配置值。
// 更换供电电压时须同步修改，供电压降不会被软件自动补偿。
#define SYSTEM_BUS_VOLTAGE 15.0f
/* GB4310 电机规格。数据表给出的电阻/电感是相间值；
 * 以星形绕组换算为 FOC 单相模型值时各除以 2。
 * 额定 24 V 是电机规格，不代替上面的控制板实际 15 V 母线配置。
 */
#define MOTOR_MODEL_NAME               "GB4310" // 既有电机规格名称，不代替实机型号确认
#define MOTOR_RATED_VOLTAGE_V          24.0f    // 规格额定电压 V；实际控制母线仍为 SYSTEM_BUS_VOLTAGE
#define MOTOR_RATED_CURRENT_A          0.9f     // 规格额定电流 A，作为默认主机 Iq 限幅
#define MOTOR_PEAK_CURRENT_A           1.9f     // 规格峰值电流 A，作为现有软件最大 Iq 边界
#define MOTOR_RATED_TORQUE_NM          0.2f     // 规格额定转矩 N·m
#define MOTOR_PEAK_TORQUE_NM           0.49f    // 规格峰值转矩 N·m
#define MOTOR_RATED_SPEED_RPM          504.0f   // 规格额定机械转速 rpm
#define MOTOR_PEAK_SPEED_RPM           1028.0f  // 规格峰值机械转速 rpm
#define MOTOR_SPEED_CONSTANT_RPM_PER_V 43.0f    // 既有速度常数 rpm/V，用于供电速度边界估算
#define MOTOR_SUPPLY_LIMITED_SPEED_RPM                                                                                 \
    (SYSTEM_BUS_VOLTAGE * MOTOR_SPEED_CONSTANT_RPM_PER_V) // 按配置母线与速度常数估算的目标转速边界 rpm
#define MOTOR_TORQUE_CONSTANT_NM_PER_A     0.23f          // 既有电磁转矩与 Iq 换算系数 N·m/A
#define MOTOR_INTERPHASE_RESISTANCE_OHM    10.32f         // 规格相间电阻 Ω，星形单相模型取一半
#define MOTOR_INTERPHASE_INDUCTANCE_H      0.00476f       // 规格相间电感 H，星形单相模型取一半
#define MOTOR_NOMINAL_PHASE_RESISTANCE_OHM (MOTOR_INTERPHASE_RESISTANCE_OHM * 0.5f) // 由相间值换算的单相标称电阻 Ω
#define MOTOR_NOMINAL_PHASE_INDUCTANCE_H   (MOTOR_INTERPHASE_INDUCTANCE_H * 0.5f)   // 由相间值换算的单相标称电感 H
#define MOTOR_POLE_PAIRS                   14U                                      // 既有电机极对数配置
#define MOTOR_ROTOR_INERTIA_KG_M2          0.0000161f                               // 既有转子惯量配置 kg·m²

#define MOTOR_TORQUE_CURRENT_LIMIT_A        MOTOR_PEAK_CURRENT_A           // 力矩给定的硬限幅 A
#define MOTOR_HOST_DEFAULT_IQ_LIMIT_A       MOTOR_RATED_CURRENT_A          // 主机取得控制权后的默认电流限幅 A
#define MOTOR_HOST_HEARTBEAT_TIMEOUT_MS     2000U                          // 主机控制权下的心跳失联停机门限 ms
#define MOTOR_IQ_SLEW_A_PER_S               1.0f                           // 一般触觉/力矩给定变化率 A/s
#define MOTOR_HOST_IQ_SLEW_A_PER_S          0.0f                           // 主机默认 Iq 斜率 A/s；0 保留立即给定语义
#define MOTOR_HOST_IQ_SLEW_MAX_A_PER_S      1000.0f                        // 可接受主机 Iq 斜率上限 A/s
#define MOTOR_HOST_SPEED_MAX_RPM            MOTOR_SUPPLY_LIMITED_SPEED_RPM // 主机目标速度限幅 rpm
#define MOTOR_HOST_SPEED_SLEW_RPM_PER_S     0.0f                           // 主机默认速度斜率 rpm/s；0 为立即给定
#define MOTOR_HOST_SPEED_SLEW_MAX_RPM_PER_S 100000.0f                      // 可接受主机速度斜率上限 rpm/s
/* 诊断用开环强拖：旋转磁场升至机械 250 rpm，不经过位置/速度/电流闭环。
 * amplitude 是 motor_open_loop_drive 的 0..500 调制度，不是电流给定。
 */
#define MOTOR_FORCE_DRAG_SPEED_RPM       250.0f           // 诊断开环旋转磁场对应的机械目标速度 rpm
#define MOTOR_FORCE_DRAG_POLE_PAIRS      MOTOR_POLE_PAIRS // 强拖电角速度换算使用的极对数
#define MOTOR_FORCE_DRAG_ALIGN_AMPLITUDE 100.0f           // 固定相位对齐幅值，使用开环 0..500 调制度
#define MOTOR_FORCE_DRAG_RUN_AMPLITUDE   280.0f           // 运行强拖幅值，使用开环 0..500 调制度
#define MOTOR_FORCE_DRAG_ALIGN_MS        300U             // 固定相位对齐时间 ms
#define MOTOR_FORCE_DRAG_RAMP_RPM_PER_S  250.0f           // 强拖机械速度爬升斜率 rpm/s
/* Speed闭环需要及时从驱动切换到制动；Free触觉仍使用上面的柔和斜率。 */
#define MOTOR_SPEED_IQ_SLEW_A_PER_S 5.0f

/* FREE 模式摩擦补偿的轻微反向速度阻尼，现有限幅为 10 mA。 */
#define MOTOR_FREE_FRICTION_DAMPING_A_PER_RAD_S 0.0001f
#define MOTOR_FREE_FRICTION_DAMPING_MAX_A       0.01f // 自由模式附加速度阻尼电流上限 A
#define MOTOR_PARAMETERS_AUTO_IDENTIFY          0     // 0 禁用上电自动辨识，沿用当前配置
/* 保留旧宏供现有代码兼容；运行时由 motor_input_source_t 决定 POT/HOST。 */
#define MOTOR_TORQUE_USE_POT      1
#define MOTOR_TORQUE_POT_CENTER   2047.5f // 12 位 ADC 的中点，对应零力矩
#define MOTOR_TORQUE_POT_DEADBAND 80.0f   // 中点两侧各 80 个计数置零，减少噪声引起的出力
/* 电位器两端限制为 +/-0.2 A，对应 GB4310 约 +/-0.046 N·m。 */
#define MOTOR_TORQUE_POT_MAX_CURRENT_A 0.2f

#define MOTOR_TORQUE_POT_MAX_NM                                                                                        \
    (MOTOR_TORQUE_POT_MAX_CURRENT_A * MOTOR_TORQUE_CONSTANT_NM_PER_A) // 由电位器 Iq 限幅与转矩常数换算的目标转矩 N·m

#define MOTOR_SYSTEM_TASK_DT_SEC 0.001f // 系统控制任务每次更新的时间步长 s
/*
 * MT6826S 15位SPI角度的速度估算始终每1 ms发布；只调整滚动位置窗长度。
 * 窗口切换使用迟滞，避免临界转速附近来回切换。
 */
#define SPEED_EST_WINDOW_TICKS      20U   // 低速位置差分窗口，单位 1 ms 周期
#define SPEED_EST_MID_WINDOW_TICKS  10U   // 中速位置差分窗口，单位 1 ms 周期
#define SPEED_EST_HIGH_WINDOW_TICKS 5U    // 高速位置差分窗口，单位 1 ms 周期
#define SPEED_EST_LOW_TO_MID_RPM    35.0f // 低速切到中速窗口的迟滞门限 rpm
#define SPEED_EST_MID_TO_LOW_RPM    25.0f // 中速退回低速窗口的迟滞门限 rpm
#define SPEED_EST_MID_TO_HIGH_RPM   85.0f // 中速切到高速窗口的迟滞门限 rpm
#define SPEED_EST_HIGH_TO_MID_RPM   70.0f // 高速退回中速窗口的迟滞门限 rpm

#endif
