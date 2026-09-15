#ifndef MOTOR_SYSTEM_H
#define MOTOR_SYSTEM_H

#include <stdint.h>
#include "motor_music.h"

// -----------------------------------------
// 全局软硬件核心参数配置区
// -----------------------------------------
// 实际使用固定 15V 电源，硬件未采样母线电压，因此算法使用此配置值。
// 更换供电电压时须同步修改，供电压降不会被软件自动补偿。
#define SYSTEM_BUS_VOLTAGE    15.0f
// -----------------------------------------

// 电机运行状态。
typedef enum
{
    MOTOR_STATE_STOPPED = 0,
    MOTOR_STATE_WAIT_PARAMETERS,
    MOTOR_STATE_IDENTIFYING,
    MOTOR_STATE_SENSORED_RUN,
    MOTOR_STATE_MUSIC,
    MOTOR_STATE_FAULT
} MotorState;

/* 控制模式决定 Iq 由谁生成，与运行/停机/故障状态分开管理。
 * 音乐播放期间暂停所选模式；保留位置模式供后续主动切换使用。
 */
typedef enum
{
    MOTOR_CONTROL_POSITION = 0, // 电位器给位置，经位置环、速度环和前馈生成 Iq
    MOTOR_CONTROL_TORQUE       // 电位器或手动给力矩，直接换算成 Iq
} MotorControlMode;

#define MOTOR_DEFAULT_CONTROL_MODE     MOTOR_CONTROL_TORQUE // 初始化后默认绕过位置/速度环
#define MOTOR_TORQUE_CURRENT_LIMIT_A   2.5f // 力矩给定最终换算出的 Iq 绝对值上限，单位 A
/* BM3514H 商品页：291 KV。按常用 FOC 近似 Kt=8.27/KV 估算。
 * 单位 N·m/A，Iq 峰值定义；非实测标定，现有 R/L 辨识不测 Kt。
 * 参考 https://docs.odriverobotics.com/v/latest/manual/control.html
 * 设为 0 表示未配置，电位器力矩输出为零。
 */
#define MOTOR_TORQUE_CONSTANT_NM_PER_A  (8.27f / 291.0f)
/* 默认电位器给定；改为 0 可使用手动给定接口。 */
#define MOTOR_TORQUE_USE_POT           1
#define MOTOR_TORQUE_POT_CENTER        2047.5f // 12 位 ADC 的中点，对应零力矩
#define MOTOR_TORQUE_POT_DEADBAND      80.0f   // 中点两侧各 80 个计数置零，减少噪声引起的出力
/* 电位器两端对应 +/- 此力矩，仍受电流上限约束。
 * 取 BM3514H 商品页标注的 0.05 N·m 作为给定范围；并不代表已验证的连续输出能力。
 */
#define MOTOR_TORQUE_POT_MAX_NM        0.05f

// 电机实时运行数据
typedef struct
{
    float speed_rpm;       // 实时转速 (RPM)
    uint16_t pot_raw;      // 电位器原始数值 (0-4095)
    float target_torque_nm; // 限流后的目标电磁力矩，非实测轴端力矩
} MotorRunData;

// 电机系统关键变量。
typedef struct
{
    MotorState state;
    MotorRunData run_data; // 实时的运行状态数据 (转速、电位器值等)
} MotorSystem;

#define MOTOR_SYSTEM_TASK_DT_SEC       0.001f
#define SPEED_EST_LOW_RPM_THRESHOLD    50.0f
#define SPEED_EST_MID_RPM_THRESHOLD    200.0f
#define SPEED_EST_HIGH_RPM_THRESHOLD   500.0f
#define SPEED_EST_LOW_PERIOD_TICKS     20U     // 50Hz，低速测速窗口更长，降低量化抖动
#define SPEED_EST_MID_PERIOD_TICKS     5U      // 200Hz
#define SPEED_EST_HIGH_PERIOD_TICKS    2U      // 500Hz
#define SPEED_EST_MAX_PERIOD_TICKS     1U      // 1000Hz，高速测速

// 电机系统全局实例。
extern MotorSystem g_motor_system;

// 初始化电机系统。
void Motor_System_Init(void);
// 周期任务入口。
void Motor_System_Task(void);
MotorState Motor_System_GetState(void);

/* 系统级功能入口：负责检查参数、传感器和当前运行状态。 */
uint8_t Motor_System_StartControl(void);
void Motor_System_StopControl(void);
/* 选择力矩或位置控制；仅在模式改变时清零 Iq 和外环历史，不自动启动。
 * 返回 1 表示模式有效并已接受，0 表示枚举值无效；重复设置相同模式不清零。
 */
uint8_t Motor_System_SetControlMode(MotorControlMode mode);
/* 查询所选模式，供业务逻辑/调试使用；返回值不表示电机已使能或正在输出。 */
MotorControlMode Motor_System_GetControlMode(void);
/* 手动给定仅在 MOTOR_TORQUE_USE_POT=0 时生效；电位器模式返回 0。
 * 直接给定电磁方向 Iq (A)，Id=0。
 * 有感运行且已选择 TORQUE 时可设置，限幅至 +/- MOTOR_TORQUE_CURRENT_LIMIT_A。
 * 非有限值被拒绝并清零目标。手动模式停机/故障/切换后需重新给定。
 * 电位器模式在恢复运行后自动采用当前旋钮位置。
 * 返回 1 表示接受（超限值会被限幅），0 表示拒绝并清零存储的手动目标。
 */
uint8_t Motor_System_SetTorqueCurrent(float iq_a);
/* 手动力矩入口，参数单位 N·m：用 Iq=T/Kt 转换后交给电流给定接口检查。
 * Kt 未配置或输入非有限时清零并返回 0；正负沿用 Iq 电磁方向。
 * 返回值及运行条件与 SetTorqueCurrent 相同，默认电位器模式不接受手动给定。
 */
uint8_t Motor_System_SetTorqueNm(float torque_nm);
/* 传感器恢复后需主动清故障，防止电机突然自动恢复出力。 */
uint8_t Motor_System_ClearFault(void);
/* 主动辨识并保存；必须确认电机无负载且可以自由转动。 */
uint8_t Motor_System_IdentifyAndSave(void);
/* 播放有限次数或循环播放，正常位置/力矩控制会在音乐期间暂停。 */
uint8_t Motor_System_PlaySong(const MotorMusicNote *song,
                              uint16_t note_count,
                              uint16_t play_count);
uint8_t Motor_System_PlaySongLoop(const MotorMusicNote *song,
                                  uint16_t note_count);
void Motor_System_StopMusic(void);
/* 当前仅启停并行无感观测器，不会替换 AS5600 换相角度。 */
uint8_t Motor_System_EnableSensorlessObserver(uint8_t enable);
// OLED上显示临时调试信息。
void Motor_ShowDebugInfo_OLED(void);
void Motor_SimulateSpring_Task(void);

#endif
