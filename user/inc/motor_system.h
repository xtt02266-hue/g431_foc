#ifndef MOTOR_SYSTEM_H
#define MOTOR_SYSTEM_H

#include <stdint.h>
#include "motor_music.h"
#include "motor_config.h"

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
    MOTOR_CONTROL_TORQUE,
    MOTOR_CONTROL_FREE,
    MOTOR_CONTROL_DAMPING,
    MOTOR_CONTROL_SPRING,
    MOTOR_CONTROL_DETENT,
    MOTOR_CONTROL_LIMIT
} MotorControlMode;

typedef enum
{
    MOTOR_INPUT_POT = 0,
    MOTOR_INPUT_HOST,
    MOTOR_INPUT_INTERNAL
} MotorInputSource;

typedef enum
{
    MOTOR_OWNER_LOCAL = 0,
    MOTOR_OWNER_HOST
} MotorControlOwner;

typedef enum
{
    MOTOR_CMD_OK = 0,
    MOTOR_CMD_INVALID_LENGTH,
    MOTOR_CMD_INVALID_VALUE,
    MOTOR_CMD_UNSUPPORTED,
    MOTOR_CMD_NOT_OWNER,
    MOTOR_CMD_MUST_STOP_FIRST,
    MOTOR_CMD_NOT_READY,
    MOTOR_CMD_FAULT_ACTIVE,
    MOTOR_CMD_BUSY,
    MOTOR_CMD_NOT_RUNNING,
    MOTOR_CMD_INVALID_COMBINATION
} MotorCommandResult;

typedef struct
{
    float spring_k_a_per_rad;
    float damping_b_a_per_rad_s;
    float detent_k_a_per_rad;
    float limit_k_a_per_rad;
    float limit_half_range_deg;
    uint16_t detent_count;
} MotorHapticParams;

typedef struct
{
    MotorState state;
    MotorControlMode mode;
    MotorInputSource source;
    MotorControlOwner owner;
    uint8_t run_requested;
    float iq_limit_a;
    float continuous_angle_rad;
    float relative_center_angle_rad;
    uint32_t config_revision;
} MotorControlSnapshot;

#define MOTOR_DEFAULT_CONTROL_MODE     MOTOR_CONTROL_TORQUE // 初始化后默认绕过位置/速度环

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
MotorInputSource Motor_System_GetInputSource(void);
MotorControlOwner Motor_System_GetControlOwner(void);
MotorCommandResult Motor_System_ClaimHost(void);
void Motor_System_ReleaseHost(void);
void Motor_System_HostHeartbeat(void);
MotorCommandResult Motor_System_HostStart(void);
MotorCommandResult Motor_System_HostSetMode(MotorControlMode mode,
                                            MotorInputSource source);
MotorCommandResult Motor_System_HostSetIq(float iq_a, float *accepted_iq_a);
MotorCommandResult Motor_System_HostSetIqLimit(float limit_a);
MotorCommandResult Motor_System_HostSetHapticParams(const MotorHapticParams *params);
void Motor_System_GetHapticParams(MotorHapticParams *params);
void Motor_System_GetControlSnapshot(MotorControlSnapshot *snapshot);
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

#endif
