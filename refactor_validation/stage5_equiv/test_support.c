/* Stage 5 行为等价验证：所有被测源文件外部依赖的桩实现与全局定义。
 * 本文件不包含任何被测算法，只提供确定性输入并记录副作用调用顺序。 */
#include <string.h>
#include "test_support.h"
#include "motor_system.h"
#include "motor_current_loop.h"
#include "motor_speed_loop.h"
#include "motor_position_loop.h"
#include "motor_trajectory.h"
#include "motor_feedforward.h"
#include "motor_identify.h"
#include "motor_sensorless.h"
#include "motor_music.h"
#include "svpwm.h"
#include "as5600.h"
#include "user_io.h"
#include "tim.h"
#include "oled.h"
#include "vofa_usart.h"
#include "mt6826s.h"

TestState g_test;

/* --- 被测源文件引用的全局量定义 --- */
MotorCurrentLoopState g_foc_state;
PID_Controller        speed_pid;
MotorSpeedEstimator   speed_est;
PID_Controller        g_pi_pos;
MotorIdentifiedParams g_identified_params;
volatile uint16_t     g_user_pot_raw;
TIM_HandleTypeDef     htim1;
TIM_HandleTypeDef     htim2;
UART_HandleTypeDef    huart2;

void Test_ResetAll(void)
{
    memset(&g_test, 0, sizeof(g_test));
}

void Test_ResetCounters(void)
{
    g_test.pid_reset_count = 0;
    g_test.pos_reset_count = 0;
    g_test.traj_clear_count = 0;
    g_test.ff_calc_count = 0;
    g_test.pos_run_count = 0;
    g_test.music_task_count = 0;
    g_test.evlen = 0;
    g_test.event[0] = '\0';
}

void Test_Event(char c)
{
    if (g_test.evlen < (sizeof(g_test.event) - 1u)) {
        g_test.event[g_test.evlen++] = c;
        g_test.event[g_test.evlen] = '\0';
    }
}

/* --- HAL / 系统 --- */
uint32_t HAL_GetTick(void) { return 0u; }

/* --- user_io --- */
uint16_t Pot_ReadRaw(void) { return 2048u; }
void Led_Toggle(void) { }

/* --- as5600 --- */
uint8_t AS5600_IsDataFresh(uint32_t max_age_ms) { (void)max_age_ms; return g_test.as5600_fresh; }
uint16_t AS5600_ReadRawAngle(void) { return g_test.as5600_angle; }
void AS5600_RequestRead_DMA(void) { }
void AS5600_BackgroundTask(void) { }

/* --- 速度环/测速 --- */
void Motor_SpeedEstimator_Init(float filter_alpha) { (void)filter_alpha; }
float Motor_SpeedEstimator_Update(uint16_t angle_raw, float dt_seconds)
{
    (void)angle_raw; (void)dt_seconds; return 30.0f;
}
void Motor_SpeedLoop_Init(void) { }
void Motor_SpeedLoop_SetTarget(float target_speed_rpm)
{
    Test_Event('S');
    speed_pid.target = target_speed_rpm;
}
float Motor_SpeedLoop_Update(float current_speed_rpm)
{
    Test_Event('s');
    speed_pid.measure = current_speed_rpm;
    return 0.5f;
}
float Motor_SpeedLoop_FieldWeakening(float current_rpm) { (void)current_rpm; return 0.0f; }

/* --- 位置环 --- */
void Motor_PositionLoop_Init(void) { }
void Motor_PositionLoop_Reset(void)
{
    g_test.pos_reset_count++;
    Test_Event('O');
}
float Motor_PositionLoop_Run(float target_position, float actual_position)
{
    (void)target_position; (void)actual_position;
    g_test.pos_run_count++;
    Test_Event('p');
    return 42.0f;
}

/* --- PID --- */
void PID_Reset(PID_Controller *pid)
{
    (void)pid;
    g_test.pid_reset_count++;
    Test_Event('P');
}

/* --- 轨迹规划 --- */
void Motor_Trajectory_Reset(float actual_position, float actual_speed_rpm)
{
    (void)actual_position; (void)actual_speed_rpm;
}
void Motor_Trajectory_Clear(void)
{
    g_test.traj_clear_count++;
    Test_Event('T');
}
void Motor_Trajectory_Step(float target_position, float actual_position,
                           float actual_speed_rpm, float target_speed_rpm)
{
    (void)target_position; (void)actual_position; (void)actual_speed_rpm;
    (void)target_speed_rpm;
    Test_Event('t');
}
float Motor_Trajectory_GetPosition(void) { return 101.0f; }
float Motor_Trajectory_GetVelocity(void) { return 0.0f; }
float Motor_Trajectory_GetAccel(void) { return 1.5f; }
float Motor_Trajectory_GetFilteredTarget(void) { return 100.0f; }

/* --- 前馈 --- */
MotorFeedforwardResult Motor_Feedforward_Calculate(float speed_loop_iq,
                                                   float target_speed_rpm,
                                                   float target_accel_rpm_s,
                                                   float min_iq, float max_iq)
{
    (void)target_speed_rpm; (void)min_iq; (void)max_iq;
    MotorFeedforwardResult r;
    g_test.ff_calc_count++;
    Test_Event('F');
    r.speed_loop_iq = speed_loop_iq;
    r.friction_iq   = 0.1f;
    r.inertia_iq    = 0.2f;
    r.accel_rpm_s   = target_accel_rpm_s;
    r.output_iq     = speed_loop_iq + 0.3f;
    return r;
}
float Motor_Feedforward_FrictionIq(float speed_rpm) { (void)speed_rpm; return 0.1f; }
float Motor_Feedforward_InertiaIq(float accel_rpm_s) { (void)accel_rpm_s; return 0.2f; }
float Motor_Feedforward_ApplyIq(float speed_loop_iq, float friction_iq,
                                float inertia_iq, float min_iq, float max_iq)
{
    (void)friction_iq; (void)inertia_iq; (void)min_iq; (void)max_iq;
    return speed_loop_iq;
}

/* --- 辨识 --- */
MotorIdentifyState Motor_Identify_GetState(void) { return IDENTIFY_STATE_DONE; }
MotorIdentifiedParams Motor_Identify_GetResult(void)
{
    MotorIdentifiedParams p;
    p.resistance = 1.0f;
    p.inductance = 0.0012f;
    p.pole_pairs = 7u;
    p.zero_angle_offset = 0.0f;
    p.uvw_dir = 1;
    return p;
}
void Motor_Identify_Start(void) { }
void Motor_Identify_UseResult(const MotorIdentifiedParams *params) { (void)params; }
void Motor_Identify_Task(void) { }
void Motor_OpenLoop_Drive(float elec_angle, float amplitude) { (void)elec_angle; (void)amplitude; }

/* --- 参数模块 --- */
uint8_t Motor_Parameters_IsReady(void)
{
    return (g_test.param_status == MOTOR_PARAMETERS_READY) ? 1u : 0u;
}
uint8_t Motor_Parameters_IsBusy(void)
{
    return ((g_test.param_status == MOTOR_PARAMETERS_IDENTIFYING) ||
            (g_test.param_status == MOTOR_PARAMETERS_SAVE_PENDING)) ? 1u : 0u;
}
uint8_t Motor_Parameters_HasStoredData(void) { return 1u; }
MotorParametersStatus Motor_Parameters_GetStatus(void) { return g_test.param_status; }
void Motor_Parameters_Init(void) { }
uint8_t Motor_Parameters_IdentifyAndSave(void) { return 0u; }
void Motor_Parameters_ControlTask1ms(void) { }
void Motor_Parameters_BackgroundTask(void) { }

/* --- 电流环 --- */
uint8_t Motor_CurrentLoop_IsEnabled(void) { return g_test.current_loop_enabled ? 1u : 0u; }
void Motor_CurrentLoop_Enable(uint8_t enable)
{
    g_test.current_loop_enabled = enable ? 1 : 0;
    Test_Event(enable ? 'C' : 'c');
}
void Motor_CurrentLoop_AutoTunePIDWithBandwidth(float resistance, float inductance,
                                                 float bus_voltage, float bandwidth_hz)
{
    (void)resistance; (void)inductance; (void)bus_voltage; (void)bandwidth_hz;
    Test_Event('B');
}
void Motor_CurrentLoop_AutoTunePID(float resistance, float inductance, float bus_voltage)
{
    (void)resistance; (void)inductance; (void)bus_voltage;
    Test_Event('B');
}

/* --- SVPWM --- */
SVPWM_State g_svpwm;
void SVPWM_Enable(void) { g_test.svpwm_enabled = 1; Test_Event('E'); }
uint8_t SVPWM_IsEnabled(void) { return g_test.svpwm_enabled ? 1u : 0u; }
void SVPWM_Disable(void) { g_test.svpwm_enabled = 0; Test_Event('D'); }

/* --- 无感观测 --- */
void Motor_Sensorless_Init(void) { }
void Motor_Sensorless_Enable(uint8_t enable) { Test_Event(enable ? 'n' : 'N'); }
uint8_t Motor_Sensorless_IsEnabled(void) { return 0u; }
void Motor_Sensorless_Reset(void) { }
void Motor_Sensorless_SetDirection(int8_t direction) { (void)direction; }
uint8_t Motor_Sensorless_ConfigureMotor(float resistance, float inductance, uint16_t pole_pairs)
{
    (void)resistance; (void)inductance; (void)pole_pairs;
    return 1u;
}
MotorSensorlessOutput Motor_Sensorless_GetOutput(void)
{
    MotorSensorlessOutput out;
    memset(&out, 0, sizeof(out));
    return out;
}

/* --- 音乐 --- */
uint8_t Motor_Music_IsPlaying(void) { return g_test.music_playing ? 1u : 0u; }
void Motor_Music_Init(void) { }
void Motor_Music_Task1ms(void) { g_test.music_task_count++; Test_Event('m'); }
void Motor_Music_Stop(void) { g_test.music_playing = 0; Test_Event('x'); }
void Motor_Music_Start(const MotorMusicNote *song, uint16_t note_count, uint8_t repeat)
{
    (void)song; (void)note_count; (void)repeat;
}
void Motor_Music_StartTimes(const MotorMusicNote *song, uint16_t note_count, uint16_t play_count)
{
    (void)song; (void)note_count; (void)play_count;
}
void Motor_Music_StartDemo(void) { }
void Motor_Music_StartDemoTimes(uint16_t play_count) { (void)play_count; }
void Motor_Music_SetAmplitude(float amplitude_a) { (void)amplitude_a; }
float Motor_Music_ProcessIqTarget(float normal_iq_target) { return normal_iq_target; }

/* --- 前台显示（baseline 版 motor_system.c 会引用） --- */
void OLED_Init(void) { }
void OLED_Clear(void) { }
void OLED_ShowChar(uint8_t line, uint8_t column, char ch) { (void)line; (void)column; (void)ch; }
void OLED_ShowString(uint8_t line, uint8_t column, char *str) { (void)line; (void)column; (void)str; }
void OLED_ShowNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    (void)line; (void)column; (void)number; (void)length;
}
void OLED_ShowSignedNum(uint8_t line, uint8_t column, int32_t number, uint8_t length)
{
    (void)line; (void)column; (void)number; (void)length;
}
void OLED_ShowHexNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    (void)line; (void)column; (void)number; (void)length;
}
void OLED_ShowBinNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    (void)line; (void)column; (void)number; (void)length;
}
void VOFA_JustFloat_Send(float *data, uint8_t count) { (void)data; (void)count; }
uint16_t MT6826S_ReadRawAngle15(void) { return 0u; }
