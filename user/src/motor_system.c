#include "motor_system.h"
#include "motor_current_loop.h"
#include "motor_speed_loop.h"
#include "motor_position_loop.h"
#include "motor_feedforward.h"
#include "motor_trajectory.h"
#include "as5600.h"
#include "oled.h"
#include "tim.h"
#include <math.h>
#include <stddef.h>
#include "user_io.h"
#include "vofa_usart.h"
#include "motor_identify.h"
#include "svpwm.h"
#include "mt6826s.h"
#include "motor_music.h"
#include "motor_parameters.h"
#include "motor_sensorless.h"

#define MOTOR_AS5600_MAX_SAMPLE_AGE_MS  10U
#define MOTOR_CURRENT_LOOP_CONTROL_BW_HZ 250.0f
#define MOTOR_CURRENT_LOOP_MUSIC_BW_HZ  2000.0f

// 电位器低通滤波系数 (一阶 EMA, 1kHz 更新率)
// α 越小滤波越强、响应越慢; α=1.0 则无滤波
// α=0.10f → 截止频率 ~16Hz, 适合人手旋转电位器
// α=0.05f → 截止频率 ~8Hz,  更平滑但略有滞后感
#define POT_LPF_ALPHA  0.10f

// 电机系统运行状态。
MotorSystem g_motor_system = {
    .state = MOTOR_STATE_WAIT_PARAMETERS,
};

static volatile uint8_t g_run_requested = 1U;
/* 故障采用锁存方式，传感器恢复后仍需调用 ClearFault。 */
static volatile uint8_t g_fault_latched = 0U;

static void Motor_System_ResetOuterLoops(void);
static void Motor_System_ForceSafeStop(void);
static void Motor_System_UpdateOperatingState(void);
static void Motor_System_TuneCurrentLoopBandwidth(float bandwidth_hz);

// 调试用全局变量，用于 VOFA+ 波形观察，不参与控制逻辑。
static float g_debug_pot_target_pos = 0.0f;       // 电位器目标位置
static float g_pot_target_filtered = 0.0f;         // 电位器目标位置经低通滤波后的值
static float g_debug_uvw_dir = 1.0f;              // UVW 方向系数 (1 或 -1)
static float g_debug_friction_iq = 0.0f;           // 摩擦补偿电流 (A)
static float g_debug_inertia_iq = 0.0f;            // 惯性补偿电流 (A)
static float g_debug_speed_loop_iq = 0.0f;         // 速度环基础 Iq 输出 (A)
static float g_debug_target_accel_rpm_s = 0.0f;    // 惯性补偿使用的目标加速度 (RPM/s)

// 浮点数绝对值
static float Motor_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint16_t Motor_SpeedEstimator_GetPeriodTicks(float rpm)
{
    float abs_rpm = Motor_AbsFloat(rpm);

    if (abs_rpm >= SPEED_EST_HIGH_RPM_THRESHOLD) {
        return SPEED_EST_MAX_PERIOD_TICKS;
    } else if (abs_rpm >= SPEED_EST_MID_RPM_THRESHOLD) {
        return SPEED_EST_HIGH_PERIOD_TICKS;
    } else if (abs_rpm >= SPEED_EST_LOW_RPM_THRESHOLD) {
        return SPEED_EST_MID_PERIOD_TICKS;
    } else {
        return SPEED_EST_LOW_PERIOD_TICKS;
    }
}

// 自适应采样率速度估算器
// 策略: 根据当前估算转速动态调整编码器采样间隔 (分频比)
//   低速 (<50 RPM):  每 20ms 采样一次 (50Hz)，用更长窗口抑制量化噪声
//   中速 (50~200):   每 5ms 采样一次 (200Hz)
//   中高速 (200~500): 每 2ms 采样一次 (500Hz)
//   高速 (>500 RPM): 每 1ms 采样一次 (1000Hz)，保证响应速度
// 这样在低速时避免了 AS5600 12-bit 编码器因采样过快导致的量化抖动
static float Motor_UpdateSpeedEstimatorAdaptive(void)
{
    static uint16_t ticks = 0;
    static float dt_acc = 0.0f;

    if (!speed_est.initialized) {
        ticks = 0;
        dt_acc = 0.0f;
        g_motor_system.run_data.speed_rpm =
            Motor_SpeedEstimator_Update(AS5600_ReadRawAngle(), MOTOR_SYSTEM_TASK_DT_SEC);
        return g_motor_system.run_data.speed_rpm;
    }

    ticks++;
    dt_acc += MOTOR_SYSTEM_TASK_DT_SEC;

    if (ticks >= Motor_SpeedEstimator_GetPeriodTicks(g_motor_system.run_data.speed_rpm)) {
        g_motor_system.run_data.speed_rpm =
            Motor_SpeedEstimator_Update(AS5600_ReadRawAngle(), dt_acc);
        ticks = 0;
        dt_acc = 0.0f;
    }

    return g_motor_system.run_data.speed_rpm;
}

// 初始化系统状态与目标值。
void Motor_System_Init(void)
{
    g_motor_system.state = MOTOR_STATE_WAIT_PARAMETERS;
    g_run_requested = 1U;
    g_fault_latched = 0U;

    // 初始目标电流归零
    g_foc_state.target_q = 0.0f;
    g_foc_state.target_d = 0.0f;
    
    // 初始化速度估计器 
    // 参数为低通滤波系数 filter_alpha (0.001 ~ 1.0)
    // 越接近 0 (如 0.08f)：信号越平滑，抗噪声能力强，低速越稳，但响应会有滞后
    // 越接近 1 (如 0.80f)：几乎无滤波，对转速变化响应极快，但低速极容易受噪声抖动
    Motor_SpeedEstimator_Init(0.4f);
    
    // 初始化速度环 PID (参数已移至 motor_speed_loop.h)
    Motor_SpeedLoop_Init();

    // 初始化位置环 PID
    Motor_PositionLoop_Init();

    // Music output is disabled until explicitly started.
    Motor_Music_Init();

    // Sensorless estimation is observation-only and disabled by default.
    Motor_Sensorless_Init();
}

MotorState Motor_System_GetState(void)
{
    return g_motor_system.state;
}

uint8_t Motor_System_StartControl(void)
{
    if ((Motor_Parameters_IsReady() == 0U) ||
        (g_fault_latched != 0U) ||
        (AS5600_IsDataFresh(MOTOR_AS5600_MAX_SAMPLE_AGE_MS) == 0U)) {
        return 0U;
    }

    g_run_requested = 1U;
    return 1U;
}

void Motor_System_StopControl(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_run_requested = 0U;
    Motor_System_ForceSafeStop();
    g_motor_system.state = MOTOR_STATE_STOPPED;
    if (primask == 0U) {
        __enable_irq();
    }
}

uint8_t Motor_System_ClearFault(void)
{
    if ((Motor_Parameters_IsReady() == 0U) ||
        (AS5600_IsDataFresh(MOTOR_AS5600_MAX_SAMPLE_AGE_MS) == 0U)) {
        return 0U;
    }

    g_fault_latched = 0U;
    return 1U;
}

uint8_t Motor_System_IdentifyAndSave(void)
{
    if ((Motor_Parameters_IsBusy() != 0U) ||
        (AS5600_IsDataFresh(MOTOR_AS5600_MAX_SAMPLE_AGE_MS) == 0U)) {
        return 0U;
    }

    g_fault_latched = 0U;
    g_run_requested = 1U;
    return Motor_Parameters_IdentifyAndSave();
}

uint8_t Motor_System_PlaySong(const MotorMusicNote *song,
                              uint16_t note_count,
                              uint16_t play_count)
{
    if ((song == NULL) || (note_count == 0U) || (play_count == 0U) ||
        (Motor_Parameters_IsReady() == 0U) ||
        (g_fault_latched != 0U) ||
        (Motor_CurrentLoop_IsEnabled() == 0U)) {
        return 0U;
    }

    Motor_Music_StartTimes(song, note_count, play_count);
    return 1U;
}

uint8_t Motor_System_PlaySongLoop(const MotorMusicNote *song,
                                  uint16_t note_count)
{
    if ((song == NULL) || (note_count == 0U) ||
        (Motor_Parameters_IsReady() == 0U) ||
        (g_fault_latched != 0U) ||
        (Motor_CurrentLoop_IsEnabled() == 0U)) {
        return 0U;
    }

    Motor_Music_Start(song, note_count, 1U);
    return 1U;
}

void Motor_System_StopMusic(void)
{
    Motor_Music_Stop();
}

uint8_t Motor_System_EnableSensorlessObserver(uint8_t enable)
{
    if (enable == 0U) {
        Motor_Sensorless_Enable(0U);
        return 1U;
    }

    if ((g_motor_system.state != MOTOR_STATE_SENSORED_RUN) ||
        (Motor_CurrentLoop_IsEnabled() == 0U)) {
        return 0U;
    }

    Motor_Sensorless_Enable(1U);
    return 1U;
}

static void Motor_System_ResetOuterLoops(void)
{
    /* 清掉位置/速度环历史量，避免功能切换后积分残留。 */
    Motor_SpeedLoop_SetTarget(0.0f);
    PID_Reset(&speed_pid);
    PID_Reset(&g_pi_pos);
    Motor_Trajectory_Clear();
    g_foc_state.target_q = 0.0f;
    g_foc_state.target_d = 0.0f;
}

static void Motor_System_TuneCurrentLoopBandwidth(float bandwidth_hz)
{
    MotorIdentifiedParams id = Motor_Identify_GetResult();

    Motor_CurrentLoop_AutoTunePIDWithBandwidth(id.resistance,
                                               id.inductance,
                                               SYSTEM_BUS_VOLTAGE,
                                               bandwidth_hz);
}

static void Motor_System_ForceSafeStop(void)
{
    /* 所有功能统一从这里撤销转矩输出。 */
    Motor_System_ResetOuterLoops();
    if (Motor_Music_IsPlaying() != 0U) {
        Motor_Music_Stop();
    }
    if (Motor_Sensorless_IsEnabled() != 0U) {
        Motor_Sensorless_Enable(0U);
    }
    if (Motor_CurrentLoop_IsEnabled() != 0U) {
        Motor_CurrentLoop_Enable(0U);
    }
    if (SVPWM_IsEnabled() != 0U) {
        SVPWM_Disable();
    }
}

static void Motor_System_UpdateOperatingState(void)
{
    MotorParametersStatus parameter_status = Motor_Parameters_GetStatus();

    /*
     * Identification owns the PWM after its start routine has stopped all
     * closed-loop features. Do not clear its open-loop duty every 1 ms.
     */
    if (Motor_Parameters_IsBusy() != 0U) {
        g_motor_system.state = MOTOR_STATE_IDENTIFYING;
        return;
    }

    if (parameter_status == MOTOR_PARAMETERS_ERROR) {
        g_fault_latched = 1U;
    }

    if (parameter_status == MOTOR_PARAMETERS_NO_DATA) {
        Motor_System_ForceSafeStop();
        g_motor_system.state = MOTOR_STATE_WAIT_PARAMETERS;
        return;
    }

    /* AS5600 是当前实际换相角度源，数据过期必须立即停机。 */
    if ((g_fault_latched != 0U) ||
        (AS5600_IsDataFresh(MOTOR_AS5600_MAX_SAMPLE_AGE_MS) == 0U)) {
        g_fault_latched = 1U;
        Motor_System_ForceSafeStop();
        g_motor_system.state = MOTOR_STATE_FAULT;
        return;
    }

    if (g_run_requested == 0U) {
        Motor_System_ForceSafeStop();
        g_motor_system.state = MOTOR_STATE_STOPPED;
        return;
    }

    /* 参数和传感器均有效后，统一完成整定与闭环使能。 */
    if (Motor_CurrentLoop_IsEnabled() == 0U) {
        MotorIdentifiedParams id = Motor_Identify_GetResult();

        Motor_System_TuneCurrentLoopBandwidth(
            MOTOR_CURRENT_LOOP_CONTROL_BW_HZ);
        (void)Motor_Sensorless_ConfigureMotor(id.resistance,
                                              id.inductance,
                                              id.pole_pairs);
        SVPWM_Enable();
        Motor_CurrentLoop_Enable(1U);
    }

    g_motor_system.state = (Motor_Music_IsPlaying() != 0U)
                               ? MOTOR_STATE_MUSIC
                               : MOTOR_STATE_SENSORED_RUN;
}

// 周期任务：读取电位器位置目标, 通过位置规划器生成速度目标, 再由速度环生成 Iq。
// 调用频率: 1kHz (由 TIM2 中断驱动, dt = 1ms)
// 控制链路: 电位器 → 位置环 → 速度环(仅前馈) → Iq → FOC → SVPWM → 电机
void Motor_System_Task(void)
{
    static uint8_t music_was_active = 0U;
#if MOTOR_MUSIC_AUTOPLAY_DEMO
    static uint8_t demo_autoplay_checked = 0U;
#endif

    // ========== 步骤 1: 速度估算 (自适应采样率) ==========
    // 低速时用长窗口降噪，高速时用短窗口提高响应
    if (AS5600_IsDataFresh(MOTOR_AS5600_MAX_SAMPLE_AGE_MS) != 0U) {
        g_motor_system.run_data.speed_rpm = Motor_UpdateSpeedEstimatorAdaptive();
    } else {
        g_motor_system.run_data.speed_rpm = 0.0f;
    }
    
    // 如果编码器接线/电机相序不同，编码器读数的正反方向可能会和 Iq 的正扭矩方向相反。
    // 我们必须用系统辨识出的 uvw_dir (1 或 -1) 来把转速的正负号与电机电磁正方向统一，否则会导致 PID 变成正反馈（越差越使劲）！
    int8_t identified_direction = Motor_Identify_GetResult().uvw_dir;
    if ((identified_direction != 1) && (identified_direction != -1)) {
        identified_direction = 1;
    }
    float current_rpm = g_motor_system.run_data.speed_rpm *
                        (float)identified_direction;

    if (current_rpm > 20.0f) {
        Motor_Sensorless_SetDirection(1);
    } else if (current_rpm < -20.0f) {
        Motor_Sensorless_SetDirection(-1);
    }
    
    // ========== 步骤 2: 读取电位器目标位置 ==========
    // 电位器 ADC DMA 原始值 → 反向映射 (4095 - raw) → 低通滤波 → 目标位置
    g_motor_system.run_data.pot_raw = Pot_ReadRaw();

    float target_pos_raw = 4095.0f - (float)g_motor_system.run_data.pot_raw;
    // 一阶 EMA 低通滤波: filtered += α * (raw - filtered)
    g_pot_target_filtered += POT_LPF_ALPHA * (target_pos_raw - g_pot_target_filtered);
    float target_pos = g_pot_target_filtered;
    g_debug_pot_target_pos = target_pos;
    g_foc_state.target_d = 0.0f;

#if MOTOR_MUSIC_AUTOPLAY_DEMO
    if ((g_motor_system.state == MOTOR_STATE_SENSORED_RUN) &&
        (demo_autoplay_checked == 0U)) {
        demo_autoplay_checked = 1U;
        Motor_Music_StartDemo();
        g_motor_system.state = MOTOR_STATE_MUSIC;
    }
#endif

    // Music mode owns only the q-axis current target. Pause the outer loops so
    // position/speed control cannot fight the alternating audio torque.
    if (g_motor_system.state == MOTOR_STATE_MUSIC) {
        if (music_was_active == 0U) {
            Motor_System_TuneCurrentLoopBandwidth(
                MOTOR_CURRENT_LOOP_MUSIC_BW_HZ);
            Motor_SpeedLoop_SetTarget(0.0f);
            PID_Reset(&speed_pid);
            PID_Reset(&g_pi_pos);
            Motor_Trajectory_Clear();
            music_was_active = 1U;
        }

        Motor_Music_Task1ms();
        g_debug_speed_loop_iq = 0.0f;
        g_debug_friction_iq = 0.0f;
        g_debug_inertia_iq = 0.0f;
        g_debug_target_accel_rpm_s = 0.0f;
        g_foc_state.target_q = 0.0f;
        g_foc_state.target_d = 0.0f;
        return;
    }

    if (music_was_active != 0U) {
        Motor_System_TuneCurrentLoopBandwidth(
            MOTOR_CURRENT_LOOP_CONTROL_BW_HZ);
    }
    music_was_active = 0U;
    
    // 3. FOC 闭环开始工作后，开始让位置环介入产生速度，速度环介入产生 Iq
    // 控制链路: 电位器目标 → 位置环 (P) → 目标机械转速 → 乘 uvw_dir → 目标电磁转速
    //          → 速度环 (PI) → Iq 电流 → 摩擦前馈叠加 → 限幅 → FOC 电流环
    if (g_motor_system.state == MOTOR_STATE_SENSORED_RUN)
    {
        float actual_pos = (float)AS5600_ReadRawAngle();
        float actual_mech_rpm = g_motor_system.run_data.speed_rpm;
        
        // 位置/速度闭环直接使用电位器目标；轨迹规划器只旁路提供惯性前馈加速度。
        float target_mech_rpm =
            Motor_PositionLoop_Run(target_pos, actual_pos);

        // 惯性前馈轨迹规划只用于估算加速度:
        // 输入目标位置、实际位置、实际机械速度和位置环目标机械速度；
        // 输出 Motor_Trajectory_GetAccel()，不再改变位置环/速度环目标。
        Motor_Trajectory_Step(target_pos,
                              actual_pos,
                              actual_mech_rpm,
                              target_mech_rpm);
        
        // 将机械期望转速乘以 uvw_dir，转换为电磁期望转速，给到速度环，防止正反馈。
        // 摩擦补偿和惯性补偿也必须使用同一个电磁方向坐标系。
        float uvw_dir = (float)Motor_Identify_GetResult().uvw_dir;
        float target_elec_rpm = target_mech_rpm * uvw_dir;
        g_debug_uvw_dir = uvw_dir;

        // 位置环输出的目标速度直接传给速度环。
        Motor_SpeedLoop_SetTarget(target_elec_rpm);

        // 速度闭环 PID 输出基础 Iq。
        float speed_loop_iq = Motor_SpeedLoop_Update(current_rpm);

        // 轨迹规划器只用于生成目标加速度，供惯性前馈使用。
        float target_accel_rpm_s = Motor_Trajectory_GetAccel() * uvw_dir;
        MotorFeedforwardResult feedforward =
            Motor_Feedforward_Calculate(speed_loop_iq,
                                        target_elec_rpm,
                                        target_accel_rpm_s,
                                        MOTOR_SPEED_PID_OUT_MIN,
                                        MOTOR_SPEED_PID_OUT_MAX);

        // 调试量，便于在 VOFA+ 中观察各分量。
        g_debug_speed_loop_iq = feedforward.speed_loop_iq;
        g_debug_friction_iq = feedforward.friction_iq;
        g_debug_inertia_iq = feedforward.inertia_iq;
        g_debug_target_accel_rpm_s = feedforward.accel_rpm_s;
        // 正确的合成顺序：速度 PID + 摩擦前馈 + 惯性前馈，最后统一限流。
        // 不要再用 target_q = friction_iq 覆盖速度 PID 输出。
        g_foc_state.target_q = feedforward.output_iq;
    }
    else
    {
        // 辨识未完成: 清零所有 PID 积分和电流输出，防止误动作
        Motor_SpeedLoop_SetTarget(0.0f);
        PID_Reset(&speed_pid);
        PID_Reset(&g_pi_pos);
        Motor_Trajectory_Clear();
        g_debug_uvw_dir = (float)Motor_Identify_GetResult().uvw_dir;
        g_debug_speed_loop_iq = 0.0f;
        g_debug_friction_iq = 0.0f;
        g_debug_inertia_iq = 0.0f;
        g_debug_target_accel_rpm_s = 0.0f;
        g_foc_state.target_q = 0.0f;
        g_foc_state.target_d = 0.0f;
    }
    
    // D 轴弱磁控制 (Field Weakening) 策略 — 当前禁用
    // 弱磁用于高速时主动注入负 Id 来压低反电动势，扩展转速范围
    //g_foc_state.target_d = Motor_SpeedLoop_FieldWeakening(current_rpm);
}

// ---------------------------------------------------------
// 状态灯 2Hz 闪烁任务 (设计为 1ms 调用一次)
// ---------------------------------------------------------
static void Motor_StatusLed_Task(void)
{
    // 频率 2Hz = 周期 500ms = 亮 250ms，灭 250ms
    static uint16_t led_cnt = 0;
    led_cnt++;
    if (led_cnt >= 250) {
        led_cnt = 0;
        Led_Toggle();
    }
}

// ---------------------------------------------------------
// 定时器更新中断回调函数 (TIM2, 假设为 1000Hz / 1ms 周期)
// 这是整个电机控制系统的"心跳"中断，所有实时控制逻辑在此统一调度:
//   1. 辨识阶段: 执行开环辨识任务
//   2. 辨识刚完成: 自动整定电流环 PI 参数 + 使能 SVPWM + 开启 FOC 闭环
//   3. 正常运行时: 执行 Motor_System_Task (位置/速度/Iq 级联控制)
//   4. 每 250ms 翻转一次状态 LED (2Hz 闪烁)
// 注意: 中断中不得执行耗时操作 (如 OLED 刷新、VOFA 发送等)，那些应放在 main 循环中
// ---------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
        Motor_Parameters_ControlTask1ms();
        Motor_System_UpdateOperatingState();
        
        // 调用系统普通任务 (1ms 周期刷新电位器和目标)
        Motor_System_Task();
       //Motor_SimulateSpring_Task();   // 弹簧模拟 (与位置控制互斥，二选一)
        // ================================================
        // ============ 状态灯 2Hz 闪烁 ============
        Motor_StatusLed_Task();
        // =========================================
        
    }
}

void Motor_ShowDebugInfo_OLED(void)
{
    static uint32_t last_refresh_ms = 0U;
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_refresh_ms) < 2U) {
        return;
    }
    last_refresh_ms = now;

    // 获取实时的 FOC 内部状态（电流、坐标变换后结果）
    // 注意：这里读取全局变量，如果有严谨强迫症可以加关中断，但对于只是观察调试没关系。
    MotorIdentifyState id_state = Motor_Identify_GetState();
    MotorParametersStatus parameter_status = Motor_Parameters_GetStatus();
    
    // 1. OLED 界面显示关键调度状态
    if (g_motor_system.state == MOTOR_STATE_FAULT) {
        OLED_ShowString(1, 1, "Fault");
    } else if (parameter_status == MOTOR_PARAMETERS_NO_DATA) {
        OLED_ShowString(1, 1, "NoParam");
    } else if (parameter_status == MOTOR_PARAMETERS_ERROR) {
        OLED_ShowString(1, 1, "ParamErr");
    } else if (parameter_status != MOTOR_PARAMETERS_READY) {
        // 辨识进行中：显示状态编号 + 目标 q 电流
        OLED_ShowString(1, 1, "Idt");     // "Idt" = Identify (辨识中)
        OLED_ShowString(1, 4, "Tq:");
        OLED_ShowSignedNum(1, 7, (int32_t)(g_foc_state.target_q * 1000.0f), 5);  // 显示 mA 级
        OLED_ShowNum(1, 14, id_state, 2);   // 辨识状态码
    } else {
        // 辨识完成：OLED 显示暂时关闭以节省 CPU
       // OLED_ShowString(1, 1, "Run");
        //OLED_ShowSignedNum(1, 7, (int32_t)(g_foc_state.target_q * 1000.0f), 5);
    }
if(0)  // 开启 OLED 诊断显示：d/q电流、ADC原始值、角度、电位器
       // 改为 if(1) 可开启详细诊断界面 (会增加 CPU 负载)
{
    // 第2行：D 轴实际电流 (mA) + ADC U 相原始值
    OLED_ShowChar(2, 1, 'd');
    OLED_ShowSignedNum(2, 2, (int32_t)(g_foc_state.park.d * 1000.0f), 4);
    OLED_ShowString(2, 7, "U:");
    OLED_ShowNum(2, 9, g_foc_state.sample.iu_raw, 4);

    // 第3行：Q 轴实际电流 (mA) + ADC W 相原始值
    OLED_ShowChar(3, 1, 'q');
    OLED_ShowSignedNum(3, 2, (int32_t)(g_foc_state.park.q * 1000.0f), 4);
    OLED_ShowString(3, 7, "A:");
    // OLED_ShowNum(3, 9, AS5600_ReadRawAngle(), 4); 

}

    MotorSensorlessOutput sensorless = Motor_Sensorless_GetOutput();
    float vofa_data[20];
    vofa_data[0] = 4095-Pot_ReadRaw();                   // CH0:  电位器原始 ADC (反向)
    vofa_data[1] = (float)AS5600_ReadRawAngle();         // CH1:  编码器实际位置 (counts)
    vofa_data[2] = speed_pid.target;                     // CH2:  速度环目标转速 (电磁方向, RPM)
    vofa_data[3] = speed_pid.measure;                    // CH3:  速度环实测转速 (电磁方向, RPM)
    vofa_data[4] = g_debug_speed_loop_iq*1000;           // CH4:  速度环 PID 基础输出 (mA)
    vofa_data[5] = g_debug_friction_iq*1000;             // CH5:  摩擦前馈补偿电流 (mA)
    vofa_data[6] = g_debug_inertia_iq*1000;              // CH6:  惯性前馈补偿电流 (mA)
    vofa_data[7] = g_foc_state.pi_q.target*1000;          // CH7:  实际送入电流环的目标 Iq (mA)
    vofa_data[8] = g_foc_state.park.q*1000;              // CH8:  实测 Q 轴电流 (mA)
    vofa_data[9] = g_debug_target_accel_rpm_s;           // CH9:  轨迹规划加速度 (电磁方向, RPM/s)
    vofa_data[10] = (float)MT6826S_ReadRawAngle15();     // CH10: MT6826S 15-bit 机械角度 (0~32767)
    vofa_data[11] = Motor_Trajectory_GetAccel()*0.1;         // CH11: 轨迹规划加速度 (RPM/s)
    vofa_data[12] = Motor_Trajectory_GetFilteredTarget();// CH12: 轨迹规划滤波后目标位置 (counts)
    vofa_data[13] = g_debug_pot_target_pos;              // CH13: 电位器反向映射目标位置 (counts)
    vofa_data[14] = Motor_Trajectory_GetPosition();      // CH14: 惯性前馈规划器内部位置 (不参与位置环)
    vofa_data[15] = sensorless.electrical_angle_rad;     // CH15: sensorless electrical angle (rad)
    vofa_data[16] = sensorless.mechanical_speed_rpm;     // CH16: sensorless mechanical speed (RPM)
    vofa_data[17] = sensorless.bemf_magnitude_volts;     // CH17: estimated back-EMF magnitude (V)
    vofa_data[18] = (float)sensorless.valid;             // CH18: sensorless estimate valid flag
    vofa_data[19] = (float)g_motor_system.state;          // CH19: system operating state
    VOFA_JustFloat_Send(vofa_data, 20);
    // 适当的软件延时，刷新太快 OLED 会闪
    // 这里设定 50ms (即20Hz刷新率)，对 OLED 友好，对 VOFA 观察手动转动也足够
   // HAL_Delay(1);
}

