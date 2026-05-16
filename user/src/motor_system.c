#include "motor_system.h"
#include "motor_current_loop.h"
#include "motor_speed_loop.h"
#include "motor_position_loop.h"
#include "as5600.h"
#include "oled.h"
#include "tim.h"
#include <math.h>
#include "user_io.h"
#include "vofa_usart.h"
#include "motor_identify.h"
#include "svpwm.h"

// 电机系统运行状态。
MotorSystem g_motor_system = {
    .state = MOTOR_STATE_STOPPED,
};

// 电位器原始值映射到目标速度 (单位: RPM)
// 假设目标最高 1500 RPM
#define POT_SPEED_MAX     300.0f      // 最大速度设定 (RPM)
#define POT_DEADZONE      10U         // 中位死区 (±100 LSB)，避免微小漂移

static float Motor_MapPotToSpeed(uint16_t pot_raw)
{
    // 以中点 2048 为零点，计算偏差
    int32_t offset = (int32_t)pot_raw - 2048;

    // 死区：中位附近强制输出 0，手感更好
    if (offset > -(int32_t)POT_DEADZONE && offset < (int32_t)POT_DEADZONE) {
        return 0.0f;
    }

    // 线性映射：偏差 → 速度
    // offset 范围约 ±2048，映射到 ±POT_SPEED_MAX
    float speed = (float)offset * (POT_SPEED_MAX / 2048.0f);

    // 限幅
    if (speed >  POT_SPEED_MAX) speed =  POT_SPEED_MAX;
    if (speed < -POT_SPEED_MAX) speed = -POT_SPEED_MAX;

    return speed;
}

// 初始化系统状态与目标值。
void Motor_System_Init(void)
{
    g_motor_system.state = MOTOR_STATE_STOPPED;

    // 初始目标电流归零
    g_foc_state.target_q = 0.0f;
    g_foc_state.target_d = 0.0f;
    
    // 初始化速度估计器 (这里使用 alpha=0.15f 的滤波强度，你可以自己调试)
    Motor_SpeedEstimator_Init(0.05f);
    
    // 初始化速度环 PID (参数已移至 motor_speed_loop.h)
    Motor_SpeedLoop_Init();

    // 初始化位置环 PID
    Motor_PositionLoop_Init();
}

// 周期任务：更新电位器输入，映射为速度，并计算速度环输出。
void Motor_System_Task(void)
{
    // 定时器进入此函数为 1ms 一次，故 dt=0.001s
    float dt = 0.001f;

    // 1. 每 1ms 测算最新速度 (RPM)，传入 dt=0.001s
    float raw_rpm = Motor_SpeedEstimator_Update(AS5600_ReadRawAngle(), dt);
    
    // 如果编码器接线/电机相序不同，编码器读数的正反方向可能会和 Iq 的正扭矩方向相反。
    // 我们必须用系统辨识出的 uvw_dir (1 或 -1) 来把转速的正负号与电机电磁正方向统一，否则会导致 PID 变成正反馈（越差越使劲）！
    float current_rpm = raw_rpm * Motor_Identify_GetResult().uvw_dir;
    
    // 2. 将电位器值直接映射为目标位置 (范围 0~4095)，并进行反向处理
    float target_pos = 4095.0f - (float)Pot_ReadRaw();
    
    // 3. FOC 闭环开始工作后，开始让位置环介入产生速度，速度环介入产生 Iq
    if (Motor_Identify_GetState() == IDENTIFY_STATE_DONE)
    {
        float actual_pos = (float)AS5600_ReadRawAngle();
        
        // 处理 0~4095 过零点“最短路径”问题，避免在 0 和 4095 之间来回疯抖
        float pos_error = target_pos - actual_pos;
        if (pos_error > 2048.0f) {
            actual_pos += 4096.0f;
        } else if (pos_error < -2048.0f) {
            actual_pos -= 4096.0f;
        }
        
        // 计算位置环，输出期望的机械转速
        float target_mech_rpm = Motor_PositionLoop_Run(target_pos, actual_pos);
        
        // 将机械期望转速乘以 uvw_dir，转换为电磁期望转速，给到速度环，防止正反馈
        float target_elec_rpm = target_mech_rpm * (float)Motor_Identify_GetResult().uvw_dir;
        Motor_SpeedLoop_SetTarget(target_elec_rpm);
        
        // 计算速度环，输出期望电流
        g_foc_state.target_q = Motor_SpeedLoop_Update(current_rpm);
    }
    else
    {
        Motor_SpeedLoop_SetTarget(0.0f);
        PID_Reset(&speed_pid);
        PID_Reset(&g_pi_pos);
        g_foc_state.target_q = 0.0f;
    }
    
    // D 轴弱磁控制 (Field Weakening) 策略
    // 当速度较高时给入一定的负 D 轴电流以削弱反电动势，换取更高转速
    // 这里的策略非常简单：超过 1000 RPM 后，每增加 100 RPM，D 轴增加 -0.1A，最多 -1.0A
    float fw_rpm_threshold = 1000.0f;
    float fw_gain = 0.001f;  // (即 0.1A / 100RPM)
    float fw_max_current = 0.0f;
    
    // 取当前转速的绝对值
    float abs_rpm = current_rpm;
    if (abs_rpm < 0) abs_rpm = -abs_rpm;
    
    if (abs_rpm > fw_rpm_threshold)
    {
        g_foc_state.target_d = -(abs_rpm - fw_rpm_threshold) * fw_gain;
        // 限幅，防止弱磁电流过大导致定子严重发热甚至失磁退磁
        if (g_foc_state.target_d < fw_max_current)
        {
            g_foc_state.target_d = fw_max_current;
        }
    }
    else
    {
        // 速度较低时无需弱磁，保持最大转矩电流比 (MTPA)
        g_foc_state.target_d = 0.0f;
    }
}

// ---------------------------------------------------------
// 定时器更新中断回调函数 (TIM2, 假设为 1000Hz / 1ms 周期)
// ---------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 请确保在 stm32g4xx_it.c 中调用了 HAL_TIM_IRQHandler(&htim2)
    // 且 htim2 已被正确初始化并启动：HAL_TIM_Base_Start_IT(&htim2);
    // 同时如果要在其他文件中使用外部的 htim2，需要包含对应的头文件 (例如 #include "tim.h")
    
    // extern TIM_HandleTypeDef htim2; // 开启这行如果下方报错未定义
    
     if (htim == &htim2)
    {
        // ============ 状态机主调度 ============
        // 1. 如果正在校准，执行校准任务
        if (Motor_Identify_GetState() != IDENTIFY_STATE_DONE)
        {
             Motor_Identify_Task();
        }
        // 2. 辨识刚完成时：自动整定 PID + 启用 FOC 闭环 + SVPWM 输出
        else
        {
             static uint8_t loop_was_enabled = 0;
             if (!loop_was_enabled)
             {
                 loop_was_enabled = 1;

                 // 根据辨识出的 R/L 自动计算电流环 PI 增益
                 MotorIdentifiedParams id = Motor_Identify_GetResult();
                 Motor_CurrentLoop_AutoTunePID(id.resistance, id.inductance, SYSTEM_BUS_VOLTAGE);

                 g_svpwm.enabled = 1;                    // 使能 SVPWM 输出
                 Motor_CurrentLoop_Enable(1);             // 使能 FOC 电流闭环
             }
             // 闭环已接管 CCR，不再调用 Motor_OpenLoop_Drive（否则会覆盖 SVPWM 输出！）
        }
        
        // 调用系统普通任务 (1ms 周期刷新电位器和目标)
        Motor_System_Task();
        // ================================================
        
        // ============ 状态灯 2Hz 闪烁 ============
        // 频率 2Hz = 周期 500ms = 亮 250ms，灭 250ms
        static uint16_t led_cnt = 0;
        led_cnt++;
        if (led_cnt >= 250) {
            led_cnt = 0;
            Led_Toggle();
        }
        // =========================================
        
        // 其他需要在 1ms 下执行的逻辑，如按键扫描延时、速度环更新等
    }
}

// ---------------------------------------------------------
// 综合测试任务：OLED 显示与 VOFA+ 上位机波形观察 (放在 main 的 while(1) 中调用)
// ---------------------------------------------------------
void Motor_ShowDebugInfo_OLED(void)
{
    // 获取实时的 FOC 内部状态（电流、坐标变换后结果）
    // 注意：这里读取全局变量，如果有严谨强迫症可以加关中断，但对于只是观察调试没关系。
    MotorIdentifyState id_state = Motor_Identify_GetState();
    
    // 1. OLED 界面显示关键调度状态
    if (id_state != IDENTIFY_STATE_DONE) {
        // 辨识进行中：显示状态编号 + 目标 q 电流
        OLED_ShowString(1, 1, "Idt");
        OLED_ShowString(1, 4, "Tq:");
        OLED_ShowSignedNum(1, 7, (int32_t)(g_foc_state.target_q * 1000.0f), 5);
        OLED_ShowNum(1, 14, id_state, 2);
    } else {
        // 辨识完成：显示 Run + 目标 q 电流
       // OLED_ShowString(1, 1, "Run");
        //OLED_ShowSignedNum(1, 7, (int32_t)(g_foc_state.target_q * 1000.0f), 5);
    }
if(0)  // 开启 OLED 诊断显示：d/q电流、ADC原始值、角度、电位器
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
    // 2. VOFA+ 诊断波形 (4 通道 或者更多)
    //    现在可以用来观察速度闭环参数。CH0 vs CH1 看速度跟随，CH2 vs CH3 看电流跟随
    float vofa_data[7];
    vofa_data[0] = -speed_pid.target;             // 目标速度 (RPM)
    vofa_data[1] = speed_est.speed_rpm;          // 实际转速 (RPM)
    vofa_data[2] = g_pi_pos.target;              // 目标位置
    vofa_data[3] = g_pi_pos.measure;             // 实际位置
    vofa_data[4] = g_foc_state.pi_q.output;      // PID 输出 Vq (V)
    vofa_data[5] = g_foc_state.sample.iu_a;      // U相实际电流 (A)
    vofa_data[6]= -(g_foc_state.sample.iu_a + g_foc_state.sample.iw_a); // V相实际电流 (A)，理论上应该等于 -Iu -Iw
    VOFA_JustFloat_Send(vofa_data, 7);
    // 适当的软件延时，刷新太快 OLED 会闪
    // 这里设定 50ms (即20Hz刷新率)，对 OLED 友好，对 VOFA 观察手动转动也足够
   // HAL_Delay(1);
}


