#include "motor_system.h"
#include "motor_current_loop.h"
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

// 电位器原始值映射到 q 轴目标电流 (安培)。
// ADC 范围 0~4095，中位 ≈ 2048 对应 0A，两端对应 ±MAX_CURRENT。
#define POT_CURRENT_MAX   0.5f    // 最大 q 轴电流 (A)
#define POT_DEADZONE      100U     // 中位死区 (±80 LSB)，避免微小漂移

static float Motor_MapPotToCurrent(uint16_t pot_raw)
{
    // 以中点 2048 为零点，计算偏差
    int32_t offset = (int32_t)pot_raw - 2048;

    // 死区：中位附近强制输出 0，手感更好
    if (offset > -(int32_t)POT_DEADZONE && offset < (int32_t)POT_DEADZONE) {
        return 0.0f;
    }

    // 线性映射：偏差 → 电流 (A)
    // offset 范围约 ±2048，映射到 ±POT_CURRENT_MAX
    float current = (float)offset * (POT_CURRENT_MAX / 2048.0f);

    // 限幅
    if (current >  POT_CURRENT_MAX) current =  POT_CURRENT_MAX;
    if (current < -POT_CURRENT_MAX) current = -POT_CURRENT_MAX;

    return current;
}

// 初始化系统状态与目标值。
void Motor_System_Init(void)
{
    g_motor_system.state = MOTOR_STATE_STOPPED;

    // 初始目标电流归零
    g_foc_state.target_q = 0.0f;
    g_foc_state.target_d = 0.0f;
}

// 周期任务：更新电位器输入，映射为 q 轴目标电流。
void Motor_System_Task(void)
{
    // 电位器 → q 轴电流目标 (A)，D 轴目标保持 0（Id=0 控制）
    g_foc_state.target_q = Motor_MapPotToCurrent(Pot_ReadRaw());
    g_foc_state.target_d = 0.0f;
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
                 Motor_CurrentLoop_AutoTunePID(id.resistance, id.inductance, 12.0f);

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
      //  OLED_ShowString(1, 1, "Run");
       // OLED_ShowSignedNum(1, 7, (int32_t)(g_foc_state.target_q * 1000.0f), 5);
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
    OLED_ShowSignedNum(3, 9, (int32_t)(g_foc_state.park.q * 1000.0f), 4);

}
    // 2. VOFA+ 诊断波形 (4 通道)
    //    CH0 vs CH1 对比看跟随，CH2 看PID输出，CH3 看是否有真实电流
    float vofa_data[7];
    vofa_data[0] = g_foc_state.target_q;         // Q轴目标 (A)
    vofa_data[1] = g_foc_state.park.q;           // Q轴实际 (A)
    vofa_data[2] = g_foc_state.pi_q.output;      // PID 输出 Vq (V)
    vofa_data[3] = g_foc_state.park.d;      // U相实际电流 (A)
    vofa_data[4] = g_foc_state.sample.iw_a;      // W相实际电流 (A)
    vofa_data[5] = g_foc_state.sample.iu_a;      // U相实际电流 (A)
    vofa_data[6]= -(g_foc_state.sample.iu_a + g_foc_state.sample.iw_a); // V相实际电流 (A)，理论上应该等于 -Iu -Iw
    VOFA_JustFloat_Send(vofa_data, 7);
    // 适当的软件延时，刷新太快 OLED 会闪
    // 这里设定 50ms (即20Hz刷新率)，对 OLED 友好，对 VOFA 观察手动转动也足够
   // HAL_Delay(1);
}


