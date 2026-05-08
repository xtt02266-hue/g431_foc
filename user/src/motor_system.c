#include "motor_system.h"
#include "motor_publicdata.h"
#include "motor_current_loop.h"
#include "as5600.h"
#include "oled.h"
#include "tim.h"
#include <math.h>
#include "user_io.h"
#include "vofa_usart.h"
#include "motor_identify.h"

// 电机系统运行状态与目标值缓存。
MotorSystem g_motor_system = {
    .state = MOTOR_STATE_STOPPED,
    .pot_raw = 0U,
    .target = 0U,
};

// 电位器原始值映射到目标量（占位实现）。
static uint16_t Motor_MapPotToTarget(uint16_t pot_raw)
{
    return pot_raw;
}

// 初始化系统状态与目标值。
void Motor_System_Init(void)
{
    g_motor_system.state = MOTOR_STATE_STOPPED;
    g_motor_system.pot_raw = g_motor_publicdata.pot_raw;
    g_motor_system.target = Motor_MapPotToTarget(g_motor_system.pot_raw);
}

// 周期任务：更新输入与目标值。
void Motor_System_Task(void)
{
    g_motor_system.pot_raw = g_motor_publicdata.pot_raw;
    g_motor_system.target = Motor_MapPotToTarget(g_motor_system.pot_raw);
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
        // 2. 如果校准完毕，可以在这里设置目标等，但暂时不动电机 (等待SVPWM就绪)
        else
        {
             // 临时安全：关闭任何底层开环强拖，保证即使辨识完成也不会乱跑
             Motor_OpenLoop_Drive(0.0f, 0.0f);
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
        // 如果正在校准，OLED显示校准进度
        OLED_ShowString(1, 1, "State: Idt     ");
        OLED_ShowNum(2, 1, id_state, 2);
    } else {
        // 如果校准完毕，显示我们通过参数计算出的电量
        OLED_ShowString(1, 1, "State: Run     ");
        
        // 显示当前 Id 和 Iq (放大1000倍转为mA显示)
        OLED_ShowString(2, 1, "Id:      mA");
        OLED_ShowSignedNum(2, 4, (int32_t)(g_foc_state.park.d * 1000.0f), 5);
        OLED_ShowString(3, 1, "Iq:      mA");
        OLED_ShowSignedNum(3, 4, (int32_t)(g_foc_state.park.q * 1000.0f), 5);
    }

    // 始终显示 AS5600 原始角度（第4行）—— 如果一直是 0，说明 I2C 没通！
    OLED_ShowString(4, 1, "Ang:");
    OLED_ShowNum(4, 5, AS5600_ReadRawAngle(), 4);

    // 2. VOFA+ 串口发送波形数据 (使用 Just_Float 协议)
    // 你可以在上位机中查看波形，看看用手转动电机时，电流和角度的变化
    float vofa_data[4];
    vofa_data[0] = g_foc_state.sample.iu_a * 1000.0f; // U相真实物理电流 (mA)
    vofa_data[1] = g_foc_state.sample.iw_a * 1000.0f; // W相真实物理电流 (mA)
    vofa_data[2] = g_foc_state.park.d * 1000.0f;      // D轴电流 (mA)
    vofa_data[3] = g_foc_state.park.q * 1000.0f;      // Q轴电流 (mA)
    
    VOFA_JustFloat_Send(vofa_data, 4); // 发送四个通道浮点数
    
    // 适当的软件延时，刷新太快 OLED 会闪
    // 这里设定 50ms (即20Hz刷新率)，对 OLED 友好，对 VOFA 观察手动转动也足够
    HAL_Delay(50);
}

