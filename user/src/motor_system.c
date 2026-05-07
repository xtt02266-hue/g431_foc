#include "motor_system.h"
#include "motor_publicdata.h"
#include "motor_current_loop.h"
#include "as5600.h"
#include "oled.h"
#include "tim.h"
#include <math.h>
#include "user_io.h"

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
// 临时开环测试
// ---------------------------------------------------------
void Motor_OpenLoop_Drive(float elec_angle, float amplitude)
{
    // 幅值限幅 (中心点为500)
    if (amplitude > 500.0f) amplitude = 500.0f;
    if (amplitude < 0.0f)   amplitude = 0.0f;

    // 计算三相占空比 (相差120度 => 2.0944弧度)
    // 占空比计算：中心值(500) + 正弦波分量
    float valA = 500.0f + amplitude * sinf(elec_angle);
    float valB = 500.0f + amplitude * sinf(elec_angle - 2.094395f); 
    float valC = 500.0f + amplitude * sinf(elec_angle + 2.094395f);

    // 将 0~1000 的设定分辨率映射到实际的定时器 ARR (4250)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(valA * 4.25f));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(valB * 4.25f));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(valC * 4.25f));
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
        // 调用辨识状态机 (1ms 周期调度)
        // Motor_Identify_Task();
        
        // 调用系统普通任务 (1ms 周期刷新电位器和目标)
        Motor_System_Task();
        
        // ============ 临时部署：开环强拖测试 ============
        static float open_loop_angle = 0.0f;
        
        // 【注意】如果你在 1ms (1000Hz) 中断里每次加 1.0f 弧度，
        // 电频率将达到 1000 rad/s (约 159 Hz)，对于开环启动来说可能太快了会引发失步。
        // 这里我暂时帮你把默认步长设为了 0.05f（转速更安全），你随时可以改回 1.0f。
        float speed_step = 0.05f; 
        
        float drive_power = 200.0f; // 强拖的力度 (0~500)
        
        open_loop_angle += speed_step;
        if (open_loop_angle > 6.283185f) {
            open_loop_angle -= 6.283185f;
        }
        
        Motor_OpenLoop_Drive(open_loop_angle, drive_power);
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
// 临时测试任务：OLED 调试显示 (主循环调用)
// ---------------------------------------------------------
void Motor_ShowDebugInfo_OLED(void)
{
    // 获取 AS5600 角度 (0~4095)
    uint16_t raw_angle = AS5600_ReadRawAngle();
    
    // 获取最新的电流采样数据 (mA 级别展示)
    MotorCurrentSample current = Motor_CurrentLoop_GetLastSample();
    
    // OLED 显示第一行：角度
    OLED_ShowString(1, 1, "Ang:");
    OLED_ShowNum(1, 5, raw_angle, 4);

    // OLED 显示第二行：Iu
    OLED_ShowString(2, 1, "Iu:");
    // 这里将 A 转换成 mA，方便整数显示
    OLED_ShowSignedNum(2, 4, (int32_t)(current.iu_a * 1000), 5); // xxxx mA

    // OLED 显示第三行：Iw
    OLED_ShowString(3, 1, "Iw:");
    OLED_ShowSignedNum(3, 4, (int32_t)(current.iw_a * 1000), 5);

    // OLED 显示第四行：电位器采样值 (Pot_Raw)
    OLED_ShowString(4, 1, "Pot:");
    OLED_ShowNum(4, 5, g_motor_system.pot_raw, 4);

    // 适当的软件延时，刷新太快 OLED 会闪，且 I2C 这里是阻塞式的，不宜调用过于频繁
    HAL_Delay(100);
}

