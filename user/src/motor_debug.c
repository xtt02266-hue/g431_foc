#include "motor_debug.h"
#include "motor_system.h"
#include "motor_current_loop.h"
#include "motor_identify.h"
#include "motor_parameters.h"
#include "oled.h"
#include "vofa_usart.h"
#include "pc_protocol.h"

// 详细诊断 OLED 界面开关；默认关闭，仅用于本地调试，不作为全局配置。
#define MOTOR_DEBUG_OLED_DETAILS 0

void Motor_ShowDebugInfo_OLED(void)
{
    static uint32_t last_refresh_ms = 0U;
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_refresh_ms) < 5U) {
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
if (MOTOR_DEBUG_OLED_DETAILS)  // 开启 OLED 诊断显示：d/q电流、ADC原始值、角度、电位器
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

    /* Studio v2 遥测统一封帧，避免裸 JustFloat 与 ACK 在同一上行流混杂。 */
    PC_Protocol_SendTelemetry();
}
