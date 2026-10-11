#include "motor_debug.h"
#include "motor_system.h"
#include "motor_current_loop.h"
#include "motor_identify.h"
#include "motor_parameters.h"
#include "oled.h"
#include "vofa_usart.h"
#include "pc_protocol.h"

// 详细诊断 OLED 界面开关；默认关闭，仅用于本地调试，不作为全局配置。
#define MOTOR_DEBUG_OLED_DETAILS 0 // 0 关闭详细 OLED 分支，保持当前 VOFA 调试行为

// 前台输出 VOFA 观测量；详细 OLED 分支受原配置开关控制。
void motor_debug_show_info_oled(void)
{
    static uint32_t last_refresh_ms = 0U;
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_refresh_ms) < 5U) {
        return;
    }
    last_refresh_ms = now;

    // 获取实时的 FOC 内部状态（电流、坐标变换后结果）
    // 注意：这里读取全局变量，当前读取不新增临界区，观测字段可能来自相邻控制周期。
    motor_identify_state_t id_state = motor_identify_get_state();
    motor_parameters_status_t parameter_status = motor_parameters_get_status();

    // 1. OLED 界面显示关键调度状态
    if (g_motor_system.state == MOTOR_STATE_FAULT) {
        oled_show_string(1, 1, "Fault");
    } else if (parameter_status == MOTOR_PARAMETERS_NO_DATA) {
        oled_show_string(1, 1, "NoParam");
    } else if (parameter_status == MOTOR_PARAMETERS_ERROR) {
        oled_show_string(1, 1, "ParamErr");
    } else if (parameter_status != MOTOR_PARAMETERS_READY) {
        // 辨识进行中：显示状态编号 + 目标 q 电流
        oled_show_string(1, 1, "Idt"); // "Idt" = Identify (辨识中)
        oled_show_string(1, 4, "Tq:");
        oled_show_signed_num(1, 7, (int32_t)(g_foc_state.target_q * 1000.0f), 5); // 显示 mA 级
        oled_show_num(1, 14, id_state, 2);                                        // 辨识状态码
    } else {
        // 辨识完成：OLED 显示暂时关闭以节省 CPU
        // oled_show_string(1, 1, "Run");
        //oled_show_signed_num(1, 7, (int32_t)(g_foc_state.target_q * 1000.0f), 5);
    }
    // 详细界面刷新会增加前台负载；沿用配置开关控制 d/q 电流及 ADC 观测。
    if (MOTOR_DEBUG_OLED_DETAILS) {
        // 第2行：D 轴实际电流 (mA) + ADC U 相原始值
        oled_show_char(2, 1, 'd');
        oled_show_signed_num(2, 2, (int32_t)(g_foc_state.park.d * 1000.0f), 4);
        oled_show_string(2, 7, "U:");
        oled_show_num(2, 9, g_foc_state.sample.iu_raw, 4);

        // 第3行：Q 轴实际电流 (mA) + ADC W 相原始值
        oled_show_char(3, 1, 'q');
        oled_show_signed_num(3, 2, (int32_t)(g_foc_state.park.q * 1000.0f), 4);
        oled_show_string(3, 7, "A:");
        // oled_show_num(3, 9, motor_encoder_get_raw_angle(), 5);
    }

    /* Studio v2 遥测统一封帧，避免裸 JustFloat 与 ACK 在同一上行流混杂。 */
    pc_protocol_send_telemetry();
}
