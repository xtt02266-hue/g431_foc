#include "hardware_init.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "i2c.h"
#include "user_io.h"
#include "motor_current_loop.h"
#include "motor_identify.h"
#include "svpwm.h"
#include "pc_protocol.h"
#include "pc_transport.h"
#include "motor_encoder.h"
#include "motor_system.h"
#include "motor_parameters.h"
#include "board_profile.h"
#include "motor_angle_reference.h"

// OLED_Init 由显示驱动实现，这里做前置声明。
void OLED_Init(void);

// ---------------------------------------------------------
// 电流偏置(Bias)校准：在不施加任何电压时获取真正的零点电压偏移
// ---------------------------------------------------------
static void Hardware_CalibrateCurrentBias(void)
{
    uint32_t bias_sum_u = 0;
    uint32_t bias_sum_w = 0;
    const uint16_t calibrate_count = 500;
    
    HAL_Delay(10); // 等待 ADC 稳妥上电
    
    for (uint16_t i = 0; i < calibrate_count; i++)
    {
        // 在这里因为没开启 PWM 通道强推电机，我们手动软件触发注入采样
        // 然后累加求平均值
        HAL_ADCEx_InjectedStart(&hadc1); 
        HAL_ADCEx_InjectedPollForConversion(&hadc1, 10);
        
        bias_sum_u += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        bias_sum_w += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        
    }
    
    // 计算平均原始值并求出基准电压
    float bias_raw_u = (float)bias_sum_u / calibrate_count;
    float bias_raw_w = (float)bias_sum_w / calibrate_count;
    // 两路运放零点并不完全相同，分别换算并保存，避免公共偏置制造相反零漂。
    MotorCurrentParams params = Motor_CurrentLoop_GetParams();
    float bias_u_volts = (bias_raw_u * params.vref_volts) / params.adc_max;
    float bias_w_volts = (bias_raw_w * params.vref_volts) / params.adc_max;
    Motor_CurrentLoop_SetPhaseBiasVolts(bias_u_volts, bias_w_volts);
    HAL_ADCEx_InjectedStop(&hadc1);
    
}

// 硬件外设初始化与启动时序。
void hardware_init(void)
{
    HAL_Delay(100); // 上电后等待外设稳定（尤其是 ADC 和 I2C）
    // 0. 执行 ADC 内部自校准以提高采样精度（必须在 ADC 启动前调用）
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    UserIO_StartDma();

    /* SPI1已由CubeMX初始化；先建立首个15位角度快照，再启动FOC采样中断。 */
#if BOARD_SENSORED_CONTROL_ENABLE
    Motor_Encoder_Init();
#endif
    Motor_AngleReference_Init();


    // 2. 启动 TIM1 通道 4（用作 ADC 的触发信号 CC4）
    //此时不开启PWM输出（占空比全0），对运放偏置进行初始标定
    HAL_TIM_Base_Init(&htim1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

    // 3. 首先配置并开启 ADC 注入通道，等待被 TIM1 触发
    Motor_CurrentLoop_Init();
    HAL_Delay(10);
    // 调用封装好的偏置校准函数
    Hardware_CalibrateCurrentBias();
    HAL_Delay(10);
    HAL_ADCEx_InjectedStart_IT(&hadc1);


    // 4. 启动 TIM1 的 6 路互补 PWM 输出（驱动三相半桥）。
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

    // 初始化 SVPWM 模块（三相输出 50% 零矢量，上电安全）
    SVPWM_Init();

    // 5. 启动 TIM2 (用于 1ms / 1000Hz 周期任务调度)

    // 6. 启动所选的上位机传输后端（UART DMA 或 USB CDC）。
    PC_Protocol_Init();
    PC_Transport_Init();

    OLED_Init();
    HAL_Delay(50);
    
    Motor_System_Init();

    // Load parameters from Flash, or schedule one identification if absent.
    Motor_Parameters_Init();
    HAL_TIM_Base_Start_IT(&htim2);
}
