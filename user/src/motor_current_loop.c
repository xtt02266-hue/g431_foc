#include "motor_current_loop.h"
#include "adc.h"

static float g_vref_volts = MOTOR_CURRENT_VREF_VOLTS;
static float g_bias_volts = MOTOR_CURRENT_BIAS_VOLTS;
static float g_shunt_ohms = MOTOR_CURRENT_SHUNT_OHMS;
static float g_gain = MOTOR_CURRENT_GAIN;
static float g_adc_max = MOTOR_CURRENT_ADC_MAX;

static volatile MotorCurrentSample g_last_sample;

static float Motor_CurrentLoop_RawToCurrent(uint16_t raw)
{
    float v_in = ((float)raw * g_vref_volts) / g_adc_max;
    float v_shunt = v_in - g_bias_volts;
    return v_shunt / (g_shunt_ohms * g_gain);
}

static void Motor_CurrentLoop_Run(uint16_t iu_raw, uint16_t iw_raw)
{
    float iu_a = Motor_CurrentLoop_RawToCurrent(iu_raw);
    float iw_a = Motor_CurrentLoop_RawToCurrent(iw_raw);

    g_last_sample.iu_raw = iu_raw;
    g_last_sample.iw_raw = iw_raw;
    g_last_sample.iu_a = iu_a;
    g_last_sample.iw_a = iw_a;

    // TODO: add current loop control here (Clarke/Park + PI + SVPWM).
}

void Motor_CurrentLoop_Init(void)
{
    g_vref_volts = MOTOR_CURRENT_VREF_VOLTS;
    g_bias_volts = MOTOR_CURRENT_BIAS_VOLTS;
    g_shunt_ohms = MOTOR_CURRENT_SHUNT_OHMS;
    g_gain = MOTOR_CURRENT_GAIN;
    g_adc_max = MOTOR_CURRENT_ADC_MAX;

    g_last_sample.iu_raw = 0U;
    g_last_sample.iw_raw = 0U;
    g_last_sample.iu_a = 0.0f;
    g_last_sample.iw_a = 0.0f;
}

void Motor_CurrentLoop_SetBiasVolts(float bias_volts)
{
    g_bias_volts = bias_volts;
}

void Motor_CurrentLoop_SetVrefVolts(float vref_volts)
{
    g_vref_volts = vref_volts;
}

void Motor_CurrentLoop_SetGain(float gain)
{
    g_gain = gain;
}

void Motor_CurrentLoop_SetShuntOhms(float shunt_ohms)
{
    g_shunt_ohms = shunt_ohms;
}

MotorCurrentSample Motor_CurrentLoop_GetLastSample(void)
{
    MotorCurrentSample sample;

    __disable_irq();
    sample.iu_raw = g_last_sample.iu_raw;
    sample.iw_raw = g_last_sample.iw_raw;
    sample.iu_a = g_last_sample.iu_a;
    sample.iw_a = g_last_sample.iw_a;
    __enable_irq();

    return sample;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1)
    {
        return;
    }

    uint16_t iu_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint16_t iw_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

    Motor_CurrentLoop_Run(iu_raw, iw_raw);
}
