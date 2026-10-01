#include "motor_encoder_calibration.h"
#include "motor_system.h"
#include "motor_encoder.h"
#include "mt6826s.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <string.h>

/* MT6826S Rev1.1 sections 8.6.4/5 and 10.2. No blanket EEPROM-program
 * command: only the runtime AUTO_CAL_FREQ field is changed, preserving all
 * reserved bits. The chip performs its documented internal self-calibration. */
#define FREQ_REG 0x00EU
#define KEY_REG 0x155U
#define STATUS_REG 0x113U
static volatile uint8_t phase, reason, reboot_required, cleanup;
static uint8_t step, original_freq, freq_known, freq_written;
static volatile uint8_t trigger_attempted;
static uint8_t chip_status, saw_running, settle_started;
static uint32_t started, stage_started, settled_since;
static volatile uint32_t operation_started, finished_at;
static uint32_t crc_start, transfer_start, last_poll, counts;
static uint16_t last_angle;
static int32_t travel;
static float measured_rpm;

uint8_t Motor_EncoderCal_IsActive(void)
{
    return (phase >= ENC_CAL_RAMP && phase <= ENC_CAL_RUNNING) || cleanup;
}
uint8_t Motor_EncoderCal_RequiresPowerCycle(void) { return reboot_required; }

void Motor_EncoderCal_Begin(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    reason=ENC_CAL_OK;
    step=chip_status=saw_running=settle_started=freq_known=freq_written=0U;
    trigger_attempted=cleanup=reboot_required=0U;
    started=stage_started=HAL_GetTick(); counts=0U; travel=0;
    crc_start=MT6826S_GetCrcErrorCount();
    transfer_start=MT6826S_GetTransferErrorCount();
    phase=ENC_CAL_RAMP;
    if (!mask) __enable_irq();
}

void Motor_EncoderCal_NotifyStop(void)
{
    if (phase >= ENC_CAL_RAMP && phase <= ENC_CAL_RUNNING) {
        phase=ENC_CAL_ABORTED; reason=ENC_CAL_CANCELLED;
        finished_at=HAL_GetTick();
        reboot_required=trigger_attempted;
        cleanup=1U; operation_started=HAL_GetTick();
    }
}

static void finish(uint8_t result_phase, uint8_t result_reason)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (phase < ENC_CAL_RAMP || phase > ENC_CAL_RUNNING) {
        if (!mask) __enable_irq();
        return;
    }
    phase=result_phase; reason=result_reason;
    finished_at=HAL_GetTick();
    reboot_required=trigger_attempted;
    cleanup=(result_phase == ENC_CAL_SUCCESS || chip_status==3U) ? 0U : 1U;
    operation_started=HAL_GetTick();
    if (result_phase == ENC_CAL_SUCCESS || chip_status==3U) MT6826S_SuspendAngleReads();
    Motor_System_StopControl();
    if (!mask) __enable_irq();
}

/* Main loop only. Each SPI operation is bounded to 1ms, arbitration skips a
 * busy angle DMA rather than waiting in the 1kHz/20kHz motor interrupts. */
void Motor_EncoderCal_BackgroundTask(void)
{
    uint32_t now=HAL_GetTick();
    if (cleanup) {
        uint8_t r=1U;
        if (trigger_attempted) r=MT6826S_WriteRegister(KEY_REG,0U);
        else if (freq_known && freq_written) r=MT6826S_WriteRegister(FREQ_REG,original_freq);
        if (r || (uint32_t)(now-operation_started)>100U) {
            cleanup=0U;
            if (reboot_required) MT6826S_SuspendAngleReads();
        }
        return;
    }
    if (!Motor_EncoderCal_IsActive()) return;
    MotorControlSnapshot s; Motor_System_GetControlSnapshot(&s);
    measured_rpm=g_motor_system.run_data.speed_rpm;
    if (s.owner!=MOTOR_OWNER_HOST || !s.run_requested) {
        finish(ENC_CAL_ABORTED,ENC_CAL_CONTROL_LOST); return;
    }
    if (s.state==MOTOR_STATE_FAULT || !Motor_Encoder_IsDataFresh(2U) ||
        MT6826S_GetStatus()!=0U || MT6826S_GetCrcErrorCount()!=crc_start ||
        MT6826S_GetTransferErrorCount()!=transfer_start) {
        finish(ENC_CAL_FAILED,ENC_CAL_SENSOR); return;
    }
    if ((uint32_t)(now-started)>35000U) {
        finish(ENC_CAL_FAILED,ENC_CAL_TIMEOUT); return;
    }
    if (phase==ENC_CAL_RAMP) {
        if (s.state==MOTOR_STATE_SENSORED_RUN && s.target_speed_rpm>=249.9f) {
            phase=ENC_CAL_SETTLE; stage_started=now;
        }
    } else if (phase==ENC_CAL_SETTLE) {
        if (fabsf(fabsf(measured_rpm)-250.0f)<=25.0f) {
            if (!settle_started) { settled_since=now; settle_started=1U; }
            if ((uint32_t)(now-settled_since)>=1000U) {
                phase=ENC_CAL_CONFIGURE; operation_started=now;
            }
        } else settle_started=0U;
        if ((uint32_t)(now-stage_started)>5000U) finish(ENC_CAL_FAILED,ENC_CAL_SPEED);
    } else if (phase==ENC_CAL_CONFIGURE) {
        uint8_t r=0U, value=0U;
        switch (step) {
        case 0:
            r=MT6826S_ReadRegister(FREQ_REG,&original_freq);
            if (r==1U) freq_known=1U;
            break;
        case 1:
            freq_written=1U; /* a timed-out write may still have reached the chip */
            r=MT6826S_WriteRegister(FREQ_REG,(original_freq & 0x8FU) | 0x40U);
            break;
        case 2:
            r=MT6826S_ReadRegister(FREQ_REG,&value);
            if (r==1U && value!=((original_freq & 0x8FU) | 0x40U)) {
                finish(ENC_CAL_FAILED,ENC_CAL_READBACK); return;
            }
            break;
        case 3: r=MT6826S_WriteRegister(KEY_REG,0U); break;
        default:
            /* Conservatively latch restart before a possibly uncertain write. */
            trigger_attempted=1U;
            r=MT6826S_WriteRegister(KEY_REG,0x5EU);
            if (r==0U) trigger_attempted=0U; /* busy means no bytes sent */
            if (r==1U && phase==ENC_CAL_CONFIGURE) {
                phase=ENC_CAL_RUNNING; stage_started=now; last_poll=now;
                last_angle=Motor_Encoder_GetRawAngle();
            }
            break;
        }
        if (phase!=ENC_CAL_CONFIGURE && phase!=ENC_CAL_RUNNING) return;
        if (r==2U) { finish(ENC_CAL_FAILED,ENC_CAL_SPI); return; }
        if (r==1U) { ++step; operation_started=now; }
        if ((uint32_t)(now-operation_started)>100U) finish(ENC_CAL_FAILED,ENC_CAL_SPI);
    } else if (phase==ENC_CAL_RUNNING) {
        uint16_t angle=Motor_Encoder_GetRawAngle();
        int32_t delta=(int32_t)angle-last_angle;
        if (delta>16384) delta-=32768;
        if (delta<-16384) delta+=32768;
        travel+=delta; last_angle=angle;
        counts=(uint32_t)(travel<0 ? -travel : travel);
        if (!isfinite(measured_rpm) || fabsf(measured_rpm)<200.0f || fabsf(measured_rpm)>=400.0f) {
            finish(ENC_CAL_FAILED,ENC_CAL_SPEED); return;
        }
        if ((uint32_t)(now-last_poll)<100U) return;
        uint8_t value=0U, r=MT6826S_ReadRegister(STATUS_REG,&value);
        if (r==2U) { finish(ENC_CAL_FAILED,ENC_CAL_SPI); return; }
        if (r==0U) return;
        last_poll=now; chip_status=(value>>6U)&3U;
        if (phase!=ENC_CAL_RUNNING) return;
        if (chip_status==1U) saw_running=1U;
        if (chip_status==2U) finish(ENC_CAL_FAILED,ENC_CAL_CHIP_FAILED);
        else if (chip_status==3U)
            finish(saw_running && counts>18UL*32768UL ? ENC_CAL_SUCCESS : ENC_CAL_FAILED,
                   saw_running && counts>18UL*32768UL ? ENC_CAL_OK : ENC_CAL_UNCONFIRMED);
        else if ((uint32_t)(now-stage_started)>30000U)
            finish(ENC_CAL_FAILED,ENC_CAL_TIMEOUT);
    }
}

void Motor_EncoderCal_GetStatus(MotorEncoderCalStatus *s)
{
    memset(s,0,sizeof(*s));
    s->phase=phase; s->reason=reason; s->chip_status=chip_status;
    s->flags=(Motor_EncoderCal_IsActive()?1U:0U) | (reboot_required?2U:0U) |
             (phase!=ENC_CAL_IDLE?4U:0U) | (saw_running?8U:0U);
    s->elapsed_ms=phase==ENC_CAL_IDLE?0U:
        ((phase>=ENC_CAL_RAMP && phase<=ENC_CAL_RUNNING)?HAL_GetTick():finished_at)-started;
    s->calibrated_counts=counts; s->measured_rpm=measured_rpm;
}
