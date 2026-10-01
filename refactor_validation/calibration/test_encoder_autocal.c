#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_encoder_calibration.h"
#include "motor_system.h"
#ifdef NDEBUG
#error "Encoder calibration simulation requires active assertions"
#endif
MotorSystem g_motor_system;
static MotorControlSnapshot snapshot;
static unsigned now, mask, stopped, suspended, crc, transfers, warnings;
static unsigned busy_once, spi_error, bad_readback, stale_success, fail_chip;
static unsigned injected_stop, key_started_at, key_writes, polls;
static uint16_t angle;
static uint8_t frequency;
uint32_t HAL_GetTick(void) {return now;}
uint32_t __get_PRIMASK(void) {return mask;}
void __disable_irq(void) {mask=1;}
void __enable_irq(void) {mask=0;}
void Motor_System_GetControlSnapshot(MotorControlSnapshot *s) {*s=snapshot;}
void Motor_System_StopControl(void) {++stopped;snapshot.run_requested=0;Motor_EncoderCal_NotifyStop();}
uint8_t Motor_Encoder_IsDataFresh(uint32_t ms) {(void)ms;return 1;}
uint16_t Motor_Encoder_GetRawAngle(void) {return angle;}
uint32_t MT6826S_GetCrcErrorCount(void) {return crc;}
uint32_t MT6826S_GetTransferErrorCount(void) {return transfers;}
uint8_t MT6826S_GetStatus(void) {return warnings;}
void MT6826S_SuspendAngleReads(void) {++suspended;}
uint8_t MT6826S_ReadRegister(uint16_t reg,uint8_t *v) {
 assert(!suspended);
 if(busy_once){busy_once=0;return 0;}
 if(spi_error)return 2;
 if(reg==0xE)*v=frequency;
 else {assert(reg==0x113);++polls;
  *v=stale_success?0xc0:fail_chip?0x80:now-key_started_at>5000?0xc0:0x40;}
 return 1;
}
uint8_t MT6826S_WriteRegister(uint16_t reg,uint8_t v) {
 assert(!suspended);
 if(busy_once){busy_once=0;return 0;}
 if(spi_error)return 2;
 if(reg==0xE) {frequency=v;if(bad_readback)frequency^=0x10;}
 else {assert(reg==0x155);
  if(v==0x5e){key_started_at=now;++key_writes;
   if(injected_stop){Motor_System_StopControl();}}
  else assert(v==0);}
 return 1;
}
static MotorEncoderCalStatus status(void) {MotorEncoderCalStatus s;Motor_EncoderCal_GetStatus(&s);return s;}
static void tick(void) {++now;angle=(angle-137)&32767;Motor_EncoderCal_BackgroundTask();}
static void reset(void) {
 now=mask=stopped=suspended=crc=transfers=warnings=0;
 busy_once=spi_error=bad_readback=stale_success=fail_chip=injected_stop=0;
 key_started_at=key_writes=polls=angle=0;frequency=0xbd;
 snapshot=(MotorControlSnapshot){MOTOR_STATE_SENSORED_RUN,MOTOR_OWNER_HOST,1,250};
 g_motor_system.run_data.speed_rpm=-250;Motor_EncoderCal_Begin();
}
static void configured(void) {for(unsigned i=0;i<1100 && status().phase!=ENC_CAL_RUNNING;i++)tick();}
int main(void) {
 reset();busy_once=1;configured();assert(status().phase==ENC_CAL_RUNNING);
 assert(frequency==0xcd);assert(key_writes==1);
 for(unsigned i=0;i<5500 && status().phase==ENC_CAL_RUNNING;i++)tick();
 assert(status().phase==ENC_CAL_SUCCESS && status().calibrated_counts>18*32768);
 assert(Motor_EncoderCal_RequiresPowerCycle() && !Motor_EncoderCal_IsActive());
 assert(stopped==1 && suspended==1);unsigned elapsed=status().elapsed_ms;
 tick();assert(status().elapsed_ms==elapsed);

 reset();stale_success=1;configured();for(unsigned i=0;i<110;i++)tick();
 assert(status().phase==ENC_CAL_FAILED && status().reason==ENC_CAL_UNCONFIRMED);
 assert(suspended==1 && key_writes==1);

 reset();fail_chip=1;configured();for(unsigned i=0;i<110;i++)tick();
 assert(status().phase==ENC_CAL_FAILED && status().reason==ENC_CAL_CHIP_FAILED);
 assert(Motor_EncoderCal_RequiresPowerCycle() && suspended==1);

 reset();configured();++crc;tick();assert(status().phase==ENC_CAL_FAILED && status().reason==ENC_CAL_SENSOR);
 tick();assert(suspended==1);

 reset();g_motor_system.run_data.speed_rpm=0;
 for(unsigned i=0;i<5010;i++)tick();assert(status().phase==ENC_CAL_FAILED && status().reason==ENC_CAL_SPEED);
 assert(!Motor_EncoderCal_RequiresPowerCycle() && key_writes==0);

 reset();Motor_System_StopControl();tick();assert(status().phase==ENC_CAL_ABORTED);
 assert(!Motor_EncoderCal_RequiresPowerCycle() && !Motor_EncoderCal_IsActive());

 reset();injected_stop=1;configured();tick();
 assert(status().phase==ENC_CAL_ABORTED && Motor_EncoderCal_RequiresPowerCycle());
 assert(key_writes==1 && suspended==1); /* STOP during SPI cannot be overwritten */

 reset();bad_readback=1;configured();tick();
 assert(status().phase==ENC_CAL_FAILED && status().reason==ENC_CAL_READBACK && !key_writes);

 puts("PASS: MT6826S calibration busy arbitration, reserved bits, success proof, stale result rejection, chip failure, CRC failure, speed timeout, STOP and interrupt-during-trigger");
}
