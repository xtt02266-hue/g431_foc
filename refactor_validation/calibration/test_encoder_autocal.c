#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_encoder_calibration.h"
#include "motor_system.h"
#ifdef NDEBUG
#error "Encoder calibration simulation requires active assertions"
#endif
motor_system_t g_motor_system;
static motor_control_snapshot_t snapshot;
static unsigned now, mask, stopped, suspended, crc, transfers, warnings;
static unsigned busy_once, spi_error, bad_readback, stale_success, fail_chip;
static unsigned injected_stop, key_started_at, key_writes, polls;
static uint16_t angle;
static uint8_t frequency;
uint32_t HAL_GetTick(void) {return now;}
uint32_t __get_PRIMASK(void) {return mask;}
void __disable_irq(void) {mask=1;}
void __enable_irq(void) {mask=0;}
void motor_system_get_control_snapshot(motor_control_snapshot_t *s) {*s=snapshot;}
void motor_system_stop_control(void) {++stopped;snapshot.run_requested=0;motor_encoder_cal_notify_stop();}
uint8_t motor_encoder_is_data_fresh(uint32_t ms) {(void)ms;return 1;}
uint16_t motor_encoder_get_raw_angle(void) {return angle;}
uint32_t mt6826_s_get_crc_error_count(void) {return crc;}
uint32_t mt6826_s_get_transfer_error_count(void) {return transfers;}
uint8_t mt6826_s_get_status(void) {return warnings;}
void mt6826_s_suspend_angle_reads(void) {++suspended;}
uint8_t mt6826_s_read_register(uint16_t reg,uint8_t *v) {
 assert(!suspended);
 if(busy_once){busy_once=0;return 0;}
 if(spi_error)return 2;
 if(reg==0xE)*v=frequency;
 else {assert(reg==0x113);++polls;
  *v=stale_success?0xc0:fail_chip?0x80:now-key_started_at>5000?0xc0:0x40;}
 return 1;
}
uint8_t mt6826_s_write_register(uint16_t reg,uint8_t v) {
 assert(!suspended);
 if(busy_once){busy_once=0;return 0;}
 if(spi_error)return 2;
 if(reg==0xE) {frequency=v;if(bad_readback)frequency^=0x10;}
 else {assert(reg==0x155);
  if(v==0x5e){key_started_at=now;++key_writes;
   if(injected_stop){motor_system_stop_control();}}
  else assert(v==0);}
 return 1;
}
static motor_encoder_cal_status_t status(void) {motor_encoder_cal_status_t s;motor_encoder_cal_get_status(&s);return s;}
static void tick(void) {++now;angle=(angle-137)&32767;motor_encoder_cal_background_task();}
static void reset(void) {
 now=mask=stopped=suspended=crc=transfers=warnings=0;
 busy_once=spi_error=bad_readback=stale_success=fail_chip=injected_stop=0;
 key_started_at=key_writes=polls=angle=0;frequency=0xbd;
 snapshot=(motor_control_snapshot_t){MOTOR_STATE_SENSORED_RUN,MOTOR_OWNER_HOST,1,250};
 g_motor_system.run_data.speed_rpm=-250;motor_encoder_cal_begin();
}
static void configured(void) {for(unsigned i=0;i<1100 && status().phase!=ENC_CAL_RUNNING;i++)tick();}
int main(void) {
 reset();busy_once=1;configured();assert(status().phase==ENC_CAL_RUNNING);
 assert(frequency==0xcd);assert(key_writes==1);
 for(unsigned i=0;i<5500 && status().phase==ENC_CAL_RUNNING;i++)tick();
 assert(status().phase==ENC_CAL_SUCCESS && status().calibrated_counts>18*32768);
 assert(motor_encoder_cal_requires_power_cycle() && !motor_encoder_cal_is_active());
 assert(stopped==1 && suspended==1);unsigned elapsed=status().elapsed_ms;
 tick();assert(status().elapsed_ms==elapsed);

 reset();stale_success=1;configured();for(unsigned i=0;i<110;i++)tick();
 assert(status().phase==ENC_CAL_FAILED && status().reason==ENC_CAL_UNCONFIRMED);
 assert(suspended==1 && key_writes==1);

 reset();fail_chip=1;configured();for(unsigned i=0;i<110;i++)tick();
 assert(status().phase==ENC_CAL_FAILED && status().reason==ENC_CAL_CHIP_FAILED);
 assert(motor_encoder_cal_requires_power_cycle() && suspended==1);

 reset();configured();++crc;tick();assert(status().phase==ENC_CAL_FAILED && status().reason==ENC_CAL_SENSOR);
 tick();assert(suspended==1);

 reset();g_motor_system.run_data.speed_rpm=0;
 for(unsigned i=0;i<5010;i++)tick();assert(status().phase==ENC_CAL_FAILED && status().reason==ENC_CAL_SPEED);
 assert(!motor_encoder_cal_requires_power_cycle() && key_writes==0);

 reset();motor_system_stop_control();tick();assert(status().phase==ENC_CAL_ABORTED);
 assert(!motor_encoder_cal_requires_power_cycle() && !motor_encoder_cal_is_active());

 reset();injected_stop=1;configured();tick();
 assert(status().phase==ENC_CAL_ABORTED && motor_encoder_cal_requires_power_cycle());
 assert(key_writes==1 && suspended==1); /* STOP during SPI cannot be overwritten */

 reset();bad_readback=1;configured();tick();
 assert(status().phase==ENC_CAL_FAILED && status().reason==ENC_CAL_READBACK && !key_writes);

 puts("PASS: MT6826S calibration busy arbitration, reserved bits, success proof, stale result rejection, chip failure, CRC failure, speed timeout, STOP and interrupt-during-trigger");
}
