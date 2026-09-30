#include "motor_calibration.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void drain(void) {
    MotorCalSample records[3]; uint8_t n;
    while ((n=Motor_Calibration_Peek(records,3))) Motor_Calibration_Consume(n);
}
static MotorCalStatus status(void) {
    MotorCalStatus s; Motor_Calibration_GetStatus(&s); return s;
}
static void reset(void) { Motor_Calibration_Abort(CAL_CANCELLED); drain(); }

static void run(int stall_first, int friction) {
    assert(Motor_Calibration_Begin(friction ? 4 : 1));
    double position=0; unsigned ticks=0;
    while (Motor_Calibration_IsActive() && ticks++<300000) {
        MotorCalStatus s=status();
        float speed=s.target_rpm;
        if (friction) speed *= 1.0f + .10f*sinf(ticks*.0062831853f);
        if (stall_first && s.speed_index==0) speed=0;
        position+=speed*32768.0/60000.0;
        uint16_t angle=(uint16_t)((int64_t)position & 32767);
        Motor_Calibration_Step(angle,speed,s.target_rpm,0,1);
        MotorCalSample sample={0}; sample.tick_ms=ticks;
        sample.flags=Motor_Calibration_SampleFlags();
        Motor_Calibration_PublishControl(&sample);
        Motor_Calibration_Capture(angle,.01f,.009f,0,.02f);
        /* Several ADC cycles in a control period must not duplicate a record. */
        Motor_Calibration_Capture(angle,.01f,.009f,0,.02f);
        if (ticks%3==0) drain();
    }
    MotorCalStatus s=status();
    assert(ticks<300000 && s.phase==(friction && stall_first ? CAL_FAILED : CAL_DONE) && s.dropped==0);
    assert(s.passed_mask==(stall_first?6:7));
    assert(s.speed_reasons[0]==(stall_first?CAL_STALL:CAL_OK));
    assert(s.produced>30000 && s.produced<=ticks);
    assert(s.reason==(friction && stall_first ? CAL_INSUFFICIENT : CAL_OK));
    drain();
}

int main(int argc, char **argv) {
    assert(sizeof(MotorCalSample)==64);
    assert(!Motor_Calibration_Begin(6));
    run(0,0);run(1,0);run(0,1);run(1,1);
    assert(Motor_Calibration_Begin(5));
    unsigned point_ticks=0;
    while (Motor_Calibration_IsActive() && point_ticks++<1000000) {
        float target=Motor_Calibration_PointTarget();
        Motor_Calibration_StepPoint((uint16_t)target,0,0,1);
        MotorCalStatus ps=status();
        if (ps.phase==CAL_CAPTURE) {
            assert(!Motor_Calibration_PointAssist());
            MotorCalSample rec={0};rec.tick_ms=point_ticks;rec.flags=Motor_Calibration_SampleFlags();
            Motor_Calibration_PublishControl(&rec);Motor_Calibration_Capture((uint16_t)target,.01f,.01f,0,0);
            float saved;Motor_Calibration_PeekTargets(&saved,1);assert(saved==target);drain();
        }
    }
    assert(status().phase==CAL_DONE && status().passed_mask==7 && status().dropped==0);
    assert(status().produced>=MOTOR_CAL_POINT_CAPTURE_MS*128*6);
    assert(Motor_Calibration_Begin(5));
    assert(Motor_Calibration_PointAssist());
    Motor_Calibration_StepPoint(32512,0,0,1);
    assert(!Motor_Calibration_PointAssist() && status().phase==CAL_ZERO);
    for (unsigned i=0;i<MOTOR_CAL_POINT_SETTLE_MS-1;i++) Motor_Calibration_StepPoint(32512,0,0,1);
    assert(status().phase==CAL_ZERO);
    Motor_Calibration_StepPoint(32512,0,0,1);
    assert(Motor_Calibration_PointTarget()==0 && Motor_Calibration_PointAssist());
    Motor_Calibration_StepPoint(0,0,0,1);
    for (unsigned i=0;i<MOTOR_CAL_POINT_SETTLE_MS;i++) Motor_Calibration_StepPoint(0,0,0,1);
    assert(status().phase==CAL_CAPTURE);
    Motor_Calibration_StepPoint(0,0,CAL_BAD_SENSOR,1);
    assert(status().phase==CAL_SETTLE && status().accepted_turns==0);
    for (unsigned i=0;i<MOTOR_CAL_POINT_SETTLE_MS-1;i++) Motor_Calibration_StepPoint(0,0,0,1);
    assert(status().phase==CAL_SETTLE);
    Motor_Calibration_StepPoint(0,0,0,1);assert(status().phase==CAL_CAPTURE);
    for (unsigned i=0;i<MOTOR_CAL_POINT_CAPTURE_MS-1;i++) Motor_Calibration_StepPoint(0,0,0,1);
    assert(status().accepted_turns==0 && Motor_Calibration_PointTarget()==0);
    Motor_Calibration_StepPoint(0,0,0,1);
    assert(status().accepted_turns==1 && Motor_Calibration_PointTarget()==256);
    reset();
    assert(Motor_Calibration_Begin(5));
    for (unsigned i=0;i<10000;i++) Motor_Calibration_StepPoint(0,10,0,1);
    assert(status().phase==CAL_FAILED && status().reason==CAL_UNSTABLE);
    assert(Motor_Calibration_Begin(1));
    Motor_Calibration_Step(0,0,60,0,0);
    assert(status().reason==CAL_SENSOR && !Motor_Calibration_IsActive());
    reset();assert(Motor_Calibration_Begin(1));
    for (unsigned i=0;i<20000;i++) Motor_Calibration_Step((i*20)&32767,30,60,0,1);
    assert(status().phase==CAL_ZERO && status().speed_reasons[0]==CAL_UNSTABLE);
    for (unsigned i=0;i<10001;i++) Motor_Calibration_Step(0,30,0,0,1);
    assert(status().reason==CAL_TIMEOUT);
    reset();assert(Motor_Calibration_Begin(1));
    MotorCalSample sample={0};
    for (unsigned i=0;i<MOTOR_CAL_RING_SIZE+1;i++) Motor_Calibration_Push(&sample);
    assert(status().dropped==1);
    Motor_Calibration_Step(0,60,60,0,1);
    assert(status().reason==CAL_OVERFLOW);
    assert(!Motor_Calibration_Begin(1)); /* pending old records must drain first */
    reset();assert(Motor_Calibration_Begin(1));
    Motor_Calibration_Abort(CAL_CANCELLED);
    assert(status().phase==CAL_ABORTED && status().target_rpm==0);
    reset();assert(Motor_Calibration_Begin(1));
    sample.tick_ms=123;sample.flags=2<<8;sample.target_rpm=60;sample.speed_rpm=59;
    sample.window_rpm=58;sample.pi_iq=.001f;sample.friction_iq=.002f;sample.cogging_iq=.003f;
    sample.window_ticks=20;sample.sensor_age_ms=1;sample.crc_errors=4;sample.transfer_errors=5;
    Motor_Calibration_PublishControl(&sample);
    Motor_Calibration_Capture(12345,.01f,.009f,.008f,.007f);
    MotorCalSample received;
    assert(Motor_Calibration_Peek(&received,1)==1);
    assert(received.sequence==1 && received.angle==12345 && received.tick_ms==123);
    if (argc>1) { FILE *f=fopen(argv[1],"wb");assert(f);assert(fwrite(&received,64,1,f)==1);fclose(f); }
    drain();
    float targets[3];MotorCalSample replay[3];
    assert(Motor_Calibration_Replay(1,1,replay,targets)==1 && replay[0].angle==12345);
    assert(!Motor_Calibration_Replay(0,1,replay,targets));
    assert(!Motor_Calibration_Replay(1,4,replay,targets));
    assert(!Motor_Calibration_Replay(2,1,replay,targets));
    for (unsigned i=0;i<MOTOR_CAL_RING_SIZE;i++) { Motor_Calibration_Push(&sample);drain(); }
    assert(!Motor_Calibration_Replay(1,1,replay,targets)); /* overwritten, never fabricate */
    assert(Motor_Calibration_Replay(2,3,replay,targets)==3 && replay[2].sequence==4);
    assert(Motor_Calibration_Replay(MOTOR_CAL_RING_SIZE+1,1,replay,targets)==1);
    reset();
    puts("PASS: 3-speed reversal, skip stalled speed, settling timeout, sensor, ring overflow, cancel, ADC dedup, wire record");
    return 0;
}
