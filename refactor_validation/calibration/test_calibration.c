#include "motor_calibration.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#ifdef NDEBUG
#error "Calibration simulation requires assertions; compile with -UNDEBUG"
#endif

static void drain(void) {
    MotorCalSample records[3]; uint8_t n;
    while ((n=Motor_Calibration_Peek(records,3))) Motor_Calibration_Consume(n);
}
static MotorCalStatus status(void) {
    MotorCalStatus s; Motor_Calibration_GetStatus(&s); return s;
}
static void reset(void) { Motor_Calibration_Abort(CAL_CANCELLED); drain(); }
static void point_step(uint16_t angle,float reference,float actual) {
    static uint32_t tick;
    Motor_Calibration_StepPoint(angle,0,0,1);
    if (status().phase==CAL_CAPTURE) {
        MotorCalSample rec={0};rec.tick_ms=++tick;rec.flags=Motor_Calibration_SampleFlags();
        Motor_Calibration_PublishControl(&rec);
        Motor_Calibration_Capture(angle,reference,actual,0,0);
        drain();
    }
}
static void reach_first_point(void) {
    for (unsigned i=0;i<2*(MOTOR_CAL_POINT_SETTLE_MS+1);i++)
        point_step((uint16_t)Motor_Calibration_PointTarget(),.01f,.01f);
    assert(status().phase==CAL_CAPTURE && Motor_Calibration_PointTarget()==0);
}

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

static void test_average_policy(void) {
    uint8_t mode,repeats,wait,policy;uint16_t count;
    const uint8_t request[]={5,0,2,1,1,1};
    assert(Motor_Calibration_DecodeStartPolicy(request,6,&mode,&count,&repeats,&wait,&policy));
    assert(mode==5 && count==512 && repeats==1 && wait==1 && policy==1);
    assert(Motor_Calibration_DecodeStartPolicy(request,5,&mode,&count,&repeats,&wait,&policy) && policy==0);
    uint8_t bad[]={5,0,2,1,1,2};
    assert(!Motor_Calibration_DecodeStartPolicy(bad,6,&mode,&count,&repeats,&wait,&policy));
    assert(!Motor_Calibration_DecodeStartPolicy(request,4,&mode,&count,&repeats,&wait,&policy));
    assert(!Motor_Calibration_BeginWithPolicy(1,256,3,0,1));
    assert(Motor_Calibration_BeginWithPolicy(5,512,1,1,1));reach_first_point();
    /* New 6mA floor admits a 5mA standard deviation; old contract stays unchanged. */
    for (unsigned i=0;i<500;i++) point_step(0,i&1?.015f:.005f,i&1?.015f:.005f);
    assert(status().accepted_turns==1);reset();
    assert(Motor_Calibration_BeginWithPolicy(5,512,1,1,1));reach_first_point();
    /* Two noisy windows are discarded. Only a fresh 1000ms window may waive variance. */
    for (unsigned attempt=0;attempt<2;attempt++) {
        assert(!(Motor_Calibration_SampleFlags()&CAL_POINT_AVERAGE));
        for (unsigned i=0;i<500;i++) point_step(0,i&1?.03f:-.01f,i&1?.03f:-.01f);
        assert(status().phase==CAL_SETTLE && !status().accepted_turns);
        for (unsigned i=0;i<1000;i++) point_step(0,.01f,.01f);
    }
    assert(Motor_Calibration_SampleFlags()&CAL_POINT_AVERAGE);
    for (unsigned i=0;i<999;i++) point_step(0,i&1?.03f:-.01f,i&1?.03f:-.01f);
    assert(!status().accepted_turns && status().phase==CAL_CAPTURE);
    point_step(0,.01f,.01f);
    assert(status().accepted_turns==1 && Motor_Calibration_PointTarget()==64);
    assert(!(Motor_Calibration_SampleFlags()&CAL_POINT_AVERAGE));reset();
    /* Mean current tracking failure is never waived, even after entering fallback. */
    assert(Motor_Calibration_BeginWithPolicy(5,512,1,1,1));reach_first_point();
    for (unsigned i=0;i<8000;i++) point_step(0,.01f,.03f);
    assert(!status().accepted_turns && Motor_Calibration_PointTarget()==0);
    for (unsigned i=0;i<2000 && status().phase!=CAL_CAPTURE;i++) point_step(0,.01f,.01f);
    assert(status().phase==CAL_CAPTURE);
    assert(Motor_Calibration_SampleFlags()&CAL_POINT_AVERAGE);
    Motor_Calibration_StepPoint(4,0,0,1);
    assert(status().phase==CAL_SETTLE && !status().accepted_turns);
    for (unsigned i=0;i<1000;i++) point_step(0,.01f,.01f);
    assert(status().phase==CAL_CAPTURE);
    /* An ADC-only position excursion is still rejected by the complete-window checks. */
    MotorCalSample rec={0};rec.flags=Motor_Calibration_SampleFlags();
    Motor_Calibration_PublishControl(&rec);Motor_Calibration_Capture(4,.01f,.01f,0,0);drain();
    for (unsigned i=0;i<1000;i++) point_step(0,.01f,.01f);
    assert(status().phase==CAL_SETTLE && !status().accepted_turns);
    Motor_Calibration_StepPoint(0,0,0,0);
    assert(status().phase==CAL_FAILED && status().reason==CAL_SENSOR);reset();
}

int main(int argc, char **argv) {
    assert(sizeof(MotorCalSample)==64);
    assert(!Motor_Calibration_Begin(6));
    uint8_t mode; uint16_t count;
    const uint8_t legacy[]={5}, modern[]={5,0,1}, highres[]={5,0,2}, invalid[]={5,0,4}, wrong_mode[]={1,0,1};
    assert(Motor_Calibration_DecodeStartRequest(legacy,1,&mode,&count) && mode==5 && count==128);
    assert(Motor_Calibration_DecodeStartRequest(modern,3,&mode,&count) && count==256);
    assert(Motor_Calibration_DecodeStartRequest(highres,3,&mode,&count) && count==512);
    assert(!Motor_Calibration_DecodeStartRequest(modern,2,&mode,&count));
    assert(!Motor_Calibration_DecodeStartRequest(invalid,3,&mode,&count));
    assert(!Motor_Calibration_DecodeStartRequest(wrong_mode,3,&mode,&count));
    assert(!Motor_Calibration_DecodeStartRequest(legacy,0,&mode,&count));
    uint8_t repeats,wait;
    const uint8_t single[]={5,0,1,1,1}, bad_repeats[]={5,0,1,2,1}, bad_flags[]={5,0,1,1,2};
    assert(Motor_Calibration_DecodeStartOptions(single,5,&mode,&count,&repeats,&wait) && mode==5 && count==256 && repeats==1 && wait==1);
    assert(Motor_Calibration_DecodeStartOptions(legacy,1,&mode,&count,&repeats,&wait) && count==128 && repeats==3 && !wait);
    assert(Motor_Calibration_DecodeStartOptions(modern,3,&mode,&count,&repeats,&wait) && count==256 && repeats==3 && !wait);
    assert(!Motor_Calibration_DecodeStartOptions(single,4,&mode,&count,&repeats,&wait));
    assert(!Motor_Calibration_DecodeStartOptions(bad_repeats,5,&mode,&count,&repeats,&wait));
    assert(!Motor_Calibration_DecodeStartOptions(bad_flags,5,&mode,&count,&repeats,&wait));
    const uint8_t high_single[]={5,0,2,1,1};
    assert(Motor_Calibration_DecodeStartOptions(high_single,5,&mode,&count,&repeats,&wait) && count==512 && repeats==1 && wait==1);
    assert(!Motor_Calibration_BeginWithOptions(1,256,1,1));
    assert(!Motor_Calibration_BeginWithPoints(5,1024));
    run(0,0);run(1,0);run(0,1);run(1,1);
    assert(Motor_Calibration_Begin(5));
    assert(Motor_Calibration_PointCount()==512 && Motor_Calibration_PointSchema()==4);
    unsigned point_ticks=0;
    while (Motor_Calibration_IsActive() && point_ticks++<7000000) {
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
    assert(status().produced>=MOTOR_CAL_POINT_CAPTURE_MS*MOTOR_CAL_POINT_COUNT*6);
    assert(Motor_Calibration_Begin(5));
    assert(Motor_Calibration_PointAssist());
    Motor_Calibration_StepPoint(32768-32768/MOTOR_CAL_POINT_COUNT,0,0,1);
    assert(!Motor_Calibration_PointAssist() && status().phase==CAL_ZERO);
    for (unsigned i=0;i<MOTOR_CAL_POINT_SETTLE_MS-1;i++) Motor_Calibration_StepPoint(32768-32768/MOTOR_CAL_POINT_COUNT,0,0,1);
    assert(status().phase==CAL_ZERO);
    Motor_Calibration_StepPoint(32768-32768/MOTOR_CAL_POINT_COUNT,0,0,1);
    assert(Motor_Calibration_PointTarget()==0 && Motor_Calibration_PointAssist());
    Motor_Calibration_StepPoint(0,0,0,1);
    for (unsigned i=0;i<MOTOR_CAL_POINT_SETTLE_MS;i++) Motor_Calibration_StepPoint(0,0,0,1);
    assert(status().phase==CAL_CAPTURE);
    Motor_Calibration_StepPoint(0,0,CAL_BAD_SENSOR,1);
    assert(status().phase==CAL_SETTLE && status().accepted_turns==0);
    for (unsigned i=0;i<MOTOR_CAL_POINT_SETTLE_MS-1;i++) Motor_Calibration_StepPoint(0,0,0,1);
    assert(status().phase==CAL_SETTLE);
    point_step(0,.01f,.01f);assert(status().phase==CAL_CAPTURE);
    for (unsigned i=0;i<MOTOR_CAL_POINT_CAPTURE_MS-1;i++) point_step(0,.01f,.01f);
    assert(status().accepted_turns==0 && Motor_Calibration_PointTarget()==0);
    point_step(0,.01f,.01f);
    assert(status().accepted_turns==1 && Motor_Calibration_PointTarget()==32768/MOTOR_CAL_POINT_COUNT);
    reset();
    /* A complete but noisy current window retries the same target, then accepts recovery. */
    assert(Motor_Calibration_Begin(5));reach_first_point();
    for (unsigned i=0;i<MOTOR_CAL_POINT_CAPTURE_MS-1;i++)
        point_step(0,i&1?.03f:-.01f,i&1?.03f:-.01f);
    point_step(0,.01f,.01f);
    assert(status().phase==CAL_SETTLE && status().accepted_turns==0 && status().rejected_turns==1);
    assert(Motor_Calibration_PointTarget()==0);
    for (unsigned i=0;i<MOTOR_CAL_POINT_SETTLE_MS-1;i++) point_step(0,.01f,.01f);
    assert(status().phase==CAL_SETTLE);
    point_step(0,.01f,.01f);assert(status().phase==CAL_CAPTURE);
    for (unsigned i=0;i<MOTOR_CAL_POINT_CAPTURE_MS;i++) point_step(0,.01f,.01f);
    assert(status().accepted_turns==1 && Motor_Calibration_PointTarget()==32768/MOTOR_CAL_POINT_COUNT);
    reset();
    /* Recover on the same point after 16 seconds; retries retain the total deadline. */
    assert(Motor_Calibration_Begin(5));reach_first_point();
    for (unsigned i=0;i<14000;i++) point_step(0,i&1?.03f:-.01f,i&1?.03f:-.01f);
    assert(Motor_Calibration_IsActive() && status().accepted_turns==0 && Motor_Calibration_PointTarget()==0);
    for (unsigned i=0;i<4000 && !status().accepted_turns;i++) point_step(0,.01f,.01f);
    assert(status().accepted_turns==1 && Motor_Calibration_PointTarget()==32768/MOTOR_CAL_POINT_COUNT);
    reset();
    /* Persistently bad current must still hit the cumulative deadline, rather than reset it on every retry. */
    assert(Motor_Calibration_Begin(5));reach_first_point();
    for (unsigned i=0;i<MOTOR_CAL_POINT_TIMEOUT_MS+2000 && Motor_Calibration_IsActive();i++)
        point_step(0,i&1?.03f:-.01f,i&1?.03f:-.01f);
    assert(status().phase==CAL_FAILED && status().reason==CAL_UNSTABLE && !status().accepted_turns);
    reset();
    /* Tracking failure, ADC angle outlier and absent ADC samples must not advance. */
    for (unsigned kind=0;kind<3;kind++) {
        assert(Motor_Calibration_Begin(5));reach_first_point();
        for (unsigned i=0;i<MOTOR_CAL_POINT_CAPTURE_MS-1;i++) {
            if (kind==2) Motor_Calibration_StepPoint(0,0,0,1);
            else if (kind==1) {
                Motor_Calibration_StepPoint(0,0,0,1);
                MotorCalSample rec={0};rec.flags=Motor_Calibration_SampleFlags();
                Motor_Calibration_PublishControl(&rec);Motor_Calibration_Capture(4,.01f,.01f,0,0);drain();
            } else point_step(0,.01f,.02f);
        }
        point_step(0,.01f,.01f);
        assert(status().phase==CAL_SETTLE && status().accepted_turns==0 && Motor_Calibration_PointTarget()==0);
        reset();
    }
    /* Boundary and wrap-around: 3 counts may settle; 4 must restart capture. */
    assert(Motor_Calibration_Begin(5));
    uint16_t guide=(uint16_t)Motor_Calibration_PointTarget();
    Motor_Calibration_StepPoint(guide+4,0,0,1);
    assert(Motor_Calibration_PointAssist());
    Motor_Calibration_StepPoint(guide+3,0,0,1);
    assert(!Motor_Calibration_PointAssist());
    for (unsigned i=0;i<MOTOR_CAL_POINT_SETTLE_MS;i++) Motor_Calibration_StepPoint(guide+3,0,0,1);
    assert(Motor_Calibration_PointTarget()==0);
    Motor_Calibration_StepPoint(32765,0,0,1); /* -3 across the single-turn boundary */
    assert(!Motor_Calibration_PointAssist());
    for (unsigned i=0;i<MOTOR_CAL_POINT_SETTLE_MS;i++) Motor_Calibration_StepPoint(32765,0,0,1);
    assert(status().phase==CAL_CAPTURE);
    Motor_Calibration_StepPoint(32764,0,0,1); /* -4 */
    assert(status().phase==CAL_SETTLE && status().accepted_turns==0);
    reset();
    assert(Motor_Calibration_Begin(5));
    for (unsigned i=0;i<10000;i++) Motor_Calibration_StepPoint(0,10,0,1);
    assert(Motor_Calibration_IsActive()); /* old deadline no longer aborts this point */
    for (unsigned i=10000;i<MOTOR_CAL_POINT_TIMEOUT_MS-1;i++) Motor_Calibration_StepPoint(0,10,0,1);
    assert(Motor_Calibration_IsActive());
    Motor_Calibration_StepPoint(0,10,0,1);
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
    assert(Motor_Calibration_BeginWithPoints(5,128));
    assert(Motor_Calibration_PointCount()==128 && Motor_Calibration_PointSchema()==2);
    unsigned legacy_ticks=0;
    while (Motor_Calibration_IsActive() && legacy_ticks++<2000000)
        point_step((uint16_t)Motor_Calibration_PointTarget(),.01f,.01f);
    assert(status().phase==CAL_DONE && status().accepted_turns==128 && status().passed_mask==7);
    reset();
    /* Negotiated single pair visits all 256 points in both directions, then finishes. */
    assert(Motor_Calibration_BeginWithOptions(5,256,1,1));
    assert(Motor_Calibration_PointRepeats()==1 && Motor_Calibration_PointRetryUntilGood());
    unsigned single_ticks=0;
    while (Motor_Calibration_IsActive() && single_ticks++<1200000)
        point_step((uint16_t)Motor_Calibration_PointTarget(),.01f,.01f);
    assert(status().phase==CAL_DONE && status().passed_mask==1 && status().speed_index==0);
    assert(status().produced==MOTOR_CAL_POINT_CAPTURE_MS*256U*2U);
    reset();
    assert(Motor_Calibration_BeginWithOptions(5,512,1,1));
    assert(Motor_Calibration_PointSchema()==4 && Motor_Calibration_PointCount()==512);
    single_ticks=0;
    while (Motor_Calibration_IsActive() && single_ticks++<2200000)
        point_step((uint16_t)Motor_Calibration_PointTarget(),.01f,.01f);
    assert(status().phase==CAL_DONE && status().passed_mask==1 && status().accepted_turns==512);
    assert(status().produced==MOTOR_CAL_POINT_CAPTURE_MS*512U*2U);
    reset();
    /* Bad capture may wait past 30 seconds on the same target and recover later. */
    assert(Motor_Calibration_BeginWithOptions(5,256,1,1));reach_first_point();
    for (unsigned i=0;i<35000;i++) point_step(0,i&1?.03f:-.01f,i&1?.03f:-.01f);
    assert(Motor_Calibration_IsActive() && !status().accepted_turns && Motor_Calibration_PointTarget()==0);
    for (unsigned i=0;i<4000 && !status().accepted_turns;i++) point_step(0,.01f,.01f);
    assert(status().accepted_turns==1 && Motor_Calibration_PointTarget()==128);
    reset();assert(status().phase==CAL_ABORTED);
    /* Waiting outside position/speed tolerance does not time out, but sensor fault still stops. */
    assert(Motor_Calibration_BeginWithOptions(5,256,1,1));
    for (unsigned i=0;i<35000;i++) Motor_Calibration_StepPoint(0,10,0,1);
    assert(Motor_Calibration_IsActive());
    Motor_Calibration_StepPoint(0,0,0,0);
    assert(status().phase==CAL_FAILED && status().reason==CAL_SENSOR);
    reset();
    test_average_policy();
    puts("PASS: legacy calibration, additive policy, raised variance, two rejects then fresh 1s average, position/current/sensor protections");
    return 0;
}
