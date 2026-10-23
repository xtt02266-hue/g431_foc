#include "motor_calibration.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#ifdef NDEBUG
#error "Calibration simulation requires assertions; compile with -UNDEBUG"
#endif

static void drain(void) {
    motor_cal_sample_t records[3]; uint8_t n;
    while ((n=motor_calibration_peek(records,3))) motor_calibration_consume(n);
}
static motor_cal_status_t status(void) {
    motor_cal_status_t s; motor_calibration_get_status(&s); return s;
}
static void reset(void) { motor_calibration_abort(CAL_CANCELLED); drain(); }
static void point_step(uint16_t angle,float reference,float actual) {
    static uint32_t tick;
    motor_calibration_step_point(angle,0,0,1);
    if (status().phase==CAL_CAPTURE) {
        motor_cal_sample_t rec={0};rec.tick_ms=++tick;rec.flags=motor_calibration_sample_flags();
        motor_calibration_publish_control(&rec);
        motor_calibration_capture(angle,reference,actual,0,0);
        drain();
    }
}
static void reach_first_point(void) {
    for (unsigned i=0;i<2*(MOTOR_CAL_POINT_SETTLE_MS+1);i++)
        point_step((uint16_t)motor_calibration_point_target(),.01f,.01f);
    assert(status().phase==CAL_CAPTURE && motor_calibration_point_target()==0);
}

static void run(int stall_first, int friction) {
    assert(motor_calibration_begin(friction ? 4 : 1));
    double position=0; unsigned ticks=0;
    while (motor_calibration_is_active() && ticks++<300000) {
        motor_cal_status_t s=status();
        float speed=s.target_rpm;
        if (friction) speed *= 1.0f + .10f*sinf(ticks*.0062831853f);
        if (stall_first && s.speed_index==0) speed=0;
        position+=speed*32768.0/60000.0;
        uint16_t angle=(uint16_t)((int64_t)position & 32767);
        motor_calibration_step(angle,speed,s.target_rpm,0,1);
        motor_cal_sample_t sample={0}; sample.tick_ms=ticks;
        sample.flags=motor_calibration_sample_flags();
        motor_calibration_publish_control(&sample);
        motor_calibration_capture(angle,.01f,.009f,0,.02f);
        /* Several ADC cycles in a control period must not duplicate a record. */
        motor_calibration_capture(angle,.01f,.009f,0,.02f);
        if (ticks%3==0) drain();
    }
    motor_cal_status_t s=status();
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
    assert(motor_calibration_decode_start_policy(request,6,&mode,&count,&repeats,&wait,&policy));
    assert(mode==5 && count==512 && repeats==1 && wait==1 && policy==1);
    assert(motor_calibration_decode_start_policy(request,5,&mode,&count,&repeats,&wait,&policy) && policy==0);
    uint8_t bad[]={5,0,2,1,1,2};
    assert(!motor_calibration_decode_start_policy(bad,6,&mode,&count,&repeats,&wait,&policy));
    assert(!motor_calibration_decode_start_policy(request,4,&mode,&count,&repeats,&wait,&policy));
    assert(!motor_calibration_begin_with_policy(1,256,3,0,1));
    assert(motor_calibration_begin_with_policy(5,512,1,1,1));reach_first_point();
    /* New 6mA floor admits a 5mA standard deviation; old contract stays unchanged. */
    for (unsigned i=0;i<500;i++) point_step(0,i&1?.015f:.005f,i&1?.015f:.005f);
    assert(status().accepted_turns==1);reset();
    assert(motor_calibration_begin_with_policy(5,512,1,1,1));reach_first_point();
    /* Two noisy windows are discarded. Only a fresh 1000ms window may waive variance. */
    for (unsigned attempt=0;attempt<2;attempt++) {
        assert(!(motor_calibration_sample_flags()&CAL_POINT_AVERAGE));
        for (unsigned i=0;i<500;i++) point_step(0,i&1?.03f:-.01f,i&1?.03f:-.01f);
        assert(status().phase==CAL_SETTLE && !status().accepted_turns);
        for (unsigned i=0;i<1000;i++) point_step(0,.01f,.01f);
    }
    assert(motor_calibration_sample_flags()&CAL_POINT_AVERAGE);
    for (unsigned i=0;i<999;i++) point_step(0,i&1?.03f:-.01f,i&1?.03f:-.01f);
    assert(!status().accepted_turns && status().phase==CAL_CAPTURE);
    point_step(0,.01f,.01f);
    assert(status().accepted_turns==1 && motor_calibration_point_target()==64);
    assert(!(motor_calibration_sample_flags()&CAL_POINT_AVERAGE));reset();
    /* Mean current tracking failure is never waived, even after entering fallback. */
    assert(motor_calibration_begin_with_policy(5,512,1,1,1));reach_first_point();
    for (unsigned i=0;i<8000;i++) point_step(0,.01f,.03f);
    assert(!status().accepted_turns && motor_calibration_point_target()==0);
    for (unsigned i=0;i<2000 && status().phase!=CAL_CAPTURE;i++) point_step(0,.01f,.01f);
    assert(status().phase==CAL_CAPTURE);
    assert(motor_calibration_sample_flags()&CAL_POINT_AVERAGE);
    /* Policy 1 checks mean absolute position error: one excursion may qualify,
     * but a complete window outside the configured tolerance must be rejected. */
    uint16_t outside=(uint16_t)MOTOR_CAL_POINT_MAX_ERROR_COUNTS+1U;
    point_step(outside,.01f,.01f);
    assert(status().phase==CAL_CAPTURE && !status().accepted_turns);
    for (unsigned i=1;i<MOTOR_CAL_POINT_AVERAGE_MS;i++) point_step(0,.01f,.01f);
    assert(status().accepted_turns==1);reset();
    assert(motor_calibration_begin_with_policy(5,512,1,1,1));reach_first_point();
    for (unsigned attempt=0;attempt<2;attempt++) {
        for (unsigned i=0;i<MOTOR_CAL_POINT_CAPTURE_MS;i++) point_step(0,i&1?.03f:-.01f,i&1?.03f:-.01f);
        assert(status().phase==CAL_SETTLE && !status().accepted_turns);
        for (unsigned i=0;i<MOTOR_CAL_POINT_SETTLE_MS;i++) point_step(0,.01f,.01f);
    }
    assert(motor_calibration_sample_flags()&CAL_POINT_AVERAGE);
    for (unsigned i=0;i<MOTOR_CAL_POINT_AVERAGE_MS;i++) point_step(outside,.01f,.01f);
    assert(status().phase==CAL_SETTLE && !status().accepted_turns);
    motor_calibration_step_point(0,0,0,0);
    assert(status().phase==CAL_FAILED && status().reason==CAL_SENSOR);reset();
}

int main(int argc, char **argv) {
    assert(sizeof(motor_cal_sample_t)==64);
    assert(!motor_calibration_begin(6));
    uint8_t mode; uint16_t count;
    const uint8_t legacy[]={5}, modern[]={5,0,1}, highres[]={5,0,2}, invalid[]={5,0,4}, wrong_mode[]={1,0,1};
    assert(motor_calibration_decode_start_request(legacy,1,&mode,&count) && mode==5 && count==128);
    assert(motor_calibration_decode_start_request(modern,3,&mode,&count) && count==256);
    assert(motor_calibration_decode_start_request(highres,3,&mode,&count) && count==512);
    assert(!motor_calibration_decode_start_request(modern,2,&mode,&count));
    assert(!motor_calibration_decode_start_request(invalid,3,&mode,&count));
    assert(!motor_calibration_decode_start_request(wrong_mode,3,&mode,&count));
    assert(!motor_calibration_decode_start_request(legacy,0,&mode,&count));
    uint8_t repeats,wait;
    const uint8_t single[]={5,0,1,1,1}, bad_repeats[]={5,0,1,2,1}, bad_flags[]={5,0,1,1,2};
    assert(motor_calibration_decode_start_options(single,5,&mode,&count,&repeats,&wait) && mode==5 && count==256 && repeats==1 && wait==1);
    assert(motor_calibration_decode_start_options(legacy,1,&mode,&count,&repeats,&wait) && count==128 && repeats==3 && !wait);
    assert(motor_calibration_decode_start_options(modern,3,&mode,&count,&repeats,&wait) && count==256 && repeats==3 && !wait);
    assert(!motor_calibration_decode_start_options(single,4,&mode,&count,&repeats,&wait));
    assert(!motor_calibration_decode_start_options(bad_repeats,5,&mode,&count,&repeats,&wait));
    assert(!motor_calibration_decode_start_options(bad_flags,5,&mode,&count,&repeats,&wait));
    const uint8_t high_single[]={5,0,2,1,1};
    assert(motor_calibration_decode_start_options(high_single,5,&mode,&count,&repeats,&wait) && count==512 && repeats==1 && wait==1);
    assert(!motor_calibration_begin_with_options(1,256,1,1));
    assert(!motor_calibration_begin_with_points(5,1024));
    run(0,0);run(1,0);run(0,1);run(1,1);
    assert(motor_calibration_begin(5));
    assert(motor_calibration_point_count()==512 && motor_calibration_point_schema()==4);
    unsigned point_ticks=0;
    while (motor_calibration_is_active() && point_ticks++<7000000) {
        float target=motor_calibration_point_target();
        motor_calibration_step_point((uint16_t)target,0,0,1);
        motor_cal_status_t ps=status();
        if (ps.phase==CAL_CAPTURE) {
            assert(!motor_calibration_point_assist());
            motor_cal_sample_t rec={0};rec.tick_ms=point_ticks;rec.flags=motor_calibration_sample_flags();
            motor_calibration_publish_control(&rec);motor_calibration_capture((uint16_t)target,.01f,.01f,0,0);
            float saved;motor_calibration_peek_targets(&saved,1);assert(saved==target);drain();
        }
    }
    assert(status().phase==CAL_DONE && status().passed_mask==7 && status().dropped==0);
    assert(status().produced>=MOTOR_CAL_POINT_CAPTURE_MS*MOTOR_CAL_POINT_COUNT*6);
    assert(motor_calibration_begin(5));
    assert(motor_calibration_point_assist());
    motor_calibration_step_point(32768-32768/MOTOR_CAL_POINT_COUNT,0,0,1);
    assert(!motor_calibration_point_assist() && status().phase==CAL_ZERO);
    for (unsigned i=0;i<MOTOR_CAL_POINT_SETTLE_MS-1;i++) motor_calibration_step_point(32768-32768/MOTOR_CAL_POINT_COUNT,0,0,1);
    assert(status().phase==CAL_ZERO);
    motor_calibration_step_point(32768-32768/MOTOR_CAL_POINT_COUNT,0,0,1);
    assert(motor_calibration_point_target()==0 && motor_calibration_point_assist());
    motor_calibration_step_point(0,0,0,1);
    for (unsigned i=0;i<MOTOR_CAL_POINT_SETTLE_MS;i++) motor_calibration_step_point(0,0,0,1);
    assert(status().phase==CAL_CAPTURE);
    motor_calibration_step_point(0,0,CAL_BAD_SENSOR,1);
    assert(status().phase==CAL_SETTLE && status().accepted_turns==0);
    for (unsigned i=0;i<MOTOR_CAL_POINT_SETTLE_MS-1;i++) motor_calibration_step_point(0,0,0,1);
    assert(status().phase==CAL_SETTLE);
    point_step(0,.01f,.01f);assert(status().phase==CAL_CAPTURE);
    for (unsigned i=0;i<MOTOR_CAL_POINT_CAPTURE_MS-1;i++) point_step(0,.01f,.01f);
    assert(status().accepted_turns==0 && motor_calibration_point_target()==0);
    point_step(0,.01f,.01f);
    assert(status().accepted_turns==1 && motor_calibration_point_target()==32768/MOTOR_CAL_POINT_COUNT);
    reset();
    /* A complete but noisy current window retries the same target, then accepts recovery. */
    assert(motor_calibration_begin(5));reach_first_point();
    for (unsigned i=0;i<MOTOR_CAL_POINT_CAPTURE_MS-1;i++)
        point_step(0,i&1?.03f:-.01f,i&1?.03f:-.01f);
    point_step(0,.01f,.01f);
    assert(status().phase==CAL_SETTLE && status().accepted_turns==0 && status().rejected_turns==1);
    assert(motor_calibration_point_target()==0);
    for (unsigned i=0;i<MOTOR_CAL_POINT_SETTLE_MS-1;i++) point_step(0,.01f,.01f);
    assert(status().phase==CAL_SETTLE);
    point_step(0,.01f,.01f);assert(status().phase==CAL_CAPTURE);
    for (unsigned i=0;i<MOTOR_CAL_POINT_CAPTURE_MS;i++) point_step(0,.01f,.01f);
    assert(status().accepted_turns==1 && motor_calibration_point_target()==32768/MOTOR_CAL_POINT_COUNT);
    reset();
    /* Recover on the same point after 16 seconds; retries retain the total deadline. */
    assert(motor_calibration_begin(5));reach_first_point();
    for (unsigned i=0;i<14000;i++) point_step(0,i&1?.03f:-.01f,i&1?.03f:-.01f);
    assert(motor_calibration_is_active() && status().accepted_turns==0 && motor_calibration_point_target()==0);
    for (unsigned i=0;i<4000 && !status().accepted_turns;i++) point_step(0,.01f,.01f);
    assert(status().accepted_turns==1 && motor_calibration_point_target()==32768/MOTOR_CAL_POINT_COUNT);
    reset();
    /* Persistently bad current must still hit the cumulative deadline, rather than reset it on every retry. */
    assert(motor_calibration_begin(5));reach_first_point();
    for (unsigned i=0;i<MOTOR_CAL_POINT_TIMEOUT_MS+2000 && motor_calibration_is_active();i++)
        point_step(0,i&1?.03f:-.01f,i&1?.03f:-.01f);
    assert(status().phase==CAL_FAILED && status().reason==CAL_UNSTABLE && !status().accepted_turns);
    reset();
    /* Tracking failure, ADC angle outlier and absent ADC samples must not advance. */
    for (unsigned kind=0;kind<3;kind++) {
        assert(motor_calibration_begin(5));reach_first_point();
        for (unsigned i=0;i<MOTOR_CAL_POINT_CAPTURE_MS-1;i++) {
            if (kind==2) motor_calibration_step_point(0,0,0,1);
            else if (kind==1) {
                motor_calibration_step_point(0,0,0,1);
                motor_cal_sample_t rec={0};rec.flags=motor_calibration_sample_flags();
                motor_calibration_publish_control(&rec);motor_calibration_capture(4,.01f,.01f,0,0);drain();
            } else point_step(0,.01f,.02f);
        }
        point_step(0,.01f,.01f);
        assert(status().phase==CAL_SETTLE && status().accepted_turns==0 && motor_calibration_point_target()==0);
        reset();
    }
    /* Test the configured boundary and one count outside it, including wrap-around. */
    assert(motor_calibration_begin(5));
    uint16_t guide=(uint16_t)motor_calibration_point_target();
    uint16_t allowed=(uint16_t)MOTOR_CAL_POINT_MAX_ERROR_COUNTS;
    motor_calibration_step_point(guide+allowed+1U,0,0,1);
    assert(motor_calibration_point_assist());
    motor_calibration_step_point(guide+allowed,0,0,1);
    assert(!motor_calibration_point_assist());
    for (unsigned i=0;i<MOTOR_CAL_POINT_SETTLE_MS;i++) motor_calibration_step_point(guide+allowed,0,0,1);
    assert(motor_calibration_point_target()==0);
    motor_calibration_step_point(32768U-allowed,0,0,1);
    assert(!motor_calibration_point_assist());
    for (unsigned i=0;i<MOTOR_CAL_POINT_SETTLE_MS;i++) motor_calibration_step_point(32768U-allowed,0,0,1);
    assert(status().phase==CAL_CAPTURE);
    motor_calibration_step_point(32768U-allowed-1U,0,0,1);
    assert(status().phase==CAL_SETTLE && status().accepted_turns==0);
    reset();
    assert(motor_calibration_begin(5));
    for (unsigned i=0;i<10000;i++) motor_calibration_step_point(0,10,0,1);
    assert(motor_calibration_is_active()); /* old deadline no longer aborts this point */
    for (unsigned i=10000;i<MOTOR_CAL_POINT_TIMEOUT_MS-1;i++) motor_calibration_step_point(0,10,0,1);
    assert(motor_calibration_is_active());
    motor_calibration_step_point(0,10,0,1);
    assert(status().phase==CAL_FAILED && status().reason==CAL_UNSTABLE);
    assert(motor_calibration_begin(1));
    motor_calibration_step(0,0,60,0,0);
    assert(status().reason==CAL_SENSOR && !motor_calibration_is_active());
    reset();assert(motor_calibration_begin(1));
    for (unsigned i=0;i<20000;i++) motor_calibration_step((i*20)&32767,30,60,0,1);
    assert(status().phase==CAL_ZERO && status().speed_reasons[0]==CAL_UNSTABLE);
    for (unsigned i=0;i<10001;i++) motor_calibration_step(0,30,0,0,1);
    assert(status().reason==CAL_TIMEOUT);
    reset();assert(motor_calibration_begin(1));
    motor_cal_sample_t sample={0};
    for (unsigned i=0;i<MOTOR_CAL_RING_SIZE+1;i++) motor_calibration_push(&sample);
    assert(status().dropped==1);
    motor_calibration_step(0,60,60,0,1);
    assert(status().reason==CAL_OVERFLOW);
    assert(!motor_calibration_begin(1)); /* pending old records must drain first */
    reset();assert(motor_calibration_begin(1));
    motor_calibration_abort(CAL_CANCELLED);
    assert(status().phase==CAL_ABORTED && status().target_rpm==0);
    reset();assert(motor_calibration_begin(1));
    sample.tick_ms=123;sample.flags=2<<8;sample.target_rpm=60;sample.speed_rpm=59;
    sample.window_rpm=58;sample.pi_iq=.001f;sample.friction_iq=.002f;sample.cogging_iq=.003f;
    sample.window_ticks=20;sample.sensor_age_ms=1;sample.crc_errors=4;sample.transfer_errors=5;
    motor_calibration_publish_control(&sample);
    motor_calibration_capture(12345,.01f,.009f,.008f,.007f);
    motor_cal_sample_t received;
    assert(motor_calibration_peek(&received,1)==1);
    assert(received.sequence==1 && received.angle==12345 && received.tick_ms==123);
    if (argc>1) { FILE *f=fopen(argv[1],"wb");assert(f);assert(fwrite(&received,64,1,f)==1);fclose(f); }
    drain();
    float targets[3];motor_cal_sample_t replay[3];
    assert(motor_calibration_replay(1,1,replay,targets)==1 && replay[0].angle==12345);
    assert(!motor_calibration_replay(0,1,replay,targets));
    assert(!motor_calibration_replay(1,4,replay,targets));
    assert(!motor_calibration_replay(2,1,replay,targets));
    for (unsigned i=0;i<MOTOR_CAL_RING_SIZE;i++) { motor_calibration_push(&sample);drain(); }
    assert(!motor_calibration_replay(1,1,replay,targets)); /* overwritten, never fabricate */
    assert(motor_calibration_replay(2,3,replay,targets)==3 && replay[2].sequence==4);
    assert(motor_calibration_replay(MOTOR_CAL_RING_SIZE+1,1,replay,targets)==1);
    reset();
    assert(motor_calibration_begin_with_points(5,128));
    assert(motor_calibration_point_count()==128 && motor_calibration_point_schema()==2);
    unsigned legacy_ticks=0;
    while (motor_calibration_is_active() && legacy_ticks++<2000000)
        point_step((uint16_t)motor_calibration_point_target(),.01f,.01f);
    assert(status().phase==CAL_DONE && status().accepted_turns==128 && status().passed_mask==7);
    reset();
    /* Negotiated single pair visits all 256 points in both directions, then finishes. */
    assert(motor_calibration_begin_with_options(5,256,1,1));
    assert(motor_calibration_point_repeats()==1 && motor_calibration_point_retry_until_good());
    unsigned single_ticks=0;
    while (motor_calibration_is_active() && single_ticks++<1200000)
        point_step((uint16_t)motor_calibration_point_target(),.01f,.01f);
    assert(status().phase==CAL_DONE && status().passed_mask==1 && status().speed_index==0);
    assert(status().produced==MOTOR_CAL_POINT_CAPTURE_MS*256U*2U);
    reset();
    assert(motor_calibration_begin_with_options(5,512,1,1));
    assert(motor_calibration_point_schema()==4 && motor_calibration_point_count()==512);
    single_ticks=0;
    while (motor_calibration_is_active() && single_ticks++<2200000)
        point_step((uint16_t)motor_calibration_point_target(),.01f,.01f);
    assert(status().phase==CAL_DONE && status().passed_mask==1 && status().accepted_turns==512);
    assert(status().produced==MOTOR_CAL_POINT_CAPTURE_MS*512U*2U);
    reset();
    /* Bad capture may wait past 30 seconds on the same target and recover later. */
    assert(motor_calibration_begin_with_options(5,256,1,1));reach_first_point();
    for (unsigned i=0;i<35000;i++) point_step(0,i&1?.03f:-.01f,i&1?.03f:-.01f);
    assert(motor_calibration_is_active() && !status().accepted_turns && motor_calibration_point_target()==0);
    for (unsigned i=0;i<4000 && !status().accepted_turns;i++) point_step(0,.01f,.01f);
    assert(status().accepted_turns==1 && motor_calibration_point_target()==128);
    reset();assert(status().phase==CAL_ABORTED);
    /* Waiting outside position/speed tolerance does not time out, but sensor fault still stops. */
    assert(motor_calibration_begin_with_options(5,256,1,1));
    for (unsigned i=0;i<35000;i++) motor_calibration_step_point(0,10,0,1);
    assert(motor_calibration_is_active());
    motor_calibration_step_point(0,0,0,0);
    assert(status().phase==CAL_FAILED && status().reason==CAL_SENSOR);
    reset();
    test_average_policy();
    puts("PASS: legacy calibration, additive policy, raised variance, two rejects then fresh 1s average, position/current/sensor protections");
    return 0;
}
