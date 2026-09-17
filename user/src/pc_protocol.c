#include "pc_protocol.h"
#include "motor_system.h"
#include "motor_parameters.h"
#include "motor_current_loop.h"
#include "motor_debug.h"
#include "motor_encoder.h"
#include "mt6826s.h"
#include "usart.h"
#include <math.h>
#include <string.h>

#define SOF0 0xA5U
#define SOF1 0x5AU
#define TYPE_COMMAND 1U
#define TYPE_RESPONSE 2U
#define TYPE_STATUS 3U
#define TYPE_TELEMETRY 4U
#define RX_RING_SIZE 2048U
#define MAX_FRAME_SIZE (10U + PC_PROTOCOL_MAX_PAYLOAD)

enum { CMD_HELLO=0x01, CMD_GET_STATUS=0x02, CMD_CLAIM=0x03,
       CMD_RELEASE=0x04, CMD_HEARTBEAT=0x05, CMD_START=0x10,
       CMD_STOP=0x11, CMD_CLEAR_FAULT=0x12, CMD_IDENTIFY=0x13,
       CMD_SET_MODE=0x20,
       CMD_SET_IQ=0x21, CMD_SET_IQ_LIMIT=0x22,
       CMD_SET_HAPTIC=0x23, CMD_GET_PARAMS=0x24,
       CMD_SET_FRICTION=0x25, CMD_GET_FRICTION=0x26,
       CMD_SET_SPEED=0x27, CMD_SET_COGGING_CONFIG=0x28,
       CMD_GET_COGGING_CONFIG=0x29, CMD_COGGING_TABLE_BEGIN=0x2A,
       CMD_COGGING_TABLE_CHUNK=0x2B, CMD_COGGING_TABLE_COMMIT=0x2C,
       CMD_GET_COGGING_TABLE_CHUNK=0x2D, CMD_SAVE_COGGING=0x2E };

static volatile uint8_t s_ring[RX_RING_SIZE];
static volatile uint16_t s_head, s_tail;
static uint8_t s_parse[MAX_FRAME_SIZE];
static uint16_t s_parse_len;
static uint8_t s_tx[MAX_FRAME_SIZE];
static uint8_t s_tx_pending[MAX_FRAME_SIZE];
static uint16_t s_tx_pending_len;
static uint8_t s_tx_busy;
static uint32_t s_token;
static uint32_t s_token_counter;
static uint32_t s_crc_errors, s_overflows, s_telemetry_drops;
static uint16_t s_telemetry_seq, s_status_seq;
static uint32_t s_sample_seq, s_last_status_ms;
static uint8_t s_last_request[PC_PROTOCOL_MAX_PAYLOAD];
static uint16_t s_last_request_len, s_last_request_seq;
static uint8_t s_last_response[PC_PROTOCOL_MAX_PAYLOAD];
static uint16_t s_last_response_len;

static uint16_t crc16(const uint8_t *p, uint16_t n)
{
    uint16_t crc = 0xFFFFU;
    while (n-- != 0U) {
        crc ^= (uint16_t)(*p++) << 8U;
        for (uint8_t bit=0U; bit<8U; ++bit) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1U) ^ 0x1021U)
                                  : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

/* HELLO and STOP are safe to execute repeatedly.  Keeping them out of the
 * replay cache also lets a newly connected host restart its sequence counter
 * without colliding with the previous connection. */
static uint8_t command_uses_replay_cache(uint8_t cmd)
{
    return (cmd != CMD_HEARTBEAT) && (cmd != CMD_HELLO) && (cmd != CMD_STOP);
}

static void put_u16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8U); }
static void put_u32(uint8_t *p, uint32_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8U); p[2]=(uint8_t)(v>>16U); p[3]=(uint8_t)(v>>24U); }
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1]<<8U); }
static int16_t get_i16(const uint8_t *p) { return (int16_t)get_u16(p); }
static uint32_t get_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1]<<8U) | ((uint32_t)p[2]<<16U) | ((uint32_t)p[3]<<24U); }
static float get_f32(const uint8_t *p) { float v; memcpy(&v,p,4U); return v; }
static void put_f32(uint8_t *p, float v) { memcpy(p,&v,4U); }

static uint16_t make_frame(uint8_t *dst, uint8_t type, uint16_t seq,
                           const uint8_t *payload, uint16_t length)
{
    dst[0]=SOF0; dst[1]=SOF1; dst[2]=PC_PROTOCOL_VERSION; dst[3]=type;
    put_u16(&dst[4],seq); put_u16(&dst[6],length);
    if (length != 0U) memcpy(&dst[8],payload,length);
    put_u16(&dst[8U+length],crc16(&dst[2],(uint16_t)(6U+length)));
    return (uint16_t)(10U+length);
}

static uint8_t queue_frame(uint8_t type, uint16_t seq,
                           const uint8_t *payload, uint16_t length,
                           uint8_t priority)
{
    uint8_t frame[MAX_FRAME_SIZE];
    uint16_t n = make_frame(frame,type,seq,payload,length);
    if ((s_tx_busy == 0U) && (huart2.gState == HAL_UART_STATE_READY)) {
        memcpy(s_tx,frame,n); s_tx_busy=1U;
        if (HAL_UART_Transmit_DMA(&huart2,s_tx,n) == HAL_OK) return 1U;
        s_tx_busy=0U;
    }
    if ((priority != 0U) && (s_tx_pending_len == 0U)) {
        memcpy(s_tx_pending,frame,n); s_tx_pending_len=n; return 1U;
    }
    return 0U;
}

static uint16_t build_status(uint8_t *p)
{
    MotorControlSnapshot s;
    Motor_System_GetControlSnapshot(&s);
    put_u32(&p[0],HAL_GetTick()); p[4]=(uint8_t)s.state; p[5]=(uint8_t)s.mode;
    p[6]=(uint8_t)s.source; p[7]=(uint8_t)s.owner; p[8]=s.run_requested;
    p[9]=(uint8_t)Motor_Parameters_GetStatus();
    p[10]=Motor_Encoder_IsDataFresh(2U); p[11]=MT6826S_GetStatus();
    uint32_t faults = 0U;
    if (p[10] == 0U) faults |= 1U;
    if (p[9] == (uint8_t)MOTOR_PARAMETERS_ERROR) faults |= 2U;
    put_u32(&p[12],faults); p[16]=0U; p[17]=p[18]=p[19]=0U;
    put_u32(&p[20],s.config_revision); put_f32(&p[24],s.iq_limit_a);
    put_u32(&p[28],s_crc_errors); put_u32(&p[32],s_overflows);
    put_u32(&p[36],s_telemetry_drops); return 40U;
}

static void response(uint16_t seq, uint8_t cmd, MotorCommandResult result,
                     const uint8_t *body, uint16_t body_len)
{
    uint8_t p[PC_PROTOCOL_MAX_PAYLOAD]; MotorControlSnapshot s;
    Motor_System_GetControlSnapshot(&s);
    p[0]=cmd; p[1]=(uint8_t)result; put_u32(&p[2],s_token);
    put_u32(&p[6],s.config_revision);
    if (body_len != 0U) memcpy(&p[10],body,body_len);
    uint16_t response_len=(uint16_t)(10U+body_len);
    if (command_uses_replay_cache(cmd)) {
        s_last_response_len=response_len;
        memcpy(s_last_response,p,response_len);
    }
    (void)queue_frame(TYPE_RESPONSE,seq,p,response_len,1U);
}

static uint8_t token_ok(uint32_t token)
{
    return (s_token != 0U) && (token == s_token) &&
           (Motor_System_GetControlOwner() == MOTOR_OWNER_HOST);
}

static void handle_command(uint16_t seq, const uint8_t *p, uint16_t n)
{
    if (n < 5U) return;
    uint8_t cmd=p[0]; uint32_t token=get_u32(&p[1]); const uint8_t *b=&p[5];
    uint16_t bn=(uint16_t)(n-5U); MotorCommandResult r=MOTOR_CMD_OK;
    uint8_t out[PC_PROTOCOL_MAX_PAYLOAD-10U]; uint16_t out_n=0U;
    if (command_uses_replay_cache(cmd) && (seq == s_last_request_seq) &&
        (n == s_last_request_len)) {
        if (memcmp(p,s_last_request,n)==0) {
            (void)queue_frame(TYPE_RESPONSE,seq,s_last_response,s_last_response_len,1U);
        } else {
            MotorControlSnapshot cs; uint8_t conflict[10];
            Motor_System_GetControlSnapshot(&cs);
            conflict[0]=cmd; conflict[1]=11U; put_u32(&conflict[2],s_token);
            put_u32(&conflict[6],cs.config_revision);
            (void)queue_frame(TYPE_RESPONSE,seq,conflict,sizeof(conflict),1U);
        }
        return;
    }
    if (cmd == CMD_HELLO) {
        if (bn != 0U) r=MOTOR_CMD_INVALID_LENGTH;
        else {
            out[0]=1U; out[1]=4U; out[2]=0U; put_u16(&out[3],4U);
            put_u32(&out[5],HAL_GetUIDw0()); put_u32(&out[9],HAL_GetUIDw1()); put_u32(&out[13],HAL_GetUIDw2());
            put_u32(&out[17],0xFFU); put_u32(&out[21],0x7FU);
            put_f32(&out[25],MOTOR_TORQUE_CURRENT_LIMIT_A); put_f32(&out[29],MOTOR_HOST_DEFAULT_IQ_LIMIT_A);
            put_u16(&out[33],PC_PROTOCOL_MAX_PAYLOAD); put_u16(&out[35],MOTOR_HOST_HEARTBEAT_TIMEOUT_MS); out_n=37U;
        }
    } else if (cmd == CMD_GET_STATUS) {
        if (bn != 0U) r=MOTOR_CMD_INVALID_LENGTH; else out_n=build_status(out);
    } else if (cmd == CMD_CLAIM) {
        if (bn != 0U) r=MOTOR_CMD_INVALID_LENGTH;
        else { r=Motor_System_ClaimHost(); if (r==MOTOR_CMD_OK) { ++s_token_counter; if (s_token_counter==0U) ++s_token_counter; s_token=s_token_counter; Motor_System_HostHeartbeat(); put_u32(out,s_token); out_n=4U; } }
    } else if (cmd == CMD_STOP) {
        Motor_System_StopControl(); Motor_System_ReleaseHost(); s_token=0U;
    } else if (cmd == CMD_CLEAR_FAULT) {
        r=Motor_System_ClearFault() ? MOTOR_CMD_OK : MOTOR_CMD_NOT_READY;
    } else if (!token_ok(token)) {
        r=MOTOR_CMD_NOT_OWNER;
    } else if (cmd == CMD_IDENTIFY) {
        if (bn != 0U) r=MOTOR_CMD_INVALID_LENGTH;
        else if (Motor_Encoder_IsDataFresh(2U) == 0U) r=MOTOR_CMD_NOT_READY;
        else r=Motor_Parameters_IdentifyAndSave() ? MOTOR_CMD_OK : MOTOR_CMD_BUSY;
    } else if (cmd == CMD_RELEASE) {
        Motor_System_ReleaseHost(); s_token=0U;
    } else if (cmd == CMD_HEARTBEAT) {
        Motor_System_HostHeartbeat(); out_n=build_status(out);
    } else if (cmd == CMD_START) {
        r=Motor_System_HostStart();
    } else if (cmd == CMD_SET_MODE) {
        if (bn!=2U) r=MOTOR_CMD_INVALID_LENGTH;
        else r=Motor_System_HostSetMode((MotorControlMode)b[0],(MotorInputSource)b[1]);
    } else if (cmd == CMD_SET_IQ) {
        float accepted=0.0f;
        if (bn!=4U) r=MOTOR_CMD_INVALID_LENGTH;
        else { r=Motor_System_HostSetIq(get_f32(b),&accepted); put_f32(out,accepted); out_n=4U; }
    } else if (cmd == CMD_SET_IQ_LIMIT) {
        if (bn!=4U) r=MOTOR_CMD_INVALID_LENGTH;
        else { r=Motor_System_HostSetIqLimit(get_f32(b)); put_f32(out,Motor_System_GetControlOwner()==MOTOR_OWNER_HOST?get_f32(b):0.0f); out_n=4U; }
    } else if (cmd == CMD_SET_HAPTIC) {
        MotorHapticParams hp;
        if (bn!=22U) r=MOTOR_CMD_INVALID_LENGTH;
        else { hp.spring_k_a_per_rad=get_f32(&b[0]); hp.damping_b_a_per_rad_s=get_f32(&b[4]); hp.detent_k_a_per_rad=get_f32(&b[8]); hp.limit_k_a_per_rad=get_f32(&b[12]); hp.limit_half_range_deg=get_f32(&b[16]); hp.detent_count=get_u16(&b[20]); r=Motor_System_HostSetHapticParams(&hp); }
    } else if (cmd == CMD_GET_PARAMS) {
        MotorControlSnapshot cs; MotorHapticParams hp; Motor_System_GetControlSnapshot(&cs); Motor_System_GetHapticParams(&hp);
        out[0]=(uint8_t)cs.mode; out[1]=(uint8_t)cs.source; put_f32(&out[2],cs.iq_limit_a); put_f32(&out[6],MOTOR_IQ_SLEW_A_PER_S);
        put_f32(&out[10],hp.spring_k_a_per_rad); put_f32(&out[14],hp.damping_b_a_per_rad_s); put_f32(&out[18],hp.detent_k_a_per_rad); put_f32(&out[22],hp.limit_k_a_per_rad); put_f32(&out[26],hp.limit_half_range_deg); put_u16(&out[30],hp.detent_count); out_n=32U;
    } else if (cmd == CMD_SET_FRICTION) {
        MotorFrictionConfig fc;
        if (bn!=17U) r=MOTOR_CMD_INVALID_LENGTH;
        else {
            fc.enabled=b[0]; fc.coulomb_iq_a=get_f32(&b[1]);
            fc.viscous_iq_a_per_rpm=get_f32(&b[5]);
            fc.smooth_speed_rpm=get_f32(&b[9]); fc.max_iq_a=get_f32(&b[13]);
            r=Motor_System_HostSetFrictionConfig(&fc);
            if (r==MOTOR_CMD_OK) { memcpy(out,b,17U); out_n=17U; }
        }
    } else if (cmd == CMD_GET_FRICTION) {
        MotorFrictionConfig fc;
        if (bn!=0U) r=MOTOR_CMD_INVALID_LENGTH;
        else {
            Motor_System_GetFrictionConfig(&fc); out[0]=fc.enabled;
            put_f32(&out[1],fc.coulomb_iq_a);
            put_f32(&out[5],fc.viscous_iq_a_per_rpm);
            put_f32(&out[9],fc.smooth_speed_rpm); put_f32(&out[13],fc.max_iq_a);
            out_n=17U;
        }
    } else if (cmd == CMD_SET_SPEED) {
        float accepted=0.0f;
        if (bn!=4U) r=MOTOR_CMD_INVALID_LENGTH;
        else { r=Motor_System_HostSetSpeed(get_f32(b),&accepted); put_f32(out,accepted); out_n=4U; }
    } else if (cmd == CMD_SET_COGGING_CONFIG) {
        MotorCoggingConfig cc;
        if (bn!=19U) r=MOTOR_CMD_INVALID_LENGTH;
        else {
            cc.enabled=b[0]; cc.gain=get_f32(&b[1]); cc.max_iq_a=get_f32(&b[5]);
            cc.fade_start_rpm=get_f32(&b[9]); cc.fade_end_rpm=get_f32(&b[13]);
            cc.phase_offset_counts=get_u16(&b[17]);
            r=Motor_System_HostSetCoggingConfig(&cc);
            if (r==MOTOR_CMD_OK) { memcpy(out,b,19U); out_n=19U; }
        }
    } else if (cmd == CMD_GET_COGGING_CONFIG) {
        MotorCoggingConfig cc; MotorControlSnapshot cs;
        if (bn!=0U) r=MOTOR_CMD_INVALID_LENGTH;
        else {
            Motor_System_GetCoggingConfig(&cc); Motor_System_GetControlSnapshot(&cs);
            out[0]=cc.enabled;
            put_f32(&out[1],cc.gain); put_f32(&out[5],cc.max_iq_a);
            put_f32(&out[9],cc.fade_start_rpm); put_f32(&out[13],cc.fade_end_rpm);
            put_u16(&out[17],cc.phase_offset_counts);
            put_u32(&out[19],cs.cogging_table_revision);
            put_u32(&out[23],cs.cogging_table_crc);
            out[27]=Motor_Cogging_IsPersisted(); out_n=28U;
        }
    } else if (cmd == CMD_COGGING_TABLE_BEGIN) {
        uint16_t transaction=0U;
        if (bn!=6U) r=MOTOR_CMD_INVALID_LENGTH;
        else {
            r=Motor_System_HostBeginCoggingTable(get_u16(&b[0]),get_u32(&b[2]),
                                                 &transaction);
            if (r==MOTOR_CMD_OK) { put_u16(out,transaction); out_n=2U; }
        }
    } else if (cmd == CMD_COGGING_TABLE_CHUNK) {
        int16_t values[MOTOR_COGGING_CHUNK_MAX_POINTS];
        if ((bn<5U) || (b[4]==0U) || (b[4]>MOTOR_COGGING_CHUNK_MAX_POINTS) ||
            (bn!=(uint16_t)(5U+2U*b[4]))) r=MOTOR_CMD_INVALID_LENGTH;
        else {
            for (uint16_t i=0U;i<b[4];++i) values[i]=get_i16(&b[5U+2U*i]);
            r=Motor_System_HostWriteCoggingChunk(get_u16(&b[0]),get_u16(&b[2]),
                                                 b[4],values);
            if (r==MOTOR_CMD_OK) {
                put_u16(&out[0],get_u16(&b[2])); out[2]=b[4]; out_n=3U;
            }
        }
    } else if (cmd == CMD_COGGING_TABLE_COMMIT) {
        MotorControlSnapshot cs;
        if (bn!=2U) r=MOTOR_CMD_INVALID_LENGTH;
        else {
            r=Motor_System_HostCommitCoggingTable(get_u16(b));
            if (r==MOTOR_CMD_OK) {
                Motor_System_GetControlSnapshot(&cs);
                put_u32(&out[0],cs.cogging_table_revision);
                put_u32(&out[4],cs.cogging_table_crc); out_n=8U;
            }
        }
    } else if (cmd == CMD_GET_COGGING_TABLE_CHUNK) {
        int16_t values[MOTOR_COGGING_CHUNK_MAX_POINTS];
        if ((bn!=3U) || (b[2]==0U) ||
            (b[2]>MOTOR_COGGING_CHUNK_MAX_POINTS)) r=MOTOR_CMD_INVALID_LENGTH;
        else {
            r=Motor_System_HostReadCoggingChunk(get_u16(&b[0]),b[2],values);
            if (r==MOTOR_CMD_OK) {
                put_u16(&out[0],get_u16(&b[0])); out[2]=b[2];
                for (uint16_t i=0U;i<b[2];++i) {
                    put_u16(&out[3U+2U*i],(uint16_t)values[i]);
                }
                out_n=(uint16_t)(3U+2U*b[2]);
            }
        }
    } else if (cmd == CMD_SAVE_COGGING) {
        MotorControlSnapshot cs;
        if (bn!=0U) r=MOTOR_CMD_INVALID_LENGTH;
        else {
            r=Motor_System_HostSaveCogging();
            Motor_System_GetControlSnapshot(&cs);
            if (r==MOTOR_CMD_OK) {
                put_u32(&out[0],cs.cogging_table_revision);
                put_u32(&out[4],cs.cogging_table_crc);
                out[8]=Motor_Cogging_IsPersisted(); out_n=9U;
            }
        }
    } else r=MOTOR_CMD_UNSUPPORTED;
    response(seq,cmd,r,out,out_n);
    if (command_uses_replay_cache(cmd)) {
        s_last_request_seq=seq; s_last_request_len=n; memcpy(s_last_request,p,n);
    }
}

void PC_Protocol_Init(void)
{
    s_head=s_tail=s_parse_len=0U; s_tx_busy=0U; s_tx_pending_len=0U;
    s_token=0U; s_token_counter=0U; s_crc_errors=s_overflows=s_telemetry_drops=0U;
    s_last_request_len=0U; s_last_request_seq=0U; s_last_response_len=0U;
    s_sample_seq=0U; s_telemetry_seq=s_status_seq=0U; s_last_status_ms=HAL_GetTick();
}

void PC_Protocol_FeedFromISR(const uint8_t *data, uint16_t length)
{
    for (uint16_t i=0U;i<length;++i) {
        uint16_t next=(uint16_t)((s_head+1U)%RX_RING_SIZE);
        if (next==s_tail) { ++s_overflows; break; }
        s_ring[s_head]=data[i]; s_head=next;
    }
}

void PC_Protocol_Task(void)
{
    uint16_t budget=512U;
    while ((s_tail!=s_head) && (budget--!=0U)) {
        uint8_t byte=s_ring[s_tail]; s_tail=(uint16_t)((s_tail+1U)%RX_RING_SIZE);
        if ((s_parse_len==0U) && (byte!=SOF0)) continue;
        if ((s_parse_len==1U) && (byte!=SOF1)) { s_parse_len=(byte==SOF0)?1U:0U; continue; }
        s_parse[s_parse_len++]=byte;
        if (s_parse_len>=8U) {
            uint16_t payload_len=get_u16(&s_parse[6]);
            if ((payload_len>PC_PROTOCOL_MAX_PAYLOAD) || (s_parse_len>=(uint16_t)sizeof(s_parse))) { s_parse_len=0U; continue; }
            uint16_t frame_len=(uint16_t)(10U+payload_len);
            if (s_parse_len==frame_len) {
                uint16_t got=get_u16(&s_parse[8U+payload_len]);
                uint16_t calc=crc16(&s_parse[2],(uint16_t)(6U+payload_len));
                if ((got==calc) && (s_parse[2]==PC_PROTOCOL_VERSION) && (s_parse[3]==TYPE_COMMAND)) {
                    handle_command(get_u16(&s_parse[4]),&s_parse[8],payload_len);
                } else ++s_crc_errors;
                s_parse_len=0U;
            }
        }
    }
    uint32_t now=HAL_GetTick();
    if ((uint32_t)(now-s_last_status_ms)>=100U) {
        uint8_t p[40]; s_last_status_ms=now;
        (void)queue_frame(TYPE_STATUS,++s_status_seq,p,build_status(p),0U);
    }
}

void PC_Protocol_SendTelemetry(void)
{
    uint8_t p[86]; MotorControlSnapshot s; Motor_System_GetControlSnapshot(&s);
    put_u16(&p[0],4U); put_u32(&p[2],HAL_GetTick()); put_u32(&p[6],++s_sample_seq);
    float values[19] = { Motor_System_GetDebugPotTarget(), g_foc_state.pi_q.target,
        g_foc_state.park.q, g_foc_state.park.d, g_motor_system.run_data.speed_rpm,
        g_foc_state.pi_q.output, (float)Motor_Encoder_GetRawAngle()*MOTOR_ENCODER_RAD_PER_COUNT,
        s.continuous_angle_rad, g_foc_state.target_q, s.relative_center_angle_rad,
        s.target_speed_rpm, s.speed_loop_iq_a, s.friction_iq_a,
        s.speed_loop_iq_a + s.friction_iq_a, s.cogging_iq_a,
        s.cogging_effective_gain, (float)s.cogging_table_revision,
        (float)(s.cogging_table_crc & 0xFFFFU),
        (float)(s.cogging_table_crc >> 16U) };
    memcpy(&p[10],values,sizeof(values));
    if (queue_frame(TYPE_TELEMETRY,++s_telemetry_seq,p,sizeof(p),0U)==0U) ++s_telemetry_drops;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance!=USART2) return;
    s_tx_busy=0U;
    if (s_tx_pending_len!=0U) {
        uint16_t n=s_tx_pending_len; memcpy(s_tx,s_tx_pending,n); s_tx_pending_len=0U; s_tx_busy=1U;
        if (HAL_UART_Transmit_DMA(&huart2,s_tx,n)!=HAL_OK) s_tx_busy=0U;
    }
}

uint32_t PC_Protocol_GetCrcErrorCount(void) { return s_crc_errors; }
uint32_t PC_Protocol_GetRxOverflowCount(void) { return s_overflows; }
uint32_t PC_Protocol_GetTelemetryDropCount(void) { return s_telemetry_drops; }
