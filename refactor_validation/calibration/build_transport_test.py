"""Extract the production queue/CRC functions for a native HAL simulation."""
from pathlib import Path
root=Path(__file__).resolve().parents[2]
source=(root/'user/src/pc_protocol.c').read_text(encoding='utf-8')
def function(signature):
    start=source.index(signature);brace=source.index('{',start);depth=1;end=brace+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]+'\n'
parts=[function(s) for s in ('static uint16_t crc16(', 'static void put_u16(',
        'static uint16_t make_frame(', 'static void start_queued_tx(', 'static uint8_t queue_frame(', 'void HAL_UART_TxCpltCallback(')]
shim=r'''
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include "motor_calibration.h"
#define SOF0 0xA5
#define SOF1 0x5A
#define TYPE_CALIBRATION 5
#define TYPE_TELEMETRY 4
#define PC_PROTOCOL_VERSION 2
#define MAX_FRAME_SIZE 266
#define USART2 ((void*)2)
#define HAL_UART_STATE_READY 0
#define HAL_OK 0
typedef struct {void *Instance;int gState;} UART_HandleTypeDef;
static UART_HandleTypeDef huart2={USART2,0};
static uint8_t s_tx[MAX_FRAME_SIZE],s_tx_pending[MAX_FRAME_SIZE],s_tx_stream[MAX_FRAME_SIZE];
static volatile uint16_t s_tx_pending_len,s_tx_stream_len;
static volatile uint8_t s_tx_busy;
static unsigned mask;
static unsigned launch_failures;
static uint64_t now_us,done_us;
static unsigned launched,cal_launched,wire_sequence,main_ready_us;
static unsigned last_types[4];
static uint32_t __get_PRIMASK(void) {return mask;}
static void __disable_irq(void) {mask=1;}
static void __enable_irq(void) {mask=0;}
static int HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n) {
    assert(u->gState==0);
    if (launch_failures) {--launch_failures;return 1;}
    u->gState=1;
    done_us=now_us+(n*10000000ULL+921599)/921600;
    if (launched<4) last_types[launched]=p[3];
    ++launched;
    if (p[3]==5) {
        ++cal_launched;
        for (unsigned i=0;i<p[14];i++) {
            unsigned sequence;memcpy(&sequence,p+20+i*64,4);
            assert(sequence==++wire_sequence);
        }
    }
    return HAL_OK;
}
'''
driver=r'''
static uint16_t reference(const uint8_t *p,unsigned n) {
    uint16_t crc=0xffff;
    while(n--) {crc^=(uint16_t)(*p++)<<8;
        for(unsigned b=0;b<8;b++)crc=(uint16_t)((crc<<1)^((crc&0x8000)?0x1021:0));}
    return crc;
}
static void complete(void) {huart2.gState=0;HAL_UART_TxCpltCallback(&huart2);}
int main(void) {
    uint8_t bytes[256];for(unsigned i=0;i<256;i++)bytes[i]=(uint8_t)(i*73);
    for(unsigned n=0;n<=256;n++)assert(crc16(bytes,n)==reference(bytes,n));
    assert(crc16((uint8_t*)"123456789",9)==0x29b1);
    assert(Motor_Calibration_Begin(1));
    assert(queue_frame(3,1,bytes,40,0));
    assert(queue_frame(4,2,bytes,126,0)); /* prepared while DMA is active */
    assert(!queue_frame(4,3,bytes,126,0)); /* a busy stream slot cannot overwrite */
    assert(queue_frame(2,4,bytes,32,1)); /* ACK has its own priority slot */
    now_us=done_us;complete();assert(last_types[1]==2);
    now_us=done_us;complete();assert(last_types[2]==4);
    now_us=done_us;complete();assert(!s_tx_busy);
    assert(queue_frame(3,1,bytes,40,0));
    assert(queue_frame(4,2,bytes,126,0));
    launch_failures=1;
    now_us=done_us;complete();
    assert(!s_tx_busy && s_tx_stream_len==136); /* failed launch must retain queued frame */
    start_queued_tx();assert(s_tx_busy && !s_tx_stream_len);
    now_us=done_us;complete();assert(!s_tx_busy);
    now_us=done_us=0;wire_sequence=0;cal_launched=0;
    unsigned next_sample=1000,next_telem=50000,next_status=100000,next_ack=100000;
    unsigned max_backlog=0;
    while(now_us<2000000) {
        if (now_us>=next_sample) {
            MotorCalSample record={0};record.tick_ms=next_sample/1000;record.flags=256;
            Motor_Calibration_Push(&record);next_sample+=1000;
        }
        if (s_tx_busy && now_us>=done_us) complete();
        if (now_us>=main_ready_us) {
            if (now_us>=next_ack && !s_tx_pending_len) {
                assert(queue_frame(2,1,bytes,40,1));next_ack+=100000;main_ready_us=(unsigned)now_us+800;
            } else if (now_us>=next_status && !s_tx_pending_len) {
                assert(queue_frame(3,1,bytes,40,1));next_status+=100000;main_ready_us=(unsigned)now_us+800;
            } else if (!s_tx_stream_len) {
                if (now_us>=next_telem) {
                    assert(queue_frame(4,1,bytes,126,0));next_telem+=50000;main_ready_us=(unsigned)now_us+800;
                } else {
                    MotorCalSample records[3];uint8_t count=Motor_Calibration_Peek(records,3);
                    if (count==3) {
                        uint8_t payload[204]={0};payload[6]=3;memcpy(payload+12,records,192);
                        assert(queue_frame(5,0,payload,204,0));Motor_Calibration_Consume(3);
                        main_ready_us=(unsigned)now_us+800;
                    }
                }
            }
        }
        MotorCalStatus s;Motor_Calibration_GetStatus(&s);assert(s.dropped==0);
        unsigned backlog=s.produced-wire_sequence;
        if (backlog>max_backlog)max_backlog=backlog;
        now_us+=50;
    }
    printf("PASS: CRC unchanged, ACK priority, stream backpressure, 1kHz + telemetry + status + heartbeat, max backlog %u/64\n",max_backlog);
    assert(cal_launched>600 && max_backlog<64);
}
'''
(root/'.codex_tmp/calibration_transport_test.c').write_text(shim+'\n'.join(parts)+driver,encoding='utf-8')
print('Generated native test using actual firmware transport functions.')
