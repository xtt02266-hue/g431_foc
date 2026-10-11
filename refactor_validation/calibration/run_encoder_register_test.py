from pathlib import Path
import subprocess
import argparse
import shutil
root=Path(__file__).resolve().parents[2]
(root/'.codex_tmp').mkdir(exist_ok=True)
parser=argparse.ArgumentParser(description='Verify production encoder SPI access without hardware')
parser.add_argument('--zig', default=shutil.which('zig'))
args=parser.parse_args()
if not args.zig: parser.error('Zig is required: add zig to PATH or pass --zig /path/to/zig')
source=(root/'user/src/mt6826s.c').read_text(encoding='utf-8')
def function(signature):
    a=source.index(signature);b=source.index('{',a)+1;depth=1
    while depth:
        depth+=(source[b]=='{')-(source[b]=='}');b+=1
    return source[a:b]+'\n'
s=r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#ifdef NDEBUG
#error assertions required
#endif
typedef struct {unsigned dummy;} SPI_HandleTypeDef;
typedef unsigned HAL_StatusTypeDef;
#define HAL_OK 0U
#define MT6826S_BURST_FRAME_BYTES 6U
static SPI_HandleTypeDef hspi1;
static uint8_t g_mt6826s_busy,g_register_lock,g_angle_reads_suspended,g_mt6826s_timeout_reported,g_mt6826s_ok;
static uint8_t g_mt6826s_tx[6],g_mt6826s_rx[6],last_tx[3];
static uint32_t g_mt6826s_start_ms,g_mt6826s_transfer_error_count,mask,now,spi_calls,dma_calls;
static unsigned error;
static uint32_t HAL_GetTick(void){return now;}
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __enable_irq(void){mask=0;}
static void mt6826s_select(void){}
static void mt6826s_unselect(void){}
static unsigned HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *h,const uint8_t*t,uint8_t*r,uint16_t n){(void)h;(void)t;(void)r;(void)n;++dma_calls;return 0;}
void mt6826s_request_read_dma(void);
static unsigned HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h,uint8_t*t,uint8_t*r,uint16_t n,uint32_t timeout){
 assert(h==&hspi1 && n==3 && timeout==1 && g_register_lock && !mask);
 ++spi_calls;memcpy(last_tx,t,3);r[2]=0xbd;
 mt6826s_request_read_dma();assert(dma_calls==0);return error;
}
'''
for name in ('void mt6826s_request_read_dma(', 'static uint8_t mt6826s_register_transfer(',
             'uint8_t mt6826s_read_register(', 'uint8_t mt6826s_write_register(',
             'void mt6826s_suspend_angle_reads('):s+=function(name)
s+=r'''
int main(void){
 uint8_t v=0;
 g_mt6826s_busy=1;assert(mt6826s_read_register(0xe,&v)==0 && !spi_calls && !g_register_lock && !mask);
 g_mt6826s_busy=0;g_register_lock=1;assert(mt6826s_read_register(0xe,&v)==0 && !spi_calls);g_register_lock=0;
 assert(mt6826s_read_register(0xe,&v)==1 && v==0xbd && !g_register_lock);
 assert(last_tx[0]==0x30 && last_tx[1]==0xe && last_tx[2]==0);
 assert(mt6826s_write_register(0x155,0x5e)==1);
 assert(last_tx[0]==0x61 && last_tx[1]==0x55 && last_tx[2]==0x5e);
 assert(mt6826s_read_register(0x1000,&v)==2 && mt6826s_read_register(0xe,0)==2);
 error=1;g_mt6826s_ok=1;assert(mt6826s_read_register(0x113,&v)==2);
 assert(!g_mt6826s_ok && g_mt6826s_transfer_error_count==1 && !g_register_lock);
 mt6826s_suspend_angle_reads();mt6826s_request_read_dma();assert(!dma_calls);
 assert(mt6826s_read_register(0xe,&v)==2);
 puts("PASS: production SPI register read/write bytes, DMA exclusion, busy return, bounded timeout, invalidation and post-calibration suspension");
}
'''
p=root/'.codex_tmp/encoder_register_test.c';p.write_text(s,encoding='utf-8')
exe=p.with_suffix('.exe')
subprocess.run([args.zig,'cc','-std=c11','-O1','-UNDEBUG',str(p),'-o',str(exe)],check=True)
subprocess.run([str(exe)],check=True)
