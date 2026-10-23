"""Compile existing C calibration assertions for Cortex-M4 and execute them in Unicorn.

No board is attached. Only puts/assert callbacks are hooked; C state machines and
test loops execute directly. Encoder tests retain their existing peripheral shims.
"""
from pathlib import Path
import argparse
import json
import subprocess
import sys
import struct

ROOT = Path(__file__).resolve().parents[2]
TEMP = ROOT / '.codex_tmp/simplification_20261012'
sys.path[:0] = [str(ROOT / '.codex_tmp/style_validation/python_modules'), str(Path(__file__).parent)]
from run_checks import run
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
import unicorn.arm_const as arm
from elftools.elf.elffile import ELFFile


def execute(elf_path):
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    uc.ctl_set_cpu_model(arm.UC_CPU_ARM_CORTEX_M4)
    for address, size in [(0x08000000, 0x20000), (0x20000000, 0x8000), (0x10000000, 0x1000), (0xE000E000, 0x2000)]:
        uc.mem_map(address, size)
    uc.reg_write(arm.UC_ARM_REG_C1_C0_2, 0xF << 20)
    uc.reg_write(arm.UC_ARM_REG_FPEXC, 0x40000000)
    uc.mem_write(0xE000ED88, struct.pack('<I', 0xF << 20))
    symbols = {}
    with elf_path.open('rb') as file:
        elf = ELFFile(file)
        for section in elf.iter_sections():
            if section['sh_flags'] & 2 and section['sh_size'] and section['sh_type'] != 'SHT_NOBITS':
                uc.mem_write(section['sh_addr'], section.data())
        for sym in elf.get_section_by_name('.symtab').iter_symbols():
            symbols[sym.name] = sym['st_value']
    def string(address):
        data = bytearray()
        while True:
            value = uc.mem_read(address + len(data), 1)[0]
            if value == 0:
                return data.decode('utf-8', errors='replace')
            data.append(value)
    messages = []
    def hook(uc, address, size, user):
        if address == (symbols['__wrap_puts'] & ~1):
            messages.append(string(uc.reg_read(arm.UC_ARM_REG_R0)))
            uc.reg_write(arm.UC_ARM_REG_R0, 0)
            uc.reg_write(arm.UC_ARM_REG_PC, uc.reg_read(arm.UC_ARM_REG_LR))
        else:
            raise AssertionError((string(uc.reg_read(arm.UC_ARM_REG_R0)),
                                  uc.reg_read(arm.UC_ARM_REG_R1), string(uc.reg_read(arm.UC_ARM_REG_R3))))
    for name in ['__wrap_puts', '__wrap___assert_func']:
        address = symbols[name] & ~1
        uc.hook_add(UC_HOOK_CODE, hook, begin=address, end=address)
    uc.reg_write(arm.UC_ARM_REG_SP, 0x20008000 - 128)
    uc.reg_write(arm.UC_ARM_REG_LR, 0x10000001)
    address = symbols['main'] | 1
    for batch in range(8):
        uc.emu_start(address | 1, 0x10000000, count=2_000_000_000)
        address = uc.reg_read(arm.UC_ARM_REG_PC)
        if address == 0x10000000:
            break
        print(f'{elf_path.name}: completed instruction batch {batch + 1}; continuing full C assertions.', flush=True)
    pc = uc.reg_read(arm.UC_ARM_REG_PC)
    if pc != 0x10000000:
        nearest = sorted((address, name) for name, address in symbols.items() if 0x08000000 <= address <= pc)[-3:]
        details = {name: bytes(uc.mem_read(symbols[name], 4)).hex() for name in ['elapsed', 'point_index', 'stable_ticks'] if name in symbols}
        raise AssertionError(('Instruction budget exhausted', hex(pc), nearest, details))
    assert uc.reg_read(arm.UC_ARM_REG_R0) == 0
    assert messages, 'Test did not report completion'
    print('\n'.join(messages))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=['all', 'calibration', 'encoder'], default='all')
    parser.add_argument('--average-only', action='store_true')
    parser.add_argument('--source-baseline', action='store_true')
    args = parser.parse_args()
    entries = json.loads((TEMP / 'current/compile_commands.json').read_text())
    argv = next(e['command'].split() for e in entries if e['file'].endswith('/firmware/app/motor_system.c'))
    for option in ['-c', '-o']:
        index = argv.index(option)
        del argv[index:index + 2]
    argv = [a for a in argv if a != '-O0'] + ['-O1', '-UNDEBUG', '-Werror']
    support = TEMP / 'assert_support.c'
    support.write_text('int __wrap_puts(const char *p) {(void)p;return 0;}\n'
                       'void __wrap___assert_func(const char *f,int l,const char *n,const char *s) '
                       '{(void)f;(void)l;(void)n;(void)s;for(;;){}}\n')
    script = TEMP / 'test.ld'
    script.write_text('SECTIONS { .text 0x08000000 : { *(.text*) *(.rodata*) } '
                      '.data 0x20000000 : { *(.data*) } .bss : { *(.bss*) *(COMMON) } end = .; }\n')
    shim = TEMP / 'encoder_shim'
    shim.mkdir(exist_ok=True)
    (shim / 'motor_system.h').write_text('''#include <stdint.h>
enum {MOTOR_OWNER_LOCAL,MOTOR_OWNER_HOST};
enum {MOTOR_STATE_STOPPED,MOTOR_STATE_WAIT_PARAMETERS,MOTOR_STATE_IDENTIFYING,MOTOR_STATE_SENSORED_RUN,MOTOR_STATE_MUSIC,MOTOR_STATE_FAULT};
typedef struct {uint8_t state,owner,run_requested;float target_speed_rpm;} motor_control_snapshot_t;
typedef struct {struct {float speed_rpm;} run_data;uint8_t state;} motor_system_t;
extern motor_system_t g_motor_system;
void motor_system_get_control_snapshot(motor_control_snapshot_t*);
void motor_system_stop_control(void);
''')
    (shim / 'motor_encoder.h').write_text('#include <stdint.h>\nuint8_t motor_encoder_is_data_fresh(uint32_t);\nuint16_t motor_encoder_get_raw_angle(void);\n')
    (shim / 'stm32g4xx_hal.h').write_text('#include <stdint.h>\nuint32_t HAL_GetTick(void);\nuint32_t __get_PRIMASK(void);\nvoid __disable_irq(void);\nvoid __enable_irq(void);\n')
    for name, source, driver in [
        ('calibration', ROOT / 'firmware/calibration/motor_calibration.c', ROOT / 'refactor_validation/calibration/test_calibration.c'),
        ('encoder', ROOT / 'firmware/calibration/motor_encoder_calibration.c', ROOT / 'refactor_validation/calibration/test_encoder_autocal.c')]:
        if args.case != 'all' and args.case != name:
            continue
        if args.source_baseline:
            source = TEMP / 'safety_baseline' / source.relative_to(ROOT)
        driver_copy = TEMP / (name + '_driver.c')
        driver_text = driver.read_text(encoding='utf-8').replace('mt6826_s_', 'mt6826s_')
        if args.average_only and name == 'calibration':
            driver_text = driver_text.replace('int main(int argc, char **argv)', 'int legacy_main(int argc, char **argv)')
            driver_text += '\nint main(void) {test_average_policy();puts("PASS: full average-policy assertions");return 0;}\n'
        driver_copy.write_text(driver_text, encoding='utf-8')
        includes = ['-I' + str(shim)] if name == 'encoder' else []
        objects = []
        for i, path in enumerate([source, driver_copy, support]):
            obj = TEMP / (name + str(i) + '.o')
            run([argv[0], *includes, *argv[1:], '-c', path, '-o', obj])
            objects.append(obj)
        target = TEMP / (name + '_test.elf')
        run([argv[0], '-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard',
             '-nostartfiles', '--specs=nano.specs', '--specs=nosys.specs', '-Wl,--gc-sections', '-Wl,-e,main',
             '-Wl,--wrap=puts', '-Wl,--wrap=__assert_func', '-T', script, *objects, '-lm', '-o', target])
        execute(target)


if __name__ == '__main__':
    main()
