"""执行真实 ARM 齿槽模块，检查内置表、Flash 恢复和输出门控；不连接设备。"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import math
import struct
import sys
import zlib

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'refactor_validation/style_20261011'))
from verify_commands import Firmware, SCRATCH, RETURN, STACK, REGISTERS, FLOAT_REGISTERS, arm

FLASH_PAGE = 0x0801F000
FLASH_PAGE_SIZE = 2048
BUILTIN_CRC = 0x8706A889
BUILTIN_REVISION = 10
RECORD_SIZE = 1072
RECORD_TAG_OFFSET = 1064
RECORD_CRC_OFFSET = 1068
POINTS = 512
TABLE_BUFFER = SCRATCH + 0x2000
CONFIG_BUFFER = SCRATCH + 0x3000
CSV_PATH = ROOT / 'calibration_results/hybrid_512_20261022/HYBRID_UNVALIDATED_512.csv'


class CoggingFirmware(Firmware):
    """仅替换 Flash 物理操作，保留真实记录编码、CRC、表交换及控制计算。"""

    def call(self, name, parameters=(), float_result=False):
        # 沿用既有框架的AAPCS装载方式；初始化含3次CRC，扩大指令预算而非替换CRC实现。
        integer_index = float_index = 0
        stack = bytearray()
        for kind, value in parameters:
            if kind == 'float' and float_index < 16:
                self.uc.reg_write(FLOAT_REGISTERS[float_index], struct.unpack('<I', struct.pack('<f', value))[0])
                float_index += 1
            elif kind != 'float' and integer_index < 4:
                self.uc.reg_write(REGISTERS[integer_index], int(value) & 0xFFFFFFFF)
                integer_index += 1
            else:
                stack.extend(struct.pack('<f' if kind == 'float' else '<I', value))
        stack_address = STACK - ((len(stack) + 7) & ~7)
        if stack:
            self.uc.mem_write(stack_address, bytes(stack))
        self.uc.reg_write(arm.UC_ARM_REG_SP, stack_address)
        self.uc.reg_write(arm.UC_ARM_REG_LR, RETURN | 1)
        self.uc.emu_start(self.address(name) | 1, RETURN, count=2000000)
        if self.uc.reg_read(arm.UC_ARM_REG_PC) != RETURN:
            raise RuntimeError(f'{name}: did not return within instruction budget')
        return self.uc.reg_read(arm.UC_ARM_REG_S0 if float_result else arm.UC_ARM_REG_R0)

    def reset(self, *args, **kwargs):
        super().reset(*args, **kwargs)
        self.flash_operations = []
        self.erase_failure = False
        self.program_failure = False
        for name in ('HAL_FLASH_Unlock', 'HAL_FLASH_Lock', 'HAL_FLASHEx_Erase', 'HAL_FLASH_Program'):
            self.stub(name, 0)

    def hook(self, uc, address, size, user):
        boundary = self.stubs.get(address)
        if boundary:
            name, result = boundary
            if name == 'HAL_FLASHEx_Erase':
                descriptor = uc.reg_read(arm.UC_ARM_REG_R0)
                erase_type, bank, page, count = struct.unpack('<4I', bytes(uc.mem_read(descriptor, 16)))
                self.flash_operations.append({'operation': 'erase', 'page': page, 'count': count, 'bank': bank})
                assert page == 62 and count == 1, '只允许模拟齿槽专属页擦除'
                if self.erase_failure:
                    result = 1
                else:
                    uc.mem_write(FLASH_PAGE, bytes([255]) * FLASH_PAGE_SIZE)
                    uc.mem_write(uc.reg_read(arm.UC_ARM_REG_R1), struct.pack('<I', 0xFFFFFFFF))
            elif name == 'HAL_FLASH_Program':
                target = uc.reg_read(arm.UC_ARM_REG_R1)
                word = struct.pack('<II', uc.reg_read(arm.UC_ARM_REG_R2), uc.reg_read(arm.UC_ARM_REG_R3))
                assert FLASH_PAGE <= target <= FLASH_PAGE + FLASH_PAGE_SIZE - 8
                self.flash_operations.append({'operation': 'program', 'address': target})
                if self.program_failure:
                    result = 1
                else:
                    uc.mem_write(target, word)
            if name.startswith('HAL_FLASH'):
                uc.reg_write(arm.UC_ARM_REG_R0, result)
                uc.reg_write(arm.UC_ARM_REG_PC, uc.reg_read(arm.UC_ARM_REG_LR))
                return
        super().hook(uc, address, size, user)

    def floating(self, name, parameters=()):
        bits = self.call(name, parameters, float_result=True)
        return struct.unpack('<f', struct.pack('<I', bits))[0]

    def config(self):
        self.call('motor_cogging_get_config', [('uint32_t', CONFIG_BUFFER)])
        enabled, gain, limit, start, end, phase = struct.unpack('<B3xffffH2x', bytes(self.uc.mem_read(CONFIG_BUFFER, 24)))
        return {'enabled': enabled, 'gain': gain, 'limit_a': limit, 'fade_start_rpm': start,
                'fade_end_rpm': end, 'phase_counts': phase}

    def set_config(self, enabled=1, gain=.9, limit=.08, start=100., end=200., phase=0):
        self.call('motor_cogging_set_config', [('uint8_t', enabled), ('float', gain), ('float', limit),
                  ('float', start), ('float', end), ('uint16_t', phase)])

    def table(self):
        values = []
        for offset in range(0, POINTS, 120):
            count = min(120, POINTS - offset)
            assert self.call('motor_cogging_table_read_chunk', [('uint16_t', offset), ('uint8_t', count),
                             ('uint32_t', TABLE_BUFFER)]) == 1
            values.extend(struct.unpack('<' + 'h' * count, bytes(self.uc.mem_read(TABLE_BUFFER, count * 2))))
        return values

    def initialized(self, record=None):
        self.reset()
        self.uc.mem_write(FLASH_PAGE, bytes([255]) * FLASH_PAGE_SIZE)
        if record is not None:
            self.uc.mem_write(FLASH_PAGE, record)
        self.call('motor_cogging_init')


def encode_record(values, tag=BUILTIN_CRC, enabled=0, gain=.7, limit=.015,
                  start=40., end=120., phase=128, revision=42):
    """按现有线上格式构造独立测试记录，不调用被测模块的保存函数。"""
    table_crc = zlib.crc32(struct.pack('<512h', *values)) & 0xFFFFFFFF
    body = struct.pack('<IHHB3xffffHHII512hI', 0x434F4747, 2, RECORD_SIZE, enabled,
                       gain, limit, start, end, phase, POINTS, table_crc, revision, *values, tag)
    assert len(body) == RECORD_CRC_OFFSET
    return body + struct.pack('<I', zlib.crc32(body) & 0xFFFFFFFF)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--elf', type=Path, default=ROOT / 'build/Debug/g431_foc.elf')
    parser.add_argument('--report', type=Path,
                        default=ROOT / '.codex_tmp/builtin_lut_20261022/verification.json')
    args = parser.parse_args()
    results = []

    def check(label, condition, **evidence):
        results.append({'assertion': label, 'passed': bool(condition), **evidence})
        assert condition, label

    f = CoggingFirmware(args.elf, True, {})
    with CSV_PATH.open(encoding='utf-8-sig', newline='') as stream:
        rows = list(csv.DictReader(stream))
    expected = [round(float(row['iq_a']) / .0001) for row in rows]
    expected_bytes = struct.pack('<512h', *expected)
    builtin_address, builtin_size = f.symbols['s_motor_cogging_builtin_table']
    check('CSV 512点角度网格', len(rows) == POINTS and all(int(row['index']) == i and int(row['angle_count']) == i * 64
          for i, row in enumerate(rows)))
    check('内置数组驻留程序Flash且占1024字节', 0x08000000 <= builtin_address < 0x0801E800 and builtin_size == 1024,
          address=hex(builtin_address), bytes=builtin_size)
    check('ELF内置表与CSV实际协议量化逐点相同', bytes(f.uc.mem_read(builtin_address, builtin_size)) == expected_bytes)
    check('独立CRC匹配', (zlib.crc32(expected_bytes) & 0xFFFFFFFF) == BUILTIN_CRC)

    f.initialized()
    config = f.config()
    check('空白页自动加载内置512点', f.table() == expected)
    check('空白页活动CRC和修订号', f.call('motor_cogging_get_active_table_crc') == BUILTIN_CRC and
          f.call('motor_cogging_get_table_revision') == BUILTIN_REVISION)
    check('默认使能且使用0.9增益和0.08A限幅', config['enabled'] == 1 and math.isclose(config['gain'], .9, abs_tol=1e-7)
          and math.isclose(config['limit_a'], .08, abs_tol=1e-7) and config['fade_start_rpm'] == 100.
          and config['fade_end_rpm'] == 200. and config['phase_counts'] == 0, config=config)
    check('内置配置可重启恢复标志', f.call('motor_cogging_is_persisted') == 1)
    check('初始化不擦写参数页', not f.flash_operations)
    check('停机初始化有效增益为零', f.floating('motor_cogging_get_effective_gain') == 0.)
    check('尚未运行时所有抽查角度补偿为零', all(f.floating('motor_cogging_compensation',
          [('uint16_t', angle), ('float', 0.)]) == 0. for angle in (0, 64, 32767)))

    custom = [((index * 13) % 301) - 150 for index in range(POINTS)]
    for label, record in [
        ('旧来源标签0', encode_record(custom, tag=0)),
        ('来源标签不匹配', encode_record(custom, tag=BUILTIN_CRC ^ 1)),
        ('记录CRC损坏', encode_record(custom)[:-1] + bytes([encode_record(custom)[-1] ^ 1])),
    ]:
        f.initialized(record)
        check(label + '回退内置表', f.table() == expected and f.config()['enabled'] == 1)
        check(label + '不擦除旧记录', bytes(f.uc.mem_read(FLASH_PAGE, len(record))) == record and not f.flash_operations)
    bad_table_crc = bytearray(encode_record(custom))
    bad_table_crc[40] ^= 1
    bad_table_crc[-4:] = struct.pack('<I', zlib.crc32(bad_table_crc[:-4]) & 0xFFFFFFFF)
    f.initialized(bytes(bad_table_crc))
    check('表CRC损坏且记录CRC正常时回退内置', f.table() == expected)

    f.initialized(encode_record(custom, enabled=0))
    check('匹配来源标签恢复用户表', f.table() == custom)
    loaded = f.config()
    check('匹配记录保留禁用状态及配置', loaded['enabled'] == 0 and loaded['phase_counts'] == 128
          and math.isclose(loaded['gain'], .7, abs_tol=1e-7) and math.isclose(loaded['limit_a'], .015, abs_tol=1e-7))
    check('匹配记录修订号与持久标志', f.call('motor_cogging_get_table_revision') == 42
          and f.call('motor_cogging_is_persisted') == 1)

    f.initialized()
    f.set_config(enabled=0, gain=.25, limit=.02, start=20., end=90., phase=64)
    check('修改配置撤销持久标志', f.call('motor_cogging_is_persisted') == 0)
    check('实际保存路径成功', f.call('motor_cogging_save_to_flash') == 1)
    saved = bytes(f.uc.mem_read(FLASH_PAGE, RECORD_SIZE))
    check('保存记录写入来源标签', struct.unpack_from('<I', saved, RECORD_TAG_OFFSET)[0] == BUILTIN_CRC)
    check('保存记录独立CRC正确', struct.unpack_from('<I', saved, RECORD_CRC_OFFSET)[0] ==
          (zlib.crc32(saved[:RECORD_CRC_OFFSET]) & 0xFFFFFFFF))
    check('保存只操作齿槽页并写满记录', len([op for op in f.flash_operations if op['operation'] == 'erase']) == 1
          and len([op for op in f.flash_operations if op['operation'] == 'program']) == RECORD_SIZE // 8)
    f.initialized(saved)
    loaded = f.config()
    check('保存后重启恢复禁用配置和内置表', loaded['enabled'] == 0 and loaded['phase_counts'] == 64
          and math.isclose(loaded['gain'], .25, abs_tol=1e-7) and f.table() == expected)

    # 完整上传失败不破坏当前表，成功事务沿用公开接口切换活动表。
    custom_bytes = struct.pack('<512h', *custom)
    custom_crc = zlib.crc32(custom_bytes) & 0xFFFFFFFF
    f.initialized()

    def upload(expected_crc, chunks=True):
        check('上传BEGIN接受512点', f.call('motor_cogging_table_begin', [('uint16_t', POINTS), ('uint32_t', expected_crc)]) == 1)
        transaction = f.call('motor_cogging_get_transaction_id')
        if chunks:
            for offset in range(0, POINTS, 120):
                count = min(120, POINTS - offset)
                f.uc.mem_write(TABLE_BUFFER, struct.pack('<' + 'h' * count, *custom[offset:offset + count]))
                check(f'上传片段{offset}+{count}', f.call('motor_cogging_table_write_chunk',
                      [('uint16_t', transaction), ('uint16_t', offset), ('uint8_t', count), ('uint32_t', TABLE_BUFFER)]) == 1)
        return transaction

    transaction = upload(custom_crc, chunks=False)
    check('未收齐拒绝提交且保留内置表', f.call('motor_cogging_table_commit', [('uint16_t', transaction)]) == 0
          and f.table() == expected)
    transaction = upload(custom_crc ^ 1)
    check('CRC错误拒绝提交且保留内置表', f.call('motor_cogging_table_commit', [('uint16_t', transaction)]) == 0
          and f.table() == expected)
    transaction = upload(custom_crc)
    check('完整正确上传成功切换', f.call('motor_cogging_table_commit', [('uint16_t', transaction)]) == 1
          and f.table() == custom and f.call('motor_cogging_get_active_table_crc') == custom_crc)
    check('RAM上传修订号增加且未持久化', f.call('motor_cogging_get_table_revision') == BUILTIN_REVISION + 1
          and f.call('motor_cogging_is_persisted') == 0)

    f.initialized()
    f.call('motor_cogging_runtime_start')
    f.call('motor_cogging_task1ms', [('uint8_t', 1)])
    check('首个运行tick按500ms渐入', math.isclose(f.floating('motor_cogging_get_effective_gain'), .9 / 500, abs_tol=1e-8))
    for _ in range(498):
        f.call('motor_cogging_task1ms', [('uint8_t', 1)])
    check('第499ms尚未满增益', math.isclose(f.floating('motor_cogging_get_effective_gain'), .9 * 499 / 500, abs_tol=1e-7))
    f.call('motor_cogging_task1ms', [('uint8_t', 1)])
    check('第500ms达到配置增益', math.isclose(f.floating('motor_cogging_get_effective_gain'), .9, abs_tol=1e-7))
    for angle in (0, 64, 96, 32704, 32767):
        idx, fraction = angle // 64, (angle % 64) / 64
        interpolation = (expected[idx] + (expected[(idx + 1) % POINTS] - expected[idx]) * fraction) * .0001 * .9
        actual = f.floating('motor_cogging_compensation', [('uint16_t', angle), ('float', 0.)])
        check(f'插值与环绕角度{angle}', math.isclose(actual, interpolation, abs_tol=2e-8), actual_a=actual)
    for speed, fraction in ((100., 1.), (150., .5), (-150., .5), (200., 0.), (300., 0.)):
        expected_iq = expected[1] * .0001 * .9 * fraction
        actual = f.floating('motor_cogging_compensation', [('uint16_t', 64), ('float', speed)])
        check(f'速度淡出{speed:g}rpm', math.isclose(actual, expected_iq, abs_tol=2e-8), actual_a=actual)
    check('非有限速度无输出', f.floating('motor_cogging_compensation', [('uint16_t', 64), ('float', math.nan)]) == 0.)
    f.set_config(gain=10., limit=.001)
    for _ in range(500):
        f.call('motor_cogging_task1ms', [('uint8_t', 1)])
    for angle, limit in ((0, -.001), (64, .001)):
        actual = f.floating('motor_cogging_compensation', [('uint16_t', angle), ('float', 0.)])
        check(f'正负补偿限幅{angle}', math.isclose(actual, limit, abs_tol=1e-9), actual_a=actual)
    f.call('motor_cogging_task1ms', [('uint8_t', 0)])
    check('不适用运行模式清零渐入', f.floating('motor_cogging_get_effective_gain') == 0.
          and f.floating('motor_cogging_compensation', [('uint16_t', 64), ('float', 0.)]) == 0.)
    f.call('motor_cogging_task1ms', [('uint8_t', 1)])
    f.call('motor_cogging_runtime_stop')
    check('停机立即清零补偿', f.floating('motor_cogging_get_effective_gain') == 0.
          and f.floating('motor_cogging_compensation', [('uint16_t', 64), ('float', 0.)]) == 0.)

    report = {'status': 'passed_offline', 'elf': str(args.elf.resolve()),
              'elf_sha256': hashlib.sha256(args.elf.read_bytes()).hexdigest(),
              'candidate_csv': str(CSV_PATH), 'builtin_crc32': f'0x{BUILTIN_CRC:08X}',
              'assertion_count': len(results), 'assertions': results,
              'method': 'Unicorn运行实际ARM ELF；仅Flash物理接口和时钟为测试桩，沿用现有AAPCS执行框架。',
              'limitations': ['未烧录、未连接MCU、未测试实际Flash擦写或中断时序。',
                              '补偿表索引0～4仍为估算值；测试通过不代表电机补偿收益已验证。']}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(f'PASS: {len(results)} assertions; {args.report}')


if __name__ == '__main__':
    main()
