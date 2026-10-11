"""Compile baseline/current project C files and compare code, data, relocations and public symbols."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess


def object_contents(path, renames):
    data = path.read_bytes()
    header = struct.unpack_from('<16sHHIIIIIHHHHHH', data)
    if header[0][:6] != b'\x7fELF\x01\x01':
        raise ValueError('Expected little-endian ELF32 object')
    headers = [struct.unpack_from('<10I', data, header[6] + i * header[11]) for i in range(header[12])]
    names_header = headers[header[13]]
    names = data[names_header[4]:names_header[4] + names_header[5]]

    def normalize(name):
        if name in renames:
            return renames[name]
        for current, original in renames.items():
            if name.endswith('.' + current):
                return name[:-len(current)] + original
        return name

    def section_name(index):
        offset = headers[index][0]
        return normalize(names[offset:names.index(b'\0', offset)].decode('utf-8'))

    symbols = []
    for section in headers:
        if section[1] != 2:
            continue
        strings_section = headers[section[6]]
        strings = data[strings_section[4]:strings_section[4] + strings_section[5]]
        for offset in range(section[4], section[4] + section[5], section[9]):
            name_offset, value, size, info, other, index = struct.unpack_from('<IIIBBH', data, offset)
            name = strings[name_offset:strings.index(b'\0', name_offset)].decode('utf-8')
            if info & 15 == 3:
                name = section_name(index)
            symbols.append((normalize(name), value, size, info, other, index))

    allocated = {}
    for index, section in enumerate(headers):
        if not section[2] & 2:
            continue
        contents = b'' if section[1] == 8 else data[section[4]:section[4] + section[5]]
        allocated[section_name(index)] = [section[1], section[2], section[5], section[8],
                                          hashlib.sha256(contents).hexdigest()]
    relocations = {}
    for section in headers:
        if section[1] != 9 or not headers[section[7]][2] & 2:
            continue
        entries = []
        for offset in range(section[4], section[4] + section[5], section[9]):
            position, info = struct.unpack_from('<II', data, offset)
            symbol = symbols[info >> 8]
            entries.append([position, info & 255, symbol[0], symbol[1]])
        relocations[section_name(section[7])] = entries
    public = {name: [value, size, info & 15, section_name(index)]
              for name, value, size, info, other, index in symbols
              if info >> 4 == 1 and 0 < index < len(headers) and headers[index][2] & 2}
    return {'sections': allocated, 'relocations': relocations, 'public_symbols': public}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--build-dir', type=Path, required=True)
    parser.add_argument('--renames', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    baseline = args.baseline.resolve()
    build = args.build_dir.resolve()
    output = build.parent / 'objects'
    output.mkdir(exist_ok=True)
    commands = json.loads((build / 'compile_commands.json').read_text(encoding='utf-8'))
    entry = next(command for command in commands if command['file'].endswith('/user/src/motor_system.c'))
    # CMake emitted short Windows executable paths and no space-containing arguments.
    command = entry['command'].split()
    assert not any('"' in arg for arg in command), 'Space-containing paths need an argument parser'
    for flag in ('-o', '-c'):
        index = command.index(flag)
        del command[index:index + 2]
    command = [arg for arg in command if not arg.startswith('-g')]
    command.append('-g0')
    mappings = json.loads(args.renames.read_text(encoding='utf-8'))
    environment = dict(os.environ, SOURCE_DATE_EPOCH='0')

    def verify(source_profile):
        source, profile = source_profile
        results = []
        warnings = []
        for variant in ('baseline', 'current'):
            argv = [arg.replace('-DFOC_PROFILE_SENSORLESS=0', f'-DFOC_PROFILE_SENSORLESS={profile}')
                    for arg in command]
            path = source
            if variant == 'baseline':
                path = baseline / source.relative_to(root)
                argv = [arg.replace(str(root / 'user/inc').replace('\\', '/'),
                                    str(baseline / 'user/inc').replace('\\', '/')) for arg in argv]
            target = output / f'{source.stem}_{profile}_{variant}.o'
            process = subprocess.run(argv + ['-c', str(path), '-o', str(target)], cwd=root,
                                     env=environment, capture_output=True, text=True)
            if process.returncode:
                raise RuntimeError(f'{source.name} profile={profile} {variant}: {process.stderr}')
            if process.stderr.strip():
                warnings.append({'variant': variant, 'text': process.stderr.strip()})
            inverse = {new: old for old, new in mappings.get(source.name, {}).items()} if variant == 'current' else {}
            results.append(object_contents(target, inverse))
        changed = [key for key in results[0] if results[0][key] != results[1][key]]
        return {'source': source.name, 'profile_sensorless': profile, 'equivalent': not changed,
                'changed': changed, 'warnings': warnings}

    inputs = [(path, profile) for path in sorted((root / 'user/src').glob('*.c')) for profile in (0, 1)]
    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(verify, inputs))

    header_results = []
    for header in sorted((root / 'user/inc').glob('*.h')):
        if header.name == 'oled_font.h':
            continue
        probe = output / 'header_probe.c'
        probe.write_text(f'#include "{header.name}"\n', encoding='utf-8')
        process = subprocess.run(command + ['-Werror', '-fsyntax-only', str(probe)], cwd=root,
                                 capture_output=True, text=True)
        header_results.append({'header': header.name, 'passed': process.returncode == 0,
                               'diagnostic': process.stderr.strip()})
    passed = all(result['equivalent'] for result in results) and all(result['passed'] for result in header_results)
    args.report.write_text(json.dumps({'passed': passed, 'units': results, 'headers': header_results}, indent=2) + '\n',
                           encoding='utf-8')
    print(f'Checked {len(results)} C/profile pairs and {len(header_results)} standalone headers.')
    for result in results:
        if not result['equivalent']:
            print('FAIL:', result['source'], result['profile_sensorless'], result['changed'])
    for result in header_results:
        if not result['passed']:
            print('FAIL:', result['header'], result['diagnostic'])
    print('PASS' if passed else 'FAIL')
    raise SystemExit(not passed)


if __name__ == '__main__':
    main()
