"""Compare all runtime ELF sections, including load bytes and zero-filled RAM layout."""
import argparse
import hashlib
import json
import re
from pathlib import Path
import struct


def runtime_sections(path, allow_build_timestamp=False):
    data = Path(path).read_bytes()
    if data[:6] != b'\x7fELF\x01\x01':
        raise ValueError('Expected a little-endian ELF32 firmware')
    header = struct.unpack_from('<16sHHIIIIIHHHHHH', data)
    offset, size, count, names_index = header[6], header[11], header[12], header[13]
    headers = [struct.unpack_from('<10I', data, offset + i * size) for i in range(count)]
    names_header = headers[names_index]
    names = data[names_header[4]:names_header[4] + names_header[5]]
    result = {}
    for section in headers:
        name_offset, kind, flags, address, file_offset, length = section[:6]
        if not flags & 2:
            continue
        end = names.index(b'\0', name_offset)
        name = names[name_offset:end].decode('ascii')
        contents = b'' if kind == 8 else data[file_offset:file_offset + length]
        ignored = []
        if allow_build_timestamp and name == '.rodata':
            timestamp = re.compile(rb'[A-Z][a-z]{2} [ 0-9][0-9] [0-9]{4} [0-9]{2}:[0-9]{2}:[0-9]{2}\x00')
            matches = list(timestamp.finditer(contents))
            if len(matches) != 1:
                raise ValueError('Expected exactly one protocol build timestamp in .rodata')
            match = matches[0]
            ignored.append({'offset': match.start(), 'size': match.end() - match.start()})
            contents = contents[:match.start()] + bytes(match.end() - match.start()) + contents[match.end():]
        result[name] = {
            'address': address, 'size': length, 'type': kind, 'flags': flags,
            'alignment': section[8], 'sha256': hashlib.sha256(contents).hexdigest(),
            'ignored_build_timestamp': ignored,
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('before', type=Path)
    parser.add_argument('after', type=Path)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--allow-build-timestamp', action='store_true',
                        help='Normalize only the single __DATE__ __TIME__ protocol string in .rodata')
    args = parser.parse_args()
    before = runtime_sections(args.before, args.allow_build_timestamp)
    after = runtime_sections(args.after, args.allow_build_timestamp)
    changed = [name for name in sorted(before.keys() | after.keys())
               if before.get(name) != after.get(name)]
    result = {'equivalent': not changed, 'changed_sections': changed,
              'before': before, 'after': after}
    if args.report:
        args.report.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(('PASS' if not changed else 'FAIL') + ': ' + str(len(before)) +
          ' runtime sections; changed=' + ','.join(changed))
    raise SystemExit(bool(changed))


if __name__ == '__main__':
    main()
