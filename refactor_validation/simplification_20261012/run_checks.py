"""Build/test the safety-fixed snapshot and the current firmware; outputs stay ignored.

--capture creates an immutable local snapshot before structural edits. Existing
command/runtime/session cases are reused with identical modern API names on both
sides. No assertions or numeric tolerances in those suites are removed.
"""
from pathlib import Path
import argparse
import io
import json
import os
import re
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[2]
TEMP = ROOT / '.codex_tmp/simplification_20261012'
BASE = TEMP / 'safety_baseline'
STYLE = ROOT / 'refactor_validation/style_20261011'
ARCH = ROOT / 'refactor_validation/architecture_20261011'
sys.path[:0] = [str(STYLE), str(ARCH)]


def run(argv, cwd=ROOT):
    result = subprocess.run([str(a) for a in argv], cwd=cwd, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if result.returncode:
        print(result.stdout)
        raise subprocess.CalledProcessError(result.returncode, argv)
    return result.stdout


def build(source, name, retain=True):
    build_dir = TEMP / name
    symbols = set()
    if retain:
        for path in (source / 'firmware').rglob('*.h'):
            symbols.update(re.findall(r'\b((?:motor_|pc_protocol_|pid_|svpwm_)\w+)\s*\(',
                                      path.read_text(encoding='utf-8')))
    retain_file = TEMP / (name + '_retain.cmake')
    response_file = TEMP / (name + '_retain.rsp')
    response_file.write_text('\n'.join('-Wl,--undefined=' + s for s in sorted(symbols)) + '\n')
    retain_file.write_text('add_link_options("@' + response_file.as_posix() + '")\n')
    run(['cmake', '-S', source, '-B', build_dir, '-G', 'Ninja',
         '-DCMAKE_TOOLCHAIN_FILE=' + str(source / 'cmake/gcc-arm-none-eabi.cmake'),
         '-DCMAKE_BUILD_TYPE=Debug', '-DCMAKE_PROJECT_g431_foc_INCLUDE=' + str(retain_file)])
    output = run(['cmake', '--build', build_dir])
    (TEMP / (name + '.log')).write_text(output, encoding='utf-8')
    print(output.strip().splitlines()[-4:])
    return build_dir / 'g431_foc.elf'


def capture():
    if BASE.exists():
        raise RuntimeError('Safety baseline already exists; do not overwrite it.')
    BASE.mkdir(parents=True)
    archive = subprocess.check_output(['git', 'archive', '--format=zip', 'HEAD'], cwd=ROOT)
    with zipfile.ZipFile(io.BytesIO(archive)) as files:
        files.extractall(BASE)
    # Copy the reviewed safety fix over the pinned pre-change tree.
    for path in (ROOT / 'firmware').rglob('*'):
        if path.is_file():
            target = BASE / path.relative_to(ROOT)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(path.read_bytes())
    (TEMP / 'baseline_commit.txt').write_text(run(['git', 'rev-parse', 'HEAD']).strip() + '\n')


def suites(baseline, current):
    sys.argv = ['verify_commands.py', '--baseline', str(baseline), '--current', str(current), '--sensored-only']
    text = (STYLE / 'verify_commands.py').read_text(encoding='utf-8')
    text = text.replace('Firmware(args.baseline, False, renames)', 'Firmware(args.baseline, True, renames)')
    # The old suite compared former struct arguments with today's scalar APIs.
    # Both snapshots now use the same scalar ABI, so send current arguments to both.
    text = text.replace('[(baseline, baseline_args), (current, current_args)]',
                        '[(baseline, current_args), (current, current_args)]')
    sys.argv += ['--report', str(TEMP / 'commands.json')]
    exec(compile(text, str(STYLE / 'verify_commands.py'), 'exec'),
         {'__name__': '__main__', '__file__': str(STYLE / 'verify_commands.py')})
    for filename in ['verify_runtime.py', 'verify_calibration_sessions.py']:
        text = (ARCH / filename).read_text(encoding='utf-8')
        text = text.replace("TEMP/'baseline.elf'", repr(str(baseline)))
        text = text.replace("TEMP/'current.elf'", repr(str(current)))
        text = text.replace("'motor_control_update_state' if f is current else 'motor_system_update_operating_state'",
                            "'motor_control_update_state'")
        text = text.replace("(HERE/'runtime_equivalence.json')", "(Path(" + repr(str(TEMP / 'runtime.json')) + "))")
        text = text.replace("(HERE/'calibration_session_equivalence.json')",
                            "(Path(" + repr(str(TEMP / 'sessions.json')) + "))")
        text = text.replace('against 2f6fb3d', 'against the safety-fixed snapshot')
        text = text.replace("'baseline_commit':'2f6fb3d022037055b2bc99174480761adcf9c91a'",
                            "'baseline_kind':'local safety-fixed source snapshot'")
        # Constructors expect Path objects, rather than strings.
        text = text.replace(repr(str(baseline)), 'Path(' + repr(str(baseline)) + ')')
        text = text.replace(repr(str(current)), 'Path(' + repr(str(current)) + ')')
        exec(compile(text, str(ARCH / filename), 'exec'),
             {'__name__': '__main__', '__file__': str(ARCH / filename)})


def compile_all():
    entries = json.loads((TEMP / 'current/compile_commands.json').read_text())
    argv = next(e['command'].split() for e in entries if e['file'].endswith('/firmware/app/motor_system.c'))
    for option in ['-c', '-o']:
        index = argv.index(option)
        del argv[index:index + 2]
    for path in (ROOT / 'firmware').rglob('*.c'):
        run(argv + ['-Werror', '-fsyntax-only', path])
    probe = TEMP / 'probe.c'
    for path in (ROOT / 'firmware').rglob('*.h'):
        if path.name == 'oled_font.h':
            continue
        probe.write_text('#include "' + path.name + '"\n', encoding='utf-8')
        run(argv + ['-Werror', '-fsyntax-only', probe])
    print('PASS: all project C units and standalone headers (-Werror).')


def stack_usage():
    observations = []
    for version, source in [('baseline', BASE), ('current', ROOT)]:
        entries = json.loads((TEMP / version / 'compile_commands.json').read_text())
        argv = next(e['command'].split() for e in entries if e['file'].endswith('/firmware/app/motor_system.c'))
        for option in ['-c', '-o']:
            index = argv.index(option)
            del argv[index:index + 2]
        for relative in ['app/motor_runtime.c', 'communication/pc_protocol.c', 'calibration/motor_calibration.c',
                         'calibration/motor_encoder_calibration.c', 'app/motor_calibration_session.c',
                         'app/motor_control.c', 'calibration/motor_parameters.c']:
            obj = TEMP / (version + '_' + Path(relative).stem + '_stack.o')
            run(argv + ['-Werror', '-fstack-usage', '-c', source / 'firmware' / relative, '-o', obj])
            for line in obj.with_suffix('.su').read_text().splitlines():
                location, size, kind = line.split('\t')
                observations.append({'version': version, 'function': location.rsplit(':', 1)[-1],
                                     'frame_bytes': int(size), 'kind': kind})
    (TEMP / 'stack_frames.json').write_text(json.dumps(observations, indent=2) + '\n')
    print('PASS: compiler stack-frame report saved locally (not worst-case interrupt stack).')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', action='store_true')
    parser.add_argument('--build-only', action='store_true')
    parser.add_argument('--rebuild-baseline', action='store_true')
    parser.add_argument('--production', action='store_true')
    parser.add_argument('--stack', action='store_true')
    args = parser.parse_args()
    os.environ['SOURCE_DATE_EPOCH'] = '0'
    TEMP.mkdir(parents=True, exist_ok=True)
    if args.capture:
        capture()
    if args.production:
        build(ROOT, 'production', retain=False)
        sys.exit(0)
    baseline = TEMP / 'baseline/g431_foc.elf'
    if not baseline.exists() or args.rebuild_baseline:
        baseline = build(BASE, 'baseline')
    current = build(ROOT, 'current')
    if args.stack:
        stack_usage()
    if not args.build_only:
        compile_all()
        suites(baseline, current)
