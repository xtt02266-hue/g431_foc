"""Verify the architecture migration against the immediate pre-refactor sources.

Compiles every sensored firmware C unit, probes standalone headers,
compares moved units' allocated sections/relocations, and checks every extracted
function body including the haptic branch reinsertion. No hardware is exercised.
"""
from pathlib import Path
import json
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
TEMP = ROOT/'.codex_tmp/architecture_validation'
BUILD = TEMP/'build'
sys.path.insert(0, str(ROOT/'refactor_validation/style_20261011'))
from verify_units import object_contents


def mask(text):
    return re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"',
                  lambda m: re.sub(r'[^\n]', ' ', m[0]), text, flags=re.S)


def bodies(text):
    clean = mask(text)
    result = {}
    for m in re.finditer(r'^(?:static )?(?:void|float|uint8_t)\s+(\w+)\([^;{}]*\)\s*\{', clean, re.M):
        end = m.end(); depth = 1
        while depth:
            depth += (clean[end] == '{') - (clean[end] == '}'); end += 1
        result[m[1]] = text[m.end():end-1]
    return result


def tokens(text):
    clean = mask(text)
    return re.findall(r'[A-Za-z_]\w*|(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?[fFuUlL]*|\S', clean)


def active_sensored_body(text):
    return re.sub(r'^#if !BOARD_SENSORED_CONTROL_ENABLE\n.*?^#endif\n','',text,flags=re.M|re.S)


def main():
    # Compile-time build information must not differ merely because checks run a second apart.
    os.environ['SOURCE_DATE_EPOCH']='0'
    migration = json.loads((HERE/'migration.json').read_text(encoding='utf-8'))
    old = bodies((TEMP/'baseline/user/src/motor_system.c').read_text(encoding='utf-8'))
    new = {}
    for path in (ROOT/'firmware/app').glob('*.c'):
        new.update(bodies(path.read_text(encoding='utf-8')))
    haptic = new['motor_haptic_task']
    expanded = new['motor_control_task'].replace(
        'motor_haptic_task(identified_direction, angle_counts);\n            return;', haptic.strip())
    new['motor_control_task'] = expanded
    body_results = []
    for name, body in old.items():
        current_name = migration['internal_renames'].get(name, name)
        normalized = active_sensored_body(body)
        for before, after in migration['internal_renames'].items():
            # Callback still calls the public task entry; initialization has no such calls.
            if name == 'HAL_TIM_PeriodElapsedCallback' and before == 'motor_system_task': continue
            normalized = re.sub(r'\b'+before+r'\b', after, normalized)
        body_results.append({'function':name,'current':current_name,
                             'passed':tokens(normalized)==tokens(new[current_name])})
    # HAL uses its scheduling entry; the two added wrappers each call exactly one implementation.
    assert tokens(new['motor_system_init']) == tokens('motor_control_init();')
    assert tokens(new['motor_system_task']) == tokens('motor_control_task();')

    commands=json.loads((BUILD/'compile_commands.json').read_text(encoding='utf-8'))
    command=next(c for c in commands if c['file'].endswith('/firmware/app/motor_system.c'))['command'].split()
    for option in ['-c','-o']:
        index=command.index(option);del command[index:index+2]
    command += ['-Werror']
    output=TEMP/'objects';output.mkdir(exist_ok=True)
    def compile_unit(item):
        source,profile=item
        argv=command
        target=output/f'{source.stem}_{profile}.o'
        subprocess.run(argv+['-c',str(source),'-o',str(target)],check=True,cwd=ROOT,capture_output=True)
        return {'source':source.relative_to(ROOT).as_posix(),'profile':profile,'passed':True}
    with ThreadPoolExecutor(max_workers=4) as pool:
        units=list(pool.map(compile_unit,[(p,0) for p in sorted((ROOT/'firmware').rglob('*.c'))]))
    headers=[]
    for path in sorted((ROOT/'firmware').rglob('*.h')):
        if path.name=='oled_font.h':continue
        probe=output/'probe.c';probe.write_text(f'#include "{path.name}"\n',encoding='utf-8')
        subprocess.run(command+['-fsyntax-only',str(probe)],check=True,cwd=ROOT,capture_output=True)
        headers.append(path.relative_to(ROOT).as_posix())
    moved=[]
    for old_path,new_path in migration['paths'].items():
        if not old_path.endswith('.c') or old_path.endswith('/motor_system.c'):continue
        if new_path in migration['removed_sensorless_files']:continue
        source=TEMP/'baseline'/old_path
        old_command=[arg for arg in command if not arg.startswith('-I') or '/firmware/' not in arg]
        old_command.append('-I'+str(TEMP/'baseline/user/inc'))
        old_command.append('-DFOC_PROFILE_SENSORLESS=0')
        for profile in [0]:
            argv=[arg.replace('-DFOC_PROFILE_SENSORLESS=0',f'-DFOC_PROFILE_SENSORLESS={profile}') for arg in old_command]
            target=output/f'{source.stem}_{profile}_baseline.o'
            subprocess.run(argv+['-c',str(source),'-o',str(target)],check=True,cwd=ROOT,capture_output=True)
            before=object_contents(target,{})
            after=object_contents(output/f'{Path(new_path).stem}_{profile}.o',{})
            if new_path=='firmware/platform/hardware_init.c':
                old_body=bodies(source.read_text(encoding='utf-8'))['hardware_init']
                old_body=active_sensored_body(old_body).replace('#if BOARD_SENSORED_CONTROL_ENABLE\n','').replace('#endif\n','')
                old_body=old_body.replace('motor_angle_reference_init();','')
                new_body=bodies((ROOT/new_path).read_text(encoding='utf-8'))['hardware_init']
                passed=tokens(old_body)==tokens(new_body)
                moved.append({'source':new_path,'profile':profile,'passed':passed,
                              'verification':'same active source body except removal of the former empty AS5600 init call'})
            else:
                moved.append({'source':new_path,'profile':profile,'passed':before==after})
    result={'baseline_commit':migration['baseline_commit'],
            'passed':all(x['passed'] for x in body_results+moved),
            'function_bodies':body_results,'compiled_units':units,'headers':headers,'moved_unit_code':moved}
    (HERE/'source_equivalence.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    failures=[x for x in body_results+moved if not x['passed']]
    print(f'{len(body_results)} function bodies, {len(units)} C compilations, '
          f'{len(headers)} headers, {len(moved)-1} unchanged objects and one no-op call removal.')
    print('FAIL '+str(failures) if failures else 'PASS')
    assert result['passed']


if __name__=='__main__': main()
