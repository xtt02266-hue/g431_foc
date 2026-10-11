"""Link validation-only firmware images retaining otherwise unused command entrypoints."""
from pathlib import Path
import json
import re
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT/'.codex_tmp/style_validation/build'
TEMP = BUILD.parent
NINJA = 'D:/software/embedded-tools/Scripts/ninja.exe'
renames = json.loads((Path(__file__).parent/'public_renames.json').read_text())
commands = json.loads((BUILD/'compile_commands.json').read_text())
entry = next(item for item in commands if item['file'].endswith('/Core/Src/main.c'))
argv = entry['command'].split()
for option in ('-o', '-c'):
    offset = argv.index(option); del argv[offset:offset+2]
source = (ROOT/'Core/Src/main.c').read_text(encoding='utf-8')
inverse = {new: old for old,new in renames.items()}
source = re.sub(r'\b('+'|'.join(map(re.escape,inverse))+r')\b',lambda m:inverse[m[0]],source)
old_main = TEMP/'baseline_main.c'
old_main.write_text(source,encoding='utf-8')
argv = [item.replace(str(ROOT/'user/inc').replace('\\','/'),str(TEMP/'baseline/user/inc').replace('\\','/')) for item in argv]
subprocess.run(argv+['-c',str(old_main),'-o',str(TEMP/'baseline_main.o')],check=True,cwd=ROOT)
line = subprocess.check_output([NINJA,'-C',str(BUILD),'-t','commands','g431_foc.elf'],text=True).splitlines()[-1]
command = line.split('&&',1)[1].rsplit('&&',1)[0].strip()
# Quotes only surround the linker script, and no selected path has spaces.
link = shlex.split(command.replace('\\','/'))
forced = ['Motor_System_SetControlMode','Motor_System_PlaySong','Motor_System_PlaySongLoop',
          'Motor_CurrentLoop_SetParams','Motor_Music_StartTimes','Motor_Sensorless_Configure']
for variant in ('baseline','current'):
    values=[]
    for item in link:
        if item.startswith('-Wl,-Map='):
            values.append('-Wl,-Map='+str(TEMP/f'commands_{variant}.map')); continue
        if item.endswith('.obj') and '/user/src/' in item:
            stem=Path(item).name.removesuffix('.c.obj')
            values.append(str(TEMP/'objects'/f'{stem}_0_{variant}.o')); continue
        if item.endswith('/Core/Src/main.c.obj') and variant=='baseline':
            values.append(str(TEMP/'baseline_main.o')); continue
        values.append(item)
    offset=values.index('-o'); values[offset+1]=str(TEMP/f'commands_{variant}.elf')
    values.append(str(TEMP/'objects'/f'motor_sensorless_0_{variant}.o'))
    values.extend('-Wl,--undefined='+ (renames.get(name,name) if variant=='current' else name) for name in forced)
    if variant == 'current':
        values.append('-Wl,--undefined=motor_songs_register_score')
    subprocess.run(values,check=True,cwd=BUILD)
    print('Linked validation image:',variant)
