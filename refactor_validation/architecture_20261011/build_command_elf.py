"""Retain command entrypoints for ARM emulation without changing the shipped firmware."""
from pathlib import Path
import json
import shlex
import subprocess

ROOT=Path(__file__).resolve().parents[2]
TEMP=ROOT/'.codex_tmp/architecture_validation'
BUILD=TEMP/'build'
commands=json.loads((BUILD/'compile_commands.json').read_text(encoding='utf-8'))
entry=next(c for c in commands if c['file'].endswith('/firmware/app/motor_system.c'))
argv=entry['command'].split()
for option in ['-c','-o']:
    index=argv.index(option);del argv[index:index+2]
line=subprocess.check_output(['D:/software/embedded-tools/Scripts/ninja.exe','-C',str(BUILD),'-t','commands','g431_foc.elf'],text=True).splitlines()[-1]
command=line.split('&&',1)[1].rsplit('&&',1)[0].strip()
link=shlex.split(command.replace('\\','/'))
index=link.index('-o');link[index+1]=str(TEMP/'current.elf')
link=[arg if not arg.startswith('-Wl,-Map=') else '-Wl,-Map='+str(TEMP/'current.map') for arg in link]
for name in ['motor_system_set_control_mode','motor_system_play_song','motor_system_play_song_loop',
             'motor_current_loop_set_params','motor_music_start_times','motor_songs_register_score']:
    link.append('-Wl,--undefined='+name)
subprocess.run(link,check=True,cwd=BUILD)
print('Built validation-only ARM firmware:',TEMP/'current.elf')
