#!/usr/bin/env python3
"""Flash the single RL image; --dry-run never opens the debug probe."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--openocd',type=Path,default=Path.home()/'.eide/tools/openocd_7a1adfbec_mingw32/bin/openocd.exe')
    p.add_argument('--config',type=Path,default=ROOT/'openocd_dap.cfg')
    p.add_argument('--dry-run',action='store_true')
    a=p.parse_args()
    exe=a.openocd.resolve(); scripts=exe.parent.parent/'share/openocd/scripts'
    config=(ROOT/a.config).resolve(); elf=ROOT/'build/rl/wheel_leg_rl.elf'; binary=elf.with_suffix('.bin')
    for required in (exe,config,elf,binary):
        if not required.is_file(): raise SystemExit(f'Missing: {required}')
    if not scripts.is_dir(): raise SystemExit(f'Missing OpenOCD scripts: {scripts}')
    payload=binary.read_bytes()
    if len(payload)<256: raise SystemExit('Invalid firmware: missing vector table')
    stack,entry=struct.unpack_from('<II',payload)
    if not 0x20000000<stack<=0x20020000 or not entry&1 or not 0x08000000<entry<0x08000000+len(payload):
        raise SystemExit('Invalid firmware stack/reset vector')
    config_text=(ROOT/'user/robot_config.h').read_text()
    flags={k:int(v) for k,v in re.findall(r'#define ROBOT_(ENABLE_OUTPUT|MOTOR_CALIBRATED|IMU_FRAME_VERIFIED|POLICY_CONTRACT_VERIFIED|ODOMETRY_VERIFIED) ([01])',config_text)}
    command=[str(exe),'-s',str(scripts),'-f',str(config),'-c','gdb_port disabled',
             '-c','tcl_port disabled','-c','telnet_port disabled',
             '-c',f'program {{{elf.as_posix()}}} verify reset','-c','shutdown']
    report={'dry_run':a.dry_run,'image':elf.relative_to(ROOT).as_posix(),
            'binary_sha256':hashlib.sha256(payload).hexdigest(),'production_flags':flags,'command':command}
    print(json.dumps(report,indent=2),flush=True)
    if a.dry_run: return
    stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    out=ROOT/'diagnostics'/('flash_'+stamp); out.mkdir(parents=True,exist_ok=True)
    with (out/'openocd.log').open('w',encoding='utf-8') as log:
        process=subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        for line in process.stdout:
            print(line,end='',flush=True); log.write(line); log.flush()
        code=process.wait()
    report['exit_code']=code
    (out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8',newline='\n')
    raise SystemExit(code)
if __name__=='__main__': main()
