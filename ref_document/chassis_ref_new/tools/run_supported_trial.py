"""Explicitly start one bounded trial; physical S1 middle and fresh healthy state required.

Only writes the diagnostic wheel cap (0..2 Nm) and a one-shot request token.
Never fabricates remote input, bypasses health, resets the board or clears a trace.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import monitor_can as m

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--wheel-cap',type=float,required=True);a=p.parse_args()
    if not 0<=a.wheel_cap<=2: p.error('wheel cap must be 0..2 Nm')
    if a.out.exists():p.error('Use a fresh evidence directory')
    elf=m.ROOT/'build/rl/wheel_leg_rl.elf'
    sy=m.elf_symbols(elf,('g_supported_session','g_supported_trial','g_chassis','g_remote','g_trace_transfer'))
    sy['g_trace_transfer']=(sy['g_trace_transfer'][0],92)
    m.dump_ram(a.out/'before',sy,None,m.DEFAULT_OPENOCD,elf)
    state=struct.unpack('<10I',(a.out/'before/a_g_chassis.bin').read_bytes())
    remote=(a.out/'before/a_g_remote.bin').read_bytes()
    # Use the same remote decoder ABI as monitor_can.
    channels=struct.unpack_from('<5h',remote)
    session=struct.unpack('<8I2f',(a.out/'before/a_g_supported_session.bin').read_bytes())
    port=struct.unpack('<23I',(a.out/'before/a_g_trace_transfer.bin').read_bytes())
    if remote[10]!=3 or any(abs(x)>30 for x in channels[:4]):raise RuntimeError('Physical S1 middle and centered sticks required')
    if state[8] not in (0,4) or (state[8]==4 and state[2]!=6):raise RuntimeError('Not stopped or bounded-stop state')
    if port[11]:raise RuntimeError('Previous trace frozen: archive/review it before clearing for another trial')
    token=session[1]+1;addr=sy['g_supported_session'][0]
    capword=struct.unpack('<I',struct.pack('<f',a.wheel_cap))[0]
    commands=['gdb_port disabled','tcl_port disabled','telnet_port disabled','init','targets stm32h723.cpu0',
              f'mww {addr+32:#x} {capword:#x}',f'mww {addr:#x} {token}','sleep 700']
    a.out.mkdir(parents=True,exist_ok=True)
    for name,(address,size) in sy.items():
        commands.append(f'dump_image {{{(a.out/(name+".bin")).resolve().as_posix()}}} {address:#x} {size}')
    commands+=['shutdown']
    proc=subprocess.run(m.openocd_argv(m.DEFAULT_OPENOCD,commands),cwd=m.ROOT,capture_output=True,text=True,timeout=30)
    (a.out/'trigger.log').write_text(proc.stdout+proc.stderr,encoding='utf-8');proc.check_returncode()
    after=struct.unpack('<8I2f',(a.out/'g_supported_session.bin').read_bytes())
    chassis=struct.unpack('<10I',(a.out/'g_chassis.bin').read_bytes())
    guard=struct.unpack('<3I6f',(a.out/'g_supported_trial.bin').read_bytes())
    result=dict(request=token,ack=after[1],accepted_total=after[2],accepted_this_request=after[2]==session[2]+1,
        status=after[3],permit=after[4],requested_wheel_cap_nm=after[8],applied_wheel_cap_nm=after[9],
        mode=chassis[8],fault=chassis[2],inhibit=chassis[1],guard_started=guard[0],guard_reason=guard[2],
        firmware_bin_sha256=hashlib.sha256(elf.with_suffix('.bin').read_bytes()).hexdigest())
    (a.out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    if after[1]!=token:raise RuntimeError('No request acknowledgement')
    if chassis[8] in (2,3):raise RuntimeError('Unexpected active mode after bounded interval; inspect immediately')

if __name__=='__main__':main()
