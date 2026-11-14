#!/usr/bin/env python3
"""Inspect H723 CAN2 hardware FIFO while CPU is already halted.

Reads message RAM and acknowledges consumed RX slots only. No Flash write,
CPU resume/reset, CAN TX request or motor enable is issued. Addresses verified
against Drivers/CMSIS/.../stm32h723xx.h (CAN2 0x4000a400, SRAMCAN 0x4000ac00).
"""
import argparse
import datetime
import json
from pathlib import Path
import struct
import subprocess
ROOT=Path(__file__).resolve().parents[1]
CAN2=0x4000a400
RAM=0x4000ac00
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rounds',type=int,default=4)
    a=p.parse_args()
    if not 2<=a.rounds<=8: p.error('rounds must be 2..8')
    stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    out=ROOT/'diagnostics'/('fifo_can_'+stamp); out.mkdir(parents=True,exist_ok=True)
    exe=Path.home()/'.eide/tools/openocd_7a1adfbec_mingw32/bin/openocd.exe'
    def run(commands,name):
        args=[str(exe),'-s',str(exe.parent.parent/'share/openocd/scripts'),'-f','openocd_dap.cfg']
        for command in ['gdb_port disabled','tcl_port disabled','telnet_port disabled','init','targets stm32h723.cpu0',*commands,'shutdown']:
            args+=['-c',command]
        r=subprocess.run(args,cwd=ROOT,capture_output=True,text=True,timeout=35)
        (out/name).write_text(r.stdout+r.stderr,encoding='utf-8')
        if r.returncode: raise RuntimeError((r.stdout+r.stderr)[-3000:])
    def dump(path,address,size): return f'dump_image {{{path.as_posix()}}} {address:#x} {size}'
    run([dump(out/'config.bin',CAN2,256),dump(out/'dhcsr.bin',0xe000edf0,4)],'config.log')
    data=(out/'config.bin').read_bytes(); config=struct.unpack_from('<I',data,0xa0)[0]
    cccr=struct.unpack_from('<I',data,0x18)[0]
    halted=bool(struct.unpack('<I',(out/'dhcsr.bin').read_bytes())[0]&(1<<17))
    size=(config>>16)&0x7f; start=RAM+(config&0xfffc)
    rxesc=struct.unpack_from('<I',data,0xbc)[0]&7
    payload=(8,12,16,20,24,32,48,64)[rxesc]; stride=8+payload
    if not halted: raise SystemExit('CPU is running; do not race its FIFO consumer')
    if cccr&1 or not 1<=size<=64 or not RAM<=start<start+size*stride<=RAM+10240:
        raise SystemExit(f'CAN2 FIFO not configured/active: CCCR={cccr:#x}, RXF0C={config:#x}')
    commands=[]
    for i in range(a.rounds):
        commands += [f'sleep 300',dump(out/f'reg_{i}.bin',CAN2,256),dump(out/f'fifo_{i}.bin',start,size*stride),
          f'mem2array rx 32 {CAN2+0xa4:#x} 1',
          f'if {{($rx(0)&127)>0}} {{mww {CAN2+0xa8:#x} [expr {{((($rx(0)>>8)&63)+($rx(0)&127)-1)%{size}}}]}}']
    run(commands,'capture.log')
    rounds=[]; fresh={}
    for i in range(a.rounds):
        regs=(out/f'reg_{i}.bin').read_bytes(); status=struct.unpack_from('<I',regs,0xa4)[0]
        fill=status&127; get=(status>>8)&63; raw=(out/f'fifo_{i}.bin').read_bytes(); frames=[]
        for j in range(min(fill,size)):
            slot=(get+j)%size; offset=slot*stride; w0,w1=struct.unpack_from('<II',raw,offset)
            if w0&(1<<29) or w0&(1<<30): continue
            canid=(w0>>18)&0x7ff; dlc=(w1>>16)&15
            frame={'id':hex(canid),'dlc':dlc,'data':raw[offset+8:offset+8+min(dlc,8)].hex(' ')}
            frames.append(frame)
            if i>0: fresh[hex(canid)]=fresh.get(hex(canid),0)+1
        rounds.append({'round':i,'fill':fill,'get':get,'ecr':hex(struct.unpack_from('<I',regs,0x40)[0]),
                       'psr':hex(struct.unpack_from('<I',regs,0x44)[0]),'frames':frames})
    expected=('0x301','0x302','0x303','0x304','0x202','0x203')
    report={'cpu_was_halted':halted,'can2_base':hex(CAN2),'fifo_address':hex(start),'fifo_size':size,
            'all_six_fresh':all(fresh.get(k,0)>0 for k in expected),'fresh_counts':fresh,
            'expected_ids':expected,'rounds':rounds,'cpu_resumed':False,'flash_written':False,'can_tx_requested':False}
    (out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='rounds'},indent=2)); print('Evidence:',out/'result.json')
if __name__=='__main__': main()
