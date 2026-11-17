"""Read the frozen bounded-run trace without halting, resetting or moving motors."""
import argparse
import ctypes as C
import hashlib
import json
import struct
import subprocess
from pathlib import Path
import monitor_can as monitor

U,F=C.c_uint32,C.c_float
REASONS=['confidence','tilt','forward_speed','body_angular_speed','pose','joint_speed','remote','sticks','health','sample_gap']
class TrialFrame(C.LittleEndianStructure):
    _fields_=[(n,U) for n in ('begin','tick','mode','inhibit','fault','entry_block','stable_ms','inference_us','loop_us')]+[
        ('motor_tick',U*6),('raw',C.c_uint16*6)]+[(n,F*6) for n in ('q','qd','motor_velocity','torque','feedback','action')]+[
        ('omega',F*3),('gravity',F*3),('confidence',F),('residual',F),('end',U)]
class Trace(C.LittleEndianStructure):
    _fields_=[(n,U) for n in ('schema','frame_bytes','capacity','published','overwritten','frozen')]+[
        ('entry_counts',U*10),('last_entry_block',U),('frame',TrialFrame*512)]

def decode(raw):
    if len(raw)!=C.sizeof(Trace): raise ValueError('Wrong trace size')
    t=Trace.from_buffer_copy(raw)
    if (t.schema,t.frame_bytes,t.capacity)!=(1,C.sizeof(TrialFrame),512): raise ValueError('Wrong trace ABI')
    rows=[]; rejected=[]
    for i,f in enumerate(t.frame):
        if not f.begin and not f.end: continue
        if f.begin!=f.end or f.begin&1: rejected.append(i); continue
        r={name:list(getattr(f,name)) if isinstance(getattr(f,name),C.Array) else getattr(f,name) for name,_ in f._fields_}
        r['motor_age_ms']=[(f.tick-x)&0xffffffff for x in f.motor_tick]
        rows.append(r)
    rows.sort(key=lambda r:(t.published*2-r['begin'])&0xffffffff,reverse=True)
    gaps=[(b['tick']-a['tick'])&0xffffffff for a,b in zip(rows,rows[1:])]
    active=[r for r in rows if r['mode']==3]
    return dict(schema=1,frame_bytes=t.frame_bytes,sample_count=len(rows),published=t.published,frozen=bool(t.frozen),
        rejected_slots=rejected,entry_counts=dict(zip(REASONS,t.entry_counts)),last_entry_block=t.last_entry_block,
        tick_gaps_ms=gaps,continuous_2ms=bool(gaps) and all(x==2 for x in gaps),
        span_ms=((rows[-1]['tick']-rows[0]['tick'])&0xffffffff) if rows else 0,
        active_samples=len(active),samples=rows,raw_sha256=hashlib.sha256(raw).hexdigest())

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--elf',type=Path,default=monitor.ROOT/'build/rl/wheel_leg_rl.elf')
    p.add_argument('--header-only',action='store_true')
    p.add_argument('--freeze',action='store_true',help='STOP-only diagnostic freeze through firmware mailbox')
    p.add_argument('--clear',action='store_true',help='STOP-only clear/restart recording; use after archiving')
    args=p.parse_args()
    if args.out.exists(): p.error('Use a fresh directory')
    elf=args.elf.resolve()
    symbols=monitor.elf_symbols(elf,('g_trial_trace','g_supported_trial','g_trace_transfer','g_boot_reset_flags','g_chassis','g_supported_session'))
    addr,size=symbols['g_trial_trace']
    if size!=C.sizeof(Trace): raise ValueError('ELF/decoder ABI mismatch')
    port=symbols['g_trace_transfer'][0]
    monitor.dump_ram(args.out,{'port':(port,92),'guard':symbols['g_supported_trial'],
        'reset_flags':symbols['g_boot_reset_flags'],'chassis':symbols['g_chassis'],
        'session':symbols['g_supported_session']},None,monitor.DEFAULT_OPENOCD,elf)
    initial=struct.unpack('<23I',(args.out/'a_port.bin').read_bytes())
    token=initial[0]+1
    prefix=['gdb_port disabled','tcl_port disabled','telnet_port disabled','init','targets stm32h723.cpu0']
    def run(commands,log):
        proc=subprocess.run(monitor.openocd_argv(monitor.DEFAULT_OPENOCD,prefix+commands+['shutdown']),
            cwd=monitor.ROOT,capture_output=True,text=True,timeout=120)
        (args.out/log).write_text(proc.stdout+proc.stderr,encoding='utf-8')
        proc.check_returncode()
    def request(command,offset,length,seq):
        return [f'mww {port+8:#x} {command}',f'mww {port+12:#x} {offset}',
                f'mww {port+16:#x} {length}',f'mww {port:#x} {seq}','sleep 20']
    def dump(name,address,length):
        return f'dump_image {{{(args.out/name).resolve().as_posix()}}} {address:#x} {length}'
    if args.freeze or args.clear:
        if args.freeze and args.clear: p.error('Choose only freeze or clear')
        run(request(3 if args.clear else 1,0,0,token)+[dump('port_after.bin',port,92)],'command.log')
        initial=struct.unpack('<23I',(args.out/'port_after.bin').read_bytes())
        if initial[1]!=token or initial[5]: raise RuntimeError(f'Diagnostic command rejected: {initial[:6]}')
        token+=1
    h=initial[6:23]
    report=dict(schema=h[0],frame_bytes=h[1],capacity=h[2],published=h[3],frozen=bool(h[5]),
                entry_counts=dict(zip(REASONS,list(h)[6:16])),last_entry_block=h[16])
    guard=(U*3).from_buffer_copy((args.out/'a_guard.bin').read_bytes()[:12])
    report['guard']=dict(zip(('started','running','reason'),guard))
    report['reset_flags']=hex(struct.unpack('<I',(args.out/'a_reset_flags.bin').read_bytes())[0])
    session=struct.unpack('<8I2f',(args.out/'a_session.bin').read_bytes())
    report['software_session']=dict(request=session[0],ack=session[1],accepted=session[2],status=session[3],
        permit=session[4],requested_wheel_cap_nm=session[8],applied_wheel_cap_nm=session[9])
    (args.out/'header.json').write_text(json.dumps(report,indent=2)+'\n')
    if not args.header_only:
        if not h[5]: raise RuntimeError('Trace is still live; will not read torn data or change freeze state')
        commands=[]; chunks=[]
        for offset in range(0,size,8192):
            length=min(8192,size-offset); label=f'chunk_{offset:06d}'
            commands+=request(2,offset,length,token)+[dump(label+'_ack.bin',port,24),dump(label+'.bin',port+92,length)]
            chunks.append((label,token)); token+=1
        run(commands,'transfer.log')
        for label,seq in chunks:
            ack=struct.unpack('<6I',(args.out/(label+'_ack.bin')).read_bytes())
            if ack[1]!=seq or ack[5]: raise RuntimeError(f'Invalid chunk acknowledgement: {label}: {ack}')
        raw=b''.join((args.out/(label+'.bin')).read_bytes() for label,_ in chunks)
        (args.out/'trace.bin').write_bytes(raw)
        result=decode(raw)
        result['guard']=report['guard']
        result['reset_flags']=report['reset_flags']
        result['software_session']=report['software_session']
        result['transport']='CPU copies retained AXI trace into DTCM, debugger reads only DTCM; all chunk acknowledgements checked'
        (args.out/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        print(json.dumps({k:result[k] for k in ('sample_count','span_ms','active_samples','continuous_2ms','guard','entry_counts')}))
    else: print(json.dumps(report))

if __name__=='__main__': main()
