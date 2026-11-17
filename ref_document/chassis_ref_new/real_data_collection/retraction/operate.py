"""Explicit hardware operations for the independent suspended retraction image."""
from pathlib import Path
import argparse,datetime,hashlib,json,re,shutil,struct,subprocess,sys
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'tools'))
import monitor_can as m
ELF=ROOT/'build/retraction/suspended_retraction.elf'
def audit_flash(raw,elf):
    """Compare ELF load sections. Unprogrammed alignment gaps may remain 0xff."""
    lines=subprocess.run(['arm-none-eabi-objdump','-h',str(elf)],capture_output=True,text=True,check=True).stdout.splitlines()
    payload=elf.read_bytes();sections=[]
    for i,line in enumerate(lines[:-1]):
        match=re.match(r'\s*\d+\s+(\S+)\s+([\da-f]+)\s+([\da-f]+)\s+([\da-f]+)\s+([\da-f]+)\s+',line)
        if not match or not all(x in lines[i+1] for x in ('CONTENTS','ALLOC','LOAD')):continue
        name,size,_,load,offset=match.groups();size=int(size,16);load=int(load,16);offset=int(offset,16)
        if not 0x08000000<=load<0x08100000:continue
        start=load-0x08000000
        sections.append({'name':name,'address':hex(load),'bytes':size,
            'match':len(raw)>=start+size and raw[start:start+size]==payload[offset:offset+size]})
    return {'load_sections':sections,'all_load_sections_match':bool(sections) and all(s['match'] for s in sections),
            'raw_sha256':hashlib.sha256(raw).hexdigest(),'alignment_gaps_excluded':True}
def save(p,v):p.write_text(json.dumps(v,indent=2)+'\n',encoding='utf-8')
def run(out,commands,timeout=90):
    prefix=['gdb_port disabled','tcl_port disabled','telnet_port disabled','init','targets stm32h723.cpu0']
    p=subprocess.run(m.openocd_argv(m.DEFAULT_OPENOCD,prefix+commands+['shutdown']),cwd=ROOT,capture_output=True,text=True,timeout=timeout)
    (out/'operation.log').write_text(p.stdout+p.stderr,encoding='utf-8');p.check_returncode()
    if 'Error:' in p.stdout+p.stderr:raise RuntimeError('OpenOCD reported an error; inspect saved log')
def main():
    p=argparse.ArgumentParser();p.add_argument('operation',choices=['flash','status','trigger']);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    if a.out.exists():p.error('Fresh output directory required')
    out=a.out.resolve();out.mkdir(parents=True)
    manifest=json.loads((HERE/'manifest.json').read_text())
    assert hashlib.sha256(ELF.with_suffix('.bin').read_bytes()).hexdigest()==manifest['bin_sha256']
    if a.operation=='flash':
        normal=ROOT/'build/rl/wheel_leg_rl.elf'
        before=m.snapshot(out/'before',1,elf=normal);save(out/'before/result.json',before)
        b=before['b'];assert b['remote']['s1']==2 and b['remote']['valid'] and b['chassis']['mode']==0
        assert before['a']['nonzero_torque_frames']==b['nonzero_torque_frames']
        for ext in ('.elf','.bin'):shutil.copy2(normal.with_suffix(ext),out/('restore_rl'+ext));shutil.copy2(ELF.with_suffix(ext),out/('diagnostic'+ext))
        for f in HERE.iterdir():
            if f.is_file() and f.suffix in ('.c','.h','.py','.json','.log'):shutil.copy2(f,out/f.name)
        sy=m.elf_symbols(normal,('g_chassis','g_remote'))
        run(out,[f'set r [read_memory {sy["g_remote"][0]:#x} 8 28]',f'set c [read_memory {sy["g_chassis"][0]:#x} 32 10]',
            'if {[lindex $r 10] != 2 || [lindex $r 24] != 1 || [lindex $c 8] != 0} {error "S1 down/STOP required"}',
            'reset halt','wait_halt 2000',f'dump_image {{{(out/"preflash.bin").as_posix()}}} 0x08000000 {normal.with_suffix(".bin").stat().st_size}',
            f'if {{[catch {{verify_image {{{normal.as_posix()}}}}} verify_error]}} {{resume; error $verify_error}}',
            f'program {{{ELF.as_posix()}}} verify reset'])
        backup_audit=audit_flash((out/'preflash.bin').read_bytes(),normal)
        save(out/'backup_audit.json',backup_audit)
        assert backup_audit['all_load_sections_match']
        manifest.update(flashed=True,flash_verified=True,physical_result=None,time=datetime.datetime.now().isoformat())
        save(out/'session.json',manifest)
        report=m.snapshot(out/'after',1,elf=ELF);save(out/'after/result.json',report)
        assert report['b']['nonzero_torque_frames']==0 and report['b']['enable_commands']==0
        assert report['b']['chassis']['mode']==0
        print(json.dumps({'flashed':True,'mode':report['b']['chassis']['mode'],'s1':report['b']['remote']['s1'],'nonzero':0,'enables':0,'evidence':str(out)}))
    elif a.operation=='status':
        report=m.snapshot(out,1,elf=ELF)
        for sample in ('a','b'):
            report[sample]['joint_measurement_may_be_stale']=bool(report[sample]['chassis']['inhibit']&4)
        save(out/'result.json',report)
        b=report['b'];display={k:b[k] for k in ('chassis','remote','standing_entry','measurement','nonzero_torque_frames','enable_commands','tx_failed')}
        if b['joint_measurement_may_be_stale']:
            display['measurement']=None
            display['measurement_warning']='Motor health inhibit: cached whole-body q/qd may be stale. Inspect raw motor feedback; do not interpret cached velocity as current motion.'
        print(json.dumps(display))
    else:
        save(out/'manifest.json',manifest)
        sy=m.elf_symbols(ELF,('g_chassis','g_remote','g_supported_session','g_supported_trial','g_trace_transfer','g_retraction'))
        sy['g_trace_transfer']=(sy['g_trace_transfer'][0],92)
        m.dump_ram(out/'before',sy,None,m.DEFAULT_OPENOCD,ELF)
        rd=lambda n:(out/'before'/('a_'+n+'.bin')).read_bytes()
        c=struct.unpack('<10I',rd('g_chassis'));r=rd('g_remote');s=struct.unpack('<8I2f',rd('g_supported_session'));h=struct.unpack('<23I',rd('g_trace_transfer'))
        assert r[10]==3 and r[24] and all(abs(v)<=30 for v in struct.unpack_from('<4h',r))
        assert c[8]==0 or (c[8]==4 and c[2]==6)
        assert not c[1] and s[5] and not s[4] and not h[11]
        token=(s[1]+1)&0xffffffff
        wait_ms=int(manifest['ramp_ms']+manifest['hold_ms']+manifest['release_ms']+600)
        run(out,[f'mww {sy["g_supported_session"][0]:#x} {token}',f'sleep {wait_ms}']+[
            f'dump_image {{{(out/(name+".bin")).as_posix()}}} {addr:#x} {size}' for name,(addr,size) in sy.items()],30)
        after=struct.unpack('<8I2f',(out/'g_supported_session.bin').read_bytes());ch=struct.unpack('<10I',(out/'g_chassis.bin').read_bytes());guard=struct.unpack_from('<3I',(out/'g_supported_trial.bin').read_bytes())
        result={'token':token,'ack':after[1],'accepted':after[2]==s[2]+1,'status':after[3],'permit':after[4],'mode':ch[8],'fault':ch[2],'inhibit':ch[1],'guard_reason':guard[2],'bin_sha256':manifest['bin_sha256']}
        save(out/'result.json',result);print(json.dumps(result));assert ch[8] not in (2,3) and not after[4]
if __name__=='__main__':main()
