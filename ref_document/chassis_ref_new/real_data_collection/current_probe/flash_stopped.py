"""Explicitly replace a known stopped diagnostic; never grants a run token."""
import argparse,hashlib,json,shutil,struct
from pathlib import Path
import operate as o
p=argparse.ArgumentParser();p.add_argument('--previous-elf',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
out=a.out.resolve();old=a.previous_elf.resolve()
if out.exists():p.error('Fresh evidence directory required')
out.mkdir(parents=True)
manifest=json.loads((o.HERE/'manifest.json').read_text())
assert hashlib.sha256(o.ELF.with_suffix('.bin').read_bytes()).hexdigest()==manifest['bin_sha256']
before=o.m.snapshot(out/'before',1,elf=old);o.save(out/'before/result.json',before)
b=before['b'];assert b['chassis']['mode'] in (0,4) and b['remote']['valid'] and b['remote']['s1']==2
assert b['chassis']['fault'] in (0,6) and before['a']['nonzero_torque_frames']==b['nonzero_torque_frames']
sy=o.m.elf_symbols(old,('g_chassis','g_supported_session','g_remote'))
for ext in ('.elf','.bin'):
    shutil.copy2(old.with_suffix(ext),out/('restore_previous'+ext));shutil.copy2(o.ELF.with_suffix(ext),out/('diagnostic'+ext))
for f in o.HERE.iterdir():
    if f.is_file() and f.suffix in ('.c','.h','.py','.json','.log'):shutil.copy2(f,out/f.name)
o.run(out,[f'set c [read_memory {sy["g_chassis"][0]:#x} 32 10]',f'set s [read_memory {sy["g_supported_session"][0]:#x} 32 8]',f'set r [read_memory {sy["g_remote"][0]:#x} 8 28]',
    'if {([lindex $c 8] != 0 && [lindex $c 8] != 4) || [lindex $s 4] != 0 || [lindex $s 0] != [lindex $s 1] || [lindex $r 24] != 1 || [lindex $r 10] != 2} {error "Not stopped, remote absent or request pending"}',
    'reset halt','wait_halt 2000',f'dump_image {{{(out/"preflash.bin").as_posix()}}} 0x08000000 {old.with_suffix(".bin").stat().st_size}',
    f'if {{[catch {{verify_image {{{old.as_posix()}}}}} verify_error]}} {{resume; error $verify_error}}',
    f'program {{{o.ELF.as_posix()}}} verify reset'],90)
manifest.update(flashed=True,flash_verified=True,physical_result=None,previous_elf=str(old))
o.save(out/'session.json',manifest)
audit=o.audit_flash((out/'preflash.bin').read_bytes(),old);o.save(out/'backup_audit.json',audit);assert audit['all_load_sections_match']
after=o.m.snapshot(out/'after',1,elf=o.ELF);o.save(out/'after/result.json',after)
assert after['b']['nonzero_torque_frames']==after['b']['enable_commands']==0 and after['b']['chassis']['mode']==0
print(json.dumps({'flashed':True,'bin_sha256':manifest['bin_sha256'],'mode':after['b']['chassis']['mode'],'inhibit':after['b']['chassis']['inhibit'],'ready':after['b']['standing_entry']['ready'],'s1':after['b']['remote']['s1'],'nonzero':0,'enables':0}))
