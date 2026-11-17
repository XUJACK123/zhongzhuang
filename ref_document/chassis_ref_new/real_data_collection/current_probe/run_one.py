"""One explicitly requested, bounded probe; archive before allowing another."""
import argparse,json,struct,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--index',type=int,choices=(0,1,3,4),required=True);p.add_argument('--raw-nm',type=float,required=True);p.add_argument('--name',required=True);a=p.parse_args()
assert .1<=abs(a.raw_nm)<=3 and all(c.isalnum() or c=='_' for c in a.name)
base=ROOT/'diagnostics'/a.name
trigger=base.with_name(base.name+'_trigger');trace=base.with_name(base.name+'_trace');prepare=base.with_name(base.name+'_prepare')
assert not any(x.exists() for x in (trigger,trace,prepare))
def run(argv): subprocess.run([sys.executable]+[str(v) for v in argv],cwd=ROOT,check=True)
run(['tools/trial_trace.py','--elf','build/current_probe/current_probe.elf','--clear','--header-only','--out',prepare])
run([HERE/'operate.py','trigger','--index',a.index,'--raw-nm',a.raw_nm,'--out',trigger])
result=json.loads((trigger/'result.json').read_text())
assert result['accepted'] and not result['permit'] and result['mode']==4
# Always archive a stopped trial, including unexpected faults, before inspection.
run(['tools/trial_trace.py','--elf','build/current_probe/current_probe.elf','--out',trace])
with (trace/'analysis.log').open('w',encoding='utf-8') as log:
    subprocess.run([sys.executable,str(HERE/'analyze.py'),str(trace)],cwd=ROOT,stdout=log,check=True)
d=json.loads((trace/'analysis.json').read_text())
bank=(trigger/'g_motor_snapshot.bin').read_bytes()
health=dict(temperatures_c=[bank[i*40+36] for i in range(6)],
    position_lost=[bool(bank[i*40+38]) for i in range(6)],
    post_motor_speed_rad_s=[struct.unpack_from('<f',bank,i*40+20)[0] for i in range(6)],
    tx_fail_count=struct.unpack('<I',(trigger/'g_can_tx_fail.bin').read_bytes())[0],
    rx_overflow_count=struct.unpack('<I',(trigger/'g_can_rx_overflow.bin').read_bytes())[0])
(trace/'post_health.json').write_text(json.dumps(health,indent=2)+'\n')
assert result['fault']==6 and result['guard_reason'] in (1,2,3) and not result['inhibit']
assert not any(health['position_lost']) and max(health['temperatures_c'])<75
assert max(map(abs,health['post_motor_speed_rad_s']))<.06
print(json.dumps({'dm_id':d['dm_id'],'raw_nm':a.raw_nm,'stop':d.get('stop_event'),
    'max_feedback_a':d['max_abs_feedback_a'],'motion_deg':d['max_abs_raw_output_delta_deg'],
    'health':health,'evidence':str(trace)}),flush=True)
