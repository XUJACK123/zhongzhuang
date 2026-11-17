"""Offline current tracking and raw-output motion analysis, not force ground truth."""
import argparse,json,math,struct,hashlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('trace',type=Path);a=p.parse_args()
trace=a.trace;trigger=trace.with_name(trace.name.removesuffix('_trace')+'_trigger')
d=json.loads((trace/'result.json').read_text(encoding='utf-8'))
manifest=json.loads((trigger/'manifest.json').read_text(encoding='utf-8'))
index=manifest['selected_index'];signs=[-1,1,-1,1,1,1];kt=1.261575
active=[r for r in d['samples'] if r['mode']==3]
assert active and d['frozen'] and not d['rejected_slots']
assert all(all(t==0 for j,t in enumerate(r['torque']) if j!=index) for r in active)
header=struct.unpack('<3I6f',(trigger/'g_supported_trial.bin').read_bytes())
origin=header[3:];started=header[0]
run_and_post=[r for r in d['samples'] if (r['tick']-started)&0xffffffff<10000]
current=[r['torque'][index]*signs[index]/kt for r in active]
feedback=[r['feedback'][index] for r in active]
xx=sum(current)/len(current);yy=sum(feedback)/len(feedback)
variance=sum((x-xx)**2 for x in current)
slope=sum((x-xx)*(y-yy) for x,y in zip(current,feedback))/variance if variance>1e-12 else None
offset=yy-slope*xx if slope is not None else None
plateau=[r for r in active if 1100<=(r['tick']-started)&0xffffffff<1450]
motion_threshold=8*2*math.pi/8192
onsets=[]
for motor in (0,1,3,4):
    found=None
    for n in range(len(active)-2):
        if all(abs(r['q'][motor]-origin[motor])>=motion_threshold for r in active[n:n+3]):
            r=active[n];found=dict(t_ms=(r['tick']-started)&0xffffffff,
                selected_command_a=r['torque'][index]*signs[index]/kt,
                selected_feedback_a=r['feedback'][index],
                selected_nominal_command_nm=r['torque'][index]*signs[index]);break
    onsets.append(dict(index=motor,first_sustained_motion=found))
report=dict(kind='single DM electrical and raw-angle response',selected_index=index,
    dm_id=[4,3,3,1,2,2][index],raw_nominal_target_nm=manifest['raw_nominal_nm'],
    target_current_a=manifest['raw_nominal_nm']/kt,active_samples=len(active),
    raw_sample_intervals_ms=sorted(set(d['tick_gaps_ms'])),guard_reason=d['guard']['reason'],
    max_abs_command_a=max(map(abs,current)),max_abs_feedback_a=max(map(abs,feedback)),
    plateau_samples=len(plateau),plateau_feedback_mean_a=sum(r['feedback'][index] for r in plateau)/len(plateau) if plateau else None,
    current_tracking_rmse_a=math.sqrt(sum((x-y)**2 for x,y in zip(current,feedback))/len(active)),
    feedback_vs_command_slope=slope,feedback_offset_a=offset,
    max_abs_raw_output_delta_deg=[max(abs(r['q'][i]-origin[i])*180/math.pi for r in active) for i in range(6)],
    max_abs_raw_output_speed_rad_s=[max(abs(r['qd'][i]) for r in active) for i in range(6)],
    max_abs_delta_including_recorded_post_stop_deg=[max(abs(r['q'][i]-origin[i])*180/math.pi for r in run_and_post) for i in range(6)],
    max_abs_speed_including_recorded_post_stop_rad_s=[max(abs(r['qd'][i]) for r in run_and_post) for i in range(6)],
    motion_threshold_deg=motion_threshold*180/math.pi,motion_onsets=onsets,
    max_motor_feedback_age_ms=max(max(r['motor_age_ms']) for r in active),
    nominal_kt_nm_a=kt,measured_shaft_torque_nm=None,measured_force_n=None,
    independent_force_truth=False,absolute_joint_pose_used=False,
    interpretation='Nm denotes nominal current conversion; lack of motion does not isolate friction, spring or gravity. Offset and lag affect current fit. Raw q is motor output-shaft angle, not anatomical knee angle.',
    source_trace=str(trace).replace('\\','/'),source_trigger=str(trigger).replace('\\','/'),
    raw_sha256=hashlib.sha256((trace/'trace.bin').read_bytes()).hexdigest())
stop_path=trigger/'g_stop_event.bin'
if stop_path.exists():
    values=struct.unpack('<5I30f',stop_path.read_bytes())
    report['stop_event']=dict(valid=values[0],tick=values[1],fault=values[2],reason=values[3],inhibit=values[4],
        duration_ms=(values[1]-started)&0xffffffff,q=list(values[5:11]),qd=list(values[11:17]),
        canonical_torque_before_stop=list(values[17:23]),
        selected_raw_nominal_nm_before_stop=values[17+index]*signs[index])
(trace/'analysis.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
print(json.dumps(report))
