"""Summarize independent retraction measurements; not a policy replay or torque calibration."""
import argparse,json,math,struct
from pathlib import Path
def analyze(folder):
    d=json.loads((folder/'result.json').read_text());rows=d['samples'];active=[r for r in rows if r['mode']==3]
    if not active:raise ValueError('No ACTIVE samples')
    q0=active[0]['q'];start=d['guard']['started'];deg=180/math.pi
    trigger=folder.with_name(folder.name+'_trigger')
    manifest_path=trigger/'manifest.json'
    config=json.loads(manifest_path.read_text()) if manifest_path.exists() else {'ramp_ms':1500,'hold_ms':1000,'release_ms':1000,'target_delta_deg':5,'active_legs':['left']}
    release_start=config['ramp_ms']+config['hold_ms']
    guard_path=trigger/'g_supported_trial.bin'
    baseline='first ACTIVE sample'
    if guard_path.exists():q0=list(struct.unpack('<3I6f',guard_path.read_bytes())[3:]);baseline='firmware-latched guard origin'
    report={'active_samples':len(active),'record_samples':len(rows),'record_span_ms':d['span_ms'],
        'record_gaps_ms':sorted(set(d['tick_gaps_ms'])),'start_tick':start,
        'first_active_tick':active[0]['tick'],'last_active_tick':active[-1]['tick'],
        'mode_transitions':[(r['tick'],r['mode'],r['fault']) for i,r in enumerate(rows) if i and r['mode']!=rows[i-1]['mode']],
        'max_abs_torque_nm':[max(abs(r['torque'][i]) for r in active) for i in range(6)],
        'max_abs_reported_feedback_current_a':[max(abs(r['feedback'][i]) for r in active) for i in range(6)],
        'max_nominal_leg_current_command_a':{str(i):max(abs(r['torque'][i])/1.261575 for r in active) for i in (0,1,3,4)},
        'min_delta_deg':[min((r['q'][i]-q0[i])*deg for r in active) for i in range(6)],
        'max_delta_deg':[max((r['q'][i]-q0[i])*deg for r in active) for i in range(6)],
        'max_abs_qd_rad_s':[max(abs(r['qd'][i]) for r in active) for i in range(6)],
        'last_delta_deg':[(rows[-1]['q'][i]-q0[i])*deg for i in range(6)],
        'max_loop_us':max(r['loop_us'] for r in rows),
        'max_motor_age_ms':max(max(r['motor_age_ms']) for r in active),
        'end_of_hold_samples':[(r['tick']-start,(r['q'][4]-q0[4])*deg) for r in active if release_start-50<=r['tick']-start<release_start],
        'physical_angle_truth':False,'independent_torque_measurement':False,'target_delta_deg':config['target_delta_deg'],
        'position_baseline':baseline,'parameters':config,
        'order':['right_hip','right_knee','right_wheel','left_hip','left_knee','left_wheel']}
    last=[r for r in active if release_start-1000<=r['tick']-start<release_start]
    report['hold_last_second']={'samples':len(last),'complete':bool(last) and last[-1]['tick']-last[0]['tick']>=950,'legs':{}}
    if last:
        for label,h,k,sign in [('right',0,1,-1),('left',3,4,1)]:
            if label not in config['active_legs']:continue
            fold=[sign*(r['q'][k]-q0[k])*deg for r in last]
            hip=[(r['q'][h]-q0[h])*deg for r in last]
            report['hold_last_second']['legs'][label]={
                'mean_fold_deg':sum(fold)/len(fold),'max_target_error_deg':max(abs(config['target_delta_deg']-v) for v in fold),
                'knee_peak_to_peak_deg':max(fold)-min(fold),'max_hip_error_deg':max(abs(x) for x in hip),
                'motor_cap_fraction':[sum(abs(r['torque'][i])>=config.get('motor_caps_nm',[20,20,0,20,20,0])[i]-.1 for r in last)/len(last) for i in (h,k)],
                'mean_abs_reported_feedback_current_a':[sum(abs(r['feedback'][i]) for r in last)/len(last) for i in (h,k)],
                'encoder_hold_criterion_met':max(abs(config['target_delta_deg']-v) for v in fold)<=1 and max(fold)-min(fold)<=1 and max(abs(x) for x in hip)<=2}
    report['bilateral_encoder_hold_pass']=report['hold_last_second']['complete'] and len(report['hold_last_second']['legs'])==2 and all(v['encoder_hold_criterion_met'] for v in report['hold_last_second']['legs'].values())
    (folder/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);a=p.parse_args();print(json.dumps(analyze(a.folder),indent=2))
