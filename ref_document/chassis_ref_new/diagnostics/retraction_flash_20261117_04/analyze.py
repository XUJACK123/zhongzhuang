"""Summarize independent retraction measurements; not a policy replay or torque calibration."""
import argparse,json,math
from pathlib import Path
def analyze(folder):
    d=json.loads((folder/'result.json').read_text());rows=d['samples'];active=[r for r in rows if r['mode']==3]
    if not active:raise ValueError('No ACTIVE samples')
    q0=active[0]['q'];start=d['guard']['started'];deg=180/math.pi
    report={'active_samples':len(active),'record_samples':len(rows),'record_span_ms':d['span_ms'],
        'record_gaps_ms':sorted(set(d['tick_gaps_ms'])),'start_tick':start,
        'first_active_tick':active[0]['tick'],'last_active_tick':active[-1]['tick'],
        'mode_transitions':[(r['tick'],r['mode'],r['fault']) for i,r in enumerate(rows) if i and r['mode']!=rows[i-1]['mode']],
        'max_abs_torque_nm':[max(abs(r['torque'][i]) for r in active) for i in range(6)],
        'min_delta_deg':[min((r['q'][i]-q0[i])*deg for r in active) for i in range(6)],
        'max_delta_deg':[max((r['q'][i]-q0[i])*deg for r in active) for i in range(6)],
        'max_abs_qd_rad_s':[max(abs(r['qd'][i]) for r in active) for i in range(6)],
        'last_delta_deg':[(rows[-1]['q'][i]-q0[i])*deg for i in range(6)],
        'max_loop_us':max(r['loop_us'] for r in rows),
        'max_motor_age_ms':max(max(r['motor_age_ms']) for r in active),
        'end_of_hold_samples':[(r['tick']-start,(r['q'][4]-q0[4])*deg) for r in active if 2450<=r['tick']-start<=2500],
        'physical_angle_truth':False,'independent_torque_measurement':False,'target_delta_deg':5,
        'position_baseline':'first ACTIVE sample, up to 10 ms after the latched start; use raw guard origin for exact baseline',
        'order':['right_hip','right_knee','right_wheel','left_hip','left_knee','left_wheel']}
    (folder/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);a=p.parse_args();print(json.dumps(analyze(a.folder),indent=2))
