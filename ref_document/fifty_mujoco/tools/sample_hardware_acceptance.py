#!/usr/bin/env python3
"""Derive the limited reconstruction acceptance from frozen evidence reports."""
import argparse,hashlib,json,math
from pathlib import Path
PROJECT=Path(__file__).resolve().parents[1]
REPORTS=PROJECT/'reports/sample_weight'


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def summarize(path,expected,extra_expected):
    d=json.loads(path.read_text());rows=d['rows'];base=[r for r in rows if r['condition']!='extra_protocol']
    extra=[r for r in rows if r['condition']=='extra_protocol']
    finite=all(all(math.isfinite(v) for v in r.get('metrics',{}).values()) and
               len(r.get('motor_saturation_fractions',[]))==6 and
               all(math.isfinite(v) for v in r.get('motor_saturation_fractions',[])) for r in rows)
    passed=len(base)==expected and len(extra)==extra_expected and all(r['thresholds_met'] for r in rows) and finite
    valid=[r for r in base if r.get('metrics',{}).get('samples',0)>0 and 'clearance_mae_m' in r['metrics']]
    samples=sum(r['metrics']['samples'] for r in valid)
    weighted=lambda key:math.sqrt(sum(r['metrics']['samples']*r['metrics'][key]**2 for r in valid)/samples) if samples else None
    metrics={'point_vx_rmse_m_s':weighted('point_vx_rmse_m_s'),'yaw_rate_rmse_rad_s':weighted('yaw_rate_rmse_rad_s'),
        'clearance_mae_m':sum(r['metrics']['samples']*r['metrics']['clearance_mae_m'] for r in valid)/samples if samples else None,
        'worst_motor_saturation_fraction':max((max(r['motor_saturation_fractions']) for r in rows if r.get('motor_saturation_fractions')),default=None),
        'minimum_reported_spring_margin_m':min((r['minimum_spring_margin_m'] for r in rows if r.get('minimum_spring_margin_m') is not None),default=None),
        'worst_roll_p95_deg':max((r['metrics']['roll_abs_p95_deg'] for r in valid),default=None),
        'worst_pitch_p95_deg':max((r['metrics']['pitch_abs_p95_deg'] for r in valid),default=None)}
    return {'report':str(path.relative_to(PROJECT)),'sha256':digest(path),'checkpoint_sha256':d['checkpoint_sha256'],
        'seed_scope':d['seed_scope'],'primary_trials':len(base),'primary_passed':sum(r['thresholds_met'] for r in base),
        'tracking_metrics_eligible_primary_trials':len(valid),'extra_protocols':len(extra),'extra_passed':sum(r['thresholds_met'] for r in extra),
        'physical_failures':sum(r['first_failure'] is not None for r in rows),'all_metrics_finite':finite,
        'strict_all_trials_passed':passed,'metrics':metrics,'classes':d['summaries'],
        'wheel_kd':d['wheel_kd'],'knee_margin_rad':d.get('knee_margin_rad')}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    selection=json.loads((REPORTS/'hardware_candidate_selection_20261115.json').read_text())
    inputs=[('development','hardware_guarded_final_development_20261115.json',200,3),
        ('heldout_nominal','hardware_final_heldout_nominal_20261115.json',200,3),
        ('heldout_dm_gain','hardware_final_heldout_dm_20261115.json',500,0)]
    if any(not (REPORTS/name).exists() for _,name,_,_ in inputs):raise RuntimeError('complete evidence reports are required')
    checks={name:summarize(REPORTS/file,n,extra) for name,file,n,extra in inputs}
    for row in checks.values():
        if row['checkpoint_sha256']!=selection['sha256'] or row['wheel_kd']!=.14 or row['knee_margin_rad']!=.03:
            raise ValueError('frozen candidate mismatch')
    boundary=json.loads((REPORTS/'hardware_final_boundary_20261115.json').read_text())
    startup=json.loads((REPORTS/'hardware_final_startup_20261115.json').read_text())
    report={'tool':'hardware-final-reconstruction-acceptance','status':'passed_limited_reconstruction_scope' if all(x['strict_all_trials_passed'] for x in checks.values()) else 'failed_or_limited_evidence',
        'candidate':selection,'checks':checks,'nominal_reconstruction_heldout_passed':checks['heldout_nominal']['strict_all_trials_passed'],
        'dm_gain_reconstruction_heldout_passed':checks['heldout_dm_gain']['strict_all_trials_passed'],
        'request_scope':{'nominal_vx_m_s':[0,.15,-.15],'nominal_wz_rad_s':[0,.3,-.3],
            'clearance_m':.1,'extra_protocols':'specified 40s reversal/turning and fast-backward diagnostics'},
        'dm_gain_scope':'Independent four DM gains uniform0.8..1, held per episode; wheels1; exactly20% nominal cases; engineering hypothesis, not measured probability.',
        'boundary':{'trials':len(boundary['rows']),'passed':sum(r['thresholds_met'] for r in boundary['rows']),
            'all_completed':all(r.get('completed',False) for r in boundary['rows']),
            'not_passing_commands':[s for s in boundary['summaries'] if s['passed']!=s['trials']]},
        'surveyed_pose_release_completed':sum(r['completed'] for r in startup['release_rows']),
        'surveyed_pose_release_trials':len(startup['release_rows']),
        'target_model_accepted':False,'real_robot_accepted':False,'original_PhysX_parity':False,
        'not_claimed':['Unassisted startup from the supported surveyed pose','Full current-firmware high-speed/yaw request range',
            'Joint friction, mass/inertia, spring-curve or combined domain-randomization robustness','MCU timing or physical robot balance']}
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'status':report['status'],'checks':{k:[v['primary_passed'],v['primary_trials']] for k,v in checks.items()}}))


if __name__=='__main__':main()
