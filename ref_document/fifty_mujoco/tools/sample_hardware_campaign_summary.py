#!/usr/bin/env python3
"""Collect training budgets and evaluation evidence without promoting scope."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
PROJECT=Path(__file__).resolve().parents[1]


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--selected',type=Path);a=p.parse_args()
    segments=[]
    for r in sorted((PROJECT/'runs').glob('hardware31*/training_report.json')):
        d=json.loads(r.read_text())
        segments.append({'run':r.parent.name,'report':str(r.relative_to(PROJECT)),
            'report_sha256':sha(r),'seconds':d['wall_seconds'],'updates':d['last_update']-d['start_update'],
            'samples':d['transitions'],'failures':d['failures'],'episodes_ended':d['episodes_ended'],
            'stop_reason':d.get('stop_reason'),'checkpoint':d['checkpoint'],
            'max_kl':max([row['final_full_rollout_kl'] for row in d['rows']],default=0.),
            'actor_delta':d['actor_max_parameter_delta']})
    evaluations=[]
    for r in sorted((PROJECT/'reports/sample_weight').glob('hardware*.json')):
        if r.resolve()==a.out.resolve():continue
        d=json.loads(r.read_text())
        if 'summaries' in d:
            evaluations.append({'report':str(r.relative_to(PROJECT)),'sha256':sha(r),
                'tool':d.get('tool'),'checkpoint':d.get('checkpoint',d.get('actor_source',{}).get('path')),
                'suite':d.get('suite'),'scope':d.get('seed_scope'),'backend':d.get('backend'),
                'summaries':d['summaries'],'adequate_nominal_protocol':d.get('adequate_nominal_protocol',False),
                'adequate_random_protocol':d.get('adequate_random_protocol',False)})
    total=sum(s['seconds'] for s in segments)
    if total>14400:raise RuntimeError('training budget exceeded')
    report={'tool':'hardware-campaign-summary','generated_at':datetime.now(timezone.utc).isoformat(),
        'user_budget_seconds':14400,'conservative_campaign_start_utc':'2026-11-14T18:23:51Z',
        'conservative_deadline_utc':'2026-11-14T22:23:45Z','training_seconds':total,
        'training_hours':total/3600,'training_samples':sum(s['samples'] for s in segments),
        'training_segments':segments,'evaluations':evaluations,
        'selected_checkpoint':str(a.selected.resolve()) if a.selected else None,
        'selected_sha256':sha(a.selected) if a.selected else None,
        'target_model_accepted':False,'real_robot_accepted':False,
        'limitations':['Nominal standing resets do not establish recovery from the externally supported surveyed pose.',
            'Wheel Kd=0.10 requires matching deployment parameters; current firmware remains unchanged.',
            'Mass/inertias, constant gas spring, torque gains and ground contacts remain partly provisional.',
            'Original Isaac/PhysX golden-trajectory parity is unavailable.']}
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'training_seconds':total,'training_samples':report['training_samples'],
        'segments':len(segments),'evaluations':len(evaluations),'selected':report['selected_checkpoint']}))


if __name__=='__main__':main()
