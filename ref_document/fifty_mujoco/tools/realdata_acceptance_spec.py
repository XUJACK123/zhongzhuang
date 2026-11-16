#!/usr/bin/env python3
"""Resolve a predeclared held-out plan against immutable candidate files."""
import argparse
import copy
import json
from pathlib import Path
from realdata_train import PROJECT,digest


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--frozen',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args();frozen=args.frozen.resolve()
    manifest=json.loads((frozen/'manifest.json').read_text());plan=json.loads((frozen/'acceptance_plan.json').read_text())
    for name,expected in manifest['generated_sha256'].items():
        if digest(frozen/name)!=expected:raise ValueError('frozen file changed: '+name)
    # Runtime source must still match the copy made before held-out evaluation.
    for name,expected in manifest['source_sha256'].items():
        if name.startswith(('tools/','src/','configs/')) and digest(PROJECT/name)!=expected:
            raise ValueError('runtime source changed since freeze: '+name)
    configurations={role:json.loads((frozen/role/'configuration.json').read_text()) for role in ('near','full')}
    tasks=[]
    def task(role,case,overrides=None):
        weights=manifest['candidates'][role]
        result={'label':role+'_'+case['scope'],'checkpoint':str(frozen/role/'getup.pt'),
                'balance':str(frozen/role/'balance.pt'),'config':str(frozen/role/'configuration.json'),
                'expected_checkpoint_sha256':weights['getup_sha256'],'expected_balance_sha256':weights['balance_sha256'],
                'reset_range':case['reset_range'],'count':case['count'],'seed_base':case['seed_base'],
                'split':'heldout','record_traces':True,'domain':case.get('domain',False)}
        if overrides:result['configuration_overrides']=overrides
        tasks.append(result)
    for role in ('near','full'):
        for case in plan['per_candidate_nominal']:task(role,case)
    for original in plan['sensitivity']:
        case=copy.deepcopy(original);role=case['candidate'];cfg=configurations[role]
        case['reset_range']=[0,.2] if role=='near' else [1,1]
        overrides={}
        if 'initial_force_n' in case:
            spring=copy.deepcopy(cfg['spring']);spring['initial_force_n']=case['initial_force_n']
            overrides['spring']=spring
        if 'rotor_scale' in case:
            rotor=copy.deepcopy(cfg['reflected_inertia'])
            for key in ('leg_motor_output_kg_m2','wheel_output_kg_m2'):rotor[key]*=case['rotor_scale']
            rotor['sensitivity_multiplier']=case['rotor_scale'];overrides['reflected_inertia']=rotor
        if overrides:overrides['sensitivity_note']=case['scope']+'; hypothetical condition, not measured calibration'
        task(role,case,overrides)
    nominal=sum(t['count'] for t in tasks[:6]);sensitivity=sum(t['count'] for t in tasks[6:])
    if nominal!=plan['nominal_episodes'] or sensitivity!=plan['sensitivity_episodes']:raise ValueError('plan episode count mismatch')
    if args.out.exists():raise ValueError('use new held-out specification path')
    args.out.write_text(json.dumps(tasks,indent=2))
    print(json.dumps({'tasks':len(tasks),'nominal_episodes':nominal,'sensitivity_episodes':sensitivity,
                      'spec_sha256':digest(args.out),'frozen_manifest_sha256':digest(frozen/'manifest.json')}),flush=True)


if __name__=='__main__':main()
