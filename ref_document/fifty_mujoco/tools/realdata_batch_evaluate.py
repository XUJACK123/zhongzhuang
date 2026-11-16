#!/usr/bin/env python3
"""Explicit CPU evaluation matrix with source/config/weight provenance."""
import argparse
from concurrent.futures import ProcessPoolExecutor,as_completed
import json
import multiprocessing as mp
from pathlib import Path
import torch


def evaluate(task):
    from realdata_train import SamplePolicy31,verified_artifact,digest
    from realdata_parallel_train import getup_evaluate
    torch.set_num_threads(1)
    low,high=(710000,719999) if task['split']=='heldout' else (610000,619999)
    if not low<=task['seed_base']<=high-task['count']+1:raise ValueError('seed split mismatch')
    checkpoint=Path(task['checkpoint']);saved=torch.load(checkpoint,map_location='cpu',weights_only=True)
    if task.get('expected_checkpoint_sha256') and digest(checkpoint)!=task['expected_checkpoint_sha256']:
        raise ValueError('frozen get-up weight hash mismatch')
    if task.get('expected_balance_sha256') and digest(task['balance'])!=task['expected_balance_sha256']:
        raise ValueError('frozen balance weight hash mismatch')
    cfg=saved.get('configuration',saved.get('contract',{}).get('configuration'))
    if task.get('config'):cfg=json.loads(Path(task['config']).read_text())
    if task.get('configuration_overrides'):cfg.update(task['configuration_overrides'])
    if task.get('balance_s'):cfg['success']['balance_s']=task['balance_s']
    policy=SamplePolicy31(verified_artifact('model_3050.pt'),'cpu');policy.load_state_dict(saved['model'])
    settings={'task':'getup','balance_checkpoint':task['balance'],'config':cfg,
              'reset_range':task['reset_range'],'settle_seconds':1.,'domain':task.get('domain',False),
              'incremental_results_path':str(Path(task['out']).with_suffix('.rows.jsonl'))}
    if task.get('record_traces'):
        settings['trajectory_directory']=str(Path(task['out']).parent/'trajectories'/task['label'])
    report=getup_evaluate(policy,settings,count=task['count'],seed_base=task['seed_base'])
    report.update(task=task,configuration=cfg,seeds=list(range(task['seed_base'],task['seed_base']+task['count'])),
                  checkpoint_sha256=digest(checkpoint),balance_sha256=digest(task['balance']),
                  backend='CPU MuJoCo + actual C qualified observer',evaluator_sha256=digest(__file__))
    Path(task['out']).write_text(json.dumps(report,indent=2))
    return {'label':task['label'],'passed':report['passed'],'trials':report['trials'],
            'balance_ticks':[r['balance_ticks'] for r in report['rows']],
            'failures':[r['failure'] for r in report['rows']],'file':task['out']}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--spec',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--workers',type=int,default=3)
    args=p.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    tasks=json.loads(args.spec.read_text());rows=[]
    for task in tasks:task['out']=str(args.out/(task['label']+'.json'))
    (args.out/'protocol.json').write_text(json.dumps(tasks,indent=2))
    with ProcessPoolExecutor(args.workers,mp_context=mp.get_context('spawn')) as pool:
        futures=[pool.submit(evaluate,task) for task in tasks]
        for future in as_completed(futures):
            row=future.result();rows.append(row);print(json.dumps(row),flush=True)
            (args.out/'summary.json').write_text(json.dumps(rows,indent=2))


if __name__=='__main__':main()
