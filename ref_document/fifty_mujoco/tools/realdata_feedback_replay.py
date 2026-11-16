#!/usr/bin/env python3
"""CPU development replay for the GPU feedback grid; no held-out seeds."""
import argparse
from concurrent.futures import ProcessPoolExecutor,as_completed
import json
import multiprocessing as mp
from pathlib import Path
import torch


def replay(task):
    from realdata_train import SamplePolicy31,verified_artifact,digest
    from realdata_allocator import AllocatorActor
    from realdata_parallel_train import getup_evaluate
    checkpoint,reset_range,count,out=task
    torch.set_num_threads(1);checkpoint=Path(checkpoint)
    saved=torch.load(checkpoint,map_location='cpu',weights_only=True)
    cfg=saved['configuration'];policy=SamplePolicy31(verified_artifact('model_3050.pt'),'cpu')
    policy.load_state_dict(saved['model']);policy.actor=AllocatorActor(policy.actor,cfg)
    settings={'task':'getup','balance_checkpoint':str(checkpoint.resolve()),'reset_range':reset_range,
              'settle_seconds':1.,'domain':False,'config':cfg}
    report=getup_evaluate(policy,settings,count=count,seed_base=618600)
    report.update(checkpoint=str(checkpoint),checkpoint_sha256=digest(checkpoint),configuration=cfg)
    Path(out).write_text(json.dumps(report,indent=2))
    return {'file':out,'passed':report['passed'],'trials':count,
            'balance_ticks':[r['balance_ticks'] for r in report['rows']],
            'failures':[r['failure'] for r in report['rows']]}


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--grid',type=Path,required=True)
    ap.add_argument('--ranks',type=int,default=6);ap.add_argument('--workers',type=int,default=4)
    ap.add_argument('--seeds',type=int,default=2);ap.add_argument('--reset-range',nargs=2,type=float,default=[0,.1])
    args=ap.parse_args();tasks=[]
    for i in range(args.ranks):
        tasks.append((str(args.grid/f'balance_rank{i:02d}.pt'),args.reset_range,args.seeds,
                      str(args.grid/f'cpu_rank{i:02d}.json')))
    rows=[]
    with ProcessPoolExecutor(args.workers,mp_context=mp.get_context('spawn')) as executor:
        for future in as_completed([executor.submit(replay,task) for task in tasks]):
            row=future.result();rows.append(row);print(json.dumps(row),flush=True)
    (args.grid/'cpu_summary.json').write_text(json.dumps(rows,indent=2))


if __name__=='__main__':main()
