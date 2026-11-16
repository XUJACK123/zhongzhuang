#!/usr/bin/env python3
"""Select from a fixed CPU development set; never inspect held-out scores."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import statistics
from zoneinfo import ZoneInfo
from realdata_train import digest


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root',type=Path,default=Path('runs/realdata_20261117'))
    ap.add_argument('--out',type=Path,required=True);args=ap.parse_args()
    groups={'near':['getup_feedback18_near','getup_near_duration'],
            'full':['getup_full_v6','getup_full_cpu_fit_ppo']}
    result={'selected_at':datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),'selection_split':'development',
            'criterion':'Same eight CPU seeds 615000..615007; maximize complete successes, then median and minimum balance ticks; earlier experiment, then earlier update, break remaining ties. A resumed update0 does not outrank its identical source policy solely because its local counter reset.',
            'candidates':{},'comparisons':{}}
    for role,runs in groups.items():
        candidates=[]
        for name in runs:
            folder=args.root/name;contract=json.loads((folder/'contract.json').read_text())
            for file in sorted(folder.glob('cpu_development_*.json')):
                report=json.loads(file.read_text())
                if report['trials']!=8:continue
                seeds=[r['episode_parameters']['seed'] for r in report['rows']]
                if seeds!=list(range(615000,615008)):raise ValueError('unexpected development seed set')
                update=int(file.stem.rsplit('_',1)[1]);checkpoint=folder/f'update_{update:06d}.pt'
                if digest(checkpoint)!=report['checkpoint_sha256']:raise ValueError('checkpoint changed since evaluation')
                ticks=[r['balance_ticks'] for r in report['rows']]
                row={'role':role,'checkpoint':str(checkpoint),'checkpoint_sha256':digest(checkpoint),
                     'balance':contract['balance_checkpoint'],'balance_sha256':digest(contract['balance_checkpoint']),
                     'report':str(file),'report_sha256':digest(file),'passed':report['passed'],'trials':8,
                     'update':update,'balance_ticks':ticks,
                     'score':[report['passed'],statistics.median(ticks),min(ticks),-runs.index(name),-update]}
                candidates.append(row)
        if not candidates:raise ValueError('no completed CPU development evaluations for '+role)
        candidates.sort(key=lambda x:x['score'],reverse=True)
        result['candidates'][role]=candidates[0];result['comparisons'][role]=candidates
    args.out.write_text(json.dumps(result,indent=2))
    print(json.dumps(result['candidates']),flush=True)


if __name__=='__main__':main()
