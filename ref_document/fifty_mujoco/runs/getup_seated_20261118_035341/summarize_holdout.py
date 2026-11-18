"""Aggregate only verified, fully completed groups from the frozen CPU evaluation."""

from collections import Counter
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path

import numpy as np

run = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_text())


def wilson(k, n):
    if not n:
        return [None,None]
    z = 1.959963984540054
    p = k/n
    center = p+z*z/(2*n)
    half = z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))
    denom = 1+z*z/n
    return [max(0.,(center-half)/denom),min(1.,(center+half)/denom)]


def main():
    manifest = read(run/'frozen/manifest.json')
    paths = {item['label']:item for item in read(run/'frozen/heldout_spec.json')}
    groups = []
    for role in ('primary','standard_mlp'):
        for group in ('nominal','joint'):
            group_paths = {label:item for label,item in paths.items() if label.startswith(f'{role}_{group}_')}
            rows = []
            files = {}
            for label,item in group_paths.items():
                path = run/'heldout_cpu'/f'{label}.json'
                result=read(path)
                assert result['checkpoint_sha256']==manifest['weights'][role]['sha256']
                balance = 'balance' if role == 'primary' else 'standard_balance'
                assert result['balance_sha256']==manifest['weights'][balance]['sha256']
                assert result['trials']==len(result['rows'])==item['count']
                assert result['passed']==sum(bool(row['success']) for row in result['rows'])
                for row in result['rows']:
                    trace = Path(row['trajectory'])
                    assert hashlib.sha256(trace.read_bytes()).hexdigest() == row['trajectory_sha256']
                    with np.load(trace) as samples:
                        assert not np.any(samples['metrics'][:, 9:15])
                        assert not np.any(samples['motor_command'][samples['phase'] == 'STOP'])
                        if row['success']:
                            assert int((samples['phase'] == 'BALANCE').sum()) == 1000
                assert [row['seed'] for row in result['rows']]==list(range(item['seed_base'],item['seed_base']+item['count']))
                files[path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
                rows.extend(result['rows'])
            assert len(rows)==100 and len({x['seed'] for x in rows})==100
            assert all(x['support_peak_n']==x['support_peak_nm']==0. for x in rows)
            successful = [x for x in rows if x['success']]
            assert all(x['balance_ticks']==1000 and x['phase_counts']['HANDOFF']==30 for x in successful)
            perturbed = [x for x in rows if x['episode_parameters']['motor_gain'] != [1.]*6]
            failures = Counter(x['failure'] for x in rows if not x['success'])
            mechanical = sum(count for key,count in failures.items()
                             if key and any(x in key for x in ('joint_limit','spring_stroke','forbidden_contact','nonfinite')))
            info = {'label':f'{role} {group}','passed':len(successful),'trials':len(rows),
                    'interval_95':wilson(len(successful),len(rows)),
                    'actually_perturbed':len(perturbed),'actually_perturbed_passed':sum(x['success'] for x in perturbed),
                    'failure_counts':dict(failures),'mechanical_or_numeric_failures':mechanical,
                    'target_passed':len(successful)>=95 and mechanical==0,
                    'minimum_finished_balance_ticks':min((x['balance_ticks'] for x in successful),default=0),
                    'excluded_runs':0,'files':files}
            groups.append(info)
    user_clock = datetime.now().astimezone()
    start = datetime.fromisoformat('2026-11-18T03:53:41+08:00')
    summary = {'finished_at':user_clock.isoformat(),'wall_seconds':(user_clock-start).total_seconds(),
               'frozen_at':manifest['frozen_at'],'groups':groups,
               'primary_passed':all(g['target_passed'] for g in groups if g['label'].startswith('primary')),
               'physical_hardware_validated':False}
    (run/'final_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__ == '__main__':
    main()
