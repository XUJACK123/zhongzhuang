#!/usr/bin/env python3
"""Audit every retained held-out trajectory and frozen artifact hash."""
import argparse
import copy
from datetime import datetime
import json
from pathlib import Path
from zoneinfo import ZoneInfo
from realdata_train import digest
from realdata_trace_audit import audit_trace


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--frozen',type=Path,required=True);ap.add_argument('--evaluation',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);args=ap.parse_args()
    manifest=json.loads((args.frozen/'manifest.json').read_text())
    tasks=json.loads((args.evaluation/'protocol.json').read_text());issues=[];groups=[];checks=[]
    for name,expected in manifest['generated_sha256'].items():
        if digest(args.frozen/name)!=expected:issues.append('Frozen file hash mismatch: '+name)
    for task in tasks:
        file=Path(task['out']);complete=file.exists()
        if complete:
            report=json.loads(file.read_text());rows=report['rows']
            if report['checkpoint_sha256']!=task['expected_checkpoint_sha256']:issues.append('Weight mismatch: '+task['label'])
            if report['balance_sha256']!=task['expected_balance_sha256']:issues.append('Balance mismatch: '+task['label'])
            expected=copy.deepcopy(json.loads(Path(task['config']).read_text()))
            expected.update(task.get('configuration_overrides',{}))
            if report['configuration']!=expected:issues.append('Configuration mismatch: '+task['label'])
        else:
            partial=file.with_suffix('.rows.jsonl')
            rows=[json.loads(x) for x in partial.read_text().splitlines()] if partial.exists() else []
        seeds=[r.get('seed',r.get('episode_parameters',{}).get('seed')) for r in rows]
        expected_seeds=list(range(task['seed_base'],task['seed_base']+task['count']))
        if seeds!=expected_seeds[:len(seeds)] or len(seeds)>task['count']:issues.append('Seed mismatch: '+task['label'])
        if any(s is None or not 710000<=s<=719999 for s in seeds):issues.append('Non-held-out seed: '+task['label'])
        for row,seed in zip(rows,seeds):
            try:
                trace=Path(row['trajectory'])
                if digest(trace)!=row['trajectory_sha256']:raise ValueError('row trajectory hash mismatch')
                audit=audit_trace(trace)
                agrees=bool(row['success'])==audit['passed']
                if not agrees:issues.append(f'Label/trace disagreement: {task["label"]} seed {seed}')
                checks.append({'label':task['label'],'seed':seed,'success':bool(row['success']),
                               'trace_audit_passed':audit['passed'],'label_agrees':agrees,
                               'failure':row['failure'],'checks':audit['checks']})
            except Exception as error:issues.append(f'Trace audit error: {task["label"]} seed {seed}: {error}')
        passed=sum(bool(r['success']) for r in rows)
        groups.append({'label':task['label'],'expected':task['count'],'completed':len(rows),'passed':passed,
                       'complete':complete and len(rows)==task['count'],
                       'scope_passed':complete and len(rows)==task['count'] and passed==task['count'],
                       'failures':{str(reason):sum(r['failure']==reason for r in rows) for reason in set(r['failure'] for r in rows)}})
    result={'schema':'realdata_independent_cpu_acceptance_audit_v1',
            'audited_at':datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
            'frozen_manifest_sha256':digest(args.frozen/'manifest.json'),'groups':groups,'trajectory_checks':checks,
            'expected_episodes':sum(g['expected'] for g in groups),'completed_episodes':sum(g['completed'] for g in groups),
            'completed_successes':sum(g['passed'] for g in groups),'integrity_issues':issues,
            'integrity_passed':not issues and len(checks)==sum(g['completed'] for g in groups),
            'all_planned_episodes_completed':all(g['complete'] for g in groups),
            'all_tested_scopes_passed':all(g['scope_passed'] for g in groups) and not issues,
            'hardware_accepted':False,'deployed':False,
            'scope':'Frozen simulation models only. Failed and incomplete episodes are retained. No retuning from these seeds.'}
    args.out.write_text(json.dumps(result,indent=2))
    print(json.dumps({k:result[k] for k in ['expected_episodes','completed_episodes','completed_successes',
        'integrity_passed','all_planned_episodes_completed','all_tested_scopes_passed','integrity_issues']}),flush=True)
    if issues:raise SystemExit(1)


if __name__=='__main__':main()
