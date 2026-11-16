#!/usr/bin/env python3
"""Summarize retained experiments without converting development to acceptance."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
from zoneinfo import ZoneInfo

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root',type=Path,default=Path('runs/realdata_20261117'));args=ap.parse_args()
    runs=[]
    for folder in sorted(args.root.iterdir()):
        if not folder.is_dir():continue
        f=folder/'rows.jsonl'
        if not f.exists() or not f.stat().st_size:continue
        rows=[json.loads(line) for line in f.read_text().splitlines() if line.strip()]
        last=rows[-1]
        report=json.loads((folder/'training_report.json').read_text()) if (folder/'training_report.json').exists() else None
        dev=[]
        for file in sorted(folder.glob('cpu_development_*.json')):
            r=json.loads(file.read_text());dev.append({'file':str(file),'update':int(file.stem.rsplit('_',1)[1]),
                    'passed':r.get('passed'),'trials':r.get('trials'),
                    'maximum_balance_ticks':max((x.get('balance_ticks',0) for x in r.get('rows',[])),default=0)})
        runs.append({'run':folder.name,'latest':last,'training_report':report,'cpu_development':dev})
    try:
        gpu=subprocess.check_output(['nvidia-smi','--query-gpu=name,utilization.gpu,memory.used,temperature.gpu,power.draw',
                                     '--format=csv,noheader'],text=True).strip()
    except Exception as error:gpu=str(error)
    report={'updated_at':datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
            'scope':'Offline simulation experiments. Each development count has its own curriculum/configuration; do not pool as acceptance.',
            'gpu':gpu,'runs':runs,
            'recorded_last_transition_counts_sum':sum(x['latest'].get('transitions',0) for x in runs),
            'missing_real_identification':['independent spring force and hysteresis','actual charge pressure',
                'rotor inertia/gear drag','encoder reference reconciliation','natural disabled rest','real dynamic timing and sensor errors'],
            'hardware_access_this_run':False}
    (args.root/'latest_status.json').write_text(json.dumps(report,indent=2))
    selected=['getup_allocated_near','getup_feedback18_near','getup_feedback18_mixed','getup_full_v6','getup_near_duration']
    fig,axes=plt.subplots(3,1,figsize=(11,10),sharex=True)
    for name in selected:
        path=args.root/name/'rows.jsonl'
        if not path.exists() or not path.stat().st_size:continue
        rows=[json.loads(x) for x in path.read_text().splitlines()]
        x=[r['transitions']/1e6 for r in rows]
        line,=axes[0].plot(x,[100*r['cohort_successes']/r['cohort_trials'] for r in rows],label=name)
        axes[2].plot(x,[r['learner_steps_per_s'] for r in rows],label=name,color=line.get_color())
        run=next(r for r in runs if r['run']==name)
        for d in run['cpu_development']:
            match=next((r for r in rows if r['update']==d['update']),None)
            if match or d['update']==0:
                axes[1].scatter(match['transitions']/1e6 if match else 0.,100*d['passed']/d['trials'],s=28,color=line.get_color())
    axes[0].set_ylabel('GPU cohort success (%)');axes[0].legend(fontsize=8)
    axes[1].set_ylabel('CPU development success (%)')
    axes[2].set_ylabel('valid learner transitions / s');axes[2].set_xlabel('learner transitions (million), within each run')
    for ax in axes:ax.grid(alpha=.25)
    axes[0].set_ylim(0,100);axes[1].set_ylim(0,100)
    fig.suptitle('Supported get-up experiments | different curricula plotted separately | no hardware validation')
    fig.tight_layout();fig.savefig(args.root/'training_progress.png',dpi=150);plt.close(fig)
    print(json.dumps({'updated_at':report['updated_at'],'recorded_transitions_sum':report['recorded_last_transition_counts_sum'],
                      'runs':len(runs),'gpu':gpu}),flush=True)


if __name__=='__main__':main()
