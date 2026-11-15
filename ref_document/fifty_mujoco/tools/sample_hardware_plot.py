#!/usr/bin/env python3
"""Standalone scientific figures for the local hardware transfer campaign."""
import argparse
import os
os.environ.setdefault("MPLCONFIGDIR", "/tmp/zhongzhuang-matplotlib")
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
PROJECT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--selected-report',type=Path);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    scan=json.loads((PROJECT/'reports/sample_weight/hardware_kd_scan_20261115.json').read_text())['rows']
    kds=sorted(set(r['wheel_kd'] for r in scan))
    fig,axes=plt.subplots(1,2,figsize=(10,3.5))
    axes[0].plot(kds,[sum(r['thresholds_met'] for r in scan if r['wheel_kd']==k)/5 for k in kds],'o-',color='#147d92')
    axes[0].set(xlabel='Wheel velocity Kd (Nm / rad/s)',ylabel='Passing fraction (5 x 10s)',ylim=(-.05,1.05),title='Original target limits (ft413 actor)')
    axes[1].plot(kds,[max(max(r['motor_saturation_fractions']) for r in scan if r['wheel_kd']==k)*100 for k in kds],'o-',color='#c85a28')
    axes[1].axhline(5,color='black',ls='--',lw=1,label='5% threshold')
    axes[1].set(xlabel='Wheel velocity Kd (Nm / rad/s)',ylabel='Worst motor saturation (%)',title='First-failure episodes included')
    axes[1].legend(frameon=False)
    for ax in axes:ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(a.out/'controller_screening.png',dpi=180);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(11,7));offset=0.
    for name in ['hardware31_kd010_nominal_20261115','hardware31_kd010_fast_20261115','hardware31_guarded_20261115']:
        folder=PROJECT/'runs'/name
        if not (folder/'rows.jsonl').exists():continue
        rows=[json.loads(l) for l in (folder/'rows.jsonl').read_text().splitlines()]
        x=offset+np.array([r['elapsed_s'] for r in rows])/60
        label={'hardware31_kd010_nominal_20261115':'Reference actuator',
               'hardware31_kd010_fast_20261115':'Scalar actuator (new optimizer)',
               'hardware31_guarded_20261115':'Guarded targets + switching (new optimizer)'}[name]
        axes[0,0].plot(x,[r['final_full_rollout_kl'] for r in rows],label=label)
        axes[0,1].plot(x,[r['entropy'] for r in rows],label=label)
        valid=[(t,r['window_return_mean']) for t,r in zip(x,rows) if r['window_return_mean'] is not None]
        if valid:axes[1,0].plot(*np.array(valid).T,'o-',label=label)
        if (folder/'evaluations.jsonl').exists():
            ev=[json.loads(l) for l in (folder/'evaluations.jsonl').read_text().splitlines()]
            axes[1,1].plot([offset+e['elapsed_s']/60 for e in ev],[e['score'] for e in ev],'o-',label=label)
        offset=x[-1]
    axes[0,0].axhline(.02,color='black',ls='--',lw=1)
    for ax,title,y in zip(axes.flat,['Rollout KL (guard 0.02)','Exploration entropy','Finished episode return','Development tracking score'],
                          ['KL','Entropy','Return','Lower is better']):
        ax.set(title=title,xlabel='Cumulative segment wall time (min)',ylabel=y);ax.grid(alpha=.2)
    axes[0,0].legend(frameon=False,fontsize=8)
    fig.tight_layout();fig.savefig(a.out/'training_evidence.png',dpi=180);plt.close(fig)
    guarded_scan=PROJECT/'reports/sample_weight/hardware_guarded_kd_scan_20261115.json'
    if guarded_scan.exists():
        scan=json.loads(guarded_scan.read_text())['summaries']
        fig,axes=plt.subplots(1,2,figsize=(10,3.5))
        axes[0].plot([r['kd'] for r in scan],[r['passed']/r['trials'] for r in scan],'o-')
        axes[0].set(xlabel='Wheel Kd',ylabel='Passing fraction',ylim=(-.05,1.05),title='Stroke-based target limits (v4 update50)')
        axes[1].plot([r['kd'] for r in scan],[r['hard_failures'] for r in scan],'o-',color='#c85a28')
        axes[1].set(xlabel='Wheel Kd',ylabel='First physical failures / 9 trials',title='Physical failure thresholds unchanged')
        for ax in axes:ax.grid(alpha=.2)
        fig.tight_layout();fig.savefig(a.out/'guarded_controller_screening.png',dpi=180);plt.close(fig)
    if a.selected_report:
        d=json.loads(a.selected_report.read_text());rows=[r for r in d['rows'] if r.get('condition')=='nominal']
        fig,axes=plt.subplots(1,3,figsize=(12,3.5));names=['stand','forward','backward','left','right']
        for ax,key,limit,label in zip(axes,['point_vx_rmse_m_s','yaw_rate_rmse_rad_s','clearance_mae_m'],[.1,.15,.02],
                                     ['Forward RMSE (m/s)','Yaw RMSE (rad/s)','Clearance MAE (m)']):
            values=[[r['metrics'].get(key,np.nan) for r in rows if r['scenario']==name] for name in names]
            ax.boxplot(values,tick_labels=names,showfliers=True)
            ax.axhline(limit,color='black',ls='--',lw=1);ax.set(ylabel=label,title='40s heldout episodes');ax.tick_params(axis='x',rotation=30);ax.grid(axis='y',alpha=.2)
        fig.tight_layout();fig.savefig(a.out/'heldout_tracking.png',dpi=180);plt.close(fig)
    print(a.out)


if __name__=='__main__':main()
