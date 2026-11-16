#!/usr/bin/env python3
"""Independently recompute acceptance gates from a saved CPU trajectory."""
import argparse
import json
from pathlib import Path
import numpy as np
from realdata_train import digest


def audit_trace(trace):
    report=json.loads(trace.with_suffix('.json').read_text())
    file=trace.with_suffix('.npz');data=np.load(file);cfg=report['configuration']
    if digest(file)!=report['trajectory_sha256']:raise ValueError('trajectory hash changed')
    m=data['metrics'];phase=data['phase'];q=data['qpos'][:,-6:];truth=data['physical_truth']
    def check(mask,velocity):
        if not mask.any():return False
        a=m[mask];flags=truth[mask]
        leg=q[mask][:,[0,1,3,4]]-np.asarray(cfg['standing_default_rad'])[[0,1,3,4]]
        leg[:,[0,2]]=np.arctan2(np.sin(leg[:,[0,2]]),np.cos(leg[:,[0,2]]))
        return bool(np.all(np.abs(a[:,1]-cfg['standing_clearance_m'])<=.02+1e-9)
                and np.all(np.abs(a[:,2])<=velocity+1e-9) and np.all(a[:,3]<=.3+1e-9)
                and np.all(np.abs(a[:,4:6])<=8+1e-9) and np.all(np.abs(a[:,6:9])<=.5+1e-9)
                and np.all(np.abs(leg)<=.15+1e-9) and np.all(np.abs(a[:,9:15])<=1e-6)
                and np.all(a[:,15:17]>=.02*cfg['mass_kg']*9.81-1e-9) and np.all(a[:,17]>=.5)
                and np.all(flags[:,0]) and not np.any(flags[:,1]) and np.all(flags[:,2:6])
                and not np.any(flags[:,6]))
    handoff=phase=='HANDOFF';balance=phase=='BALANCE';stop=phase=='STOP'
    dwell=np.zeros(len(phase),dtype=bool)
    if handoff.any():
        end=m[handoff,0][0]-.01
        dwell=(phase=='GETUP')&(m[:,0]>=end-cfg['success']['stable_dwell_s']-1e-8)&(m[:,0]<=end+1e-8)
    passed={'finite':bool(np.isfinite(m).all() and np.isfinite(q).all()),
            'continuous_100hz_clock':bool(np.allclose(np.diff(m[:,0]),.01,atol=1e-8,rtol=0)),
            'zero_active_stop':bool(np.all(np.abs(data['motor_command'][stop])<=1e-6)),
            'motor_caps':bool(np.all(np.abs(data['motor_command'])<=np.asarray([20,20,2,20,20,2])+1e-6)),
            'action_caps':bool(np.all(np.abs(data['action'])<=np.asarray([1,1,2,1,1,2])+1e-6)),
            'unassisted_dwell':bool(dwell.sum()>=round(cfg['success']['stable_dwell_s']/.01)
                                      and check(dwell,cfg.get('handoff_velocity_limit_m_s',.08))),
            'handoff':bool(handoff.sum()>=round(cfg['success']['blend_s']/.01)
                                and check(handoff,cfg.get('blend_velocity_limit_m_s',cfg.get('handoff_velocity_limit_m_s',.08)))),
            'frozen_balance':bool(balance.sum()>=round(cfg['success']['balance_s']/.01) and check(balance,.08))}
    if report.get('physical_knee_limits_rad'):
        limits=np.asarray(report['physical_knee_limits_rad']);knees=q[:,[1,4]]
        passed['physical_knee_boundaries']=bool(np.all(knees>=limits[:,0]-1e-4) and np.all(knees<=limits[:,1]+1e-4))
    result={'scope':'Numerical audit of saved CPU simulation signals; seed/weight provenance must be audited separately.',
            'trace_sha256':digest(file),'checks':passed,'passed':all(passed.values()),
            'dwell_ticks':int(dwell.sum()),'handoff_ticks':int(handoff.sum()),'balance_ticks':int(balance.sum())}
    trace.with_suffix('.audit.json').write_text(json.dumps(result,indent=2))
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--trace',type=Path,required=True)
    args=ap.parse_args();result=audit_trace(args.trace);print(json.dumps(result),flush=True)
    if not result['passed']:raise SystemExit(1)


if __name__=='__main__':main()
