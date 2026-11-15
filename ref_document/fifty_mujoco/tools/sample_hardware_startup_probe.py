#!/usr/bin/env python3
"""Separate supported-pose release tests from nominal standing acceptance.

The spring-only motor requirement is a quasi-static, zero-external-load bound;
body weight/contact can change it. It is not a measured motor calibration.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path
import numpy as np
import torch
PROJECT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT/'src'))
from sample_hardware_evaluate import episode
from fifty_mujoco.envs.sample_env31_hardware import SampleHardwareEnv31
from fifty_mujoco.learning.sample_ppo import SamplePolicy31
from fifty_mujoco.transfer.sample_source import verified_artifact


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--wheel-kd',type=float)
    parser.add_argument('--guarded',action='store_true')
    args=parser.parse_args()
    if args.guarded:
        import sample_hardware_evaluate as serial
        from fifty_mujoco.envs.sample_env31_hardware_guarded import GuardedHardwareEnv31
        serial.SampleTargetEnv31=GuardedHardwareEnv31
    torch.set_num_threads(1)
    policy=SamplePolicy31(verified_artifact('model_3050.pt'))
    policy.load_state_dict(torch.load(args.checkpoint,map_location='cpu',weights_only=True)['model'])
    rows=[]
    for kd in ((args.wheel_kd,) if args.wheel_kd is not None else (.3,.1)):
        for leg_gain in (1.,.8):
            gain=[leg_gain,leg_gain,1.,leg_gain,leg_gain,1.]
            rows.append(episode(policy.actor,'stand',85000,10,'natural',gain,wheel_kd=kd))
    env=SampleHardwareEnv31()
    nominal=env.source.defaults
    natural=np.asarray(env.source.profile['natural_joint_pose_rad'])
    static=[]
    for fraction in np.linspace(0,1,41):
        q=nominal+fraction*(natural-nominal)
        passive,lengths=env.source.spring_torque(q,np.zeros(6))
        required,_,_=env.actuators.inner.to_motor(torch.tensor(-passive)[None],torch.tensor(q)[None],torch.zeros((1,6)))
        static.append({'natural_pose_fraction':float(fraction),'q':q.tolist(),'pin_lengths_m':lengths,
            'spring_joint_torque_nm':passive.tolist(),'spring_only_balancing_motor_torque_nm':required[0].tolist()})
    report={'tool':'hardware-startup-probe','status':'diagnostic_not_acceptance',
        'checkpoint':str(args.checkpoint),'guarded_targets':args.guarded,'checkpoint_sha256':hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
        'release_rows':rows,'spring_only_static_scan':static,
        'assumptions':['Recorded reference pose was externally supported; release test deliberately removes that support.',
            'Static scan balances only the modeled gas spring: excludes weight, ground reaction, friction and dynamics.',
            '0.8 DM gain is a feedback-derived hypothesis, not a calibrated measurement.',
            'No hardware access, motor movement or firmware changes.'],
        'target_model_accepted':False}
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'release_results':[(r['wheel_kd'],r['motor_gain'][0],r['elapsed_s'],r['first_failure']) for r in rows],
        'natural_pose_spring_only_max_motor_nm':max(abs(v) for v in static[-1]['spring_only_balancing_motor_torque_nm'])}))

if __name__=='__main__':main()
