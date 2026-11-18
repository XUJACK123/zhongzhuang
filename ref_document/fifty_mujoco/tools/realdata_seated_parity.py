#!/usr/bin/env python3
"""Compare seated reset and short action playback on both simulation backends."""

import argparse
import copy
import json
from pathlib import Path

import numpy as np
import torch

from fifty_mujoco.envs.realdata_warp_getup import WarpSupportedGetup
from fifty_mujoco.envs.sample_env31_supported import SupportedGetupEnv31
from realdata_seated_state_teacher import load_policy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('config','balance','out'):
        parser.add_argument(f'--{name}',required=True,type=Path)
    args = parser.parse_args()
    torch.set_num_threads(1)
    torch.manual_seed(811990)
    cfg = copy.deepcopy(json.loads(args.config.read_text()))
    cfg.pop('gyro_std_rad_s', None)
    cfg['seated_back']['reset_bank'] = cfg['seated_back']['reset_bank'][:1]
    cfg['getup_ramp_range_s'] = [.2,.2]
    cpu = SupportedGetupEnv31(load_policy(args.balance,'cpu'),config=cfg,settle_seconds=1.)
    gpu = WarpSupportedGetup(load_policy(args.balance,'cuda'),8,config=cfg,settle_seconds=1.)
    cpu.reset(911030)
    gpu.reset()
    quiet = torch.zeros((8,6),device='cuda')
    with torch.no_grad():
        for _ in range(100):
            gpu.step(quiet)
    rows = []

    def record(tick):
        qp, qv = gpu.plant.qpos[0].cpu().numpy(), gpu.plant.qvel[0].cpu().numpy()
        obs_cpu = cpu.env.observation()
        obs_gpu = gpu.observation[0].cpu().numpy()
        return {'tick_after_reset':tick, 'root_max_difference': float(abs(qp[:3]-cpu.data.qpos[:3]).max()),
                'joint_pos_max_difference': float(abs(qp[7:]-cpu.data.qpos[7:]).max()),
                'velocity_max_difference':float(abs(qv-cpu.data.qvel).max()),
                'observation_max_difference':float(abs(obs_cpu-obs_gpu).max()),
                'cpu_wheel_load_n':list(cpu.wheel_load_n.values()),'gpu_wheel_load_n':gpu.wheel_load[0].tolist(),
                'cpu_phase':cpu.decision.phase.value, 'gpu_phase':int(gpu.phase[0]),
                'cpu_fault':cpu.decision.failure, 'gpu_fault_code':int(gpu.first_failure[0]),
                'cpu_support_peak':[cpu.support_peak_n,cpu.support_peak_nm],
                'gpu_support_peak':gpu.support_peak[0].tolist()}

    rows.append(record(0))
    with torch.no_grad():
        for i in range(1,31):
            action = np.array([.15,.8,-1.4,-.15,-.8,-1.4],dtype=np.float32)
            cpu.step(action)
            gpu.step(torch.as_tensor(action,device='cuda').expand(8,6))
            rows.append(record(i))
    report = {'same_reset_and_200ms_ramp':True,'gyro_noise_disabled_for_comparison':True,'rows':rows,
              'meaning':'Short matching action replay; CPU closes the actual C observer. Contact trajectory differences are reported, not treated as exact parity.'}
    args.out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'initial':rows[0],'after_300ms':rows[-1]}))


if __name__ == '__main__':
    main()
