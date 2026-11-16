#!/usr/bin/env python3
"""Isolated CPU/GPU physics smoke and throughput ceiling; not task acceptance."""
import argparse
import json
from pathlib import Path
import sys
import time

import mujoco
import numpy as np
import torch
import warp as wp
import mujoco_warp as mjw

PROJECT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT/'src'))
from fifty_mujoco.envs.sample_env31_realdata import RealDataHardwareEnv31


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--worlds',type=int,default=256)
    parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    torch.set_num_threads(1);wp.init()
    e=RealDataHardwareEnv31();e.reset(610099);e.data.qvel[:]=0
    mujoco.mj_forward(e.model,e.data)
    model=mjw.put_model(e.model);data=mjw.put_data(e.model,e.data,nworld=args.worlds,nconmax=24,njmax=64)
    # One valid fixed-force integration interval tests layout compatibility.
    passive=e._spring_torque(e.source.defaults,np.zeros(6))[0]
    e.data.qfrc_applied[e.source.qvel_ids]=passive
    qfrc=wp.to_torch(data.qfrc_applied);qfrc[:,e.source.qvel_ids]=torch.tensor(passive,device='cuda',dtype=torch.float32)
    start=time.monotonic();mjw.step(model,data);wp.synchronize();compilation=time.monotonic()-start
    mujoco.mj_step(e.model,e.data)
    gpu=wp.to_torch(data.qpos)[0].cpu().numpy()
    max_error=float(np.max(np.abs(gpu-e.data.qpos)))
    with wp.ScopedCapture() as capture:
        for _ in range(4):mjw.step(model,data)
    start=time.monotonic()
    for _ in range(100):wp.capture_launch(capture.graph)
    wp.synchronize();wall=time.monotonic()-start
    report={'worlds':args.worlds,'compile_s':compilation,'one_step_qpos_max_error':max_error,
            'physics_steps_per_s':args.worlds*400/wall,'fixed_force_policy_transition_ceiling_per_s':args.worlds*20/wall,
            'benchmark_s':wall,'scope':'fixed-force physical integration only; no 31D observer, policy, contacts acceptance or reset task parity',
            'task_training_enabled':False}
    args.out.write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)


if __name__=='__main__':main()
