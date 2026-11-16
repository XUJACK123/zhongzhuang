#!/usr/bin/env python3
"""Development-only scan of feasible standing targets and teacher feedback."""
import copy
import itertools
import json
import math
from pathlib import Path
import time

import mujoco
import numpy as np
import torch

from realdata_train import RealDataHardwareEnv31, configuration, equilibrium_action, rpy_from_wxyz


def main():
    torch.set_num_threads(1)
    out=Path('runs/realdata_20261117');rows=[];start=time.monotonic()
    for height in (.32,.35,.38,.41,.44):
        cfg=copy.deepcopy(configuration())
        x,z=-.0785237,height-cfg['wheel_radius_m'];a,b=.21,.25
        delta=math.acos((x*x+z*z-a*a-b*b)/(2*a*b))
        hip=math.atan2(z,x)+math.atan2(b*math.sin(delta),a+b*math.cos(delta));knee=-delta-math.pi/2
        cfg['standing_default_rad']=[hip,knee,0,-hip,-knee,0]
        cfg['standing_clearance_m']=height+.0482748097777482-.2232509031891825
        cfg['standing_provenance']=f'Development target: {height} m hip height, CAD IK, inherited COM x; not measured.'
        cfg['support']['target_root_height_m']=height
        path=out/f'profile_height_{round(height*1000)}.json';path.write_text(json.dumps(cfg,indent=2))
        e=RealDataHardwareEnv31(config=cfg,episode_seconds=12)
        base=equilibrium_action(e)
        for kp,kd,kr in itertools.product((2.,5.,10.),(.2,.5,1.),(1.,3.,6.)):
            obs=e.reset(610090);e.data.qvel[:]=0
            samples=[]
            for step in range(1200):
                roll,pitch,yaw=rpy_from_wxyz(e.data.qpos[3:7]);omega=e._velocity6()[:3]
                action=base.copy();action[[0,3]] += [-2*pitch,2*pitch]
                action[[1,4]] += kr*roll+.4*omega[0]
                action[[2,5]]=kp*pitch+kd*omega[1]+.5*e.estimated_velocity[0]
                obs,reward,done,timeout,info=e.step(action)
                if step>200:samples.append([abs(info['clearance_m']-cfg['standing_clearance_m']),*np.abs(info['roll_pitch_deg'])])
                if done or timeout:break
            values=np.asarray(samples) if samples else np.full((1,3),1000.)
            row={'height_m':height,'kp':kp,'kd':kd,'kr':kr,'survival_s':info['time_s'],
                 'failure':info['failure'],'clearance_mae_m':float(values[:,0].mean()),
                 'tilt_p95_deg':float(np.percentile(values[:,1:],95)), 'equilibrium_action':base.tolist()}
            rows.append(row)
        best=sorted((r for r in rows if r['height_m']==height),key=lambda r:(-r['survival_s'],r['clearance_mae_m']))[:3]
        print(json.dumps({'height_m':height,'best':best,'wall_s':time.monotonic()-start}),flush=True)
        (out/'teacher_height_scan.json').write_text(json.dumps(rows,indent=2))


if __name__=='__main__':main()
