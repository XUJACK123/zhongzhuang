#!/usr/bin/env python3
"""Render one uninterrupted simulation and retain its numerical trajectory.

Set MUJOCO_GL=osmesa for CPU headless rendering. No simulator reset is performed
inside a recording; first failure or requested episode timeout ends simulation.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
os.environ.setdefault('MUJOCO_GL','osmesa')
os.environ.setdefault('LP_NUM_THREADS','1')
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import torch
PROJECT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT/'src'))
from fifty_mujoco.envs.sample_env31_hardware import SampleHardwareEnv31
from fifty_mujoco.envs.sample_env31_hardware_guarded import GuardedHardwareEnv31
from fifty_mujoco.learning.sample_ppo import SamplePolicy31
from fifty_mujoco.transfer.sample_source import verified_artifact


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=40);p.add_argument('--seed',type=int,default=87000)
    p.add_argument('--scenario',choices=['stand','forward','reversal','turning','natural_release'],default='stand')
    p.add_argument('--wheel-kd',type=float,help='explicit evaluated deployment gain override')
    a=p.parse_args();torch.set_num_threads(1)
    checkpoint=torch.load(a.checkpoint,map_location='cpu',weights_only=True)
    policy=SamplePolicy31(verified_artifact('model_3050.pt'));policy.load_state_dict(checkpoint['model'])
    kd=checkpoint['contract']['wheel_kd'] if a.wheel_kd is None else a.wheel_kd
    guarded=checkpoint['contract']['environment']=='sample_commissioned_guarded_cpu31_v4'
    cls=GuardedHardwareEnv31 if guarded else SampleHardwareEnv31
    options={'knee_margin_rad':checkpoint['contract']['knee_margin_rad']} if guarded else {}
    env=cls(a.seconds,reset_mode='natural' if a.scenario=='natural_release' else 'default',wheel_kd=kd,**options)
    protocol=json.loads((PROJECT/'configs/sample_command_evaluation.json').read_text())
    obs=env.reset(a.seed,(.15 if a.scenario=='forward' else 0.,0.,.1))
    # Visualization properties only; physical parameters are unchanged.
    for i in range(env.model.ngeom):
        name=env.model.geom(i).name
        color=[.82,.84,.88,1.] if name=='floor' else ([.1,.4,.75,.45] if name.startswith('base_link') else ([.10,.11,.13,1.] if 'wheel' in name else [.9,.48,.08,1.]))
        env.model.geom_rgba[i]=color
    env.model.vis.headlight.ambient[:]=.6;env.model.vis.headlight.diffuse[:]=.8
    renderer=mujoco.Renderer(env.model,height=480,width=640)
    camera=mujoco.MjvCamera();camera.distance=1.4;camera.azimuth=120;camera.elevation=-18
    a.out.parent.mkdir(parents=True,exist_ok=True)
    if a.out.exists():raise ValueError('use a new output filename')
    video=subprocess.Popen(['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','rgb24',
        '-s','640x480','-r','25','-i','pipe:0','-an','-c:v','libx264','-preset','fast','-threads','2','-crf','22',
        '-pix_fmt','yuv420p',str(a.out)],stdin=subprocess.PIPE)
    rows=[];observations=[];actions=[];states=[];frames=0;info=None
    for tick in range(env.max_steps):
        elapsed=tick*env.source.policy_dt
        if a.scenario=='reversal':env.request([protocol['reversal_5s_segments'][min(7,int(elapsed/5))],0.,.1])
        if a.scenario=='turning':env.request([*protocol['turning_5s_segments'][min(7,int(elapsed/5))],.1])
        with torch.no_grad():action=policy.actor(torch.tensor(obs)[None])[0].numpy()
        observations.append(obs.copy());actions.append(action.copy())
        obs,_,d,t,info=env.step(action);states.append(env.data.qpos.copy())
        rows.append([info['time_s'],*info['applied_command'],*info['point_velocity_xy'],info['yaw_rate'],
            info['clearance_m'],*info['roll_pitch_deg'],min(info['spring_margins_m']),
            *info['odometry_error_xy_m'],float(d),float(t)])
        if tick%4==0 or d or t:
            camera.lookat[:]=[*env.data.qpos[:2],.25]
            renderer.update_scene(env.data,camera=camera)
            frame=Image.fromarray(renderer.render());draw=ImageDraw.Draw(frame)
            draw.rectangle([0,0,640,73],fill=(18,24,33))
            draw.text((12,8),f'SIMULATION / COLLISION MODEL | {a.scenario} | Kd={kd:.2f}',fill='white')
            draw.text((12,29),f't={info["time_s"]:5.2f}s  vx={info["point_velocity_xy"][0]:+.3f}m/s  yaw={info["yaw_rate"]:+.3f}rad/s',fill='white')
            draw.text((12,50),f'clearance={info["clearance_m"]:.3f}m  spring margin={min(info["spring_margins_m"])*1000:.1f}mm',fill='white')
            video.stdin.write(np.asarray(frame).tobytes());frames+=1
        if d or t:break
    video.stdin.close()
    if video.wait()!=0:raise RuntimeError('ffmpeg failed')
    renderer.close()
    trajectory=a.out.with_suffix('.trajectory.npz')
    np.savez_compressed(trajectory,metrics=np.asarray(rows),observation=np.asarray(observations),
        action=np.asarray(actions),qpos=np.asarray(states))
    report={'tool':'hardware-trajectory-video','checkpoint':str(a.checkpoint),'checkpoint_sha256':hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),
        'scenario':a.scenario,'seed':a.seed,'wheel_kd':kd,'requested_seconds':a.seconds,'simulation_seconds':info['time_s'],
        'first_failure':info['failure'],'frames':frames,'fps':25,'video_seconds':frames/25,
        'automatic_reset':False,'video_sha256':hashlib.sha256(a.out.read_bytes()).hexdigest(),
        'trajectory':str(trajectory),'trajectory_sha256':hashlib.sha256(trajectory.read_bytes()).hexdigest(),
        'columns':['time_s','command_vx','command_wz','command_clearance','point_vx','point_vy','yaw_rate',
                   'clearance','roll_deg','pitch_deg','minimum_spring_margin','odometry_error_x','odometry_error_y','terminated','truncated'],
        'real_robot_accepted':False,'visualization_changes_only':'geom RGBA/headlight/camera'}
    a.out.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)


if __name__=='__main__':main()
