#!/usr/bin/env python3
"""Continuous CPU protocol trace including STOP and frozen balance ticks."""
import argparse
import json
import os
from pathlib import Path
import subprocess
os.environ.setdefault('MUJOCO_GL','osmesa')
os.environ.setdefault('LP_NUM_THREADS','1')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from PIL import Image,ImageDraw
import torch

from realdata_train import SamplePolicy31,verified_artifact,digest
from realdata_allocator import AllocatorActor
from fifty_mujoco.envs.sample_env31_supported import SupportedGetupEnv31


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--checkpoint',type=Path,required=True);ap.add_argument('--balance',type=Path,required=True)
    ap.add_argument('--config',type=Path);ap.add_argument('--allocator',action='store_true')
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--video',action='store_true')
    ap.add_argument('--seed',type=int,default=618601);ap.add_argument('--reset-range',nargs=2,type=float,default=[1,1])
    args=ap.parse_args();torch.set_num_threads(1)
    saved=torch.load(args.checkpoint,map_location='cpu',weights_only=True)
    cfg=json.loads(args.config.read_text()) if args.config else saved.get('configuration',saved.get('contract',{}).get('configuration'))
    policy=SamplePolicy31(verified_artifact('model_3050.pt'),'cpu');policy.load_state_dict(saved['model'])
    balanced=SamplePolicy31(verified_artifact('model_3050.pt'),'cpu')
    balanced.load_state_dict(torch.load(args.balance,map_location='cpu',weights_only=True)['model'])
    if args.allocator:policy.actor=AllocatorActor(policy.actor,cfg)
    env=SupportedGetupEnv31(balanced.actor,reset_range=args.reset_range,settle_seconds=1.,config=cfg)
    rows=[];phases=[];states=[];observations=[];observations_after=[];actions=[];motor=[];truth=[]
    advance=env._advance
    def record(action,scale):
        phase=env.controller.phase.value
        input_observation=env.env.observation().copy()
        obs,info,sample=advance(action,scale)
        rows.append([sample.time_s,sample.clearance_m,sample.forward_velocity_m_s,sample.displacement_m,
                     *sample.roll_pitch_deg,*sample.angular_velocity_rad_s,*env.support_wrench,
                     *env.wheel_load_n.values(),float(env.env.c_quality.confidence)])
        phases.append(phase);states.append(env.data.qpos.copy());observations.append(input_observation)
        observations_after.append(obs.copy())
        actions.append(env.env.last_action.copy());motor.append(env.env.fast_actuators.command.copy())
        truth.append([sample.both_wheels_supported,sample.body_contact,sample.telemetry_valid,
                      sample.joint_limits_valid,sample.spring_stroke_valid,sample.linkage_branch_valid,sample.forbidden_contact])
        return obs,info,sample
    env._advance=record;obs=env.reset(args.seed)
    for _ in range(env.max_steps):
        with torch.no_grad():action=policy.actor(torch.tensor(obs)[None])[0].numpy()
        obs,_,done,truncated,info=env.step(action)
        if done or truncated:break
    args.out.parent.mkdir(parents=True,exist_ok=True)
    array=np.asarray(rows);trajectory=args.out.with_suffix('.npz')
    np.savez_compressed(trajectory,metrics=array,phase=np.asarray(phases),qpos=np.asarray(states),
                        observation=np.asarray(observations),observation_after=np.asarray(observations_after),
                        action=np.asarray(actions),motor_command=np.asarray(motor),physical_truth=np.asarray(truth,dtype=bool))
    figure,axes=plt.subplots(4,1,figsize=(11,9),sharex=True)
    t=array[:,0];axes[0].plot(t,array[:,1],label='clearance');axes[0].axhline(cfg['standing_clearance_m'],c='k',ls='--')
    axes[0].set_ylabel('height (m)')
    axes[1].plot(t,array[:,2],label='estimated vx');axes[1].plot(t,array[:,3],label='displacement')
    axes[1].axhline(.08,c='r',ls='--');axes[1].axhline(-.08,c='r',ls='--');axes[1].legend(loc='upper left')
    axes[1].set_ylabel('m/s ; m')
    axes[2].plot(t,array[:,4],label='roll');axes[2].plot(t,array[:,5],label='pitch');axes[2].legend()
    axes[2].set_ylabel('angle (deg)')
    axes[3].plot(t,np.linalg.norm(array[:,9:12],axis=1),label='support force N')
    axes[3].plot(t,np.linalg.norm(array[:,12:15],axis=1),label='support moment Nm');axes[3].legend()
    axes[3].set_ylabel('support');axes[3].set_xlabel('physical simulation time (s)')
    for ax in axes:
        ax.grid(alpha=.25)
        for phase,color in [('STOP','#cccccc'),('GETUP','#f0c06e'),('HANDOFF','#91c8e8'),('BALANCE','#9dcc98')]:
            where=np.asarray(phases)==phase
            if where.any():ax.axvspan(t[where][0],t[where][-1],color=color,alpha=.15)
    figure.suptitle(f'CPU development trace | teacher={args.allocator} | success={info["success"]} | seed={args.seed}')
    figure.tight_layout();figure.savefig(args.out.with_suffix('.png'),dpi=150);plt.close(figure)
    video_path=None
    if args.video:
        video_path=args.out.with_suffix('.mp4')
        for i in range(env.model.ngeom):
            name=env.model.geom(i).name
            env.model.geom_rgba[i]=[.8,.82,.85,1] if name=='floor' else ([.12,.13,.15,1] if 'wheel' in name else [.95,.5,.12,1])
        renderer=mujoco.Renderer(env.model,height=480,width=640)
        camera=mujoco.MjvCamera();camera.distance=1.5;camera.azimuth=110;camera.elevation=-18
        process=subprocess.Popen(['ffmpeg','-v','error','-n','-f','rawvideo','-pix_fmt','rgb24','-s','640x480',
                                  '-r','25','-i','pipe:0','-an','-c:v','libx264','-threads','2','-crf','22',
                                  '-pix_fmt','yuv420p',str(video_path)],stdin=subprocess.PIPE)
        for i in range(0,len(states),4):
            env.data.qpos[:]=states[i];mujoco.mj_forward(env.model,env.data)
            camera.lookat[:]=[*states[i][:2],.25];renderer.update_scene(env.data,camera=camera)
            frame=Image.fromarray(renderer.render());draw=ImageDraw.Draw(frame)
            draw.rectangle([0,0,640,68],fill=(18,24,33))
            draw.text((10,8),f'CPU SIMULATION / COLLISION GEOMETRY | {phases[i]} | teacher={args.allocator}',fill='white')
            draw.text((10,28),f't={array[i,0]:.2f}s  height={array[i,1]:.3f}m vx={array[i,2]:+.3f}m/s',fill='white')
            draw.text((10,48),f'Fsupport={np.linalg.norm(array[i,9:12]):.2f}N Msupport={np.linalg.norm(array[i,12:15]):.2f}Nm',fill='white')
            process.stdin.write(np.asarray(frame).tobytes())
        process.stdin.close()
        if process.wait()!=0:raise RuntimeError('video encoding failed')
        renderer.close()
    report={'checkpoint':str(args.checkpoint),'checkpoint_sha256':digest(args.checkpoint),
            'balance_sha256':digest(args.balance),'configuration':cfg,'seed':args.seed,'split':'development',
            'teacher_allocator':args.allocator,'reset_range':args.reset_range,'result':info,
            'continuous_ticks':len(rows),'automatic_reset':False,'trajectory_sha256':digest(trajectory),
            'video':str(video_path) if video_path else None,
            'columns':['time_s','clearance_m','estimated_vx_m_s','displacement_m','roll_deg','pitch_deg',
                       'wx','wy','wz','support_fx','support_fy','support_fz','support_mx','support_my','support_mz',
                       'right_wheel_load_n','left_wheel_load_n','odometry_confidence'],
            'observation_timing':'observation before applied action; observation_after is post-step',
            'physical_truth_columns':['both_wheels_supported','body_contact','telemetry_valid','joint_limits_valid',
                                      'spring_stroke_valid','linkage_branch_valid','forbidden_contact'],
            'scope':'Selected development replay, not independent acceptance or hardware validation.'}
    args.out.with_suffix('.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({'success':info['success'],'failure':info['failure'],'phase_counts':info['phase_counts'],
                      'trace':str(args.out),'video':str(video_path)}),flush=True)


if __name__=='__main__':main()
