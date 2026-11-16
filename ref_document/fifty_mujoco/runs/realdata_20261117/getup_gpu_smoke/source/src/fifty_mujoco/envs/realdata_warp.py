"""Batched CUDA candidate plant with the 31-field observation and C quality law.

This backend must pass short CPU comparisons and every exported policy must be
evaluated on the CPU C-observer plant. GPU contact differences remain explicit.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np
import torch
import warp as wp
import mujoco_warp as mjw

from .sample_env31_realdata import RealDataHardwareEnv31


def rotation(quat):
    w,x,y,z=quat.unbind(-1)
    return torch.stack((1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),
                        2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),
                        2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)),-1).reshape(-1,3,3)


def angles(quat):
    w,x,y,z=quat.unbind(-1)
    return torch.stack((torch.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
                        torch.asin((2*(w*y-z*x)).clamp(-1,1)),
                        torch.atan2(2*(w*z+x*y),1-2*(y*y+z*z))),-1)


def wrap(value):
    return torch.atan2(torch.sin(value),torch.cos(value))


class QualifiedObserver:
    """Float32 batched port of state_estimator.c under fresh ideal feedback."""
    def __init__(self,num_envs,device,radius):
        self.radius=radius
        self.xy=torch.zeros((num_envs,2),device=device)
        self.velocity=torch.zeros((num_envs,3),device=device)
        self.predicted=torch.zeros_like(self.velocity)
        self.qd=torch.zeros((num_envs,6),device=device)
        self.last_q=torch.zeros_like(self.qd)
        self.confidence=torch.ones(num_envs,device=device)
        self.residual=torch.zeros(num_envs,device=device)
        self.degraded=torch.zeros(num_envs,device=device)
        self.realign=torch.zeros(num_envs,dtype=torch.bool,device=device)
        self.valid=torch.ones(num_envs,dtype=torch.bool,device=device)

    def update(self,q,qd,quat,omega,acceleration_world):
        dt=.005
        delta=q-self.last_q
        delta=torch.stack((wrap(delta[:,0]),delta[:,1],delta[:,2],wrap(delta[:,3]),delta[:,4],delta[:,5]),-1)
        self.qd.copy_(delta/dt);self.last_q.copy_(q)
        r=rotation(quat);axle=r[:,:,1]
        norm=torch.sqrt((1-axle[:,2].square()).clamp(min=1e-12))
        contact=self.radius*torch.stack((axle[:,2]*axle[:,0],axle[:,2]*axle[:,1],-(1-axle[:,2].square())),-1)/norm[:,None]
        wheels=[]
        for h,k,sign,lateral in ((0,1,1,-.260797767),(3,4,-1,.268202233)):
            a=sign*q[:,h];b=sign*(q[:,h]+q[:,k])+math.pi/2
            ad=sign*qd[:,h];bd=sign*(qd[:,h]+qd[:,k])
            zero=torch.zeros_like(a)
            point=torch.stack((.21*torch.cos(a)+.25*torch.cos(b),zero+lateral,-.21*torch.sin(a)-.25*torch.sin(b)),-1)
            pd=torch.stack((-.21*torch.sin(a)*ad-.25*torch.sin(b)*bd,zero,-.21*torch.cos(a)*ad-.25*torch.cos(b)*bd),-1)
            relative=torch.cross(omega,point,dim=-1)+pd
            relative_world=torch.bmm(r,relative[:,:,None]).squeeze(-1)
            spin=omega+torch.stack((zero,bd+qd[:,h+2],zero),-1)
            spin_world=torch.bmm(r,spin[:,:,None]).squeeze(-1)
            wheels.append(-torch.cross(spin_world,contact,dim=-1)-relative_world)
        first,second=wheels
        average=(first+second)/2
        predicted=self.predicted+acceleration_world.clamp(-10,10)*dt
        self.residual.copy_(torch.linalg.vector_norm(first-second,dim=-1))
        weight0=1/(1+(first-predicted).square().sum(-1)/.3**2)
        weight1=1/(1+(second-predicted).square().sum(-1)/.3**2)
        total=weight0+weight1
        confidence=torch.where(self.residual<=.3,total.clamp(max=1),.5*torch.maximum(weight0,weight1))
        trusted=(confidence>=.25)&(total>.001)
        self.realign.copy_(self.realign | (trusted & (self.degraded>0)))
        self.degraded.copy_(torch.where(trusted,0,self.degraded+5))
        velocity=torch.where(trusted[:,None],(weight0[:,None]*first+weight1[:,None]*second)/total[:,None].clamp(min=1e-12),
                             torch.where((self.degraded<=100)[:,None],predicted,0))
        self.xy.add_(torch.where(trusted[:,None],velocity[:,:2]*dt,0))
        self.predicted.copy_(velocity);self.velocity.copy_(torch.bmm(r.transpose(1,2),velocity[:,:,None]).squeeze(-1))
        self.confidence.copy_(confidence)
        self.valid.copy_((norm>.5)&torch.isfinite(average).all(-1)&(average.abs().amax(-1)<=5))


class RealDataWarpEnv:
    def __init__(self,num_envs=512,config=None,episode_seconds=12.,capture=True,auto_reset=False):
        wp.init();self.num_envs=num_envs;self.device=torch.device('cuda')
        self.auto_reset=auto_reset
        self.cpu=RealDataHardwareEnv31(config=config,episode_seconds=episode_seconds)
        self.cfg=self.cpu.candidate;self.cpu.reset(610099);self.cpu.data.qvel[:]=0
        mujoco.mj_forward(self.cpu.model,self.cpu.data)
        self.model=mjw.put_model(self.cpu.model)
        self.data=mjw.put_data(self.cpu.model,self.cpu.data,nworld=num_envs,nconmax=24,njmax=64)
        self.qpos=wp.to_torch(self.data.qpos);self.qvel=wp.to_torch(self.data.qvel)
        self.qacc=wp.to_torch(self.data.qacc);self.qfrc=wp.to_torch(self.data.qfrc_applied)
        self.warmstart=wp.to_torch(self.data.qacc_warmstart)
        self.xfrc=wp.to_torch(self.data.xfrc_applied)
        self.qp=torch.tensor(self.cpu.source.qpos_ids,device=self.device)
        self.qv=torch.tensor(self.cpu.source.qvel_ids,device=self.device)
        self.knee_idx=torch.tensor([1,4],device=self.device)
        self.leg_idx=torch.tensor([0,1,3,4],device=self.device)
        self.knee_qp=torch.tensor(self.cpu.source.qpos_ids[[1,4]],device=self.device)
        self.leg_qp=torch.tensor(self.cpu.source.qpos_ids[[0,1,3,4]],device=self.device)
        def tensor(value):return torch.tensor(value,device=self.device,dtype=torch.float32)
        self.default=tensor(self.cpu.source.defaults);self.limit=tensor(self.cpu.target_limits)
        self.point_offset=tensor([-.07828434314112345,.00370225,0.])
        self.clearance_center=tensor(self.cpu.source.env['env']['clearance_box_center'])
        self.clearance_half=tensor(self.cpu.source.env['env']['clearance_box_halfsize'])
        self.physical_limit=tensor(self.cpu.limits)
        self.effort=tensor(self.cpu.source.profile['joint_effort_nm'])
        self.cap=tensor([20,20,2,20,20,2])
        self.rated=tensor(self.cpu.fast_actuators.rated);self.free=tensor(self.cpu.fast_actuators.free)
        self.kp=tensor([40,60,0,40,60,0]);self.kd=tensor([2,2,self.cfg['wheel_kd'],2,2,self.cfg['wheel_kd']])
        self.bound=tensor([1,1,2,1,1,2]);self.ratio=self.cpu.fast_actuators.ratio
        coefficients=[]
        for spring in self.cpu.source.spring_config['springs']:
            f=lambda q:self.cpu.source.spring_module.geometry(q,spring)[0]**2
            a=(f(0)+f(math.pi))/2;b=(f(0)-f(math.pi))/2;c=f(math.pi/2)-a
            coefficients.append([a,b,c])
        self.spring_coeff=tensor(coefficients)
        self.action_in=torch.zeros((num_envs,6),device=self.device)
        self.torque_scale=torch.ones(num_envs,device=self.device)
        self.motor_gain=torch.ones((num_envs,6),device=self.device)
        self.spring_scale=torch.ones(num_envs,device=self.device)
        self.last_action=torch.zeros_like(self.action_in)
        self.obs=torch.zeros((num_envs,31),device=self.device)
        self.terminal_obs=torch.zeros_like(self.obs)
        self.reward=torch.zeros(num_envs,device=self.device)
        self.failure=torch.zeros(num_envs,device=self.device,dtype=torch.int32)
        self.terminated=torch.zeros(num_envs,device=self.device,dtype=torch.bool)
        self.truncated=torch.zeros_like(self.terminated)
        self.ticks=torch.zeros(num_envs,device=self.device,dtype=torch.int32)
        self.max_ticks=round(episode_seconds/.01)
        self.reference=torch.zeros((num_envs,2),device=self.device)
        self.heading=torch.zeros(num_envs,device=self.device)
        self.observer=QualifiedObserver(num_envs,self.device,self.cfg['wheel_radius_m'])
        self.saturation=torch.zeros(num_envs,device=self.device)
        self.clearance=torch.zeros(num_envs,device=self.device)
        self.rpy=torch.zeros((num_envs,3),device=self.device)
        self.point_velocity=torch.zeros((num_envs,2),device=self.device)
        self.spawn=tensor(self.cpu.data.qpos);self.spawn_velocity=torch.zeros_like(self.qvel)
        self.reset_velocity=torch.zeros_like(self.qvel)
        self.reset_velocity[:,:2]=torch.empty((num_envs,2),device=self.device).uniform_(-.02,.02)
        self.reset_velocity[:,3:6]=torch.empty((num_envs,3),device=self.device).uniform_(-.02,.02)
        self.contact_geom=wp.to_torch(self.data.contact.geom)
        self.contact_world=wp.to_torch(self.data.contact.worldid)
        self.contact_address=wp.to_torch(self.data.contact.efc_address)
        self.contact_force=wp.to_torch(self.data.efc.force)
        self.nacon=wp.to_torch(self.data.nacon)
        self.contact_slots=torch.arange(self.contact_geom.shape[0],device=self.device,dtype=torch.int32)
        self.base_geom=self.cpu.model.geom('base_link_collision_0').id
        self.floor_geom=self.cpu.model.geom('floor').id
        self.graph=None
        self.reset()
        if capture:self.capture()

    def reset(self):
        self.qpos[:]=self.spawn;self.qvel[:]=self.spawn_velocity;self.warmstart.zero_();self.qfrc.zero_();self.xfrc.zero_()
        self.ticks.zero_();self.last_action.zero_();self.reference.zero_();self.heading.zero_()
        self.observer.xy.zero_();self.observer.velocity.zero_();self.observer.predicted.zero_();self.observer.qd.zero_()
        self.observer.last_q[:]=self.qpos[:,self.qp];self.observer.confidence.fill_(1);self.observer.degraded.zero_()
        self.observer.realign.zero_();self.observer.valid.fill_(True)
        torch.cuda.synchronize();mjw.forward(self.model,self.data);wp.synchronize();self._observation()
        return self.obs

    def _controls(self,action):
        q=self.qpos[:,self.qp];qd=self.observer.qd
        position=self.default+.5*action
        position=torch.stack((position[:,0],position[:,1].clamp(self.limit[0,0],self.limit[0,1]),self.default[2].expand(self.num_envs),
                              position[:,3],position[:,4].clamp(self.limit[1,0],self.limit[1,1]),self.default[5].expand(self.num_envs)),-1)
        velocity=torch.stack((action[:,0]*0,action[:,1]*0,20*action[:,2],action[:,3]*0,action[:,4]*0,20*action[:,5]),-1)
        error=position-q;error=torch.stack((wrap(error[:,0]),error[:,1],error[:,2],wrap(error[:,3]),error[:,4],error[:,5]),-1)
        requested=torch.maximum(torch.minimum(self.kp*error+self.kd*(velocity-qd),self.effort),-self.effort)
        motors=[];speeds=[];ratios=[]
        for h,k,sign in ((0,1,1),(3,4,-1)):
            beta=sign*q[:,k]+math.pi/2
            alpha=2*torch.atan2(torch.sin(beta),self.ratio+torch.cos(beta))
            ratio=.5+self.ratio*torch.cos(alpha/2)/(2*torch.sqrt(1-self.ratio**2*torch.sin(alpha/2)**2))
            ratios.append(ratio)
            motors.extend([sign*(requested[:,h]-ratio*requested[:,k]),sign*ratio*requested[:,k],requested[:,h+2]])
            speeds.extend([sign*qd[:,h],sign*(qd[:,h]+qd[:,k]/ratio),qd[:,h+2]])
        motor=torch.stack(motors,-1);speed=torch.stack(speeds,-1)
        positive=self.cap*((self.free-speed)/(self.free-self.rated)).clamp(0,1)
        negative=-self.cap*((self.free+speed)/(self.free-self.rated)).clamp(0,1)
        delivered=torch.maximum(torch.minimum(motor,positive),negative)
        self.saturation.add_((motor-delivered).abs().gt(1e-6).float().mean(-1)/5)
        delivered=torch.maximum(torch.minimum(delivered*self.motor_gain,positive),negative)
        torque=torch.stack((delivered[:,0]+delivered[:,1],delivered[:,1]/ratios[0],delivered[:,2],
                            -(delivered[:,3]+delivered[:,4]),-delivered[:,4]/ratios[1],delivered[:,5]),-1)
        torque*=self.torque_scale[:,None]
        knees=q[:,self.knee_idx];a,b,c=self.spring_coeff.unbind(-1)
        length=torch.sqrt((a+b*torch.cos(knees)+c*torch.sin(knees)).clamp(min=1e-12))
        jac=(-b*torch.sin(knees)+c*torch.cos(knees))/(2*length)
        cfg=self.cfg['spring'];dead=cfg['stroke_m']/(1-cfg['progression_at_full_stroke']**(-1/cfg['polytropic_exponent']))
        rate=jac*qd[:,self.knee_idx]
        force=cfg['initial_force_n']*(dead/(dead-cfg['free_pin_distance_m']+length).clamp(min=1e-6))**cfg['polytropic_exponent']
        force=(force-cfg['seal_friction_n']*torch.tanh(rate/cfg['friction_smoothing_m_s'])-cfg['viscous_damping_ns_m']*rate).clamp(min=0)
        force*=self.spring_scale[:,None]
        passive=jac*force
        torque[:,1]+=passive[:,0];torque[:,4]+=passive[:,1]
        self.qfrc.zero_();self.qfrc[:,self.qv]=torque
        valid=(length>=cfg['free_pin_distance_m']-cfg['stroke_m']-.0001).all(-1)&(length<=cfg['free_pin_distance_m']+.0001).all(-1)
        self.failure.copy_(torch.where((self.failure==0)&~valid,3,self.failure))

    def _observation(self):
        q=self.qpos[:,self.qp];r=rotation(self.qpos[:,3:7]);rpy=angles(self.qpos[:,3:7])
        trusted=self.observer.confidence>=.25;realign=(~trusted)|self.observer.realign
        delta=self.reference-self.observer.xy;distance=torch.linalg.vector_norm(delta,dim=-1)
        bounded=self.observer.xy+delta*(.4/distance.clamp(min=.4))[:,None]
        self.reference.copy_(torch.where(realign[:,None],self.observer.xy,bounded))
        self.heading.copy_(torch.where(realign,rpy[:,2],self.heading));self.observer.realign.zero_()
        offset=q-self.default
        leg=torch.stack((wrap(offset[:,0]),offset[:,1],wrap(offset[:,3]),offset[:,4]),-1)
        delta=self.reference-self.observer.xy;co=torch.cos(rpy[:,2]);si=torch.sin(rpy[:,2]);yawerr=self.heading-rpy[:,2]
        reference=torch.stack(((5*(co*delta[:,0]+si*delta[:,1])).clamp(-2,2),
                               (5*(-si*delta[:,0]+co*delta[:,1])).clamp(-2,2),torch.sin(yawerr),torch.cos(yawerr)-1,
                               2*self.observer.velocity[:,0],2*self.observer.velocity[:,1]),-1)
        command=q[:,:3]*0;command[:,2]=5*self.cfg['standing_clearance_m']
        self.obs.copy_(torch.cat((self.qvel[:,3:6]*.25,-r[:,2,:],command,leg,self.observer.qd*.05,self.last_action,reference),-1).clamp(-100,100))

    def _core(self):
        action=torch.maximum(torch.minimum(self.action_in,self.bound),-self.bound)
        previous=self.last_action.clone();self.failure.zero_();self.saturation.zero_()
        r=rotation(self.qpos[:,3:7]);offset=self.point_offset
        point_before=self.qpos[:,:3]+torch.einsum('bij,j->bi',r,offset)
        for substep in range(20):
            if substep%4==0:self._controls(action)
            mjw.step(self.model,self.data);mjw.forward(self.model,self.data)
            if substep in (9,19):
                self.observer.update(self.qpos[:,self.qp],self.qvel[:,self.qv],self.qpos[:,3:7],self.qvel[:,3:6],self.qacc[:,:3])
                self.failure.copy_(torch.where((self.failure==0)&~self.observer.valid,6,self.failure))
            if substep%4==3:
                knees=self.qpos[:,self.knee_qp]
                invalid=(knees<self.physical_limit[:,0]-1e-4).any(-1)|(knees>self.physical_limit[:,1]+1e-4).any(-1)
                self.failure.copy_(torch.where((self.failure==0)&invalid,2,self.failure))
        self.ticks.add_(1);self.last_action.copy_(action)
        r=rotation(self.qpos[:,3:7]);self.rpy.copy_(angles(self.qpos[:,3:7]))
        point=self.qpos[:,:3]+torch.einsum('bij,j->bi',r,offset)
        velocity=(point-point_before)/.01;co=torch.cos(self.rpy[:,2]);si=torch.sin(self.rpy[:,2])
        self.point_velocity.copy_(torch.stack((co*velocity[:,0]+si*velocity[:,1],-si*velocity[:,0]+co*velocity[:,1]),-1))
        center=self.clearance_center;half=self.clearance_half
        up=r[:,2,:];self.clearance.copy_(self.qpos[:,2]+(up*center).sum(-1)-(up.abs()*half).sum(-1))
        tilt=torch.acos(r[:,2,2].clamp(-1,1))
        badtilt=(tilt>.70)|(self.qpos[:,2]<.15)
        self.failure.copy_(torch.where((self.failure==0)&badtilt,4,self.failure))
        world=self.contact_world.long().clamp(0,self.num_envs-1);addr=self.contact_address[:,0].long()
        valid=(self.contact_slots<self.nacon[0])&(addr>=0)
        normal=self.contact_force[world,addr.clamp(0,self.contact_force.shape[1]-1)]
        geoms=self.contact_geom
        body=((geoms[:,0]==self.base_geom)&(geoms[:,1]==self.floor_geom))|((geoms[:,1]==self.base_geom)&(geoms[:,0]==self.floor_geom))
        total=torch.zeros(self.num_envs,device=self.device);total.index_add_(0,world,torch.where(valid&body,normal.abs(),0))
        self.failure.copy_(torch.where((self.failure==0)&(total>10),5,self.failure))
        self.failure.copy_(torch.where((self.failure==0)&~self.observer.valid,6,self.failure))
        self.terminated.copy_(self.failure!=0);self.truncated.copy_((self.ticks>=self.max_ticks)&~self.terminated)
        leg=self.qpos[:,self.leg_qp]-self.default[self.leg_idx]
        rp=self.rpy[:,:2];height_error=self.clearance-self.cfg['standing_clearance_m']
        shaped=.01*(2*torch.exp(-rp.square().sum(-1)/.05**2)+torch.exp(-(self.point_velocity[:,0]/.1)**2)
                    +2*torch.exp(-(height_error/.02)**2)+torch.exp(-leg.square().sum(-1)/.15**2)
                    +torch.exp(-self.qpos[:,:2].square().sum(-1)/.1**2)
                    -.2*self.saturation-.01*(action-previous).square().sum(-1))
        self.reward.copy_(shaped-5*self.terminated.float())
        self._observation();self.terminal_obs.copy_(self.obs)
        if self.auto_reset:self.reset_done()

    def capture(self):
        stream=wp.get_stream();self.stream=wp.stream_to_torch(stream);self.event=torch.cuda.Event()
        # Eager warmup compiles every kernel and primes Torch's allocation cache.
        with torch.cuda.stream(self.stream):self._core()
        wp.synchronize();self.reset()
        with wp.ScopedCapture() as captured:
            with torch.cuda.stream(self.stream):self._core()
        wp.synchronize();self.graph=captured.graph;self.reset()

    def step(self,actions):
        if self.graph is None:
            self.action_in.copy_(actions);self._core()
        else:
            self.stream.wait_stream(torch.cuda.current_stream())
            with torch.cuda.stream(self.stream):
                self.action_in.copy_(actions);wp.capture_launch(self.graph)
            self.event.record(self.stream);torch.cuda.current_stream().wait_event(self.event)
        return self.obs,self.reward,self.terminated,self.truncated

    def reset_done(self):
        done=self.terminated|self.truncated;mask=done[:,None]
        self.qpos.copy_(torch.where(mask,self.spawn,self.qpos));self.qvel.copy_(torch.where(mask,self.reset_velocity,self.qvel))
        self.warmstart.copy_(torch.where(mask,0,self.warmstart));self.ticks.copy_(torch.where(done,0,self.ticks))
        self.last_action.copy_(torch.where(mask,0,self.last_action));self.reference.copy_(torch.where(mask,0,self.reference))
        self.heading.copy_(torch.where(done,0,self.heading));o=self.observer
        for value in (o.xy,o.velocity,o.predicted,o.qd):value.copy_(torch.where(mask,0,value))
        o.last_q.copy_(torch.where(mask,self.qpos[:,self.qp],o.last_q));o.confidence.copy_(torch.where(done,1,o.confidence))
        o.degraded.copy_(torch.where(done,0,o.degraded));o.realign.copy_(torch.where(done,False,o.realign));o.valid.copy_(torch.where(done,True,o.valid))
        self._observation()
        return self.obs
