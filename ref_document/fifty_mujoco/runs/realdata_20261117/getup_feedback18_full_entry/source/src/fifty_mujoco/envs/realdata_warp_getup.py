"""Captured supported get-up cohorts with frozen-controller balance validation.

Completed balance is coalesced onto the final GETUP/HANDOFF transition, exactly
as in the CPU task adapter. STOP and BALANCE never supply learner actions.
"""
import math

import numpy as np
import torch
import warp as wp

from .realdata_warp import RealDataWarpEnv,angles,rotation,wrap


FAILURES={0:None,1:'nonfinite_state',2:'joint_limit',3:'spring_stroke',4:'getup_tilt_limit',
          6:'invalid_telemetry',7:'forbidden_floor_contact',8:'forbidden_self_contact',
          9:'getup_timeout',10:'handoff_unstable',11:'balance_displacement',
          12:'balance_support_invalid',13:'balance_body_contact',14:'cohort_timeout',15:'balance_state_out_of_spec'}


class WarpSupportedGetup:
    def __init__(self,balance_actor,num_envs=256,config=None,reset_range=(0.,0.),settle_seconds=1.,domain=False):
        self.plant=RealDataWarpEnv(num_envs,config=config,capture=False,auto_reset=False,episode_seconds=30.)
        self.num_envs=num_envs;self.device=self.plant.device;self.cfg=self.plant.cfg
        self.balance=balance_actor.requires_grad_(False).eval()
        self.reset_range=reset_range;self.settle_seconds=settle_seconds;self.domain=domain
        self.action_in=torch.zeros((num_envs,6),device=self.device)
        self.phase=torch.zeros(num_envs,device=self.device,dtype=torch.int32)
        self.first_failure=torch.zeros_like(self.phase)
        self.stable=torch.zeros(num_envs,device=self.device)
        self.handoff_time=torch.zeros_like(self.stable);self.balance_time=torch.zeros_like(self.stable)
        self.enabled_time=torch.zeros_like(self.stable);self.ramp=torch.ones_like(self.stable)*.2
        self.release_after=torch.ones_like(self.stable);self.fade=torch.ones_like(self.stable)
        self.vertical_release_after=torch.ones_like(self.stable);self.vertical_fade=torch.ones_like(self.stable)
        self.support_start=torch.zeros((num_envs,3),device=self.device)
        self.origin=torch.zeros((num_envs,2),device=self.device)
        self.support_wrench=torch.zeros((num_envs,6),device=self.device)
        self.support_peak=torch.zeros((num_envs,2),device=self.device)
        self.phase_counts=torch.zeros((num_envs,4),device=self.device,dtype=torch.int32)
        self.wheel_load=torch.zeros((num_envs,2),device=self.device)
        self.reward=torch.zeros(num_envs,device=self.device)
        self.just_finished=torch.zeros(num_envs,device=self.device,dtype=torch.bool)
        self.terminal_metrics=torch.zeros((num_envs,6),device=self.device)
        self.reset_fraction=torch.zeros(num_envs,device=self.device)
        self.previous_leg_error=torch.zeros(num_envs,device=self.device)
        self.bodyweight=self.cfg['mass_kg']*9.81
        self.base=self.plant.cpu.base
        self.wheel_geoms=[self.plant.cpu.model.geom(n).id for n in ('right_wheel_collision_0','left_wheel_collision_0')]
        def tensor(x):return torch.tensor(x,device=self.device,dtype=torch.float32)
        support=self.cfg['support']
        self.kp=tensor(support['position_kp_n_m']);self.kd=tensor(support['position_kd_ns_m'])
        self.kr=tensor(support['rotation_kp_nm_rad']);self.dr=tensor(support['rotation_kd_nms_rad'])
        self.default=tensor(self.cfg['standing_default_rad']);self.survey=tensor(self.cfg['survey_pose_rad'])
        self.leg_idx=self.plant.leg_idx
        self.reset()
        self.plant.support_callback=self._support
        self.stream=wp.stream_to_torch(wp.get_stream());self.event=torch.cuda.Event()
        with torch.cuda.stream(self.stream):self._core()
        wp.synchronize();self.reset()
        with wp.ScopedCapture() as captured:
            with torch.cuda.stream(self.stream):self._core()
        wp.synchronize();self.graph=captured.graph;self.reset()

    @property
    def observation(self):return self.plant.obs

    def reset(self):
        p=self.plant;p.reset();n=self.num_envs
        self.reset_fraction.uniform_(*self.reset_range)
        pose=self.default+self.reset_fraction[:,None]*(self.survey-self.default)
        noise=torch.empty_like(pose).uniform_(-.005,.005);noise[:,2]=0;noise[:,5]=0;pose+=noise
        p.qpos[:,p.qp]=pose
        heights=[]
        for h,k,sign in ((0,1,1),(3,4,-1)):
            a=sign*pose[:,h];b=sign*(pose[:,h]+pose[:,k])+math.pi/2
            heights.append(.21*torch.sin(a)+.25*torch.sin(b)+self.cfg['wheel_radius_m'])
        p.qpos[:,2]=torch.maximum(*heights)
        p.qvel.zero_();p.warmstart.zero_();p.observer.last_q.copy_(pose)
        self.previous_leg_error.copy_(torch.linalg.vector_norm(pose[:,self.leg_idx]-self.default[self.leg_idx],dim=-1))
        self.support_start.copy_(p.qpos[:,:3]);self.origin.copy_(p.qpos[:,:2])
        self.ramp.uniform_(.2,.5);self.release_after.uniform_(*self.cfg['support']['release_after_s'])
        self.fade.uniform_(*self.cfg['support']['fade_s'])
        self.vertical_release_after.uniform_(*self.cfg['support'].get('vertical_release_after_s',self.cfg['support']['release_after_s']))
        self.vertical_fade.uniform_(*self.cfg['support'].get('vertical_fade_s',self.cfg['support']['fade_s']))
        for value in (self.phase,self.first_failure,self.stable,self.handoff_time,self.balance_time,self.enabled_time,
                      self.phase_counts,self.support_peak,self.support_wrench,self.wheel_load,self.reward):value.zero_()
        self.just_finished.zero_();self.terminal_metrics.zero_();p.torque_scale.zero_()
        p.motor_gain.fill_(1);p.spring_scale.fill_(1)
        if self.domain:
            nominal=torch.rand(n,device=self.device)<.2
            gain=torch.empty_like(p.motor_gain).uniform_(.8,1.);gain[:,2]=1;gain[:,5]=1
            p.motor_gain.copy_(torch.where(nominal[:,None],1,gain))
            p.spring_scale.copy_(torch.where(nominal,1,torch.empty_like(p.spring_scale).uniform_(.8,1.2)))
        torch.cuda.synchronize();self.plant._observation()
        return p.obs

    def _support(self):
        p=self.plant;c=self.cfg['support'];t=self.enabled_time
        fraction=(1-(t-self.release_after).clamp(min=0)/self.fade).clamp(min=0)
        progress=(t/self.release_after.clamp(min=.2)).clamp(0,1);progress=progress*progress*(3-2*progress)
        target=self.support_start.clone()
        target[:,2]+=progress*(c.get('target_root_height_m',.35)-self.support_start[:,2])
        force=self.kp*(target-p.qpos[:,:3])-self.kd*p.qvel[:,:3]
        force[:,2]=(force[:,2]+.9*self.bodyweight).clamp(min=0)
        norm=torch.linalg.vector_norm(force,dim=-1)
        force*=((c['force_cap_bodyweights']*self.bodyweight)/norm.clamp(min=c['force_cap_bodyweights']*self.bodyweight))[:,None]
        moment=(-self.kr*angles(p.qpos[:,3:7])-self.dr*p.qvel[:,3:6]).clamp(-c['moment_cap_nm'],c['moment_cap_nm'])
        active=(self.phase<4).float()
        self.support_wrench.copy_(torch.cat((force,moment),-1)*fraction[:,None]*active[:,None])
        if 'vertical_release_after_s' in c:
            vf=(1-(t-self.vertical_release_after).clamp(min=0)/self.vertical_fade).clamp(min=0)
            self.support_wrench[:,2]=force[:,2]*vf*active
        p.xfrc.zero_();p.xfrc[:,self.base,:]=self.support_wrench
        self.support_peak[:,0]=torch.maximum(self.support_peak[:,0],torch.linalg.vector_norm(self.support_wrench[:,:3],dim=-1))
        self.support_peak[:,1]=torch.maximum(self.support_peak[:,1],torch.linalg.vector_norm(self.support_wrench[:,3:],dim=-1))

    def _contacts(self):
        p=self.plant;geoms=p.contact_geom
        world=p.contact_world.long().clamp(0,self.num_envs-1);addr=p.contact_address[:,0].long()
        valid=(p.contact_slots<p.nacon[0])&(addr>=0)
        normal=p.contact_force[world,addr.clamp(0,p.contact_force.shape[1]-1)].abs()
        valid=valid&(normal>1e-3)
        floor=(geoms[:,0]==p.floor_geom)|(geoms[:,1]==p.floor_geom)
        body=(geoms[:,0]==p.base_geom)|(geoms[:,1]==p.base_geom)
        allowed=body.clone();self.wheel_load.zero_()
        for side,gid in enumerate(self.wheel_geoms):
            wheel=(geoms[:,0]==gid)|(geoms[:,1]==gid);allowed=allowed|wheel
            self.wheel_load[:,side].index_add_(0,world,torch.where(valid&floor&wheel,normal,0))
        body_force=torch.zeros(self.num_envs,device=self.device)
        forbidden_floor=torch.zeros_like(body_force);forbidden_self=torch.zeros_like(body_force)
        body_force.index_add_(0,world,torch.where(valid&floor&body,normal,0))
        forbidden_floor.index_add_(0,world,(valid&floor&~allowed).float())
        forbidden_self.index_add_(0,world,(valid&~floor).float())
        return body_force>10,forbidden_floor>0,forbidden_self>0

    def _core(self):
        p=self.plant;before=self.phase.clone();ended=before>=4
        old_qpos=p.qpos.clone();old_qvel=p.qvel.clone();previous=p.last_action.clone()
        self.enabled_time.copy_((p.ticks.float()*.01-self.settle_seconds).clamp(min=0))
        self._support()
        p.torque_scale.copy_((self.enabled_time/self.ramp).clamp(0,1)*((before>0)&(before<4)).float())
        frozen=self.balance(p.obs)
        weight=(self.handoff_time/.3).clamp(0,1);weight=weight*weight*(3-2*weight)
        weight=torch.where(before==3,1,torch.where(before==2,weight,0))
        applied=(1-weight[:,None])*self.action_in+weight[:,None]*frozen
        applied=torch.where(((before>0)&(before<4))[:,None],applied,0)
        p.action_in.copy_(applied);p._core()
        body_contact,forbidden_floor,forbidden_self=self._contacts()
        physical=p.failure.clone()
        # The standing-task body-floor event is permitted during GETUP only.
        physical=torch.where(physical==5,0,physical)
        physical=torch.where((physical==0)&forbidden_floor,7,physical)
        physical=torch.where((physical==0)&forbidden_self,8,physical)
        physical=torch.where((physical==0)&(p.rpy[:,:2].abs().amax(-1)>math.radians(40)),4,physical)
        self.first_failure.copy_(torch.where((self.first_failure==0)&~ended,physical,self.first_failure))
        assistance=torch.linalg.vector_norm(self.support_wrench[:,:3],dim=-1)+torch.linalg.vector_norm(self.support_wrench[:,3:],dim=-1)/.3
        wheel_supported=(self.wheel_load>=.02*self.bodyweight).all(-1)
        leg=p.qpos[:,p.leg_qp]-self.default[self.leg_idx]
        leg=torch.stack((wrap(leg[:,0]),leg[:,1],wrap(leg[:,2]),leg[:,3]),-1)
        displacement=torch.linalg.vector_norm(p.qpos[:,:2]-self.origin,dim=-1)
        velocity_limit=torch.where(before==3,.08,self.cfg.get('handoff_velocity_limit_m_s',.08))
        velocity_limit=torch.where(before==2,self.cfg.get('blend_velocity_limit_m_s',self.cfg.get('handoff_velocity_limit_m_s',.08)),velocity_limit)
        ready=((p.rpy[:,:2].abs().amax(-1)<=math.radians(8))&(p.qvel[:,3:6].abs().amax(-1)<=.5)
               &(p.observer.velocity[:,0].abs()<=velocity_limit)&((p.clearance-self.cfg['standing_clearance_m']).abs()<=.02)
               &(leg.abs().amax(-1)<=.15)&(displacement<=.3)&wheel_supported&~body_contact&(assistance<=1e-6)
               &(p.observer.confidence>=.5))
        self.stable.copy_(torch.where((before==1)&ready,self.stable+.01,torch.where(before==1,0,self.stable)))
        self.handoff_time.copy_(torch.where(before==2,self.handoff_time+.01,self.handoff_time))
        self.balance_time.copy_(torch.where(before==3,self.balance_time+.01,self.balance_time))
        failure=self.first_failure
        failure=torch.where((failure==0)&(before==1)&(self.enabled_time>self.cfg['success']['getup_timeout_s']+.001),9,failure)
        failure=torch.where((failure==0)&(before==2)&~ready,10,failure)
        failure=torch.where((failure==0)&(before==3)&(displacement>.3),11,failure)
        failure=torch.where((failure==0)&(before==3)&(~wheel_supported|(assistance>1e-6)),12,failure)
        failure=torch.where((failure==0)&(before==3)&body_contact,13,failure)
        failure=torch.where((failure==0)&(before==3)&~ready,15,failure)
        self.first_failure.copy_(failure)
        phase=before
        phase=torch.where((before==0)&(p.ticks.float()*.01>=self.settle_seconds),1,phase)
        phase=torch.where((before==1)&(self.stable>=.99999),2,phase)
        phase=torch.where((before==2)&(self.handoff_time>=.29999),3,phase)
        phase=torch.where((before==3)&(self.balance_time>=9.9999),5,phase)
        phase=torch.where((self.first_failure!=0)&~ended,4,phase)
        self.phase.copy_(phase)
        for index in range(4):self.phase_counts[:,index]+=(before==index).int()
        self.just_finished.copy_(~ended&(phase>=4))
        metrics=torch.stack((p.clearance,p.observer.velocity[:,0],displacement,
                             p.rpy[:,0],p.rpy[:,1],p.qvel[:,3:6].abs().amax(-1)),-1)
        self.terminal_metrics.copy_(torch.where(self.just_finished[:,None],metrics,self.terminal_metrics))
        potential=torch.exp(-(p.rpy[:,:2]/math.radians(15)).square().sum(-1))
        height=torch.exp(-((p.clearance-self.cfg['standing_clearance_m'])/.06).square())
        posture=torch.exp(-leg.square().sum(-1)/.2**2)
        leg_error=torch.linalg.vector_norm(leg,dim=-1)
        progress=self.previous_leg_error-leg_error
        self.previous_leg_error.copy_(leg_error)
        quiet=torch.exp(-(p.observer.velocity[:,0]/.08).square())
        position=torch.exp(-(displacement/.1).square())
        power=(p.motor_torque*p.motor_velocity).abs().sum(-1)
        self.reward.copy_(.01*(potential+height+posture+quiet+position-4.-.0001*power-.1*p.saturation)
                          +2*progress-.002*(p.last_action-previous).square().sum(-1))
        self.reward.copy_(torch.where((before==1)|(before==2),self.reward,0))
        # Freeze completed worlds; they never contribute later learner samples.
        p.qpos.copy_(torch.where(ended[:,None],old_qpos,p.qpos))
        p.qvel.copy_(torch.where(ended[:,None],old_qvel*0,p.qvel))
        # On enable, align the estimator reference without resetting physics.
        enable=(before==0)&(phase==1)
        if self.cfg.get('handoff_reanchor_reference',False):
            reanchor=(before==1)&(phase==2)
            p.reference.copy_(torch.where(reanchor[:,None],p.observer.xy,p.reference))
            p.heading.copy_(torch.where(reanchor,p.rpy[:,2],p.heading))
        p.reference.copy_(torch.where(enable[:,None],p.observer.xy,p.reference))
        p.heading.copy_(torch.where(enable,p.rpy[:,2],p.heading))
        p.last_action.copy_(torch.where(enable[:,None],0,p.last_action));p._observation()

    def step(self,actions):
        self.stream.wait_stream(torch.cuda.current_stream())
        with torch.cuda.stream(self.stream):
            self.action_in.copy_(actions);wp.capture_launch(self.graph)
        self.event.record(self.stream);torch.cuda.current_stream().wait_event(self.event)
        return self.plant.obs,self.reward,self.phase,self.first_failure
