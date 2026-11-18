"""Supported rise/release/handoff with explicit force AND moment accounting.

All support is a simulation stand. Terminal success requires a separate frozen
balance actor for ten seconds after the one-second unassisted readiness dwell.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import mujoco
import numpy as np
import torch

from .sample_env31_realdata import RealDataHardwareEnv31
from .sample_env31_hardware import rpy_from_wxyz
from ..contracts.getup31 import GetupController, Phase, Sample, blended_targets
from ..contracts.getup31 import Decision
from ..contracts.sample_actor31 import wrap_angle
from ..paths import PROJECT_ROOT
from ..transfer.firmware_runtime import Estimator, Quality, Measurement, Controller


DOMAIN_DEFAULTS = {'nominal_fraction': .2, 'motor_gain': [.8, 1.], 'spring_scale': [.8, 1.2],
                   'friction': [.6, .9], 'armature_scale': [.5, 2.], 'drag_scale': [.5, 2.]}


class CandidateController(GetupController):
    def __init__(self, plan, default):
        self.default = default
        self.odometry_confidence = 1.
        super().__init__(plan)

    def _failure(self,sample):
        failure=super()._failure(sample)
        if max(abs(x) for x in sample.roll_pitch_deg)>40:
            return failure or 'getup_tilt_limit'
        if self.spec.get('seated_back') and sample.displacement_m > self.criteria['horizontal_displacement_m_max']:
            return failure or 'getup_displacement_limit'
        if self.phase==Phase.BALANCE and not self.ready(sample):
            return failure or 'balance_state_out_of_spec'
        return failure

    def ready(self, sample):
        c = self.criteria
        velocity_limit=(c['abs_forward_velocity_m_s_max'] if self.phase==Phase.BALANCE
                        else self.spec.get('handoff_velocity_limit_m_s',c['abs_forward_velocity_m_s_max']))
        if self.phase==Phase.HANDOFF:
            velocity_limit=self.spec.get('blend_velocity_limit_m_s',velocity_limit)
        errors = [sample.joint_position_rad[i]-self.default[i] for i in (0,1,3,4)]
        leg_error = max(abs(wrap_angle(e) if i in (0,3) else e) for i,e in zip((0,1,3,4),errors))
        return (self.odometry_confidence>=.5
                and max(abs(x) for x in sample.roll_pitch_deg) <= c["abs_roll_pitch_deg_max"]
                and max(abs(x) for x in sample.angular_velocity_rad_s) <= c["abs_body_angular_velocity_rad_s_max"]
                and abs(sample.forward_velocity_m_s) <= velocity_limit
                and abs(sample.clearance_m-self.spec["actor_commands"][2]) <= c["clearance_error_m_max"]
                and leg_error <= c["abs_leg_error_to_balance_default_rad_max"]
                and sample.displacement_m <= c["horizontal_displacement_m_max"]
                and sample.both_wheels_supported and not sample.body_contact and sample.assistance_force_n <= 1e-6)


class SupportedGetupEnv31:
    def __init__(self, balance_actor, reset_range=(0.,0.), settle_seconds=0., domain=False, config=None):
        self.env = RealDataHardwareEnv31(episode_seconds=22.,config=config)
        self.balance_actor = balance_actor.requires_grad_(False).eval()
        self.reset_range, self.settle_seconds, self.domain = reset_range, settle_seconds, domain
        self.candidate = self.env.candidate
        self.seated = self.candidate.get('seated_back')
        if self.seated and settle_seconds < 1. and any(
                item.get('class', 'seated') == 'seated' for item in self.seated['reset_bank']):
            raise ValueError('seated initial states require at least one second of zero-output settle')
        self.source, self.model, self.data = self.env.source, self.env.model, self.env.data
        self.plan = json.loads((PROJECT_ROOT/"configs/hardware31_next_training_20261115.json").read_text())
        spec, cfg = self.plan["getup"], self.candidate
        spec["actor_commands"][2] = cfg["standing_clearance_m"]
        spec['seated_back'] = bool(self.seated)
        for key in ("stable_dwell_s","getup_timeout_s"):
            spec[key] = cfg["success"][key]
        spec["post_handoff_balance_s"] = cfg["success"]["balance_s"]
        spec["handoff_blend_s"] = cfg["success"]["blend_s"]
        spec['handoff_velocity_limit_m_s']=cfg.get('handoff_velocity_limit_m_s',.08)
        if 'blend_velocity_limit_m_s' in cfg:
            spec['blend_velocity_limit_m_s']=cfg['blend_velocity_limit_m_s']
        # The physical clock includes STOP and the frozen balance rollout.
        # Derive the guard from the protocol so a slower rise never truncates
        # its separate ten-second validation.
        duration=(settle_seconds+spec['getup_timeout_s']+spec['handoff_blend_s']
                  +spec['post_handoff_balance_s']+2.)
        self.max_steps=math.ceil(duration/.01)
        self.env.max_steps=self.max_steps
        self.original_apply = self.env.fast_actuators.apply
        self.env.fast_actuators.apply = self._apply
        self.env.support_callback = self._support
        self.torque_scale = 0.
        self.armature_nominal = self.model.dof_armature.copy()
        self.drag_nominal = self.model.dof_damping.copy(), self.model.dof_frictionloss.copy()

    def _apply(self,*args):
        active = self.original_apply(*args)
        self.env.fast_actuators.command *= self.torque_scale
        self.env.fast_actuators.delivered *= self.torque_scale
        return active*self.torque_scale

    def _support(self):
        cfg, e = self.candidate["support"], self.env
        if self.seated:
            self.support_wrench.fill(0.)
            e.data.xfrc_applied.fill(0.)
            return
        t = self.enabled_time
        fraction = max(0., 1-max(0.,t-self.release_after_s)/self.fade_s)
        desired = self.support_start.copy()
        if t > 0:
            progress = min(1., t/max(.2,self.release_after_s))
            progress = progress*progress*(3-2*progress)
            desired[2] += progress*(cfg.get('target_root_height_m',.44)-self.support_start[2])
        force = (np.asarray(cfg["position_kp_n_m"])*(desired-e.data.qpos[:3])
                 -np.asarray(cfg["position_kd_ns_m"])*e.data.qvel[:3]
                 +[0.,0.,.9*self.bodyweight])
        # Upward-only vertical assistance; horizontal fixture forces are explicit.
        force[2] = max(0.,force[2])
        norm = float(np.linalg.norm(force)); cap=cfg["force_cap_bodyweights"]*self.bodyweight
        if norm > cap:force *= cap/norm
        moment = (-np.asarray(cfg["rotation_kp_nm_rad"])*rpy_from_wxyz(e.data.qpos[3:7])
                  -np.asarray(cfg["rotation_kd_nms_rad"])*e._velocity6()[:3])
        moment = np.clip(moment,-cfg["moment_cap_nm"],cfg["moment_cap_nm"])
        self.support_wrench = fraction*np.r_[force,moment]
        if 'vertical_release_after_s' in cfg:
            vertical_fraction=max(0.,1-max(0.,t-self.vertical_release_after_s)/self.vertical_fade_s)
            self.support_wrench[2]=vertical_fraction*force[2]
        e.data.xfrc_applied[:] = 0
        e.data.xfrc_applied[e.base] = self.support_wrench
        self.support_peak_n = max(self.support_peak_n,float(np.linalg.norm(self.support_wrench[:3])))
        self.support_peak_nm = max(self.support_peak_nm,float(np.linalg.norm(self.support_wrench[3:])))

    def _sample(self, info):
        e=self.env
        wheel_load={"right_wheel_collision_0":0.,"left_wheel_collision_0":0.}
        body_contact=False; forbidden=False
        for i,contact in enumerate(e.data.contact):
            f=np.zeros(6);mujoco.mj_contactForce(e.model,e.data,i,f)
            magnitude=float(np.linalg.norm(f[:3]))
            if magnitude<=1e-3:continue
            names={e.model.geom(int(n)).name for n in (contact.geom1,contact.geom2)}
            if 'floor' in names:
                other=next(n for n in names if n!='floor')
                if other in wheel_load:wheel_load[other]+=abs(float(f[0]))
                elif other=='base_link_collision_0':body_contact |= magnitude>(1e-3 if self.seated else 10)
                else:forbidden=True
            else:forbidden=True
        self.wheel_load_n=wheel_load
        q=e.data.qpos[e.source.qpos_ids]
        knees=q[[1,4]]
        failure=info.get('failure')
        mechanical = bool(np.all(knees>=e.limits[:,0]-1e-4) and np.all(knees<=e.limits[:,1]+1e-4))
        self.controller.odometry_confidence=float(e.c_quality.confidence)
        # Equivalent magnitude also counts fixture moments, so a zero vertical
        # force can never hide horizontal or angular assistance.
        assistance=float(np.linalg.norm(self.support_wrench[:3])+np.linalg.norm(self.support_wrench[3:])/.3)
        return Sample(time_s=float(e.data.time),joint_position_rad=tuple(q),roll_pitch_deg=tuple(info['roll_pitch_deg']),
                      angular_velocity_rad_s=tuple(e._velocity6()[:3]),forward_velocity_m_s=float(e.estimated_velocity[0]),
                      clearance_m=info['clearance_m'],displacement_m=float(np.linalg.norm(e.data.qpos[:2]-self.origin)),
                      both_wheels_supported=all(v>=.02*self.bodyweight for v in wheel_load.values()),body_contact=bool(body_contact),
                      assistance_force_n=assistance,telemetry_valid=failure!='firmware_estimator_invalid',
                      joint_limits_valid=mechanical,spring_stroke_valid=not (failure or '').startswith('spring_stroke:'),
                      linkage_branch_valid=True,forbidden_contact=bool(forbidden),
                      balance_failure=failure if self.controller.phase in (Phase.HANDOFF,Phase.BALANCE) else None)

    def _advance(self, action, scale):
        self.torque_scale=scale;self._support()
        obs,_,terminated,truncated,info=self.env.step(action)
        if terminated and info['failure'] in ('base_ground_contact','tilt_or_low_root'):
            self.env.finished=False
        self.last_sample=self._sample(info)
        return obs,info,self.last_sample

    def reset(self,seed=510000):
        rng=np.random.default_rng(seed);e=self.env
        self.parameters={'seed':int(seed),'reset_fraction':float(rng.uniform(*self.reset_range)),
                         'domain':self.domain,'settle_s':self.settle_seconds}
        self.model.dof_armature[:]=self.armature_nominal
        self.model.dof_damping[:],self.model.dof_frictionloss[:]=self.drag_nominal
        ranges = DOMAIN_DEFAULTS | self.candidate.get('domain_ranges', {})
        gain=np.ones(6);e.spring_scale=1.
        friction, armature, damping, friction_loss = .75, 1., 1., 1.
        if self.domain and rng.random()>=ranges['nominal_fraction']:
            gain[[0,1,3,4]]=rng.uniform(*ranges['motor_gain'],4)
            e.spring_scale=float(rng.uniform(*ranges['spring_scale']))
            friction = float(rng.uniform(*ranges['friction']))
            armature = float(rng.uniform(*ranges['armature_scale']))
            damping, friction_loss = rng.uniform(*ranges['drag_scale'], 2).tolist()
        self.model.geom_friction[:,0] = friction
        self.model.dof_armature[:] *= armature
        self.model.dof_damping[:] *= damping
        self.model.dof_frictionloss[:] *= friction_loss
        e.fast_actuators.gain[:]=gain
        self.parameters.update(motor_gain=gain.tolist(),spring_scale=e.spring_scale,friction=friction,
                               armature_scale=armature,damping_scale=damping,friction_loss_scale=friction_loss)
        obs=e.reset(seed)
        fraction=self.parameters['reset_fraction']
        pose=e.source.defaults+fraction*(np.asarray(self.candidate['survey_pose_rad'])-e.source.defaults)
        pose[[0,1,3,4]]+=rng.uniform(-.005,.005,4)
        if self.seated:
            bank = self.seated['reset_bank']
            index = int(rng.integers(len(bank)))
            entry = bank[index]
            self.parameters.update(reset_index=index, reset_class=entry.get('class', 'seated'))
            self.data.qpos[:] = entry['qpos']
            self.data.qvel[:] = entry['qvel']
            pose = self.data.qpos[e.source.qpos_ids].copy()
        else:
            self.data.qpos[e.source.qpos_ids]=pose
            self.data.qvel[:]=0
            mujoco.mj_forward(self.model,self.data)
            bottoms=[]
            for name in ('right_wheel_collision_0','left_wheel_collision_0'):
                g=self.model.geom(name).id;az=self.data.geom_xmat[g].reshape(3,3)[2,2]
                extent=self.model.geom_size[g,0]*math.sqrt(max(0,1-az*az))+self.model.geom_size[g,1]*abs(az)
                bottoms.append(self.data.geom_xpos[g,2]-extent)
            self.data.qpos[2]-=min(bottoms)
        mujoco.mj_forward(self.model,self.data)
        e.c_estimator.last_q[:]=pose
        e.c_estimator.held_qd[:]=[0.]*6
        e.c_estimator.velocity_valid=True
        e.c_measurement.q[:]=pose
        e.joint_velocity=np.zeros(6)
        self.controller=CandidateController(self.plan,self.source.defaults)
        ramp = self.candidate.get('getup_ramp_range_s', [.2, .5])
        self.controller.ramp_s=float(rng.uniform(*ramp))
        self.enabled_time=0.;self.release_after_s=float(rng.uniform(*self.candidate['support']['release_after_s']))
        self.fade_s=float(rng.uniform(*self.candidate['support']['fade_s']))
        self.vertical_release_after_s=float(rng.uniform(*self.candidate['support'].get('vertical_release_after_s',[self.release_after_s]*2)))
        self.vertical_fade_s=float(rng.uniform(*self.candidate['support'].get('vertical_fade_s',[self.fade_s]*2)))
        self.origin=self.data.qpos[:2].copy();self.support_start=self.data.qpos[:3].copy()
        self.bodyweight=float(self.model.body_mass.sum()*9.81)
        self.support_wrench=np.zeros(6);self.support_peak_n=self.support_peak_nm=0.
        self.phase_counts={};self.balance_ticks=0;self.failed_reset=None;self.tick=0
        self.ready_reference_aligned = False
        for _ in range(round(self.settle_seconds/.01)):
            obs,info,s=self._advance([0.]*6,0.)
            if self.controller._failure(s):
                self.failed_reset='invalid_zero_torque_settle:'+self.controller._failure(s);break
        if self.seated and not self.failed_reset and entry.get('class', 'seated') == 'seated':
            forward = e._rotation()[:2, 0]
            wheel_centres = [self.data.geom_xpos[self.model.geom(n).id, :2]
                             for n in ('right_wheel_collision_0', 'left_wheel_collision_0')]
            offsets = [(pos-self.data.qpos[:2])@forward for pos in wheel_centres]
            self.parameters['settled_wheel_offset_m'] = [float(x) for x in offsets]
            self.parameters['settled_wheel_load_n'] = dict(self.wheel_load_n)
            if (max(offsets) >= -.01 or not s.body_contact or min(self.wheel_load_n.values()) < .005*self.bodyweight
                    or np.max(np.abs(self.data.qvel)) > .2):
                self.failed_reset = 'seated_reset_shape_or_rest_lost'
        if self.failed_reset:
            self.decision=Decision(Phase.FAILED,0.,0.,False,False,self.failed_reset)
            self.start_time=float(self.data.time)
            self.previous_sample=s
            self.finished=False
            return e.observation()
        obs,info,s=self._advance([0.]*6,0.)
        self.controller.update(s,False)
        obs,info,s=self._advance([0.]*6,0.)
        self.decision=self.controller.update(s,True)
        self.start_time=float(self.data.time)
        e.reference=e.estimated_xy.tolist();e.heading=float(rpy_from_wxyz(self.data.qpos[3:7])[2])
        e.last_action=np.zeros(6)
        self.previous_sample=s
        self.previous_leg_error=float(np.linalg.norm(np.asarray(s.joint_position_rad)[[0,1,3,4]]-self.source.defaults[[0,1,3,4]]))
        self.finished=False
        return e.observation()

    def _balance_action(self):
        with torch.no_grad():
            obs=torch.tensor(self.env.observation(),device=next(self.balance_actor.parameters()).device)[None]
            return self.balance_actor(obs)[0].cpu().numpy()

    def step(self,action):
        if self.finished:raise RuntimeError('reset required')
        total_reward=0.;last_info=None
        while True:
            before=self.decision
            if before.failure is not None or self.failed_reset:
                self.finished=True
                last_info=dict(self.env.last_info, time_s=self.enabled_time, failure=self.failed_reset or before.failure)
                total_reward-=5.;break
            applied=blended_targets(before,action,self._balance_action(),self.env.target_limits)['clipped_action']
            # Mapping into target positions happens once in the candidate plant.
            previous=np.asarray(self.env.last_action).copy()
            self.enabled_time=float(self.data.time)-self.start_time
            obs,info,state=self._advance(applied,before.torque_scale)
            physical_failure = self.controller._failure(state)
            if physical_failure and state.time_s <= self.controller.previous_time:
                # Mechanical checks may stop before any physical time advances.
                self.controller.phase, self.controller.failure = Phase.FAILED, physical_failure
                self.decision = Decision(Phase.FAILED, 0., 0., False, False, physical_failure)
            else:
                self.decision=self.controller.update(state,True)
            if (self.candidate.get('getup_ready_reanchor_reference', False)
                    and not self.ready_reference_aligned and before.phase == Phase.GETUP
                    and self.controller.ready(state)):
                self.env.reference = self.env.estimated_xy.tolist()
                self.env.heading = float(rpy_from_wxyz(self.data.qpos[3:7])[2])
                self.ready_reference_aligned = True
            if (self.candidate.get('handoff_reanchor_reference',False)
                    and before.phase==Phase.GETUP and self.decision.phase==Phase.HANDOFF):
                self.env.reference=self.env.estimated_xy.tolist()
                self.env.heading=float(rpy_from_wxyz(self.data.qpos[3:7])[2])
            self.phase_counts[before.phase.value]=self.phase_counts.get(before.phase.value,0)+1
            self.balance_ticks+=int(before.phase==Phase.BALANCE)
            self.tick+=1
            if before.phase!=Phase.BALANCE:
                err=np.asarray(state.joint_position_rad)[[0,1,3,4]]-self.source.defaults[[0,1,3,4]]
                potential=math.exp(-sum((v/15)**2 for v in state.roll_pitch_deg))
                height=math.exp(-((state.clearance_m-self.candidate['standing_clearance_m'])/.06)**2)
                posture=math.exp(-np.dot(err,err)/.2**2)
                distance=float(np.linalg.norm(err))
                progress=self.previous_leg_error-distance
                self.previous_leg_error=distance
                quiet=math.exp(-(state.forward_velocity_m_s/.08)**2)
                position=math.exp(-(state.displacement_m/.1)**2)
                power=float(np.abs(self.env.fast_actuators.delivered*self.env.fast_actuators.velocity).sum())
                saturation=sum(info['saturation_counts'])/(6*max(1,info['controls']))
                total_reward+=(.01*(potential+height+posture+quiet+position-4.-.0001*power-.1*saturation)
                               +2*progress-.002*np.square(np.asarray(applied)-previous).sum())
                if self.seated:
                    height_goal = self.candidate['standing_clearance_m']
                    last_height = np.clip(self.previous_sample.clearance_m, 0., height_goal)
                    new_height = np.clip(state.clearance_m, 0., height_goal)
                    total_reward += 12*(new_height-last_height) + .02*float(self.controller.ready(state))
                    if self.seated.get('braking_reward', False):
                        total_reward -= .04*max(0., state.displacement_m-.1)/.2
                        if new_height > .05:
                            total_reward -= .025*min(2., max(0., abs(state.forward_velocity_m_s)-.08)/.3)
            else:
                total_reward+=.01*self.candidate.get('balance_duration_reward_per_s',0.)
            self.previous_sample = state
            self.finished=self.decision.success or self.decision.failure is not None
            last_info=info
            if self.finished or self.decision.phase!=Phase.BALANCE:break
            if self.tick>self.max_steps:raise RuntimeError('protocol duration exhausted')
        failure=self.failed_reset or self.decision.failure
        total_reward+=10*bool(self.decision.success)-5*bool(failure and not self.failed_reset)
        last_info.update(failure=failure,success=bool(self.decision.success),phase=self.decision.phase.value,
                         balance_validation_completed=bool(self.decision.success),phase_counts=dict(self.phase_counts),
                         balance_ticks=self.balance_ticks,support_wrench=self.support_wrench.tolist(),
                         support_peak_n=self.support_peak_n,support_peak_nm=self.support_peak_nm,
                         wheel_load_n=self.wheel_load_n,episode_parameters=self.parameters,
                         odometry_confidence=float(self.env.c_quality.confidence),
                         final_roll_pitch_deg=list(self.last_sample.roll_pitch_deg),
                         final_clearance_m=self.last_sample.clearance_m,
                         final_joint_position_rad=list(self.last_sample.joint_position_rad),
                         final_joint_velocity_rad_s=self.env.joint_velocity.tolist(),
                         final_forward_velocity_m_s=self.last_sample.forward_velocity_m_s,
                         final_body_contact=self.last_sample.body_contact,
                         final_displacement_m=self.last_sample.displacement_m,
                         ready_at_end=bool(self.controller.ready(self.last_sample)))
        return self.env.observation(),float(total_reward),bool(self.finished),False,last_info

    def snapshot(self):
        return {**self.env.snapshot(),'task':'seated_back_getup' if self.seated else 'supported_getup_release_handoff','reset_range':list(self.reset_range),
                'zero_torque_settle_s':self.settle_seconds,'domain_randomization':self.domain,'plan':self.plan}
