"""Physical invariants and support accounting for the isolated candidate."""
import copy
from dataclasses import replace
from types import SimpleNamespace
import json
import math

import numpy as np
import pytest
import torch

from fifty_mujoco.envs.sample_env31_realdata import RealDataHardwareEnv31,configuration,spring_force
from fifty_mujoco.envs.sample_env31_supported import SupportedGetupEnv31,CandidateController
from fifty_mujoco.contracts.getup31 import Sample,Phase,Decision
from fifty_mujoco.paths import PROJECT_ROOT


@pytest.fixture(scope='module')
def cfg():
    return json.loads((PROJECT_ROOT/'configs/realdata_350_20261117.json').read_text())


def test_gas_curve_endpoints_and_progression(cfg):
    s=cfg['spring'];free=s['free_pin_distance_m']
    assert spring_force(free,0,s)==pytest.approx(s['initial_force_n'])
    assert spring_force(free-s['stroke_m'],0,s)==pytest.approx(s['initial_force_n']*s['progression_at_full_stroke'])
    values=[spring_force(v,0,s) for v in np.linspace(free-s['stroke_m'],free,25)]
    assert all(a>b for a,b in zip(values,values[1:]))


@pytest.mark.parametrize('rate',[-.5,-.02,.02,.5])
def test_hysteresis_is_dissipative(cfg,rate):
    s=cfg['spring'];length=.22
    dissipative=spring_force(length,rate,s)-spring_force(length,0,s)
    assert dissipative*rate<=0


def test_candidate_mass_radius_and_caps(cfg):
    e=RealDataHardwareEnv31(config=cfg)
    assert e.model.body_mass.sum()==pytest.approx(21.9)
    for name in ('right_wheel_collision_0','left_wheel_collision_0'):
        assert e.model.geom_size[e.model.geom(name).id,0]==pytest.approx(.7/(4*math.pi))
    np.testing.assert_equal(e.fast_actuators.limit,[20,20,2,20,20,2])
    assert np.all(e.model.dof_armature[e.source.qvel_ids]>0)
    assert e.source.external_dt==.002 and e.source.policy_dt==.01


def test_spring_virtual_work_matches_finite_difference(cfg):
    e=RealDataHardwareEnv31(config=cfg)
    q=e.source.defaults.copy();qdot=np.array([0,.1,0,0,-.1,0.])
    torque,lengths=e._spring_torque(q,qdot)
    for k,spring,force in zip((1,4),e.source.spring_config['springs'],e.last_spring_forces):
        f=lambda x:e.source.spring_module.geometry(float(x),spring)[0]
        derivative=(f(q[k]+1e-6)-f(q[k]-1e-6))/(2e-6)
        assert torque[k]==pytest.approx(force*derivative,rel=1e-7)


class ConstantActor(torch.nn.Module):
    def __init__(self):
        super().__init__();self.anchor=torch.nn.Parameter(torch.zeros(1))
    def forward(self,obs):
        return torch.zeros((obs.shape[0],6),device=obs.device)+self.anchor*0


def test_supported_zero_output_settle_and_force_release(cfg):
    torch.set_num_threads(1)
    e=SupportedGetupEnv31(ConstantActor(),config=cfg,settle_seconds=1.)
    observation=e.reset(618000)
    assert len(observation)==31 and np.isfinite(observation).all()
    assert e.failed_reset is None and e.decision.failure is None
    np.testing.assert_equal(e.env.fast_actuators.command,np.zeros(6))
    np.testing.assert_equal(e.env.fast_actuators.delivered,np.zeros(6))
    assert e.support_peak_n>0 and e.support_peak_nm>0
    e.enabled_time=e.release_after_s+e.fade_s+.1;e._support()
    np.testing.assert_equal(e.support_wrench,np.zeros(6))
    np.testing.assert_equal(e.env.data.xfrc_applied,np.zeros_like(e.env.data.xfrc_applied))


def test_actor_observation_uses_new_defaults(cfg):
    e=RealDataHardwareEnv31(config=cfg);obs=e.reset()
    np.testing.assert_allclose(obs[9:13],0,atol=1e-7)
    assert obs[8]==pytest.approx(5*cfg['standing_clearance_m'])


def test_quiet_entry_and_blend_have_separate_velocity_gates(cfg):
    plan=json.loads((PROJECT_ROOT/'configs/hardware31_next_training_20261115.json').read_text())
    plan['getup']['actor_commands'][2]=cfg['standing_clearance_m']
    plan['getup']['handoff_velocity_limit_m_s']=.035
    plan['getup']['blend_velocity_limit_m_s']=.08
    controller=CandidateController(plan,cfg['standing_default_rad'])
    s=Sample(time_s=1.,joint_position_rad=tuple(cfg['standing_default_rad']),roll_pitch_deg=(0.,0.),
             angular_velocity_rad_s=(0.,0.,0.),forward_velocity_m_s=.05,
             clearance_m=cfg['standing_clearance_m'],displacement_m=0.,both_wheels_supported=True,
             body_contact=False,assistance_force_n=0.,telemetry_valid=True,joint_limits_valid=True,
             spring_stroke_valid=True,linkage_branch_valid=True,forbidden_contact=False)
    controller.phase=Phase.GETUP
    assert not controller.ready(s)
    controller.phase=Phase.HANDOFF
    assert controller.ready(s)
    controller.phase=Phase.BALANCE
    assert controller.ready(s)
    assert controller._failure(replace(s,forward_velocity_m_s=.0801))=='balance_state_out_of_spec'
    assert not controller.ready(replace(s,assistance_force_n=.01))
    assert not controller.ready(replace(s,both_wheels_supported=False))
    # Earlier courses keep the tighter blend gate without this opt-in key.
    del plan['getup']['blend_velocity_limit_m_s']
    legacy=CandidateController(plan,cfg['standing_default_rad']);legacy.phase=Phase.HANDOFF
    assert not legacy.ready(s)


@pytest.mark.parametrize('duration_reward,expected',[(0.,10.),(.5,15.)])
def test_frozen_balance_is_one_terminal_learner_transition(cfg,duration_reward,expected):
    """A plant stub isolates clock/coalescing from physics correctness tests."""
    plan=json.loads((PROJECT_ROOT/'configs/hardware31_next_training_20261115.json').read_text())
    plan['getup']['actor_commands'][2]=cfg['standing_clearance_m']
    controller=CandidateController(plan,cfg['standing_default_rad'])
    controller.phase=Phase.BALANCE;controller.balance_at=0.;controller.previous_time=0.
    e=SupportedGetupEnv31.__new__(SupportedGetupEnv31)
    e.candidate={**cfg,'balance_duration_reward_per_s':duration_reward}
    e.controller=controller;e.decision=Decision(Phase.BALANCE,1.,1.,False,False,None)
    e.data=SimpleNamespace(time=0.);e.start_time=0.;e.finished=False;e.failed_reset=None
    e.phase_counts={};e.balance_ticks=e.tick=0;e.max_steps=2000
    e.parameters={};e.support_wrench=np.zeros(6);e.support_peak_n=e.support_peak_nm=0.
    e.wheel_load_n={'right':100.,'left':110.}
    e.env=SimpleNamespace(last_action=np.zeros(6),target_limits=[[-3.69,-2.56],[2.56,3.69]],
                          observation=lambda:np.zeros(31),c_quality=SimpleNamespace(confidence=1.),joint_velocity=np.zeros(6))
    e._balance_action=lambda:np.zeros(6)
    def advance(action,scale):
        e.data.time+=.01
        s=Sample(time_s=e.data.time,joint_position_rad=tuple(cfg['standing_default_rad']),
                 roll_pitch_deg=(0.,0.),angular_velocity_rad_s=(0.,0.,0.),forward_velocity_m_s=0.,
                 clearance_m=cfg['standing_clearance_m'],displacement_m=0.,both_wheels_supported=True,
                 body_contact=False,assistance_force_n=0.,telemetry_valid=True,joint_limits_valid=True,
                 spring_stroke_valid=True,linkage_branch_valid=True,forbidden_contact=False)
        e.last_sample=s
        return np.zeros(31),{'failure':None},s
    e._advance=advance
    _,reward,terminated,truncated,info=e.step(np.ones(6))
    assert terminated and not truncated and info['success']
    assert info['balance_ticks']==1000
    assert reward==pytest.approx(expected)
