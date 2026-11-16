import numpy as np
import pytest
import torch

from fifty_mujoco.contracts.sample_actor31 import ramp_commands
from fifty_mujoco.envs.sample_env31_hardware import SampleHardwareEnv31


def test_commissioned_timing_and_surveyed_geometry():
    e = SampleHardwareEnv31(reset_mode='natural')
    e.reset(1)
    assert e.source.external_dt == pytest.approx(.002)
    assert e.source.policy_dt == pytest.approx(.01)
    assert e.source.env['control']['decimation'] == 5
    assert e.data.qpos[2] == pytest.approx(.3, abs=2e-6)
    assert e.source.hardware['wheel_motor']['total_reduction_ratio'] == 15.88
    assert e.model.geom('right_wheel_collision_0').size[0] == .055
    assert min((c['distance_m'] for c in e.source.contacts()), default=0.) >= -1e-9
    assert e.data.geom_xpos[e.model.geom('right_wheel_collision_0').id, 2] == pytest.approx(.055)


def test_firmware_command_ramp_before_actor_and_reference_after_step():
    e = SampleHardwareEnv31()
    o = e.reset(6, (.15, .3, .1))
    expected = ramp_commands([0,0,.1], [.15,.3,.1], .01)
    assert o[6:9] == pytest.approx(np.array(expected)*[2,.25,5])
    o, _, _, _, info = e.step(np.zeros(6))
    assert info['applied_command'] == pytest.approx(expected)
    expected2 = ramp_commands(expected, [.15,.3,.1], .01)
    assert o[6:9] == pytest.approx(np.array(expected2)*[2,.25,5])
    assert e.heading == pytest.approx(expected[1]*.01)


def test_motor_caps_joint_caps_and_delivery_gain_are_distinct():
    e = SampleHardwareEnv31(motor_gain=.8)
    q = torch.tensor(e.data.qpos[e.source.qpos_ids])[None]
    z = torch.zeros_like(q)
    a = e.actuators
    a.apply(torch.full_like(q, .01), q, z)
    assert torch.allclose(a.motor_torque_requested, a.motor_torque_applied)
    assert torch.allclose(a.delivered_motor_torque, .8*a.motor_torque_applied)
    a.apply(torch.full_like(q, 1e5), q, z)
    assert torch.all(a.motor_torque_applied.abs() <= torch.tensor([20,20,2]*2)+1e-10)
    assert torch.all(a.delivered_motor_torque.abs() <= torch.tensor([16,16,1.6]*2)+1e-10)
    assert a.motor_torque_requested[0,2] == pytest.approx(e.source.profile['joint_effort_nm'][2])


def test_reset_reproducibility_and_spring_stroke_endpoints():
    e = SampleHardwareEnv31(reset_mode='mixed')
    first = e.reset(123)
    pose = e.data.qpos.copy()
    assert np.array_equal(first, e.reset(123))
    assert np.array_equal(pose, e.data.qpos)
    for mode in ('default', 'natural'):
        e = SampleHardwareEnv31(reset_mode=mode)
        q = e.data.qpos[e.source.qpos_ids]
        _, lengths = e.source.spring_torque(q, np.zeros(6))
        assert all(.175 <= x <= .25 for x in lengths)


def test_default_rollout_is_finite_and_terminal_is_retained():
    e = SampleHardwareEnv31(episode_seconds=.02)
    for _ in range(2):
        o,r,d,t,info=e.step(np.zeros(6))
        assert np.isfinite(o).all() and np.isfinite(r)
    assert d or t
    with pytest.raises(RuntimeError):
        e.step(np.zeros(6))


@pytest.mark.parametrize('kwargs', [dict(motor_gain=0), dict(spring_scale=float('nan')),
                                    dict(episode_seconds=-1), dict(reset_mode='bad')])
def test_invalid_hardware_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        SampleHardwareEnv31(**kwargs)


def test_firmware_c_estimator_matches_port_on_moving_joint_trace(tmp_path):
    import ctypes as c
    import subprocess
    from pathlib import Path
    import mujoco
    from fifty_mujoco.math3d import mat_to_quat_wxyz, rpy_to_mat
    from fifty_mujoco.transfer.sample_source import source_module

    firmware = Path(__file__).resolve().parents[2] / 'chassis_ref_new'
    library = tmp_path / 'firmware.so'
    subprocess.run(['cc','-std=c11','-O2','-shared','-fPIC','-I'+str(firmware/'user'),
        str(firmware/'user/state_estimator.c'),str(firmware/'user/algorithm/rl.c'),
        str(firmware/'user/algorithm/model/model.c'),'-lm','-o',str(library)], check=True)
    class Estimator(c.Structure):
        _fields_=[('xy',c.c_float*2),('last_q',c.c_float*6),('held_qd',c.c_float*6),
                  ('valid',c.c_bool),('tick',c.c_uint32)]
    class Measurement(c.Structure):
        _fields_=[('q',c.c_float*6),('qd',c.c_float*6),('omega',c.c_float*3),
                  ('gravity',c.c_float*3),('xy',c.c_float*2),('yaw',c.c_float),('velocity',c.c_float*2)]
    lib=c.CDLL(str(library))
    lib.estimator_update.restype=c.c_bool
    lib.estimator_update.argtypes=[c.POINTER(Estimator),c.POINTER(Measurement),
        c.POINTER(c.c_float),c.POINTER(c.c_float),c.POINTER(c.c_float),c.POINTER(c.c_float),c.c_uint32,c.c_float]
    e=SampleHardwareEnv31()
    est,measurement=Estimator(),Measurement()
    helper=source_module('actuator_model')
    base=e.source.defaults.copy()
    quat=mat_to_quat_wxyz(rpy_to_mat([.04,-.08,.2]))
    cf=lambda v:(c.c_float*len(v))(*v)
    for tick in range(101):
        t=tick*.005
        q=base+np.array([.05,-.03,.08,-.04,.025,-.06])*np.sin(3*t)
        qd=np.array([.05,-.03,.08,-.04,.025,-.06])*3*np.cos(3*t)
        # Initial stationary sample establishes the same held-difference history.
        if tick==0: qd[:]=0
        arms=q.copy();speed=qd.copy()
        for h,k,sign in ((0,1,1),(3,4,-1)):
            beta=sign*q[k]+np.pi/2
            alpha=helper.alpha_from_bend(beta,e.source.hardware)
            ratio=helper.knee_ratio(beta,e.source.hardware)
            arms[h]=sign*q[h];arms[k]=arms[h]+alpha
            speed[h]=sign*qd[h];speed[k]=sign*(qd[h]+qd[k]/ratio)
        omega=np.array([.03,-.02,.04]) if tick else np.zeros(3)
        e.data.qpos[3:7]=quat;e.data.qpos[e.source.qpos_ids]=q
        e.data.qvel[e.source.qvel_ids]=qd;e.data.qvel[3:6]=omega;e.data.time=t
        mujoco.mj_forward(e.model,e.data)
        # mj_objectVelocity supplies body-frame gyro for the very same state.
        gyro=e._velocity6()[:3]
        assert lib.estimator_update(c.byref(est),c.byref(measurement),cf(quat),cf(arms),cf(speed),cf(gyro),tick*5,.005)
        if tick:
            assert e._update_estimator()
            assert np.asarray(measurement.qd)==pytest.approx(e.joint_velocity,abs=2e-4)
            assert np.asarray(measurement.xy)==pytest.approx(e.estimated_xy,abs=2e-6)
            assert np.asarray(measurement.velocity)==pytest.approx(e.estimated_velocity[:2],abs=2e-6)
    # Use the same compiled firmware to verify PD -> joint clamp -> coupled
    # motor torque and speed-envelope ordering independently of Python algebra.
    class Controller(c.Structure):
        _fields_=[('network',c.c_float*224),('obs',c.c_float*31),('action',c.c_float*6),
                  ('command',c.c_float*3),('reference_xy',c.c_float*2),('reference_yaw',c.c_float),
                  ('target_q',c.c_float*6),('target_qd',c.c_float*6),
                  ('joint_torque',c.c_float*6),('motor_torque',c.c_float*6)]
    lib.rl_torque_step.restype=c.c_bool
    lib.rl_torque_step.argtypes=[c.POINTER(Controller),c.POINTER(Measurement)]
    rng=np.random.default_rng(9)
    for _ in range(80):
        ctrl=Controller();q=base+rng.uniform(-.025,.025,6);qd=rng.uniform(-2,2,6)
        target=q+rng.uniform(-.5,.5,6);velocity=rng.uniform(-30,30,6)
        ctrl.target_q[:]=target;ctrl.target_qd[:]=velocity
        measurement.q[:]=q;measurement.qd[:]=qd
        assert lib.rl_torque_step(c.byref(ctrl),c.byref(measurement))
        error=target-q;error[[0,3]]=np.arctan2(np.sin(error[[0,3]]),np.cos(error[[0,3]]))
        # Current exported firmware uses the accepted Kd=0.14 candidate.
        request=np.array([40,60,0]*2)*error+np.array([2,2,.14]*2)*(velocity-qd)
        e.actuators.apply(torch.tensor(request)[None],torch.tensor(q)[None],torch.tensor(qd)[None])
        assert np.array(ctrl.motor_torque)==pytest.approx(e.actuators.motor_torque_applied[0].numpy(),abs=3e-4)
