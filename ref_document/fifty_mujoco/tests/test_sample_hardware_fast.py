import numpy as np
import torch
import pytest
from fifty_mujoco.envs.sample_env31_hardware import SampleHardwareEnv31
from fifty_mujoco.envs.sample_env31_hardware_fast import FastHardwareEnv31
from fifty_mujoco.learning.sample_ppo import SamplePolicy31
from fifty_mujoco.transfer.sample_source import verified_artifact


def test_scalar_actuator_matches_reference_with_caps_asymmetry_and_high_speeds():
    e=FastHardwareEnv31(motor_gain=[.8,.9,1.05,1.02,.86,.95])
    rng=np.random.default_rng(45)
    for _ in range(500):
        q=e.source.defaults+rng.uniform(-.08,.08,6)
        qd=rng.uniform(-75,75,6);request=rng.uniform(-100,100,6)
        actual=e.fast_actuators.apply(request,q,qd)
        expected=e.actuators.apply(torch.tensor(request)[None],torch.tensor(q)[None],torch.tensor(qd)[None])[0].numpy()
        assert actual==pytest.approx(expected,abs=1e-11,rel=1e-12)
        assert e.fast_actuators.command==pytest.approx(e.actuators.motor_torque_applied[0].numpy(),abs=1e-11)
        assert e.fast_actuators.velocity==pytest.approx(e.actuators.motor_velocity[0].numpy(),abs=1e-11)


def test_fast_and_reference_closed_loop_trajectory_match():
    torch.set_num_threads(1)
    slow=SampleHardwareEnv31(episode_seconds=1.,wheel_kd=.1)
    fast=FastHardwareEnv31(episode_seconds=1.,wheel_kd=.1)
    policy=SamplePolicy31(verified_artifact('model_3050.pt'))
    a=slow.reset(847,(.15,.3,.1));b=fast.reset(847,(.15,.3,.1))
    for _ in range(100):
        with torch.no_grad():action=policy.actor(torch.tensor(a)[None])[0].numpy()
        a,ra,da,ta,ia=slow.step(action);b,rb,db,tb,ib=fast.step(action)
        assert np.allclose(a,b,atol=1e-5,rtol=1e-5)
        assert np.allclose(slow.data.qpos,fast.data.qpos,atol=1e-6,rtol=1e-6)
        assert ra==pytest.approx(rb,abs=1e-7)
        assert (da,ta)==(db,tb)
        if da or ta:break


def test_guarded_target_clamp_retains_original_physical_failure_limits():
    from fifty_mujoco.envs.sample_env31_hardware_guarded import GuardedHardwareEnv31
    e=GuardedHardwareEnv31(knee_margin_rad=.03,wheel_kd=.1)
    reference=FastHardwareEnv31(wheel_kd=.1)
    assert np.array_equal(e.model.jnt_range,reference.model.jnt_range)
    assert e.target_limits[0][0]==pytest.approx(e.limits[0,0]+.03)
    assert e.target_limits[1][1]==pytest.approx(e.limits[1,1]-.03)
    from fifty_mujoco.contracts.sample_actor31 import action_targets
    targets=action_targets([-1,-1,0,1,1,0],e.target_limits)
    assert targets['position'][1]>=e.limits[0,0]+.03-1e-12
    assert targets['position'][4]<=e.limits[1,1]-.03+1e-12
    with pytest.raises(ValueError):GuardedHardwareEnv31(knee_margin_rad=-.01)
