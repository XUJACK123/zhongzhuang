import ctypes as c
import numpy as np
import pytest
import torch

from fifty_mujoco.envs.sample_env31_qualified import QualifiedHardwareEnv31
from fifty_mujoco.transfer.firmware_runtime import Controller, Measurement, floats
from fifty_mujoco.learning.sample_ppo import SamplePolicy31
from fifty_mujoco.transfer.sample_source import verified_artifact
from fifty_mujoco.paths import PROJECT_ROOT
from fifty_mujoco.envs.sample_env31_hardware import rpy_from_wxyz


def test_qualified_trajectory_observations_match_current_c_actor():
    torch.set_num_threads(1)
    e = QualifiedHardwareEnv31(wheel_kd=.14, episode_seconds=4)
    p = SamplePolicy31(verified_artifact("model_3050.pt"))
    p.load_state_dict(torch.load(PROJECT_ROOT/"runs/hardware31_guarded_20261115/update_000100.pt",
                                 map_location="cpu", weights_only=True)["model"])
    obs = e.reset(2000000, (.15, .3, .1))
    for _ in range(300):
        m, ctrl = Measurement(), Controller()
        m.q[:], m.qd[:] = e.data.qpos[e.source.qpos_ids], e.joint_velocity
        m.omega[:], m.gravity[:] = e._velocity6()[:3], e._rotation().T @ [0., 0., -1.]
        m.xy[:], m.yaw, m.velocity[:] = e.estimated_xy, rpy_from_wxyz(e.data.qpos[3:7])[2], e.estimated_velocity[:2]
        ctrl.reference_xy[:], ctrl.reference_yaw = e.reference, e.heading
        ctrl.command[:], ctrl.action[:] = e.applied, e.last_action
        request = e.requested if e.c_quality.confidence >= .25 else [0., 0., .1]
        assert e.runtime.rl_policy_step(c.byref(ctrl), c.byref(m), floats(request), .01)
        assert np.array(ctrl.obs) == pytest.approx(obs, abs=2e-5)
        with torch.no_grad():
            action = p.actor(torch.tensor(obs)[None])[0].numpy()
        limits = np.array([1., 1., 2., 1., 1., 2.])
        assert np.array(ctrl.action) == pytest.approx(np.clip(action, -limits, limits), abs=2e-4)
        obs, _, done, truncated, info = e.step(action)
        assert not done, info
    assert e.c_quality.confidence > .25


def test_quality_loss_gates_command_and_realigns_reference():
    e = QualifiedHardwareEnv31(wheel_kd=.14)
    e.reset(2000001, (.15, .3, .1))
    e.reference = [1., -1.]
    e.c_quality.confidence = 0.
    e.reference_managed_tick = None
    observation = e.observation()
    assert e.reference == pytest.approx(e.estimated_xy)
    assert observation[6:8] == pytest.approx([0., 0.])
    assert e.requested == [.15, .3, .1]
    e.c_quality.confidence = 1.
    e.c_quality.reference_realign = True
    e.reference_managed_tick = None
    e.observation()
    assert not e.c_quality.reference_realign
    assert e.reference == pytest.approx(e.estimated_xy)


def test_reset_clears_estimator_episode_state():
    e = QualifiedHardwareEnv31(wheel_kd=.14)
    initial = e.reset(2000002).copy()
    for _ in range(3):
        e.step([0.]*6)
    e.c_quality.degraded_ms = 50
    e.c_estimator.xy[:] = [3., 4.]
    assert np.array_equal(e.reset(2000002), initial)
    assert e.c_quality.degraded_ms == 0
    assert list(e.c_estimator.xy) == [0., 0.]
