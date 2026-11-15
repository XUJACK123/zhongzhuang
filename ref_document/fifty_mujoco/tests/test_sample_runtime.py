"""Meaningful network rejection, transmission power and source MJCF checks."""

import mujoco
import numpy as np
import pytest
import torch

from fifty_mujoco.contracts.sample_actor31 import JOINT_ORDER
from fifty_mujoco.transfer.sample_checkpoint import load_sample_actor
from fifty_mujoco.transfer.sample_source import SampleSource, verified_artifact


@pytest.fixture(scope="module")
def source():
    return SampleSource()


def test_loader_rejects_missing_extra_nonfinite_and_wrong_shapes(tmp_path):
    original = torch.load(verified_artifact("model_3050.pt"), weights_only=True, map_location="cpu")
    for change in ("missing", "extra", "nan", "shape", "std"):
        checkpoint = dict(original, model_state_dict={k: v.clone() for k, v in original["model_state_dict"].items()})
        state = checkpoint["model_state_dict"]
        if change == "missing":
            del state["actor.0.bias"]
        elif change == "extra":
            state["encoder.0.weight"] = torch.zeros(3, 125)
        elif change == "nan":
            state["actor.2.weight"][0, 0] = float("nan")
        elif change == "shape":
            state["actor.0.weight"] = torch.zeros(128, 25)
        else:
            state["std"][0] = 0
        path = tmp_path / f"{change}.pt"
        torch.save(checkpoint, path)
        with pytest.raises(ValueError):
            load_sample_actor(path)


def test_actor_rejects_history_schema_and_nonfinite_input():
    actor, metadata = load_sample_actor(verified_artifact("model_3050.pt"))
    assert not metadata["full_training_resume"]
    for invalid in (torch.zeros(1, 25), torch.zeros(31), torch.full((1, 31), float("nan"))):
        with pytest.raises(ValueError):
            actor(invalid)


def test_motor_transform_conserves_power_and_inverts_on_both_sides(source):
    transmission = source.actuator_module.CoupledActuators(torch, source.hardware, JOINT_ORDER)
    rng = torch.Generator().manual_seed(721)
    q = torch.tensor(source.defaults)[None].repeat(64, 1)
    q[:, 1] += torch.linspace(-.4, .4, 64)
    q[:, 4] -= torch.linspace(-.4, .4, 64)
    tau = torch.randn(64, 6, generator=rng, dtype=torch.float64) * 10
    qd = torch.randn(64, 6, generator=rng, dtype=torch.float64) * 5
    motor, speed, ratios = transmission.to_motor(tau, q, qd)
    torch.testing.assert_close(transmission.from_motor(motor, ratios), tau)
    torch.testing.assert_close((tau * qd).sum(1), (motor * speed).sum(1))


def test_motor_envelope_limits_motoring_but_preserves_braking_and_reclips_gain(source):
    transmission = source.actuator_module.CoupledActuators(torch, source.hardware, JOINT_ORDER)
    q = torch.tensor(source.defaults)[None]
    for direction in (-1, 1):
        qd = torch.zeros(1, 6, dtype=torch.float64)
        qd[0, 2] = direction * transmission.ratings["wheel_no_load_rad_s"]
        request = torch.zeros_like(qd)
        request[0, 2] = direction * 100
        result = transmission.apply(request, q, qd, output_gain=torch.ones_like(qd)*2)
        assert result[0, 2] == pytest.approx(0)
        result = transmission.apply(-request, q, qd, output_gain=torch.ones_like(qd)*2)
        assert result[0, 2] == pytest.approx(-direction * transmission.ratings["wheel_torque_nm"])


def test_spring_torque_matches_geometric_virtual_work(source):
    for spring in source.spring_config["springs"]:
        for q in np.linspace(*spring["joint_stop_limits_rad"], 11)[1:-1]:
            eps = 1e-6
            lengths = [source.spring_module.geometry(q+d, spring)[0] for d in (-eps, eps)]
            derivative = (lengths[1]-lengths[0]) / (2*eps)
            torque, _, force = source.spring_module.evaluate(q, .2, spring)
            assert torque == pytest.approx(force * derivative, abs=1e-7)


def test_mjcf_source_inertia_is_rotated_once_and_com_preserved(source):
    from fifty_mujoco.assets.urdf_io import load_urdf
    robot = load_urdf(verified_artifact("fifty_training.urdf"))
    for name, link in robot.links.items():
        body = source.model.body(name).id
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, source.model.body_iquat[body])
        rotation = rotation.reshape(3, 3)
        reconstructed = rotation @ np.diag(source.model.body_inertia[body]) @ rotation.T
        np.testing.assert_allclose(reconstructed, link.inertial.inertia_link_frame, atol=1e-12)
        np.testing.assert_allclose(source.model.body_ipos[body], link.inertial.com, atol=1e-12)


@pytest.mark.parametrize("channel", range(6))
def test_isolated_positive_generalized_impulse_moves_named_coordinate(source, channel):
    source.reset()
    # Remove contact/gravity. A positive diagonal inverse-mass response must
    # remain positive even with the freely moving base.
    source.data.qfrc_applied[source.qvel_ids[channel]] = 1
    source.model.opt.gravity[:] = 0
    old_contype = source.model.geom_contype.copy()
    source.model.geom_contype[:] = 0
    old_affinity = source.model.geom_conaffinity.copy()
    source.model.geom_conaffinity[:] = 0
    try:
        mujoco.mj_step(source.model, source.data)
        assert source.data.qvel[source.qvel_ids[channel]] > 0
        assert np.isfinite(source.data.qpos).all()
    finally:
        source.model.opt.gravity[:] = source.env["sim"]["gravity"]
        source.model.geom_contype[:] = old_contype
        source.model.geom_conaffinity[:] = old_affinity
        source.reset()


def test_reset_has_only_two_wheel_ground_contact_paths_and_no_penetration(source):
    source.reset()
    pairs = {(c["geom1"], c["geom2"]) for c in source.contacts()}
    assert pairs == {("floor", "right_wheel_collision_0"), ("floor", "left_wheel_collision_0")}
    assert min(c["distance_m"] for c in source.contacts()) >= -1e-10
    assert source.policy_dt == pytest.approx(.01)
    assert source.external_dt == pytest.approx(.005)


def test_diagnostic_rollout_reproduces_reset_and_retains_empty_error_window(source):
    import importlib.util
    from pathlib import Path
    path = Path(__file__).resolve().parents[1] / "tools/sample_rollout_diagnostic.py"
    spec = importlib.util.spec_from_file_location("sample_diagnostic_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    actor, _ = load_sample_actor(verified_artifact("model_3050.pt"))
    row, trace = module.trial(source, actor, [.15, 0, .1], 10300, .03)
    repeated, second_trace = module.trial(source, actor, [.15, 0, .1], 10300, .03)
    assert row == repeated
    np.testing.assert_array_equal(trace, second_trace)
    assert row["completed"]
    assert trace.shape == (3, 88)
    assert row["metrics_after_2s"]["samples"] == 0
    assert row["metrics_after_2s"]["root_vx_rmse"] is None
