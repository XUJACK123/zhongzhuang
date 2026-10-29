"""Checkpoint / resume contract (doc section 5.5).

A 3-hour run once overwrote a single ``model_last.pt`` and lost the good
policy.  These tests pin the properties that prevent it: numbered snapshots
are immutable, writes are atomic, retention is bounded, ``best`` follows an
ordered metric list, and ``resume`` restores optimiser + learning rate +
counter while ``init`` deliberately does not.
"""

from __future__ import annotations

import pytest

torch = pytest.importorskip("torch")

from fifty_mujoco.learning.actor_critic import ActorCriticSequence, NetworkDims
from fifty_mujoco.learning.checkpoint import (
    CHECKPOINT_FORMAT,
    CheckpointManager,
    load_checkpoint,
    metric_is_better,
    save_checkpoint,
)
from fifty_mujoco.learning.gpu_ppo import GpuPpoConfig


def _policy(dims: NetworkDims) -> ActorCriticSequence:
    torch.manual_seed(0)
    return ActorCriticSequence(dims)


def _payload(dims: NetworkDims, update: int, lr: float = 1e-3) -> dict:
    policy = _policy(dims)
    optimizer = torch.optim.Adam(policy.parameters(), lr=lr)
    return {
        "format": CHECKPOINT_FORMAT,
        "policy": policy.state_dict(),
        "optimizer": optimizer.state_dict(),
        "dims": vars(dims),
        "cfg": {"lr": lr},
        "history": [],
        "env_steps": update * 100,
        "update": update,
        "learning_rate": lr,
        "rng": None,
        "fingerprint": {"mjcf": "abc"},
        "eval_version": "stand_eval/v1",
    }


@pytest.fixture()
def dims() -> NetworkDims:
    return NetworkDims(num_privileged=149)


def test_atomic_write_leaves_no_temporary_file(tmp_path, dims):
    path = save_checkpoint(tmp_path / "model_last.pt", _payload(dims, 1))
    assert path.is_file()
    assert not path.with_name(path.name + ".tmp").exists()
    assert load_checkpoint(path)["update"] == 1


def test_numbered_snapshots_are_never_overwritten(tmp_path, dims):
    manager = CheckpointManager(tmp_path, keep=10)
    first = manager.save_numbered(25, _payload(dims, 25))
    original = first.read_bytes()
    manager.save_numbered(50, _payload(dims, 50))
    again = manager.save_numbered(25, _payload(dims, 25, lr=9.0))
    assert again != first
    assert first.read_bytes() == original
    assert load_checkpoint(first)["update"] == 25
    assert load_checkpoint(again)["learning_rate"] == pytest.approx(9.0)


def test_retention_keeps_the_newest_snapshots(tmp_path, dims):
    manager = CheckpointManager(tmp_path, keep=3)
    for update in range(1, 8):
        manager.save_numbered(update, _payload(dims, update))
    kept = sorted(path.name for path in tmp_path.glob("model_update_*.pt"))
    assert kept == ["model_update_000005.pt", "model_update_000006.pt", "model_update_000007.pt"]


def test_metric_ordering_prefers_safety_then_survival_then_tracking(dims):
    order = ("eval_alive_frac", "eval_t_fall_p10", "eval_vx_mae")
    worse_alive = {"eval_alive_frac": 0.5, "eval_t_fall_p10": 60.0, "eval_vx_mae": 0.0}
    better_alive = {"eval_alive_frac": 1.0, "eval_t_fall_p10": 1.0, "eval_vx_mae": 5.0}
    assert metric_is_better(better_alive, worse_alive, order)
    assert not metric_is_better(worse_alive, better_alive, order)
    longer = {"eval_alive_frac": 0.5, "eval_t_fall_p10": 61.0, "eval_vx_mae": 9.0}
    assert metric_is_better(longer, worse_alive, order)
    nan_metric = {"eval_alive_frac": float("nan"), "eval_t_fall_p10": 99.0, "eval_vx_mae": 0.0}
    assert not metric_is_better(nan_metric, worse_alive, order)
    assert metric_is_better(worse_alive, nan_metric, order)


def test_best_is_promoted_only_when_it_wins(tmp_path, dims):
    manager = CheckpointManager(tmp_path, keep=2)
    assert manager.maybe_save_best(10, _payload(dims, 10), {"eval_alive_frac": 0.9, "eval_t_fall_p10": 50.0})
    assert manager.best_update == 10
    assert not manager.maybe_save_best(
        20, _payload(dims, 20), {"eval_alive_frac": 0.4, "eval_t_fall_p10": 50.0}
    )
    assert manager.best_update == 10
    assert manager.maybe_save_best(
        30, _payload(dims, 30), {"eval_alive_frac": 0.9, "eval_t_fall_p10": 55.0}
    )
    assert manager.best_update == 30
    assert load_checkpoint(manager.best_path)["update"] == 30


def test_release_is_immutable(tmp_path, dims):
    manager = CheckpointManager(tmp_path, keep=2)
    manager.maybe_save_best(10, _payload(dims, 10), {"eval_alive_frac": 1.0})
    released = manager.release("v1")
    assert released.is_file()
    with pytest.raises(FileExistsError):
        manager.release("v1")


def test_checkpoint_rejects_a_newer_format(tmp_path, dims):
    payload = _payload(dims, 1)
    payload["format"] = CHECKPOINT_FORMAT + 1
    path = save_checkpoint(tmp_path / "future.pt", payload)
    with pytest.raises(ValueError):
        load_checkpoint(path)


def test_trainer_resume_restores_optimizer_and_counter(tmp_path, dims):
    """``resume`` restores the experiment; ``init`` only the weights.

    A real ``GpuPpoTrainer`` needs CUDA, so the save/load methods are exercised
    on a lightweight stand-in that reuses them unchanged.
    """
    from fifty_mujoco.learning.gpu_ppo import GpuPpoTrainer

    class Env:
        num_privileged_obs = 149
        num_obs = 25

        class gains:
            leg_kp = 20.0
            leg_kd = 1.0
            wheel_kd = 0.2
            torque_limit_nm = 30.0
            wheel_torque_limit_nm = 5.0
            leg_position_scale = 0.5
            wheel_velocity_scale = 10.0

        spawn_z = 0.47

    class Stub:
        _checkpoint_payload = GpuPpoTrainer._checkpoint_payload
        save = GpuPpoTrainer.save
        load = GpuPpoTrainer.load

        def __init__(self):
            self.cfg = GpuPpoConfig()
            self.device = torch.device("cpu")
            self.env = Env()
            self.policy = _policy(dims)
            self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=1e-3)
            self.learning_rate = 1e-3
            self.history = []
            self.total_env_steps = 0
            self.update_count = 0
            self.best_update = 0
            self.checkpoints = CheckpointManager(tmp_path, keep=2)

    first = Stub()
    first.update_count = 7
    first.total_env_steps = 112
    first.learning_rate = 4e-4
    first.history = [{"update": 7}]
    first.save(tmp_path / "model_last.pt")

    resumed = Stub()
    info = resumed.load(tmp_path / "model_last.pt", resume=True)
    assert info["resumed"] and info["update"] == 7
    assert resumed.update_count == 7
    assert resumed.total_env_steps == 112
    assert resumed.learning_rate == pytest.approx(4e-4)
    assert resumed.optimizer.param_groups[0]["lr"] == pytest.approx(4e-4)

    fresh = Stub()
    info = fresh.load(tmp_path / "model_last.pt", resume=False)
    assert not info["resumed"]
    assert fresh.update_count == 0
    assert fresh.history == []
