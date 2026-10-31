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


def test_tracking_errors_compare_the_other_way_round(dims):
    """A MAE is an error: 0.02 m must beat 0.30 m (progress.md §23.6 item 2)."""
    order = ("eval_alive_frac", "eval_height_mae", "eval_vx_mae")
    good = {"eval_alive_frac": 1.0, "eval_height_mae": 0.02, "eval_vx_mae": 0.01}
    bad = {"eval_alive_frac": 1.0, "eval_height_mae": 0.30, "eval_vx_mae": 0.01}
    assert metric_is_better(good, bad, order)
    assert not metric_is_better(bad, good, order)
    # survival still dominates: a survivor with a worse error wins
    survivor = {"eval_alive_frac": 1.0, "eval_height_mae": 0.30, "eval_vx_mae": 0.01}
    faller = {"eval_alive_frac": 0.0, "eval_height_mae": 0.0, "eval_vx_mae": 0.0}
    assert metric_is_better(survivor, faller, order)
    # a name that is not an error keeps "larger is better"
    assert metric_is_better({"score": 2.0}, {"score": 1.0}, ("score",))
    assert not metric_is_better({"score": 1.0}, {"score": 2.0}, ("score",))


def test_resumed_ranking_survives_a_new_process(tmp_path, dims):
    """``checkpoints.json`` must restore the ranking, not just the counters."""
    first = CheckpointManager(tmp_path, keep=2)
    assert first.maybe_save_best(
        10, _payload(dims, 10), {"eval_alive_frac": 1.0, "eval_height_mae": 0.02}
    )
    reopened = CheckpointManager(tmp_path, keep=2)
    assert reopened.best_update == 10
    assert reopened.best_metrics == pytest.approx(
        {"eval_alive_frac": 1.0, "eval_height_mae": 0.02}
    )
    assert not reopened.maybe_save_best(
        20, _payload(dims, 20), {"eval_alive_frac": 1.0, "eval_height_mae": 0.30}
    )
    assert reopened.best_update == 10
    assert load_checkpoint(reopened.best_path)["update"] == 10


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


class _Env:
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


def _trainer_stub(tmp_path, dims, updates: int = 300):
    """A ``GpuPpoTrainer`` without CUDA: only the bookkeeping methods.

    A real trainer constructs a Warp/CUDA environment, so every test that is
    about *learning bookkeeping* rather than physics binds the real methods to
    this stand-in.
    """
    from fifty_mujoco.learning.gpu_ppo import GpuPpoTrainer

    class Stub:
        _checkpoint_payload = GpuPpoTrainer._checkpoint_payload
        save = GpuPpoTrainer.save
        load = GpuPpoTrainer.load
        _promote_best = GpuPpoTrainer._promote_best
        _update_range = GpuPpoTrainer._update_range

        def __init__(self):
            self.cfg = GpuPpoConfig(updates=updates)
            self.device = torch.device("cpu")
            self.env = _Env()
            self.policy = _policy(dims)
            self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=1e-3)
            self.learning_rate = 1e-3
            self.history = []
            self.total_env_steps = 0
            self.update_count = 0
            self.best_update = 0
            self.best_metrics = None
            self.init_from = None
            self.checkpoints = CheckpointManager(tmp_path, keep=2)

    return Stub()


def test_trainer_resume_restores_optimizer_and_counter(tmp_path, dims):
    """``resume`` restores the experiment; ``init`` only the weights.

    A real ``GpuPpoTrainer`` needs CUDA, so the save/load methods are exercised
    on a lightweight stand-in that reuses them unchanged.
    """
    first = _trainer_stub(tmp_path, dims)
    first.update_count = 7
    first.total_env_steps = 112
    first.learning_rate = 4e-4
    first.history = [{"update": 7}]
    first.best_update = 7
    first.best_metrics = {"eval_alive_frac": 1.0, "eval_height_mae": 0.02}
    first.save(tmp_path / "model_last.pt")

    resumed = _trainer_stub(tmp_path, dims)
    info = resumed.load(tmp_path / "model_last.pt", resume=True)
    assert info["resumed"] and info["update"] == 7
    assert resumed.update_count == 7
    assert resumed.total_env_steps == 112
    assert resumed.learning_rate == pytest.approx(4e-4)
    assert resumed.optimizer.param_groups[0]["lr"] == pytest.approx(4e-4)
    assert resumed.best_update == 7
    assert resumed.best_metrics == pytest.approx(
        {"eval_alive_frac": 1.0, "eval_height_mae": 0.02}
    )
    assert resumed.init_from == {"path": str(tmp_path / "model_last.pt"), "parts": "all"}

    fresh = _trainer_stub(tmp_path, dims)
    info = fresh.load(tmp_path / "model_last.pt", resume=False)
    assert not info["resumed"]
    assert fresh.update_count == 0
    assert fresh.history == []


def test_resume_continues_the_update_numbering(tmp_path, dims):
    """§23.6 item 3: the loop used to restart at 1 after loading ``update=7``."""
    stub = _trainer_stub(tmp_path, dims, updates=9)
    assert stub._update_range() == range(1, 10)
    stub.update_count = 7
    assert stub._update_range() == range(8, 10)
    stub.update_count = 9
    assert len(stub._update_range()) == 0


def test_best_follows_every_evaluation_not_only_survival(tmp_path, dims):
    """§23.6 item 1: equal survival with a lower error must still promote."""
    stub = _trainer_stub(tmp_path, dims)
    first = {
        "eval_alive_frac": 1.0,
        "eval_t_fall_p10": 50.0,
        "eval_t_fall_median": 55.0,
        "eval_height_mae": 0.05,
        "eval_vx_mae": 0.02,
    }
    stub.update_count = 10
    assert stub._promote_best(10, first)
    assert stub.best_update == 10

    stub.update_count = 20
    worse = dict(first, eval_height_mae=0.09)
    assert not stub._promote_best(20, worse)
    assert stub.best_update == 10
    assert load_checkpoint(stub.checkpoints.best_path)["update"] == 10

    stub.update_count = 30
    better = dict(first, eval_height_mae=0.02)
    assert stub._promote_best(30, better)
    assert stub.best_update == 30
    assert load_checkpoint(stub.checkpoints.best_path)["update"] == 30
    assert stub.best_metrics["eval_height_mae"] == pytest.approx(0.02)


def test_best_selection_without_a_checkpoint_manager(tmp_path, dims):
    stub = _trainer_stub(tmp_path, dims)
    stub.checkpoints = None
    assert stub._promote_best(5, {"eval_alive_frac": 0.5})
    assert not stub._promote_best(6, {"eval_alive_frac": 0.4})
    assert stub.best_update == 5


def test_h0_warm_start_keeps_critic_and_noise_fresh(tmp_path, dims):
    """§23.6 item 6: actor/encoder reuse must not silently reuse everything."""
    source = _payload(dims, 12)
    path = save_checkpoint(tmp_path / "base.pt", source)

    stub = _trainer_stub(tmp_path, dims)
    with torch.no_grad():
        for parameter in stub.policy.parameters():
            parameter.add_(1.0)

    info = stub.load(path, resume=False, parts=("actor", "encoder"))
    assert info["init_from"] == {"path": str(path), "parts": ["actor", "encoder"]}
    current = stub.policy.state_dict()
    for key in ("actor.0.weight", "encoder.0.weight"):
        assert torch.equal(current[key], source["policy"][key]), key
    assert not torch.equal(current["critic.0.weight"], source["policy"]["critic.0.weight"])
    assert not torch.equal(current["std"], source["policy"]["std"])
    assert stub.update_count == 0 and stub.history == []


def test_resumed_run_keeps_the_restored_best(tmp_path, dims):
    """A resume into a fresh run directory must not promote a worse policy."""
    old_dir = tmp_path / "old"
    old_dir.mkdir()
    payload = _payload(dims, 40)
    payload["best_update"] = 40
    payload["best_metrics"] = {"eval_alive_frac": 1.0, "eval_height_mae": 0.02}
    save_checkpoint(old_dir / "model_last.pt", payload)
    save_checkpoint(old_dir / "model_best.pt", payload)

    new_dir = tmp_path / "new"
    new_dir.mkdir()
    stub = _trainer_stub(new_dir, dims)
    stub.load(old_dir / "model_last.pt", resume=True)
    assert stub.best_metrics == pytest.approx(
        {"eval_alive_frac": 1.0, "eval_height_mae": 0.02}
    )
    assert (new_dir / "model_best.pt").is_file(), "the restored best must travel with the ranking"
    stub.update_count = 41
    assert not stub._promote_best(
        41, {"eval_alive_frac": 1.0, "eval_height_mae": 0.50}
    )
    assert stub.best_update == 40


def test_init_parts_rejects_a_name_that_matches_nothing(tmp_path, dims):
    path = save_checkpoint(tmp_path / "base.pt", _payload(dims, 3))
    stub = _trainer_stub(tmp_path, dims)
    with pytest.raises(ValueError, match="matched no tensors"):
        stub.load(path, resume=False, parts=("bogus",))
