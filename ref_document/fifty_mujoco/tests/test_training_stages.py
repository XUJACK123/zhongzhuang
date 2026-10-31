"""``configs/training_stages.yaml`` contract.

The file exists to stop three specific regressions:

* a documented default that no longer matches the code (progress.md §18.8),
* a curriculum stage whose prerequisite or gate does not exist,
* a randomisation entry that claims to be implemented when the environment does
  not have it (`action_delay` on the GPU path, for instance).
"""

from __future__ import annotations

import copy

import pytest
import yaml

from fifty_mujoco.cli import build_parser
from fifty_mujoco.learning.gpu_ppo import GpuPpoConfig
from fifty_mujoco.learning.stages import (
    BASELINE_FLAGS,
    TrainingStagesError,
    load_training_stages,
    parse_training_stages,
    planned_commands,
    resolve_train_gpu_args,
)
from fifty_mujoco.paths import configs_dir

STAGES_PATH = configs_dir() / "training_stages.yaml"


@pytest.fixture(scope="module")
def stages():
    return load_training_stages()


def _raw() -> dict:
    with STAGES_PATH.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _subparser_defaults(flag: str):
    """The argparse default for one ``train-gpu`` flag."""
    parser = build_parser()
    for action in parser._actions:
        if not getattr(action, "choices", None):
            continue
        for name, subparser in action.choices.items():
            if name != "train-gpu":
                continue
            for sub_action in subparser._actions:
                if flag in sub_action.option_strings:
                    return sub_action.default
    raise AssertionError(f"{flag} is not a train-gpu option")


def test_every_declared_default_matches_the_code(stages):
    for key, entry in stages.baseline.items():
        implemented_by = entry.implemented_by
        if implemented_by.startswith("--"):
            actual = _subparser_defaults(implemented_by)
        elif implemented_by.startswith("GpuPpoConfig."):
            field_name = implemented_by.split(".", 1)[1]
            actual = getattr(GpuPpoConfig, field_name, "missing")
            assert actual != "missing", f"{key}: GpuPpoConfig has no field {field_name}"
        else:  # pragma: no cover - parser rejects this earlier
            raise AssertionError(f"{key}: unrecognised implemented_by {implemented_by!r}")
        if isinstance(entry.default, list):
            assert list(actual) == pytest.approx(entry.default), key
        else:
            assert actual == pytest.approx(entry.default), key


def test_the_baseline_flags_are_real_flags(stages):
    for entry in stages.baseline.values():
        if entry.implemented_by.startswith("--"):
            assert entry.implemented_by in BASELINE_FLAGS
            _subparser_defaults(entry.implemented_by)  # raises when the flag is gone


def test_resolved_arguments_parse_with_the_real_parser(stages):
    args = resolve_train_gpu_args(stages, seed=1)
    assert args[0] == "train-gpu"
    parsed = build_parser().parse_args(args)
    assert parsed.command == "train-gpu"
    assert parsed.num_envs == 8192
    assert parsed.steps == 32
    assert parsed.epochs == 3
    assert parsed.minibatches == 2
    assert parsed.updates == 1500
    assert parsed.seed == 1
    assert parsed.stop_on_survive == 0
    assert parsed.stand_command is True
    assert parsed.eval_steps == 600


def test_the_baseline_never_relies_on_a_short_window_to_declare_success(stages):
    """§23.6 item 5: a 6 s window saturates by construction."""
    entry = stages.baseline["stop_on_survive"]
    assert entry.value == 0
    assert entry.default == 0


def test_the_standing_baseline_actually_fixes_the_command(stages):
    from fifty_mujoco.envs.warp_env import WarpEnvOptions, resolve_fixed_command

    entry = stages.baseline["stand_command"]
    assert entry.value is True and entry.default is False
    command = resolve_fixed_command(WarpEnvOptions(stand_command=True), nominal_height=0.464751)
    assert command == pytest.approx((0.0, 0.0, 0.464751))
    # an explicit fixed command still wins, and "neither" keeps resampling
    assert resolve_fixed_command(
        WarpEnvOptions(stand_command=True, fixed_command=(0.1, 0.2, 0.3)), 0.5
    ) == (0.1, 0.2, 0.3)
    assert resolve_fixed_command(WarpEnvOptions(), 0.5) is None


def test_a_short_evaluation_window_never_declares_solved():
    """§23.6 item 5: the 6 s smoke window must not report ``solved``."""
    from fifty_mujoco.learning.gpu_ppo import SOLVED_MIN_WINDOW_S, survival_streak_step

    assert SOLVED_MIN_WINDOW_S >= 60.0
    # 600 steps x 10 ms = 6 s, median saturated: still not solved, streak reset
    streak = 0
    for _ in range(5):
        streak, verdict = survival_streak_step(
            streak=streak,
            t_fall_median=6.0,
            eval_steps=600,
            eval_dt=0.01,
            required=2,
        )
        assert verdict is None
        assert streak == 0
    # a 60 s window that saturates twice does satisfy the *training* gate
    streak = 0
    streak, verdict = survival_streak_step(
        streak=streak, t_fall_median=60.0, eval_steps=6000, eval_dt=0.01, required=2
    )
    assert verdict is None and streak == 1
    streak, verdict = survival_streak_step(
        streak=streak, t_fall_median=60.0, eval_steps=6000, eval_dt=0.01, required=2
    )
    assert verdict == "solved"
    # a fall resets the streak, and a disabled gate never fires
    streak, verdict = survival_streak_step(
        streak=1, t_fall_median=12.0, eval_steps=6000, eval_dt=0.01, required=2
    )
    assert (streak, verdict) == (0, None)
    assert survival_streak_step(
        streak=9, t_fall_median=60.0, eval_steps=6000, eval_dt=0.01, required=0
    ) == (9, None)


def test_planned_commands_cover_every_baseline_seed(stages):
    commands = planned_commands(stages)
    seeds = stages.baseline["seeds"].value
    assert len(commands) == len(seeds)
    for command, seed in zip(commands, seeds, strict=True):
        assert command.startswith("fifty-mujoco train-gpu ")
        assert f"--seed {seed}" in command


def test_stages_are_ordered_and_referenced_correctly(stages):
    ids = [stage.id for stage in stages.stages]
    assert ids == sorted(ids)
    for stage in stages.stages:
        if stage.prerequisite is not None:
            assert stage.prerequisite in ids
            assert stage.prerequisite < stage.id
        if stage.gate is not None:
            assert stage.gate in stages.gates


def test_standing_fraction_drops_but_never_to_zero_after_stage_one(stages):
    for stage in stages.stages:
        if stage.keep_standing_fraction is None or stage.name in {"height_changes", "perturbations_and_terrain"}:
            continue
        assert 0.0 < stage.keep_standing_fraction <= 1.0


def test_unimplemented_randomisation_always_says_why(stages):
    for key, entry in stages.randomization.items():
        if entry.implemented:
            assert entry.where, key
        else:
            assert entry.blocked_by, key


def test_gpu_action_delay_is_recorded_as_unimplemented(stages):
    entry = stages.randomization["action_delay"]
    assert entry.implemented is False
    assert "NotImplementedError" in (entry.blocked_by or "")


def test_gates_never_duplicate_the_threshold_numbers(stages):
    for name, gate in stages.gates.items():
        for key, value in gate.items():
            if key == "planned_targets":
                continue
            assert not isinstance(value, (int, float)) or isinstance(value, bool), (
                f"gates.{name}.{key} carries a literal number"
            )


def test_a_mistyped_prerequisite_is_rejected():
    data = copy.deepcopy(_raw())
    data["stages"][1]["prerequisite"] = 99
    with pytest.raises(TrainingStagesError, match="does not exist"):
        parse_training_stages(data, STAGES_PATH)


def test_an_unimplemented_randomisation_entry_without_a_reason_is_rejected():
    data = copy.deepcopy(_raw())
    data["randomization"]["mass_and_payload"].pop("blocked_by")
    with pytest.raises(TrainingStagesError, match="blocked_by"):
        parse_training_stages(data, STAGES_PATH)


def test_a_gate_with_a_hard_coded_number_is_rejected():
    data = copy.deepcopy(_raw())
    data["gates"]["stand"]["success_rate_min"] = 0.95
    with pytest.raises(TrainingStagesError):
        parse_training_stages(data, STAGES_PATH)
