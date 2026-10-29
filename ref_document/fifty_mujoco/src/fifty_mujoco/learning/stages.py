"""Load ``configs/training_stages.yaml`` and turn it into a real command line.

The point of this module is not to describe a curriculum - the YAML does that -
but to make the description falsifiable:

* ``load_training_stages`` rejects a stage whose prerequisite does not exist, a
  randomisation entry that claims to be implemented without saying where, and a
  gate that carries a literal number instead of a reference to
  ``stand_eval.THRESHOLDS``.
* ``resolve_train_gpu_args`` produces the ``train-gpu`` argument list, and the
  tests parse it with the real parser, so a flag that stops existing breaks the
  build instead of the next training run.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping

import yaml

from ..paths import configs_dir


class TrainingStagesError(ValueError):
    """The stage file contradicts itself or the code."""


@dataclass(frozen=True)
class BaselineEntry:
    key: str
    value: Any
    default: Any
    implemented_by: str
    note: str | None = None


@dataclass(frozen=True)
class Stage:
    id: int
    name: str
    task: Mapping[str, Any]
    keep_standing_fraction: float | None
    prerequisite: int | None
    gate: str | None
    blocked_by: str | None = None
    note: str | None = None


@dataclass(frozen=True)
class RandomizationEntry:
    key: str
    implemented: bool
    where: str | None
    blocked_by: str | None
    cli_exposed: bool | None = None
    note: str | None = None


@dataclass(frozen=True)
class TrainingStages:
    path: Path
    meta: Mapping[str, Any]
    baseline: Mapping[str, BaselineEntry]
    stages: tuple[Stage, ...]
    randomization: Mapping[str, RandomizationEntry]
    gates: Mapping[str, Mapping[str, Any]] = field(default_factory=dict)

    def stage(self, stage_id: int) -> Stage:
        for entry in self.stages:
            if entry.id == stage_id:
                return entry
        raise TrainingStagesError(f"no stage with id {stage_id}")

    def implemented_randomization(self) -> list[str]:
        return [key for key, entry in self.randomization.items() if entry.implemented]

    def blocked_randomization(self) -> list[str]:
        return [key for key, entry in self.randomization.items() if not entry.implemented]


#: Training flags that the stage file is allowed to drive. Keeping the list
#: explicit means a typo in ``implemented_by`` is caught here, not by argparse.
BASELINE_FLAGS: frozenset[str] = frozenset(
    {
        "--num-envs",
        "--steps",
        "--epochs",
        "--minibatches",
        "--lr",
        "--entropy-coef",
        "--max-std",
        "--eval-every",
        "--eval-steps",
        "--save-every",
        "--checkpoint-keep",
        "--stop-on-survive",
        "--updates",
        "--seed",
    }
)

#: Keys a gate entry may carry besides the reference to the threshold table.
GATE_KEYS: frozenset[str] = frozenset(
    {"source", "keys", "implemented", "blocked_by", "planned_targets", "command", "note"}
)


def _require(mapping: Mapping[str, Any], key: str, where: str) -> Any:
    if key not in mapping:
        raise TrainingStagesError(f"{where}: missing {key!r}")
    return mapping[key]


def _check_number_free(entry: Mapping[str, Any], where: str) -> None:
    for key, value in entry.items():
        if key not in GATE_KEYS:
            raise TrainingStagesError(f"{where}: unknown key {key!r}")
        if key == "planned_targets":
            continue
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            raise TrainingStagesError(
                f"{where}.{key}: gates reference thresholds by name; the number lives in"
                " stand_eval.THRESHOLDS, not here"
            )


def parse_training_stages(data: Mapping[str, Any], path: Path) -> TrainingStages:
    meta = data.get("meta") or {}
    if not isinstance(meta, Mapping):
        raise TrainingStagesError("meta: expected a mapping")

    raw_baseline = data.get("baseline")
    if not isinstance(raw_baseline, Mapping) or not raw_baseline:
        raise TrainingStagesError("baseline: expected a non-empty mapping")
    baseline: dict[str, BaselineEntry] = {}
    for key, raw in raw_baseline.items():
        where = f"baseline.{key}"
        if not isinstance(raw, Mapping):
            raise TrainingStagesError(f"{where}: expected a mapping")
        entry = BaselineEntry(
            key=key,
            value=_require(raw, "value", where),
            default=_require(raw, "default", where),
            implemented_by=str(_require(raw, "implemented_by", where)),
            note=str(raw["note"]) if raw.get("note") else None,
        )
        if entry.implemented_by.startswith("--") and entry.implemented_by not in BASELINE_FLAGS:
            raise TrainingStagesError(
                f"{where}: {entry.implemented_by!r} is not a known train-gpu flag"
            )
        baseline[key] = entry

    raw_stages = data.get("stages")
    if not isinstance(raw_stages, list) or not raw_stages:
        raise TrainingStagesError("stages: expected a non-empty list")
    stages: list[Stage] = []
    for raw in raw_stages:
        if not isinstance(raw, Mapping):
            raise TrainingStagesError("stages: every entry must be a mapping")
        stage_id = int(_require(raw, "id", "stages"))
        fraction = raw.get("keep_standing_fraction")
        if fraction is not None and not 0.0 <= float(fraction) <= 1.0:
            raise TrainingStagesError(
                f"stages[{stage_id}].keep_standing_fraction must be within [0, 1]"
            )
        stages.append(
            Stage(
                id=stage_id,
                name=str(_require(raw, "name", "stages")),
                task=raw.get("task") or {},
                keep_standing_fraction=float(fraction) if fraction is not None else None,
                prerequisite=int(raw["prerequisite"]) if raw.get("prerequisite") is not None else None,
                gate=str(raw["gate"]) if raw.get("gate") else None,
                blocked_by=str(raw["blocked_by"]) if raw.get("blocked_by") else None,
                note=str(raw["note"]) if raw.get("note") else None,
            )
        )
    ids = [stage.id for stage in stages]
    if ids != sorted(ids) or len(set(ids)) != len(ids):
        raise TrainingStagesError(f"stages: ids must be unique and ascending, got {ids}")
    known = set(ids)
    for stage in stages:
        if stage.prerequisite is not None and stage.prerequisite not in known:
            raise TrainingStagesError(
                f"stage {stage.id}: prerequisite {stage.prerequisite} does not exist"
            )
        if stage.prerequisite is not None and stage.prerequisite >= stage.id:
            raise TrainingStagesError(
                f"stage {stage.id}: prerequisite must come before the stage"
            )

    raw_randomization = data.get("randomization") or {}
    if not isinstance(raw_randomization, Mapping):
        raise TrainingStagesError("randomization: expected a mapping")
    randomization: dict[str, RandomizationEntry] = {}
    for key, raw in raw_randomization.items():
        where = f"randomization.{key}"
        if not isinstance(raw, Mapping):
            raise TrainingStagesError(f"{where}: expected a mapping")
        implemented = bool(_require(raw, "implemented", where))
        where_code = raw.get("where")
        blocked = raw.get("blocked_by")
        if implemented and not where_code:
            raise TrainingStagesError(f"{where}: implemented entries must say where")
        if not implemented and not blocked:
            raise TrainingStagesError(f"{where}: unimplemented entries must say blocked_by")
        randomization[key] = RandomizationEntry(
            key=key,
            implemented=implemented,
            where=str(where_code) if where_code else None,
            blocked_by=str(blocked) if blocked else None,
            cli_exposed=raw.get("cli_exposed"),
            note=str(raw["note"]) if raw.get("note") else None,
        )

    gates: dict[str, Mapping[str, Any]] = {}
    raw_gates = data.get("gates") or {}
    if not isinstance(raw_gates, Mapping):
        raise TrainingStagesError("gates: expected a mapping")
    for key, raw in raw_gates.items():
        if not isinstance(raw, Mapping):
            raise TrainingStagesError(f"gates.{key}: expected a mapping")
        _check_number_free(raw, f"gates.{key}")
        if raw.get("implemented") is False and not raw.get("blocked_by"):
            raise TrainingStagesError(f"gates.{key}: unimplemented gates must say blocked_by")
        gates[key] = raw

    for stage in stages:
        if stage.gate is not None and stage.gate not in gates:
            raise TrainingStagesError(f"stage {stage.id}: unknown gate {stage.gate!r}")

    return TrainingStages(
        path=path,
        meta=meta,
        baseline=baseline,
        stages=tuple(stages),
        randomization=randomization,
        gates=gates,
    )


def load_training_stages(path: str | Path | None = None) -> TrainingStages:
    target = Path(path) if path is not None else configs_dir() / "training_stages.yaml"
    with target.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, Mapping):
        raise TrainingStagesError(f"{target}: expected a mapping at the top level")
    return parse_training_stages(data, target)


def resolve_train_gpu_args(
    stages: TrainingStages,
    seed: int | None = None,
    name: str | None = None,
) -> list[str]:
    """The ``train-gpu`` arguments the baseline describes.

    Only flags that exist on the parser are emitted, and only when the baseline
    value differs from the code default - so the command stays readable and the
    difference from "just run the defaults" is explicit.
    """
    args: list[str] = ["train-gpu"]
    for entry in stages.baseline.values():
        if entry.key == "seeds":
            continue  # a seed list is not a flag value; one run takes one seed
        if not entry.implemented_by.startswith("--"):
            continue
        if entry.value == entry.default:
            continue
        args.extend([entry.implemented_by, _format(entry.value)])
    seeds = stages.baseline.get("seeds")
    chosen = seed if seed is not None else (seeds.value[0] if seeds and seeds.value else 0)
    args.extend(["--seed", str(chosen)])
    if name:
        args.extend(["--name", name])
    return args


def describe(stages: TrainingStages) -> str:
    """Multi-line summary used by ``stage-plan``."""
    lines = [f"stages file    : {stages.path}", f"status         : {stages.meta.get('status')}"]
    lines.append(f"stages         : {len(stages.stages)}")
    for stage in stages.stages:
        state = "blocked" if stage.blocked_by else "planned"
        lines.append(
            f"  {stage.id}. {stage.name:<24} gate={stage.gate or '-':<16} [{state}]"
        )
        if stage.blocked_by:
            lines.append(f"     blocked_by: {stage.blocked_by}")
    implemented = stages.implemented_randomization()
    blocked = stages.blocked_randomization()
    lines.append(f"randomisation  : implemented {len(implemented)} / blocked {len(blocked)}")
    if implemented:
        lines.append(f"  implemented : {', '.join(implemented)}")
    if blocked:
        lines.append(f"  blocked     : {', '.join(blocked)}")
    return "\n".join(lines)


def planned_commands(stages: TrainingStages, seeds: Iterable[int] | None = None) -> list[str]:
    """One ``fifty-mujoco train-gpu ...`` line per seed."""
    chosen = list(seeds) if seeds is not None else list(
        (stages.baseline.get("seeds").value if stages.baseline.get("seeds") else [0])
    )
    return [
        "fifty-mujoco " + " ".join(resolve_train_gpu_args(stages, seed=seed))
        for seed in chosen
    ]


def _format(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return repr(value)
    return str(value)
