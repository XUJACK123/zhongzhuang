"""Versioned, atomic checkpoints (doc section 5.5).

Why this module exists
----------------------
The 3-hour run that produced ``stand_3h`` overwrote a single ``model_last.pt``
on every save, so the good policy around update 900 could not be recovered
after training collapsed.  A checkpoint therefore has to be

* **numbered** — ``model_update_000900.pt`` can never be overwritten,
* **complete** — network, optimiser, learning-rate state, exploration std,
  update counter, RNG state and the model/config fingerprint,
* **atomic** — written to a temporary file and moved into place, so a kill in
  the middle of a save cannot destroy the only good weights,
* **typed** — ``resume`` (same experiment) and ``init`` (weights only, fresh
  optimiser) are separate operations and never share a filename.

``history.json`` keeps the metric log; ``model_best.pt`` is chosen by an
ordered metric list (safety first, then survival quantiles, then tracking
error), and every released best is copied to an immutable ``release`` path
instead of being re-written later.
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Collection, Iterable, Sequence

import torch

#: Bumped when the payload layout changes in a way ``resume`` cannot bridge.
CHECKPOINT_FORMAT = 2


def sha256_file(path: str | Path, chunk: int = 1 << 20) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        while True:
            block = handle.read(chunk)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def model_fingerprint(extra_paths: Iterable[str | Path] = ()) -> dict[str, Any]:
    """Hashes of everything a policy's behaviour depends on.

    The MJCF variants and the YAML configs are hashed when present; missing
    files are recorded as ``null`` rather than silently skipped, so a report
    can tell "not built yet" from "identical".
    """
    from ..paths import assets_dir, configs_dir

    files: list[Path] = []
    files.extend(sorted(configs_dir().glob("*.yaml")))
    for variant in ("fifty_train.xml", "fifty_visual.xml"):
        files.append(assets_dir() / "mjcf" / variant)
    files.extend(Path(path) for path in extra_paths)
    return {
        str(path): (sha256_file(path) if path.is_file() else None) for path in files
    }


def save_checkpoint(path: str | Path, payload: dict[str, Any]) -> Path:
    """Write ``payload`` atomically: temp file first, then ``os.replace``."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    torch.save(payload, tmp)
    os.replace(tmp, path)
    return path


def load_checkpoint(path: str | Path, map_location: Any = "cpu") -> dict[str, Any]:
    """Load a checkpoint and reject payloads from a newer format."""
    payload = torch.load(Path(path), map_location=map_location, weights_only=False)
    fmt = int(payload.get("format", 0))
    if fmt > CHECKPOINT_FORMAT:
        raise ValueError(
            f"{path}: checkpoint format {fmt} is newer than this code ({CHECKPOINT_FORMAT})"
        )
    return payload


def rng_state() -> dict[str, Any]:
    state: dict[str, Any] = {
        "torch": torch.get_rng_state(),
        "numpy": None,
    }
    try:
        import numpy as np

        state["numpy"] = np.random.get_state()
    except Exception:  # pragma: no cover - numpy is a hard dependency in practice
        pass
    if torch.cuda.is_available():
        state["cuda"] = torch.cuda.get_rng_state_all()
    return state


def restore_rng_state(state: dict[str, Any] | None) -> None:
    """Best-effort RNG restore; a missing entry is not an error.

    A resumed run continues from the saved sampler state, but the simulator
    state itself is *not* restored, so the trajectory is not bit-identical and
    no report may claim step-by-step reproducibility.
    """
    if not state:
        return
    if state.get("torch") is not None:
        torch.set_rng_state(state["torch"])
    if state.get("numpy") is not None:
        import numpy as np

        np.random.set_state(state["numpy"])
    if state.get("cuda") is not None and torch.cuda.is_available():
        torch.cuda.set_rng_state_all(state["cuda"])


#: Metrics where smaller is better.  A tracking *error* is not a score: without
#: this set ``model_best.pt`` was chosen as if a 0.30 m height error beat a
#: 0.02 m one, because the comparison was uniformly "larger wins".
LOWER_IS_BETTER: frozenset[str] = frozenset(
    {"eval_height_mae", "eval_vx_mae", "eval_tilt_max_deg"}
)


def metric_is_better(
    new: dict[str, float],
    old: dict[str, float] | None,
    keys: Sequence[str],
    lower_is_better: Collection[str] = LOWER_IS_BETTER,
) -> bool:
    """Lexicographic comparison over ``keys`` (NaN last, direction-aware).

    The order encodes the acceptance rules: a run that falls over must never
    win on a tracking-error tie-break, and a metric that is absent is worse
    than any present value.  Keys listed in ``lower_is_better`` compare the
    other way round; everything else keeps "larger is better".
    """
    if old is None:
        return True
    for key in keys:
        new_value = new.get(key)
        old_value = old.get(key)
        if new_value is None and old_value is None:
            continue
        if new_value is None:
            return False
        if old_value is None:
            return True
        new_finite = new_value == new_value
        old_finite = old_value == old_value
        if new_finite != old_finite:
            return new_finite
        if new_finite and new_value != old_value:
            if key in lower_is_better:
                return new_value < old_value
            return new_value > old_value
    return False


#: Ordered default: stay upright first, then survive longer, then track better.
DEFAULT_BEST_ORDER: tuple[str, ...] = (
    "eval_alive_frac",
    "eval_t_fall_p10",
    "eval_t_fall_median",
    "eval_height_mae",
    "eval_vx_mae",
)


@dataclass
class CheckpointManager:
    """Numbered snapshots, ``last``/``best`` and retention for one run."""

    run_dir: Path
    keep: int = 5
    best_order: Sequence[str] = field(default_factory=lambda: DEFAULT_BEST_ORDER)
    best_metrics: dict[str, float] | None = None
    best_update: int = 0

    def __post_init__(self) -> None:
        self.run_dir = Path(self.run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.index_path = self.run_dir / "checkpoints.json"
        self._index: dict[str, Any] = self._read_index()
        # A new process must continue the *same* best ranking: without this a
        # resumed run treats its first evaluation as the first ever seen and
        # overwrites a better ``model_best.pt`` with a worse policy.
        if self.best_metrics is None and isinstance(self._index.get("best_metrics"), dict):
            self.best_metrics = {
                key: float(value) for key, value in self._index["best_metrics"].items()
            }
        if not self.best_update and self._index.get("best_update"):
            self.best_update = int(self._index["best_update"])

    # ------------------------------------------------------------------ index
    def _read_index(self) -> dict[str, Any]:
        if self.index_path.is_file():
            try:
                return json.loads(self.index_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                return {}
        return {}

    def _write_index(self) -> None:
        payload = {
            "best_update": self.best_update,
            "best_metrics": self.best_metrics,
            "best_order": list(self.best_order),
            "snapshots": sorted(path.name for path in self.run_dir.glob("model_update_*.pt")),
        }
        tmp = self.index_path.with_name(self.index_path.name + ".tmp")
        tmp.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        os.replace(tmp, self.index_path)

    def numbered_path(self, update: int) -> Path:
        return self.run_dir / f"model_update_{int(update):06d}.pt"

    def _free_numbered_path(self, update: int) -> Path:
        """A numbered snapshot is immutable: re-reaching an update gets ``_r2``."""
        path = self.numbered_path(update)
        if not path.exists():
            return path
        revision = 2
        while True:
            candidate = self.run_dir / f"model_update_{int(update):06d}_r{revision}.pt"
            if not candidate.exists():
                return candidate
            revision += 1

    @property
    def last_path(self) -> Path:
        return self.run_dir / "model_last.pt"

    @property
    def best_path(self) -> Path:
        return self.run_dir / "model_best.pt"

    # ------------------------------------------------------------------- save
    def save_numbered(self, update: int, payload: dict[str, Any]) -> Path:
        path = save_checkpoint(self._free_numbered_path(update), payload)
        self._prune()
        self._write_index()
        return path

    def save_last(self, update: int, payload: dict[str, Any]) -> Path:
        path = save_checkpoint(self.last_path, payload)
        self._write_index()
        return path

    def maybe_save_best(self, update: int, payload: dict[str, Any], metrics: dict[str, float]) -> bool:
        """Promote ``payload`` to ``model_best.pt`` if it wins the ordering."""
        if not metrics:
            return False
        if not metric_is_better(metrics, self.best_metrics, self.best_order):
            return False
        self.best_metrics = {key: float(value) for key, value in metrics.items() if value is not None}
        self.best_update = int(update)
        save_checkpoint(self.best_path, payload)
        self._write_index()
        return True

    def adopt_best(self, source: str | Path) -> Path:
        """Take an existing ``model_best.pt`` as this run's starting best.

        Used when resuming into a fresh run directory: the ranking is restored
        from the checkpoint, so the file it refers to has to come along -
        otherwise the new directory either has no best at all or promotes the
        first (possibly worse) evaluation over the restored one.
        """
        source_path = Path(source)
        if not self.best_path.is_file():
            if not source_path.is_file():
                raise FileNotFoundError(f"no best checkpoint at {source_path}")
            shutil.copy2(source_path, self.best_path)
            self._write_index()
        return self.best_path

    def release(self, version: str) -> Path:
        """Copy the accepted best into an immutable release directory."""
        if not self.best_path.is_file():
            raise FileNotFoundError("no model_best.pt to release")
        target = self.run_dir / "release" / version
        target.mkdir(parents=True, exist_ok=True)
        destination = target / "model.pt"
        if destination.exists():
            raise FileExistsError(f"{destination} already exists; releases are immutable")
        shutil.copy2(self.best_path, destination)
        if self.best_metrics is not None:
            (target / "metrics.json").write_text(
                json.dumps({"update": self.best_update, "metrics": self.best_metrics}, indent=2),
                encoding="utf-8",
            )
        return destination

    # ---------------------------------------------------------------- retention
    def _prune(self) -> None:
        if self.keep <= 0:
            return
        numbered = sorted(self.run_dir.glob("model_update_*.pt"))
        for path in numbered[: max(0, len(numbered) - self.keep)]:
            path.unlink(missing_ok=True)
