"""Independent LQR-teacher evaluation (doc section 11.6).

The teacher is judged with the same evaluator as a policy (``stand_eval``), so
its survival numbers are directly comparable with the PPO runs.  On top of
that it records what only a teacher can be blamed for:

* which reduced-model parameters it was designed with and their provenance,
* whether the teacher kept requesting torques outside the motor limit,
* an explicit ``usable_for_demonstrations`` flag: a teacher that fails must not
  generate demonstrations, and the report says so instead of leaving the
  reader to infer it.
"""

from __future__ import annotations

from typing import Any

from ..config import Config
from .stand_eval import make_scenarios, run_stand_eval


def evaluate_teacher(
    config: Config,
    teacher,
    *,
    steps: int = 6000,
    episodes: int = 64,
    seed_base: int = 1000,
    gains: dict[str, float] | None = None,
) -> dict[str, Any]:
    scenarios = make_scenarios(base_seed=seed_base, count=episodes)
    # ``bind`` hands the teacher the simulator state; the evaluator otherwise
    # treats it like any other policy, so the numbers stay comparable.
    report = run_stand_eval(config, teacher, steps=steps, scenarios=scenarios)

    saturation = getattr(teacher, "saturation", None)
    report["stage"] = "LQR-teacher"
    report["teacher"] = {
        "class": type(teacher).__name__,
        "parameters": teacher.params.as_dict() if getattr(teacher, "params", None) else None,
        "wheel_gain": None if teacher.k_wheel is None else teacher.k_wheel.tolist(),
        "gains": dict(teacher.gains),
        "gain_overrides": gains,
        "saturation_last_sample": None if saturation is None else [bool(v) for v in saturation],
        "gate": "section 11.6: 64 scenarios, >= 61 continuous 60 s successes",
        "usable_for_demonstrations": False,
    }
    teacher_ok = bool(report.get("criteria", {}).get("passed"))
    report["teacher"]["usable_for_demonstrations"] = teacher_ok
    report["status"] = "passed" if teacher_ok else "failed"
    if not teacher_ok:
        report["note"] = (
            "the teacher did not pass; demonstrations must NOT be collected from it "
            "(section 11.6) - fix the model/mapping/gains first"
        )
    return report
