"""Reproducible get-up curriculum and development-only promotion rules."""

import hashlib
import json
import math
import random

SCHEMA = "hardware_getup_curriculum_v1"


def plan_digest(plan):
    return hashlib.sha256(json.dumps(plan, sort_keys=True, allow_nan=False).encode()).hexdigest()


class Curriculum:
    def __init__(self, plan, state=None):
        self.plan = plan
        self.digest = plan_digest(plan)
        self.stages = plan["getup"]["curriculum"]
        self.index = 0
        self.consecutive = 0
        self.evaluations = {}
        if state is not None:
            if state["schema"] != SCHEMA or state["plan_sha256"] != self.digest:
                raise ValueError("curriculum resume plan mismatch")
            self.index = state["stage_index"]
            self.consecutive = state["consecutive_passes"]
            self.evaluations = dict(state["evaluations"])
            if type(self.index) is not int or not 0 <= self.index < len(self.stages):
                raise ValueError("invalid curriculum stage index")
            if type(self.consecutive) is not int or self.consecutive < 0:
                raise ValueError("invalid consecutive evaluation count")

    @property
    def stage(self):
        return self.stages[self.index]["name"]

    def state_dict(self):
        return {"schema": SCHEMA, "plan_sha256": self.digest, "stage_index": self.index,
                "stage": self.stage, "consecutive_passes": self.consecutive,
                "evaluations": dict(self.evaluations)}

    def record(self, report):
        """Repeated identical reports are idempotent, heldout reports are rejected."""
        identity = report["evaluation_id"]
        digest = plan_digest(report)
        if not isinstance(identity, str) or not identity:
            raise ValueError("evaluation id required")
        if identity in self.evaluations:
            if self.evaluations[identity] != digest:
                raise ValueError("evaluation id reused with different results")
            return False
        if report["stage"] != self.stage or report["split"] != "development":
            raise ValueError("promotion requires the current stage development split")
        low, high = self.plan["evaluation"]["seed_ranges_inclusive"]["development"]
        seeds = report["seeds"]
        if not seeds or any(type(s) is not int or not low <= s <= high for s in seeds) or len(set(seeds)) != len(seeds):
            raise ValueError("unique development seeds required")
        wins = report["successes"]
        failures = report["hard_limit_failures"]
        if type(wins) is not int or not 0 <= wins <= len(seeds) or type(failures) is not int or not 0 <= failures <= len(seeds):
            raise ValueError("invalid evaluation counts")
        if type(report["nominal_balance_regression_passed"]) is not bool:
            raise ValueError("balance regression flag required")
        minimum = self.plan["getup"]["curriculum_promotion"]["success_rate_min"]
        complete = len(seeds) == high - low + 1
        passed = complete and wins / len(seeds) >= minimum and failures == 0 and report["nominal_balance_regression_passed"]
        self.evaluations[identity] = digest
        self.consecutive = self.consecutive + 1 if passed else 0
        needed = self.plan["getup"]["curriculum_promotion"]["consecutive_development_evaluations"]
        promoted = self.consecutive >= needed and self.index < len(self.stages) - 1
        if promoted:
            self.index += 1
            self.consecutive = 0
        return promoted

    def episode_parameters(self, seed, split="train"):
        if split not in self.plan["evaluation"]["seed_ranges_inclusive"]:
            raise ValueError("unknown seed split")
        low, high = self.plan["evaluation"]["seed_ranges_inclusive"][split]
        if type(seed) is not int or not low <= seed <= high:
            raise ValueError("seed outside reserved range")
        rng = random.Random(seed)
        domain = self.plan["randomization"]
        nominal = split == "heldout_nominal" or self.stage != "joint_randomization" or rng.random() < domain["retain_nominal_episode_fraction"]
        gains, friction, spring = [1.] * 6, .75, 1.
        if not nominal:
            for index in (0, 1, 3, 4):
                gains[index] = rng.uniform(*domain["dm_gain_independent_per_leg"])
            friction = rng.uniform(*domain["ground_friction"])
            spring = rng.uniform(*domain["spring_constant_force_scale"])
        settle = rng.uniform(*self.plan["getup"]["settle_zero_torque_s"])
        if self.stage in ("near_stand", "supported_survey"):
            settle = 0.
        ramp = rng.uniform(*self.plan["getup"]["enable_ramp_s"])
        return {"seed": seed, "stage": self.stage, "nominal": nominal,
                "motor_gain": gains, "effective_friction": friction, "spring_scale": spring,
                "settle_zero_torque_s": settle, "enable_ramp_s": ramp,
                "reset_specification": self.stages[self.index],
                "requires_dynamic_reset_validation": True}


def wilson_interval(wins, count):
    if count == 0:
        return None
    z = 1.959963984540054
    rate = wins / count
    denominator = 1 + z * z / count
    center = (rate + z * z / (2 * count)) / denominator
    radius = z * math.sqrt(rate * (1 - rate) / count + z * z / (4 * count * count)) / denominator
    return [max(0., center - radius), min(1., center + radius)]


def resumed_curriculum(plan, saved, requested=None):
    previous = Curriculum(plan, saved)
    if requested is None:
        return previous
    current = Curriculum(plan, requested)
    if not previous.index <= current.index <= previous.index + 1:
        raise ValueError("resume curriculum must retain its stage or advance one stage")
    if any(current.evaluations.get(key) != value for key, value in previous.evaluations.items()):
        raise ValueError("resume curriculum discarded or changed evaluation history")
    return current


def acceptance(rows, plan, split):
    if split not in ("heldout_nominal", "heldout_combined"):
        raise ValueError("heldout split required")
    low, high = plan["evaluation"]["seed_ranges_inclusive"][split]
    if len({row["seed"] for row in rows}) != len(rows):
        raise ValueError("duplicate heldout seeds")
    if any(type(row["seed"]) is not int or not low <= row["seed"] <= high for row in rows):
        raise ValueError("heldout seed outside frozen split")
    groups = {}
    for row in rows:
        if type(row["success"]) is not bool or type(row["hard_limit_failure"]) is not bool:
            raise ValueError("trial outcome flags must be boolean")
        groups.setdefault(row["reset_class"], []).append(row)
    threshold = plan["evaluation"]["nominal_getup_success_rate_min" if split == "heldout_nominal" else "combined_getup_success_rate_min"]
    required_classes = set(plan["getup"]["final_reset_classes"])
    expected = high - low + 1
    class_results = {}
    for name, group in groups.items():
        wins = sum(r["success"] for r in group)
        class_results[name] = {"trials": len(group), "successes": wins, "rate": wins / len(group),
                               "wilson_95": wilson_interval(wins, len(group))}
    wins = sum(r["success"] for r in rows)
    rate = wins / len(rows) if rows else 0.
    hard_failures = sum(r["hard_limit_failure"] for r in rows)
    class_minimum = plan["evaluation"]["per_reset_class_success_rate_min"]
    complete = len(rows) == expected and required_classes <= set(groups)
    passed = complete and rate >= threshold and hard_failures == 0 and all(
        group["rate"] >= class_minimum for group in class_results.values())
    return {"split": split, "trials": len(rows), "expected_trials": expected,
            "complete": complete, "successes": wins, "rate": rate,
            "hard_limit_failures": hard_failures, "wilson_95": wilson_interval(wins, len(rows)),
            "classes": class_results, "passed": passed}
