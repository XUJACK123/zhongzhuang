import copy
import json
from pathlib import Path
import sys
import pytest

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT/"tools"))
from sample_qualified_acceptance import score


@pytest.fixture
def case():
    report = json.loads((PROJECT/"reports/sample_weight/qualified_baseline_nominal_20261116.json").read_text())
    protocol = {"nominal": {"seed_base": 2000000, "seeds_per_class": 2, "duration_s": 10},
                "class_stride": 1000, "metric_thresholds_source": "configs/sample_command_evaluation.json",
                "physical_failures_max": 0, "rate_min": .95, "per_class_rate_min": .9}
    frozen = {"checkpoint_sha256": report["checkpoint_sha256"], "firmware_sha256": report["firmware_sha256"]}
    return copy.deepcopy(report), protocol, frozen


def test_rescores_real_development_report(case):
    report, protocol, frozen = case
    result = score(report, protocol, "nominal", frozen)
    assert result["passed"] and result["successes"] == 10


def test_duplicate_seed_is_incomplete(case):
    report, protocol, frozen = case
    report["rows"][0]["seed"] = report["rows"][1]["seed"]
    assert not score(report, protocol, "nominal", frozen)["complete"]


def test_seed_assigned_to_wrong_class_is_incomplete(case):
    report, protocol, frozen = case
    first = next(r for r in report["rows"] if r["scenario"] == "stand")
    second = next(r for r in report["rows"] if r["scenario"] == "forward")
    first["scenario"], second["scenario"] = second["scenario"], first["scenario"]
    assert not score(report, protocol, "nominal", frozen)["complete"]


def test_checkpoint_or_firmware_change_rejects_provenance(case):
    report, protocol, frozen = case
    frozen["checkpoint_sha256"] = "changed"
    assert not score(report, protocol, "nominal", frozen)["passed"]
    frozen["checkpoint_sha256"] = report["checkpoint_sha256"]
    frozen["firmware_sha256"] = {}
    assert not score(report, protocol, "nominal", frozen)["passed"]


def test_randomized_parameters_cannot_be_labeled_nominal(case):
    report, protocol, frozen = case
    report["rows"][0]["parameters"]["motor_gain"][0] = .8
    with pytest.raises(ValueError, match="frozen domain"):
        score(report, protocol, "nominal", frozen)


@pytest.mark.parametrize("kind", ["nonfinite_saturation", "short_episode", "physical_failure", "incorrect_pass_flag"])
def test_bad_trial_cannot_inherit_pass_flag(case, kind):
    report, protocol, frozen = case
    row = report["rows"][0]
    if kind == "nonfinite_saturation":
        row["motor_saturation_fractions"][3] = float("nan")
    elif kind == "short_episode":
        row["elapsed_s"] = .5
    elif kind == "physical_failure":
        row["completed"], row["first_failure"] = False, "joint_limit"
    else:
        row["thresholds_met"] = False
    with pytest.raises(ValueError, match="rescoring"):
        score(report, protocol, "nominal", frozen)
