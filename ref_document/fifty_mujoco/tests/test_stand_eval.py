"""Standing evaluator contract (doc sections 5.4 and 14.2).

The point of these tests is not that a policy stands, but that the report
cannot be inflated: an episode that ends is counted as a failure, the survival
statistics come from the first failure, and the per-episode data agrees with
the aggregate.
"""

from __future__ import annotations

import numpy as np
import pytest

from fifty_mujoco.evaluation.stand_eval import (
    THRESHOLDS,
    make_scenarios,
    run_stand_eval,
    zero_policy,
)
def test_scenarios_are_independent_and_reproducible():
    first = make_scenarios(base_seed=1000, count=8)
    second = make_scenarios(base_seed=1000, count=8)
    assert [scenario.seed for scenario in first] == list(range(1000, 1008))
    assert [scenario.as_dict() for scenario in first] == [scenario.as_dict() for scenario in second]
    assert len({scenario.as_dict().__repr__() for scenario in first}) > 1
    for scenario in first:
        assert abs(scenario.pitch_deg) <= 2.0
        assert abs(scenario.roll_deg) <= 2.0
        assert abs(scenario.leg_delta_rad) <= 0.02
        assert abs(scenario.vx_m_s) <= 0.05


def test_episode_limit_sits_above_the_window(config, built_assets):
    del built_assets
    report = run_stand_eval(
        config,
        zero_policy,
        steps=20,
        scenarios=make_scenarios(base_seed=1000, count=2),
    )
    assert report["window_s"] == pytest.approx(20 * report["policy_dt_s"])
    assert report["episode_limit_s"] > report["window_s"]
    assert report["steps"] == 20


def test_uncontrolled_robot_fails_and_reports_why(config, built_assets):
    del built_assets
    report = run_stand_eval(
        config,
        zero_policy,
        steps=60,
        scenarios=make_scenarios(base_seed=1000, count=2),
    )
    assert report["successes"] == 0
    assert report["status"] == "failed"
    # every episode must carry a termination reason, and none may be "survived"
    for episode in report["episodes_detail"]:
        assert episode["reason"] in {"tilt", "height", "illegal_contact", "nonfinite", "time_out"}
        assert episode["success"] is False
        assert 0.0 < episode["survived_s"] <= report["window_s"]
    assert report["criteria"]["checks"]["success_rate"]["passed"] is False
    assert report["criteria"]["passed"] is False


def test_aggregate_matches_per_episode_data(config, built_assets):
    del built_assets
    report = run_stand_eval(
        config,
        zero_policy,
        steps=40,
        scenarios=make_scenarios(base_seed=1010, count=3),
    )
    survival = np.array([episode["survived_s"] for episode in report["episodes_detail"]])
    assert report["survival_median_s"] == pytest.approx(float(np.median(survival)))
    assert report["survival_min_s"] == pytest.approx(float(survival.min()))
    assert report["successes"] == int(sum(episode["success"] for episode in report["episodes_detail"]))
    assert report["success_rate"] == pytest.approx(report["successes"] / report["episodes"])
    assert report["episode_limit_s"] > report["window_s"]
    assert set(report["criteria"]["checks"]) == {
        "success_rate",
        "vx_mae",
        "height_mae",
        "tilt_p95",
        "tilt_max",
        "torque_saturation",
    }
    assert report["criteria"]["checks"]["success_rate"]["threshold"] == THRESHOLDS["success_rate_min"]


def test_policy_shape_is_validated(config, built_assets):
    del built_assets

    def bad_policy(obs, history):
        return np.zeros((obs.shape[0], 3))

    with pytest.raises(ValueError):
        run_stand_eval(config, bad_policy, steps=2, scenarios=make_scenarios(count=1))
