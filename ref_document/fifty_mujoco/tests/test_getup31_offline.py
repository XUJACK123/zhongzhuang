"""Protocol tests use synthetic measurements and perform no physics integration."""

from dataclasses import asdict
import copy
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
from fifty_mujoco.contracts.getup31 import Decision, GetupController, Phase, Sample, SCHEMA, blended_targets, reward_terms
from fifty_mujoco.contracts.sample_actor31 import SOURCE_DEFAULT_ANGLES
from fifty_mujoco.learning.getup_curriculum import Curriculum, acceptance, plan_digest, resumed_curriculum
from fifty_mujoco.transfer.getup_trace import replay, validate_raw_telemetry

SPEC = importlib.util.spec_from_file_location("getup_train", PROJECT / "tools/sample_getup_train.py")
TRAIN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TRAIN)


def sample(t, **changes):
    values = dict(time_s=t, joint_position_rad=SOURCE_DEFAULT_ANGLES, roll_pitch_deg=(0., 0.),
                  angular_velocity_rad_s=(0., 0., 0.), forward_velocity_m_s=0., clearance_m=.1,
                  displacement_m=0., both_wheels_supported=True, body_contact=False, assistance_force_n=0.,
                  telemetry_valid=True, joint_limits_valid=True, spring_stroke_valid=True,
                  linkage_branch_valid=True, forbidden_contact=False)
    values.update(changes)
    return Sample(**values)


class GetupOfflineTests(unittest.TestCase):
    def setUp(self):
        self.plan = json.loads((PROJECT / "configs/hardware31_next_training_20261115.json").read_text())
        profile = json.loads((PROJECT / "configs/sample_hardware_profile.json").read_text())
        self.limits = copy.deepcopy(profile["firmware_target_knee_limits_rad"])
        self.limits[0][0] += .03
        self.limits[1][1] -= .03
        self.controller = GetupController(self.plan)
        self.controller.update(sample(0), False)

    def run_until(self, controller, phase, first=1):
        for tick in range(first, 2000):
            decision = controller.update(sample(tick / 100), True)
            if decision.phase == phase:
                return tick, decision
        self.fail(f"phase {phase} never reached")

    def report(self, identity, **changes):
        low, high = self.plan["evaluation"]["seed_ranges_inclusive"]["development"]
        result = dict(evaluation_id=identity, stage="near_stand", split="development", seeds=list(range(low, high + 1)),
                      successes=100, hard_limit_failures=0, nominal_balance_regression_passed=True)
        result.update(changes)
        return result

    def test_requires_observed_stop_before_enable(self):
        decision = GetupController(self.plan).update(sample(0), True)
        self.assertEqual(decision.phase, Phase.STOP)
        self.assertEqual(decision.torque_scale, 0)

    def test_reference_reset_once_and_enable_ramp(self):
        first = self.controller.update(sample(.01), True)
        second = self.controller.update(sample(.02), True)
        self.assertTrue(first.reset_reference_and_velocity_history)
        self.assertFalse(second.reset_reference_and_velocity_history)
        self.assertAlmostEqual(first.torque_scale, 0)
        self.assertAlmostEqual(second.torque_scale, .05)

    def test_complete_dwell_handoff_and_ten_second_hold(self):
        tick, decision = self.run_until(self.controller, Phase.BALANCE)
        self.assertAlmostEqual(tick / 100, 1.31)
        self.assertFalse(decision.success)
        for next_tick in range(tick + 1, tick + 1001):
            decision = self.controller.update(sample(next_tick / 100), True)
            self.assertEqual(decision.success, next_tick == tick + 1000)

    def test_body_contact_is_allowed_during_getup_and_resets_dwell(self):
        for tick in range(1, 51):
            self.controller.update(sample(tick / 100), True)
        decision = self.controller.update(sample(.51, body_contact=True), True)
        self.assertEqual(decision.phase, Phase.GETUP)
        for tick in range(52, 152):
            decision = self.controller.update(sample(tick / 100), True)
            self.assertEqual(decision.phase, Phase.GETUP)
        self.assertEqual(self.controller.update(sample(1.52), True).phase, Phase.HANDOFF)

    def test_assistance_never_counts_as_stable_success(self):
        for tick in range(1, 151):
            decision = self.controller.update(sample(tick / 100, assistance_force_n=1), True)
        self.assertEqual(decision.phase, Phase.GETUP)

    def test_timeout_during_unready_getup(self):
        for tick in range(1, 504):
            decision = self.controller.update(sample(tick / 100, body_contact=True), True)
            if decision.failure:
                break
        self.assertEqual(decision.failure, "getup_timeout")
        self.assertEqual(decision.torque_scale, 0)

    def test_each_hard_failure_overrides_posture(self):
        for field in ("joint_limits_valid", "spring_stroke_valid", "linkage_branch_valid", "telemetry_valid"):
            controller = GetupController(self.plan)
            controller.update(sample(0), False)
            decision = controller.update(sample(.01, **{field: False}), True)
            self.assertEqual(decision.phase, Phase.FAILED)
            self.assertEqual(decision.torque_scale, 0)

    def test_failure_latches_until_stop_and_reenable(self):
        self.controller.update(sample(.01, forbidden_contact=True), True)
        self.assertEqual(self.controller.update(sample(.02), True).phase, Phase.FAILED)
        self.controller.update(sample(.03), False)
        self.assertEqual(self.controller.update(sample(.04), True).phase, Phase.GETUP)

    def test_telemetry_gap_fails_without_accumulating_dwell(self):
        self.controller.update(sample(.01), True)
        self.assertEqual(self.controller.update(sample(1.5), True).failure, "telemetry_time_gap")

    def test_duplicate_timestamp_rejected(self):
        self.controller.update(sample(.01), True)
        with self.assertRaisesRegex(ValueError, "increase"):
            self.controller.update(sample(.01), True)

    def test_invalid_samples_rejected(self):
        for changes in ({"clearance_m": float("nan")}, {"telemetry_valid": 1}, {"displacement_m": -1}):
            with self.assertRaises(ValueError):
                sample(.1, **changes)

    def test_continuous_hip_uses_same_wrapping_as_existing_observation(self):
        q = list(SOURCE_DEFAULT_ANGLES)
        q[0] += 2 * 3.141592653589793
        q[3] -= 2 * 3.141592653589793
        self.assertTrue(self.controller.ready(sample(.1, joint_position_rad=q)))

    def test_instability_during_handoff_stops(self):
        tick, _ = self.run_until(self.controller, Phase.HANDOFF)
        decision = self.controller.update(sample((tick + 1) / 100, roll_pitch_deg=(9., 0.)), True)
        self.assertEqual(decision.failure, "handoff_unstable")

    def test_body_contact_in_balance_stops(self):
        tick, _ = self.run_until(self.controller, Phase.BALANCE)
        decision = self.controller.update(sample((tick + 1) / 100, body_contact=True), True)
        self.assertEqual(decision.failure, "balance_body_contact")

    def test_existing_balance_failure_propagates(self):
        tick, _ = self.run_until(self.controller, Phase.BALANCE)
        decision = self.controller.update(sample((tick + 1) / 100, balance_failure="tilt_or_low_root"), True)
        self.assertEqual(decision.failure, "tilt_or_low_root")

    def test_smooth_handoff_weights_and_target_guard(self):
        tick, _ = self.run_until(self.controller, Phase.HANDOFF)
        weights = []
        for next_tick in range(tick + 1, tick + 31):
            decision = self.controller.update(sample(next_tick / 100), True)
            weights.append(decision.balance_weight)
            targets = blended_targets(decision, [10] * 6, [-10] * 6, self.limits)
            self.assertTrue(self.limits[0][0] <= targets["position"][1] <= self.limits[0][1])
            self.assertLessEqual(abs(targets["velocity"][2]), 40)
        self.assertEqual(weights, sorted(weights))
        self.assertEqual(weights[-1], 1)

    def test_previous_action_matches_applied_blend(self):
        decision = Decision(Phase.HANDOFF, 1, .25, False, False, None)
        targets = blended_targets(decision, [1] * 6, [-1] * 6, self.limits)
        self.assertEqual(targets["clipped_action"], [.5] * 6)

    def test_reward_logs_bounded_progress_and_single_success_event(self):
        decision = Decision(Phase.GETUP, 1, 0, False, False, None)
        args = (sample(.01), sample(0), decision, [0] * 6, [0] * 6, [0] * 6, [0] * 6, 0., 0., 0.)
        terms = reward_terms(*args, self.plan["getup"]["reward_weights"])
        success = reward_terms(*args, self.plan["getup"]["reward_weights"], success_event=True)
        self.assertTrue(-1 <= terms["raw"]["progress"] <= 1)
        self.assertAlmostEqual(success["total"] - terms["total"], 10)

    def test_nonfinite_reward_inputs_rejected(self):
        decision = Decision(Phase.GETUP, 1, 0, False, False, None)
        with self.assertRaises(ValueError):
            reward_terms(sample(.01), sample(0), decision, [0] * 6, [0] * 6,
                         [0] * 6, [0] * 6, float("nan"), 0, 0, self.plan["getup"]["reward_weights"])

    def test_promotion_needs_three_complete_passes_and_is_idempotent(self):
        course = Curriculum(self.plan)
        first = self.report("update1")
        self.assertFalse(course.record(first))
        self.assertFalse(course.record(first))
        self.assertEqual(course.consecutive, 1)
        self.assertFalse(course.record(self.report("update2")))
        self.assertTrue(course.record(self.report("update3")))
        self.assertEqual(course.stage, "supported_survey")

    def test_incomplete_or_failed_regression_resets_promotion(self):
        course = Curriculum(self.plan)
        course.record(self.report("a"))
        course.record(self.report("b", nominal_balance_regression_passed=False))
        self.assertEqual(course.consecutive, 0)
        course.record(self.report("c", seeds=[200000], successes=1))
        self.assertEqual(course.consecutive, 0)

    def test_heldout_and_reused_evaluation_ids_rejected(self):
        course = Curriculum(self.plan)
        with self.assertRaises(ValueError):
            course.record(self.report("a", split="heldout_nominal"))
        course.record(self.report("b"))
        with self.assertRaises(ValueError):
            course.record(self.report("b", successes=90))

    def test_curriculum_resume_and_changed_plan(self):
        course = Curriculum(self.plan)
        course.record(self.report("a"))
        self.assertEqual(Curriculum(self.plan, course.state_dict()).consecutive, 1)
        other = copy.deepcopy(self.plan)
        other["getup"]["getup_timeout_s"] = 6
        with self.assertRaisesRegex(ValueError, "mismatch"):
            Curriculum(other, course.state_dict())

    def test_resume_can_advance_curriculum_without_losing_optimizer_experiment(self):
        course = Curriculum(self.plan)
        course.record(self.report("a"))
        saved = course.state_dict()
        course.record(self.report("b"))
        course.record(self.report("c"))
        restored = resumed_curriculum(self.plan, saved, course.state_dict())
        self.assertEqual(restored.stage, "supported_survey")
        invalid = course.state_dict()
        invalid["evaluations"].pop("a")
        with self.assertRaisesRegex(ValueError, "history"):
            resumed_curriculum(self.plan, saved, invalid)

    def test_randomization_reproducible_and_wheels_unchanged(self):
        course = Curriculum(self.plan)
        course.index = len(course.stages) - 1
        first = course.episode_parameters(100010)
        self.assertEqual(first, course.episode_parameters(100010))
        self.assertEqual([first["motor_gain"][i] for i in (2, 5)], [1, 1])
        self.assertTrue(all(.8 <= first["motor_gain"][i] <= 1 for i in (0, 1, 3, 4)))
        with self.assertRaises(ValueError):
            course.episode_parameters(300000)

    def test_preparatory_stages_skip_disabled_settle_then_real_rest_uses_it(self):
        course = Curriculum(self.plan)
        self.assertEqual(course.episode_parameters(100000)["settle_zero_torque_s"], 0)
        course.index = 3
        self.assertGreaterEqual(course.episode_parameters(100000)["settle_zero_torque_s"], 1)
        course.index = 4
        self.assertTrue(course.episode_parameters(300000, "heldout_nominal")["nominal"])

    def test_acceptance_requires_complete_seeds_classes_and_limits(self):
        rows = [dict(seed=300000 + i, success=True, hard_limit_failure=False,
                     reset_class=self.plan["getup"]["final_reset_classes"][i % 3]) for i in range(200)]
        self.assertTrue(acceptance(rows, self.plan, "heldout_nominal")["passed"])
        self.assertFalse(acceptance(rows[:10], self.plan, "heldout_nominal")["passed"])
        rows[0]["hard_limit_failure"] = True
        self.assertFalse(acceptance(rows, self.plan, "heldout_nominal")["passed"])

    def test_acceptance_duplicate_seeds_rejected(self):
        rows = [dict(seed=300000, success=True, hard_limit_failure=False, reset_class="disabled_rest")] * 2
        with self.assertRaisesRegex(ValueError, "duplicate"):
            acceptance(rows, self.plan, "heldout_nominal")

    def test_success_requires_terminal_autonomous_balance_validation(self):
        base = ([0] * 31, 1., True, False, dict(policy_phase_before="HANDOFF", success=True,
                                              balance_validation_completed=True, failure=None))
        TRAIN.validate_transition(base)
        bad = copy.deepcopy(base)
        bad[4]["balance_validation_completed"] = False
        with self.assertRaises(ValueError):
            TRAIN.validate_transition(bad)

    def test_balance_only_steps_are_excluded_from_learner(self):
        result = ([0] * 31, 0., False, False, dict(policy_phase_before="BALANCE"))
        with self.assertRaisesRegex(ValueError, "GETUP/HANDOFF"):
            TRAIN.validate_transition(result)

    def test_synthetic_trace_keeps_explicit_evidence_scope(self):
        header = dict(schema=SCHEMA, record_type="header", evidence_kind="synthetic_protocol",
                      plan_sha256=plan_digest(self.plan), balance_checkpoint_sha256=self.plan["baseline"]["sha256"],
                      getup_checkpoint_sha256=None, seed=200000, reset_class="near_stand")
        records = [dict(record_type="sample", enabled=False, signals=asdict(sample(0)), getup_action=[0] * 6, balance_action=[0] * 6)]
        for tick in range(1, 1132):
            records.append(dict(record_type="sample", enabled=True, signals=asdict(sample(tick / 100)),
                                getup_action=[0] * 6, balance_action=[0] * 6))
        result = replay(header, records, self.plan, self.limits)
        self.assertTrue(result["success"])
        self.assertFalse(result["physical_validation_eligible"])
        self.assertFalse(result["simulation_validation_eligible"])

    def test_raw_capture_has_distinct_DM_torque_and_wheel_current_units(self):
        with self.assertRaises(KeyError):
            validate_raw_telemetry(dict(mcu_tick_ms=1, mode=0, inhibit=0, can_received=[1] * 6))

    def valid_raw(self):
        return dict(mcu_tick_ms=100, mode=3, inhibit=0, can_received=[100] * 6, can_last_tick_ms=[100] * 6,
                    imu_tick_ms=100, remote_tick_ms=100, joint_velocity_rad_s=[0.] * 6,
                    requested_motor_torque_nm=[0.] * 6, dm_reported_torque_nm=[0.] * 4,
                    wheel_reported_current_a=[0.] * 2, imu_quaternion_wxyz=[1., 0., 0., 0.],
                    projected_gravity=[0., 0., -1.], estimated_xy_m=[0., 0.],
                    battery_voltage_v=None, host_monotonic_s=1.)

    def test_capture_allows_unmeasured_voltage_and_rejects_stale_can(self):
        raw = self.valid_raw()
        validate_raw_telemetry(raw)
        raw["can_last_tick_ms"][2] = 0
        with self.assertRaisesRegex(ValueError, "freshness"):
            validate_raw_telemetry(raw)

    def test_capture_rejects_stale_imu_and_remote(self):
        for name in ("imu_tick_ms", "remote_tick_ms"):
            raw = self.valid_raw()
            raw["mcu_tick_ms"] = 500
            raw["can_last_tick_ms"] = [500] * 6
            raw["imu_tick_ms"] = raw["remote_tick_ms"] = 500
            raw[name] = 0
            with self.assertRaisesRegex(ValueError, "stale"):
                validate_raw_telemetry(raw)

    def plant_adapter_class(self):
        name = "fifty_mujoco.envs.sample_env31_getup"
        spec = importlib.util.spec_from_file_location(name, PROJECT / "src/fifty_mujoco/envs/sample_env31_getup.py")
        module = importlib.util.module_from_spec(spec)
        substitutes = {key: MagicMock() for key in ("mujoco", "numpy", "torch",
                       "fifty_mujoco.envs.sample_env31_hardware_guarded", "fifty_mujoco.envs.sample_env31_hardware")}
        with patch.dict(sys.modules, substitutes):
            spec.loader.exec_module(module)
        return module.GetupEnv31

    def test_support_release_clock_starts_after_disabled_settling(self):
        adapter_type = self.plant_adapter_class()
        adapter = adapter_type.__new__(adapter_type)
        adapter.spec = self.plan["getup"]
        adapter.parameters = {"stage": "support_release"}
        adapter.env = SimpleNamespace(data=SimpleNamespace(time=5., qpos=[0., 0., .28], qvel=[0., 0., 0.]))
        adapter.enabled_at, adapter.release_after_s = 1., .5
        adapter.support_height, adapter.bodyweight = .3, 200.
        adapter.settling = True
        self.assertGreater(adapter._support(), 0)
        adapter.settling = False
        self.assertEqual(adapter._support(), 0)

    def test_enable_ramp_scales_command_and_delivered_joint_force_together(self):
        adapter_type = self.plant_adapter_class()
        adapter = adapter_type.__new__(adapter_type)
        actuator = SimpleNamespace(command=20., delivered=16.)
        adapter.env = SimpleNamespace(fast_actuators=actuator)
        adapter.original_apply = lambda *args: 10.
        adapter.active_scale = .5
        self.assertEqual(adapter._apply_scaled(None, None, None), 5)
        self.assertEqual(actuator.command, 10)
        self.assertEqual(actuator.delivered, 8)


if __name__ == "__main__":
    unittest.main()
