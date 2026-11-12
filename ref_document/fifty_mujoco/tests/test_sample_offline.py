"""Offline unittest suite: use -I -S, never the full physics pytest suite."""

from __future__ import annotations

import io
import json
import math
import pickle
import sys
import unittest
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
sys.path.insert(0, str(PROJECT / "tools"))

from fifty_mujoco.contracts.sample_actor31 import (  # noqa: E402
    SOURCE_DEFAULT_ANGLES, action_targets, advance_reference, bottom_clearance,
    build_observation, clip_actions, ramp_commands, reference_features,
)
from sample_weight_offline import (  # noqa: E402
    DescriptorReader, audit, checkpoint_audit, inertia_is_physical, inside,
)


class ContractTests(unittest.TestCase):
    def assertVectorClose(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=10)

    def observation(self, **overrides):
        values = dict(angular_velocity_body=[4, -8, 12],
                      projected_gravity_body=[0, 0, -1],
                      applied_commands=[.15, -.3, .1],
                      joint_position=SOURCE_DEFAULT_ANGLES,
                      joint_velocity=[1, 2, 3, 4, 5, 6],
                      previous_clipped_action=[.1, -.2, 2, -.4, .5, -2],
                      position_xy=[0, 0], reference_xy=[0, 0], yaw=0,
                      reference_yaw=0, body_velocity_xy=[.15, -.1])
        values.update(overrides)
        return build_observation(**values)

    def test_all_observation_channels_have_expected_units_and_order(self):
        self.assertVectorClose(self.observation(),
                               [1, -2, 3, 0, 0, -1, .3, -.075, .5,
                                0, 0, 0, 0, .05, .1, .15, .2, .25, .3,
                                .1, -.2, 2, -.4, .5, -2, 0, 0, 0, 0, .3, -.2])

    def test_continuous_hip_wrap_does_not_wrap_knee(self):
        q = list(SOURCE_DEFAULT_ANGLES)
        q[0] += 2 * math.pi + .1
        q[3] -= 2 * math.pi + .2
        q[1] += .3
        self.assertVectorClose(self.observation(joint_position=q)[9:13], [.1, .3, -.2, 0])

    def test_reference_error_is_in_body_frame_and_bounded(self):
        actual = reference_features([0, 0], [0, 10], math.pi / 2, math.pi, [.2, -.3])
        self.assertVectorClose(actual, [2, 0, 1, -1, .4, -.6])

    def test_clearance_includes_collision_offset_and_tilt(self):
        self.assertAlmostEqual(bottom_clearance(.3, [0, 0, -1], [0, 0, .05], [.3, .2, .25]), .1)
        self.assertAlmostEqual(bottom_clearance(.3, [-1, 0, 0], [0, 0, .05], [.3, .2, .25]), 0)

    def test_ramp_sign_reversal_obeys_applied_command(self):
        self.assertVectorClose(ramp_commands([.15, .3, .1], [-.15, -.3, .2], .01),
                               [.135, .3 - .02 * math.pi, .1012])
        self.assertVectorClose(ramp_commands([0, 0, .1], [.001, .001, .1], .01),
                               [.001, .001, .1])

    def test_se2_straight_reverse_and_arc_are_analytic(self):
        p, yaw = advance_reference([0, 0], 0, -.2, 0, 2, offset_xy=[0, 0])
        self.assertVectorClose(p, [-.4, 0])
        self.assertEqual(yaw, 0)
        p, yaw = advance_reference([0, 0], 0, 1, 1, math.pi / 2, offset_xy=[0, 0])
        self.assertVectorClose(p, [1, 1])
        self.assertAlmostEqual(yaw, math.pi / 2)

    def test_rotating_offset_keeps_commanded_point_fixed(self):
        p, _ = advance_reference([0, 0], 0, 0, 1, math.pi / 2, offset_xy=[1, 0])
        self.assertVectorClose(p, [1, -1])

    def test_action_limits_and_physical_wheel_velocity_targets(self):
        limits = [[-4.296127953784042, -2.5592683579936732],
                  [2.559268357993676, 4.296127953784042]]
        target = action_targets([3, 3, 4, -3, -3, -4], limits)
        self.assertEqual(target["clipped_action"], [1, 1, 2, -1, -1, -2])
        self.assertEqual(target["velocity"], [0, 0, 40, 0, 0, -40])
        self.assertAlmostEqual(target["position"][0], SOURCE_DEFAULT_ANGLES[0] + .5)

    def test_knee_target_is_clamped_before_control(self):
        targets = action_targets([0, 1, 0, 0, -1, 0], [[-4, -3.5], [3.5, 4]])
        self.assertEqual(targets["position"][1], -3.5)
        self.assertEqual(targets["position"][4], 3.5)

    def test_invalid_dimensions_nonfinite_and_unclipped_history_are_rejected(self):
        for bad in ([0] * 5, [0, 0, float("nan"), 0, 0, 0]):
            with self.assertRaises(ValueError):
                clip_actions(bad)
        with self.assertRaises(ValueError):
            self.observation(previous_clipped_action=[0, 2, 0, 0, 0, 0])
        with self.assertRaises(ValueError):
            ramp_commands([0, 0, 0], [1, 0, 0], 0)


class ArchiveTests(unittest.TestCase):
    def test_arbitrary_pickle_global_is_rejected_without_import_or_call(self):
        # Protocol-0 GLOBAL followed by REDUCE would call eval in unrestricted
        # pickle.load. The restricted reader must reject before that happens.
        malicious = b"cbuiltins\neval\n(V1+1\ntR."
        with self.assertRaisesRegex(ValueError, "unsupported checkpoint global"):
            DescriptorReader(io.BytesIO(malicious)).load()

    def test_bad_persistent_storage_is_rejected(self):
        reader = DescriptorReader(io.BytesIO(pickle.dumps({})))
        for bad in (None, ("storage", "wrong", "0", "cpu", 6),
                    ("storage", "symbolic_torch_float32_storage", "../x", "cpu", 6)):
            with self.assertRaises(ValueError):
                reader.persistent_load(bad)

    def test_manifest_path_escape_is_rejected(self):
        with self.assertRaises(ValueError):
            inside(PROJECT, "../outside.txt")

    def test_all_four_real_checkpoints_are_finite_and_feedforward(self):
        folder = PROJECT.parents[1] / "sample_weight_file/artifacts"
        for iteration in (2975, 3000, 3025, 3050):
            result = checkpoint_audit(folder / f"model_{iteration}.pt")
            self.assertEqual(result["iteration"], iteration)
            self.assertEqual(result["shapes"]["actor.0.weight"], [128, 31])
            self.assertFalse(result["encoder_present"])
            self.assertFalse(result["inference_executed"])
            self.assertTrue(all(row["all_finite"] for row in result["tensors"].values()))

    def test_full_source_and_contract_audit_retains_runtime_gates(self):
        report = audit(PROJECT.parents[1])
        self.assertEqual(report["archive_integrity"]["checked"], 48)
        self.assertEqual(report["archive_integrity"]["matched"], 48)
        self.assertEqual(report["status"], "static_pass_runtime_pending")
        self.assertTrue(report["source_resolution"]["critical_unresolved"])
        self.assertEqual(report["gates"]["V2_V8"], "not_run")
        self.assertTrue(all(report["source_geometry"]["checks"].values()))
        self.assertTrue(all(r["source_hash_verified"] for r in report["mesh_references"]))

    def test_positive_inertia_can_still_violate_physical_triangle_inequality(self):
        self.assertEqual(inertia_is_physical(1, 1, 3, 0, 0, 0), (True, False))
        self.assertEqual(inertia_is_physical(1, 1, 2, 0, 0, 0), (True, True))
        self.assertEqual(inertia_is_physical(1, -1, 2, 0, 0, 0), (False, False))

    def test_no_physics_or_learning_dependencies_imported(self):
        self.assertFalse(set(sys.modules) & {"torch", "mujoco", "isaacgym", "numpy", "warp"})

    def test_protocol_seed_sets_are_disjoint_and_randomization_disabled(self):
        cfg = PROJECT / "configs"
        protocol = json.loads((cfg / "sample_command_evaluation.json").read_text(encoding="utf-8"))
        used = set()
        for name in ("smoke", "development", "heldout_nominal", "heldout_random"):
            lo, hi = protocol[name]["seeds"]
            seeds = set(range(lo, hi + 1))
            self.assertFalse(used & seeds)
            self.assertEqual(len(seeds), protocol[name]["episodes_per_class"] * 5)
            used.update(seeds)
        randomization = json.loads((cfg / "sample_randomization.json").read_text(encoding="utf-8"))
        self.assertFalse(randomization["enabled"])
        self.assertFalse(randomization["runtime_adapter_implemented"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
