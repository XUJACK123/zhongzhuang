"""Standard-library checks for offline evidence preparation. No simulator imports."""

import copy
import importlib.util
from pathlib import Path
import struct
import unittest
from unittest.mock import patch

PROJECT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("prepare_next", PROJECT / "tools/sample_hardware_prepare_next.py")
prepare_next = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(prepare_next)


def saved_blobs():
    blobs = {name: bytearray(size) for name, size in prepare_next.SIZES.items()}
    struct.pack_into("<I", blobs["g_chassis"], 0, 1234)
    struct.pack_into("<I", blobs["g_chassis"], 32, 3)
    struct.pack_into("<6f", blobs["g_rl"], 1140, 1, 2, 3, 4, 5, 6)
    struct.pack_into("<f", blobs["g_motor_snapshot"], 4 * 40 + 24, -5.88)
    return blobs


class OfflinePreparationTests(unittest.TestCase):
    def setUp(self):
        self.plan = prepare_next.read_json(PROJECT / prepare_next.PLAN)
        self.profile = prepare_next.read_json(PROJECT / self.plan["baseline"]["profile"])

    def test_current_plan_matches_contract(self):
        prepare_next.validate_plan(self.plan, self.profile)

    def test_overlapping_seed_ranges_rejected(self):
        self.plan["evaluation"]["seed_ranges_inclusive"]["heldout_nominal"] = [199999, 300000]
        with self.assertRaisesRegex(ValueError, "overlap"):
            prepare_next.validate_plan(self.plan, self.profile)

    def test_wrong_timing_rejected(self):
        self.plan["baseline"]["timing_s"]["pd"] = .001
        with self.assertRaisesRegex(ValueError, "timing"):
            prepare_next.validate_plan(self.plan, self.profile)

    def test_wrong_architecture_rejected(self):
        self.plan["baseline"]["actor_sizes"][0] = 25
        with self.assertRaisesRegex(ValueError, "architecture"):
            prepare_next.validate_plan(self.plan, self.profile)

    def test_out_of_range_reset_rejected(self):
        self.profile["natural_joint_pose_rad"][1] = 0
        with self.assertRaisesRegex(ValueError, "knee"):
            prepare_next.validate_plan(self.plan, self.profile)

    def test_arm_offsets(self):
        sample = prepare_next.decode_sample(saved_blobs())
        self.assertEqual(sample["mcu_tick_ms"], 1234)
        self.assertEqual(sample["mode"], 3)
        self.assertEqual(sample["requested_motor_torque_nm"], [1, 2, 3, 4, 5, 6])
        self.assertAlmostEqual(sample["feedback_raw_firmware_units"][4], -5.88, places=5)

    def test_partial_sample_rejected(self):
        blobs = saved_blobs()
        blobs["g_rl"] = b""
        with self.assertRaisesRegex(ValueError, "expected 1164"):
            prepare_next.decode_sample(blobs)

    def test_nonfinite_sample_rejected(self):
        blobs = saved_blobs()
        struct.pack_into("<f", blobs["g_measurement"], 0, float("nan"))
        with self.assertRaisesRegex(ValueError, "non-finite"):
            prepare_next.decode_sample(blobs)

    def test_torn_counts_marked(self):
        blobs = saved_blobs()
        struct.pack_into("<q", blobs["g_motor_snapshot"], 8, 4294964075)
        self.assertTrue(prepare_next.decode_sample(blobs)["possible_count_tearing"])

    def test_incomplete_run_keeps_only_complete_records_and_missing_log(self):
        directory = PROJECT / "tests" / "virtual_saved_watch"
        files = {directory / f"000_{name}.bin": data for name, data in saved_blobs().items()}
        files[directory / "001_g_chassis.bin"] = b""
        with patch.object(Path, "read_text", return_value="set n 3\nsleep 200\n"), \
             patch.object(Path, "glob", return_value=list(files)), \
             patch.object(Path, "exists", lambda path: path in files), \
             patch.object(Path, "is_file", return_value=False), \
             patch.object(Path, "read_bytes", lambda path: files[path]), \
             patch.object(prepare_next, "sha256", return_value="fixture_hash"):
            report = prepare_next.recover_watch(directory, directory)
            self.assertEqual(report["decoded_samples"], 1)
            self.assertEqual(len(report["rejected"]), 1)
            self.assertEqual(report["missing_indices"], [2])
            self.assertFalse(report["openocd_log_present"])

    def test_frozen_debugger_snapshot_is_not_live_can(self):
        snapshot = {"chassis": {"tick": 50, "inhibit": 0},
                    "nodes": [{"label": name, "received": 100} for name in prepare_next.LABELS]}
        data = {"a": snapshot, "b": copy.deepcopy(snapshot)}
        self.assertFalse(prepare_next.can_pair_summary(data)["all_six_increasing"])


if __name__ == "__main__":
    unittest.main()
