"""P0: the audit must reproduce the documented defects and keep sources frozen."""

from __future__ import annotations

import numpy as np

from fifty_mujoco.assets import stl


def test_all_structural_checks_pass(audit_report):
    for name, check in audit_report["checks"].items():
        assert check["passed"], f"{name} failed: {check}"
    assert audit_report["status"] == "passed"


def test_joint_origins_align_to_machine_precision(audit_report):
    worst = audit_report["checks"]["joint_origin_alignment"]["max_error_m"]
    assert worst <= 1e-12


def test_known_visual_defects_are_reproduced(audit_report):
    entries = {entry["link"]: entry for entry in audit_report["visual_transform_defects"]}
    offsets = {link: entry["offset_norm_m"] for link, entry in entries.items()}
    # Documented: the wheel mesh carries an extra ~[-0.2386, 0.0452, 0] m offset.
    assert offsets["right_wheel"] > 0.2
    assert offsets["left_wheel"] > 0.2
    # Shin meshes carry the same class of error.
    assert offsets["r1_link"] > 0.3
    assert offsets["l1_link"] > 0.3
    # Thighs are already correct in the G1 package.
    for link in ("r0_link", "l0_link"):
        assert offsets[link] < 1e-6
    # The base link differs *deliberately*: the G1 base rotation left the
    # chassis upside down, so this package applies an explicit correction.
    assert entries["base_link"]["intentional_correction"] is True
    assert offsets["base_link"] > 0.1
    assert "base_link" in audit_report["findings"]["visual_offsets"]["intentionally_corrected_links"]


def test_wheel_defect_matches_documented_vector(audit_report):
    entry = next(d for d in audit_report["visual_transform_defects"] if d["link"] == "right_wheel")
    offset = np.array(entry["offset_vs_recomputed_m"])
    assert np.linalg.norm(offset - np.array([-0.2386, 0.0452, 0.0])) < 5e-3


def test_inertial_rotation_defect_is_detected(audit_report):
    assert audit_report["findings"]["inertial_origin_rotation"]["identity"] is False


def test_source_hashes_unchanged(config, audit_report):
    assert audit_report["sources"]["source_urdf"]["sha256_matches_parameters_json"]
    for entry in audit_report["meshes"].values():
        assert entry["recorded_sha256_matches"]
        assert entry["recorded_triangles_matches"]
    # Re-hash directly, independent of the report.
    assert stl.sha256(config.source_urdf) == audit_report["sources"]["source_urdf"]["sha256"]


def test_missing_inputs_are_listed(config, audit_report):
    missing = audit_report["missing_inputs"]
    # The gas spring anchors are now configured (transferred from Fudan), but
    # every hardware-derived number is still provisional.
    assert missing["gas_spring_anchor_geometry"] is False
    assert config.gas_spring_stroke_verified is False
    assert missing["rotor_armature"] is True
    assert missing["wheel_radius_measured"] is True
