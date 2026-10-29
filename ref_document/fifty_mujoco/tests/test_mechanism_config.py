"""``configs/mechanism.yaml`` contract.

These tests exist because the numbers in that file are the only CAD-derived
geometry the project has. Two failure modes are worth pinning:

* a coordinate edited by hand that no longer matches the length table,
* an evidence label promoted (`measured`) without a source behind it.
"""

from __future__ import annotations

import copy

import pytest
import yaml

from fifty_mujoco.mechanism import MechanismConfigError, load_mechanism
from fifty_mujoco.mechanism.check import topology_gate
from fifty_mujoco.mechanism.leg import serial_leg_report, static_closure
from fifty_mujoco.mechanism.planar import TopologyNotConfirmedError
from fifty_mujoco.paths import configs_dir

MECHANISM_PATH = configs_dir() / "mechanism.yaml"

#: Lengths measured directly inside one part by the STEP audit (progress.md §18.11).
DECLARED_BY_THE_AUDIT_MM = {
    "thigh_hip_to_knee": 210.000,
    "shin_knee_to_wheel": 250.000,
    "push_rod": 135.000,
    "end_link": 96.600,
    "ternary_a_b": 135.000,
    "ternary_a_c": 67.000,
    "ternary_b_c": 200.509609,
}


@pytest.fixture(scope="module")
def mechanism():
    return load_mechanism()


def test_every_entry_declares_a_source_and_an_evidence_level(mechanism):
    for node in mechanism.nodes.values():
        assert node.status in ("cad_extracted", "computed_from_points")
        assert node.source
    for link in mechanism.all_links.values():
        assert link.status in ("cad_extracted", "computed_from_points")
        assert link.source
    for body in mechanism.bodies.values():
        assert body.status in ("cad_extracted", "candidate_inferred")
        assert body.source or body.evidence or body.confirm_in_cad


def test_nothing_in_the_file_claims_a_hardware_measurement(mechanism):
    statuses = {node.status for node in mechanism.nodes.values()}
    statuses |= {link.status for link in mechanism.all_links.values()}
    statuses |= {body.status for body in mechanism.bodies.values()}
    assert not statuses & {"measured", "validated", "datasheet"}


def test_the_length_table_matches_the_cad_coordinates(mechanism):
    closure = static_closure(mechanism)
    assert closure["status"] == "consistent"
    assert closure["worst_error_m"] * 1000.0 <= closure["tolerance_mm"]


def test_reproduces_the_step_audit_measurements(mechanism):
    for name, expected_mm in DECLARED_BY_THE_AUDIT_MM.items():
        assert mechanism.length_mm(name) == pytest.approx(expected_mm, abs=1e-6), name


def test_assembly_pose_is_flagged_as_stowed(mechanism):
    assert mechanism.meta["assembly_pose"] == "stowed"
    # The chord is 300 mm against a 464.75 mm standing height: recording the
    # assembly pose as a working length would be a 160 mm error.
    chord = mechanism.length_mm("hip_to_wheel_chord")
    assert 295.0 < chord < 305.0


def test_serial_leg_reproduces_the_three_cad_points(mechanism):
    report = serial_leg_report(mechanism)
    assert report["thigh_m"] == pytest.approx(0.210, abs=1e-12)
    assert report["shin_m"] == pytest.approx(0.250, abs=1e-12)
    pose = report["assembly_pose"]
    assert pose["inside_annulus"] is True
    assert pose["cad_reproduction_error_m"] <= 1e-9
    assert report["round_trip"]["worst_error_m"] <= 1e-12
    assert report["round_trip"]["unreachable_targets"] == 0


def test_candidate_topology_is_refused_unless_it_is_switched_on(mechanism):
    with pytest.raises(TopologyNotConfirmedError):
        topology_gate(mechanism, allow_inferred=False)
    topology_gate(mechanism, allow_inferred=True)  # must not raise


def _raw() -> dict:
    with MECHANISM_PATH.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def mechanism_path():
    return MECHANISM_PATH


def test_a_promoted_status_without_evidence_is_rejected():
    from fifty_mujoco.mechanism.loader import parse_mechanism

    data = copy.deepcopy(_raw())
    data["planar_nodes"]["HIP"]["status"] = "measured"
    with pytest.raises(MechanismConfigError, match="method"):
        parse_mechanism(data, mechanism_path())


def test_an_unknown_status_is_rejected():
    from fifty_mujoco.mechanism.loader import parse_mechanism

    data = copy.deepcopy(_raw())
    data["link_lengths"]["thigh_hip_to_knee"]["status"] = "looks-right"
    with pytest.raises(MechanismConfigError, match="not one of"):
        parse_mechanism(data, mechanism_path())


def test_a_missing_source_is_rejected():
    from fifty_mujoco.mechanism.loader import parse_mechanism

    data = copy.deepcopy(_raw())
    data["link_lengths"]["shin_knee_to_wheel"].pop("source")
    with pytest.raises(MechanismConfigError, match="source"):
        parse_mechanism(data, mechanism_path())


def test_a_link_pointing_at_an_unknown_node_is_rejected():
    from fifty_mujoco.mechanism.loader import parse_mechanism

    data = copy.deepcopy(_raw())
    data["link_lengths"]["push_rod"]["between"] = ["CRANK_PIN", "NOWHERE"]
    with pytest.raises(MechanismConfigError, match="unknown node"):
        parse_mechanism(data, mechanism_path())


def test_an_edited_coordinate_breaks_the_length_table():
    from fifty_mujoco.mechanism.loader import parse_mechanism

    data = copy.deepcopy(_raw())
    data["planar_nodes"]["KNEE"]["yz_mm"][0] += 1.0  # 1 mm typo
    broken = parse_mechanism(data, mechanism_path())
    assert static_closure(broken)["status"] == "inconsistent"
