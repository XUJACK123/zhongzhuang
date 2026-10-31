"""Axial URDF offsets must never inflate planar linkage lengths."""
from __future__ import annotations

import json
import math
from copy import deepcopy
from dataclasses import replace

import pytest

from fifty_mujoco.mechanism.loader import load_mechanism
from fifty_mujoco.mechanism.resolve import authority, describe, project_offset, resolve_nodes


@pytest.fixture(scope="module")
def mechanism():
    return load_mechanism()


@pytest.fixture(scope="module")
def resolved(mechanism):
    return resolve_nodes(mechanism)


def test_authority_is_the_perpendicular_urdf_projection(mechanism, resolved):
    assert authority(mechanism) == pytest.approx((210.0, 250.0), abs=1e-6)
    assert mechanism.meta["link_length_authority"]["decision"] == "urdf"
    for side in ("right", "left"):
        rows = resolved.urdf_projection["sides"][side]
        assert rows["thigh"]["planar_length_m"] == pytest.approx(0.210, abs=1e-9)
        assert rows["shin"]["planar_length_m"] == pytest.approx(0.250, abs=1e-9)
        assert rows["shin"]["origin_distance_m"] > rows["shin"]["planar_length_m"]
    # the URDF writes the sideways offset in the leg's own frame; the left leg's
    # local +Z points the other way, so the physical (base-frame) sign flips
    right = resolved.urdf_projection["sides"]["right"]
    left = resolved.urdf_projection["sides"]["left"]
    assert right["shin"]["axial_offset_local_m"] == pytest.approx(-0.0446)
    assert left["shin"]["axial_offset_local_m"] == pytest.approx(-0.05255)
    assert right["shin"]["axial_offset_m"] == pytest.approx(-0.0446)
    assert left["shin"]["axial_offset_m"] == pytest.approx(+0.05255)


def test_projection_is_independent_of_axial_translation():
    for axial in (-0.5, 0.0, 0.0115, 0.5):
        result = project_offset((0.21, 0.0, axial), (0.0, 0.0, 7.0))
        assert result["planar_length_m"] == pytest.approx(0.21, abs=1e-12)
        assert result["axial_offset_m"] == pytest.approx(axial, abs=1e-12)


def test_projection_does_not_assume_a_z_axis():
    a = project_offset((0.21, 0, 0.0115), (0, 0, 1))
    b = project_offset((0.21, 0.0115, 0), (0, 1, 0))
    assert a["planar_length_m"] == pytest.approx(b["planar_length_m"])
    assert a["axial_offset_m"] == pytest.approx(b["axial_offset_m"])
    with pytest.raises(ValueError, match="zero hinge axis"):
        project_offset((0.21, 0, 0), (0, 0, 0))
    with pytest.raises(ValueError, match="non-finite"):
        project_offset((math.nan, 0, 0), (0, 0, 1))


def test_a_raw_3d_norm_cannot_silently_become_planar_authority(mechanism):
    meta = deepcopy(mechanism.meta)
    meta["link_length_authority"]["lengths"]["thigh_m"] = math.hypot(0.21, 0.0115)
    with pytest.raises(ValueError, match="differs from URDF projection"):
        authority(replace(mechanism, meta=meta))


def test_no_cad_node_is_moved_and_no_end_link_is_shortened(mechanism, resolved):
    assert resolved.nodes_mm == {name: node.yz_mm for name, node in mechanism.nodes.items()}
    assert resolved.moved_mm == {}
    assert resolved.worst_move_mm == 0.0
    assert resolved.end_link_mm == pytest.approx(96.6, abs=1e-6)
    assert resolved.end_link_mm == resolved.end_link_cad_mm
    assert math.dist(resolved.nodes_mm["HIP"], resolved.nodes_mm["WHEEL"]) == pytest.approx(300.242981163, abs=1e-6)


def test_loop_lengths_and_ternary_shape_are_preserved(resolved):
    nodes = resolved.nodes_mm
    for first, second, length in (
        ("CRANK_PIN", "ROD_PIN", 135.0),
        ("ROD_PIN", "LINK_PIN_A", 135.0),
        ("LINK_PIN_A", "LINK_PIN_C", 67.0),
        ("ROD_PIN", "LINK_PIN_C", 200.509609567),
    ):
        assert math.dist(nodes[first], nodes[second]) == pytest.approx(length, abs=1e-6)


def test_describe_retains_offsets_and_has_no_adjustment(resolved):
    payload = describe(resolved)
    assert payload["end_link_change_mm"] == 0.0
    assert payload["urdf_projection"]["sides"]["left"]["thigh"]["axial_offset_m"] != 0
    assert json.loads(json.dumps(payload))["thigh_mm"] == 210.0
