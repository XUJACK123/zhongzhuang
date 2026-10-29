"""STEP audit regression on a synthetic file (the 264 MB CAD file is not in CI).

The parser has four failure modes that used to be silent, and each one is
pinned here:

* the DATA section is ASCII with ``\\X2\\`` escapes, not UTF-8/GBK,
* SwSTEP writes the instance transform as a *complex* entity, so a simple
  ``#id = TYPE(...)`` matcher finds zero transforms,
* ``CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`` points at a
  ``PRODUCT_DEFINITION_SHAPE``, not directly at the usage occurrence,
* the B-rep of a part is not the representation that carries the part's name.

The module is standard library only, so these tests need neither numpy nor
MuJoCo.
"""

from __future__ import annotations

import math

import pytest

from fifty_mujoco.assets.step_audit import (
    _line_distance,
    _planar_candidates,
    _shared_axis_nodes,
    audit_step,
    decode_step_string,
    split_top_level,
)

SYNTHETIC = """ISO-10303-21;
HEADER;
FILE_DESCRIPTION (( 'STEP AP214' ), '1' );
FILE_NAME ('synthetic','2026-01-01T00:00:00',(''),(''),'SwSTEP 2.0','SolidWorks 2026','');
FILE_SCHEMA (( 'AUTOMOTIVE_DESIGN' ));
ENDSEC;
DATA;
#1 = PRODUCT('26\\X2\\82f196c465748f66603b88c5\\X0\\2.0','x','',(#99));
#2 = PRODUCT('child part','x','',(#99));
#3 = PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE('','',#1,.NOT_KNOWN.);
#4 = PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE('','',#2,.NOT_KNOWN.);
#5 = PRODUCT_DEFINITION('design','',#3,#100);
#6 = PRODUCT_DEFINITION('design','',#4,#100);
#7 = PRODUCT_DEFINITION_SHAPE('','',#20);
#8 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('NAUO1',' ',' ',#5,#6,$);
#20 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('NAUO2',' ',' ',#5,#6,$);
#9 = ITEM_DEFINED_TRANSFORMATION('','',#11,#14);
#11 = AXIS2_PLACEMENT_3D('',#12,#13,#16);
#12 = CARTESIAN_POINT('',(0.,0.,0.));
#13 = DIRECTION('',(0.,0.,1.));
#16 = DIRECTION('',(1.,0.,0.));
#14 = AXIS2_PLACEMENT_3D('',#15,#17,#18);
#15 = CARTESIAN_POINT('',(100.,0.,0.));
#17 = DIRECTION('',(0.,0.,1.));
#18 = DIRECTION('',(1.,0.,0.));
#10 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#19,#7);
#19 =( REPRESENTATION_RELATIONSHIP ('NONE','NONE', #30, #31 ) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION ( #9 )SHAPE_REPRESENTATION_RELATIONSHIP( ) );
#30 = SHAPE_REPRESENTATION('a',(#1),#101);
#31 = ADVANCED_BREP_SHAPE_REPRESENTATION('b',(#40),#101);
#40 = MANIFOLD_SOLID_BREP('solid',#41);
#41 = CLOSED_SHELL('',(#42));
#42 = ADVANCED_FACE('',(#43),#44,.T.);
#43 = FACE_OUTER_BOUND('',#45,.T.);
#44 = CYLINDRICAL_SURFACE('',#50,8.0);
#45 = EDGE_LOOP('',(#46));
#50 = AXIS2_PLACEMENT_3D('',#51,#52,#53);
#51 = CARTESIAN_POINT('',(0.,0.,0.));
#52 = DIRECTION('',(0.,0.,1.));
#53 = DIRECTION('',(1.,0.,0.));
#99 = PRODUCT_CONTEXT('',#101,.MECHANICAL.);
#100 = PRODUCT_DEFINITION_CONTEXT('part definition',#101,'design');
#101 = APPLICATION_CONTEXT('automotive design');
ENDSEC;
END-ISO-10303-21;
"""


@pytest.fixture()
def step_file(tmp_path):
    path = tmp_path / "synthetic.step"
    path.write_text(SYNTHETIC, encoding="latin-1")
    return path


def test_split_top_level_respects_parentheses_and_quotes():
    assert split_top_level(" 'a,b', (1., 2., 3.), #5 ") == ["'a,b'", "(1., 2., 3.)", "#5"]


def test_unicode_escape_is_decoded():
    assert decode_step_string("26\\X2\\82f196c465748f66603b88c5\\X0\\2.0") == "26英雄整车总装2.0"
    assert decode_step_string("1BKB0.45-75") == "1BKB0.45-75"
    assert decode_step_string("it''s") == "it's"


def test_audit_extracts_structure_and_transform(step_file):
    report = audit_step(step_file)
    assert report["status"] == "structure_and_axis_candidates"
    assert report["counts"]["PRODUCT"] == 2
    assert report["instance_count"] == 2
    assert report["assembly"]["roots"] == ["26英雄整车总装2.0"]
    transformed = [entry for entry in report["instances"] if "transform_mm" in entry]
    assert len(transformed) == 1
    assert transformed[0]["parent"] == "26英雄整车总装2.0"
    assert transformed[0]["child"] == "child part"
    translation = transformed[0]["transform_mm"]["translation_mm"]
    assert all(math.isclose(value, target, abs_tol=1e-9) for value, target in zip(translation, (-100.0, 0.0, 0.0)))
    assert report["verification"]["transform_convention"].startswith("unverified")


def test_cylinders_are_attributed_to_the_part(step_file):
    report = audit_step(step_file)
    geometry = report["geometry"]
    assert geometry["diagnostics"]["cylinders"] == 1
    assert geometry["diagnostics"]["attributed_cylinders"] == 1
    assert geometry["reps_with_cylinders"] == 1
    entry = geometry["parts"][0]
    assert entry["candidate_parts"], "the cylinder must inherit a part name"
    assert entry["cylinder_count"] == 1
    axis = geometry["selected_parts"][0]["dominant_axes"][0] if geometry["selected_parts"] else None
    del axis  # the synthetic part name has no leg keyword, so it is not selected


def test_no_geometry_mode_skips_the_extraction(step_file):
    report = audit_step(step_file, with_geometry=False)
    assert report["status"] == "structure_only"
    assert "geometry" not in report


def test_line_distance_parallel_and_skew():
    parallel = _line_distance((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 3.0, 4.0), (-1.0, 0.0, 0.0))
    assert math.isclose(parallel["distance_mm"], 5.0, rel_tol=0, abs_tol=1e-12)
    assert math.isclose(parallel["along_axis_mm"], 0.0, abs_tol=1e-12)
    skew = _line_distance((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 2.0), (0.0, 1.0, 0.0))
    assert math.isclose(skew["distance_mm"], 2.0, abs_tol=1e-12)
    assert math.isclose(skew["angle_deg"], 90.0, abs_tol=1e-9)


def test_planar_candidates_needs_a_shared_joint():
    thigh = {
        "label": "G01 大腿",
        "status": "measured_same_part",
        "origins_mm": [[0.0, 0.0, 0.0], [0.0, 0.0, 210.0]],
        "radius_mm": [43.0, 30.0],
    }
    shin = {
        "label": "G02 小腿",
        "status": "measured_same_part",
        "origins_mm": [[0.0, 0.0, 210.0], [0.0, 250.0, 210.0]],
        "radius_mm": [21.5, 23.5],
    }
    result = _planar_candidates([thigh, shin])
    assert result["status"] == "candidate"
    assert math.isclose(result["link_lengths_mm"]["thigh_hip_to_knee"], 210.0, abs_tol=1e-6)
    assert math.isclose(result["link_lengths_mm"]["shin_knee_to_wheel"], 250.0, abs_tol=1e-6)
    # a knee that does not coincide must be reported, never patched up
    shin_bad = dict(shin, origins_mm=[[0.0, 5.0, 215.0], [0.0, 250.0, 210.0]])
    assert _planar_candidates([thigh, shin_bad])["status"] == "inconclusive"


def test_missing_file_is_reported(tmp_path):
    with pytest.raises(FileNotFoundError):
        audit_step(tmp_path / "nope.step")


def _part(name: str, rep: int, groups: list[tuple[float, float, float, int]]) -> dict:
    return {
        "rep": rep,
        "candidate_parts": [name],
        "cylinder_count": sum(group[3] for group in groups),
        "dominant_axes": [
            {"origin_mm": [0.0, y, z], "max_radius_mm": radius, "count": count, "axis": [-1.0, 0.0, 0.0]}
            for y, z, radius, count in groups
        ],
    }


def test_shared_axis_nodes_need_two_parts_on_the_same_point():
    selected = [
        _part("thigh", 1, [(70.049989, 2.417291, 43.0, 3), (-88.921958, 139.632159, 31.0, 5)]),
        _part("shin", 2, [(-88.921958, 139.632159, 29.0, 6), (-220.597818, -72.880117, 23.5, 2)]),
        _part("bolt", 3, [(-88.921958, 139.632159, 19.5, 23)]),
    ]
    result = _shared_axis_nodes(selected)
    assert result["status"] == "shared_datum_candidates"
    # hip and wheel appear in one part only; the knee is shared by all three
    assert len(result["shared_nodes"]) == 1
    node = result["shared_nodes"][0]
    assert node["part_count"] == 3
    assert node["point_mm"] == pytest.approx([-88.921958, 139.632159], abs=1e-6)
    assert node["spread_mm"] == pytest.approx(0.0, abs=1e-9)
    assert [part["part"] for part in node["parts"]] == ["thigh", "shin", "bolt"]


def test_shared_axis_nodes_are_ordered_and_tolerant():
    selected = [
        _part("a", 1, [(0.0, 0.0, 10.0, 2), (50.0, 0.0, 9.0, 2)]),
        _part("b", 2, [(0.0, 0.0004, 8.0, 2), (50.0, 0.0, 9.0, 2)]),
        _part("c", 3, [(0.0, 0.0, 7.0, 2)]),
    ]
    result = _shared_axis_nodes(selected)
    assert [node["part_count"] for node in result["shared_nodes"]] == [3, 2]
    # a point that is 0.4 microns off still counts, and the spread is reported
    assert result["shared_nodes"][0]["spread_mm"] == pytest.approx(0.0004, abs=1e-9)
    # a tolerance tighter than the offset must split the cluster
    strict = _shared_axis_nodes(selected, tolerance_mm=1e-6)
    assert [node["part_count"] for node in strict["shared_nodes"]] == [2, 2]
    assert strict["shared_nodes"][0]["point_mm"] == pytest.approx([0.0, 0.0], abs=1e-9)
    assert strict["shared_nodes"][1]["point_mm"] == pytest.approx([50.0, 0.0], abs=1e-9)


def test_shared_axis_nodes_report_absent_geometry():
    result = _shared_axis_nodes([])
    assert result["status"] == "no_geometry"
    assert result["shared_nodes"] == []
