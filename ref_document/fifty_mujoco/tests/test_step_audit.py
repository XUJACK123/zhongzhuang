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

import json
import math

import pytest

from fifty_mujoco.assets.step_audit import (
    VARIANT_PINS,
    _assembly_path,
    _drive_train,
    _gear_teeth,
    _gear_train,
    _line_distance,
    _path_multiplicity,
    _pin_carriers,
    _planar_candidates,
    _predicted_radii,
    _shared_axis_nodes,
    _sprocket_train,
    audit_step,
    classify_body,
    decode_step_string,
    split_top_level,
    sprocket_teeth,
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


# ------------------------------------------------------------------ drive train
def _instance(name: str, parent: str, child: str, translation=(0.0, 0.0, 0.0)) -> dict:
    return {
        "usage_id": int(name.replace("NAUO", "")) if name.startswith("NAUO") else 0,
        "name": name,
        "parent": parent,
        "child": child,
        "transform_mm": {
            "translation_mm": list(translation),
            "rotation": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        },
    }


def _cylinder(rep_id: int, origin, axis, radius: float) -> dict:
    return {
        "cylinder": rep_id * 10 + 1,
        "rep": rep_id,
        "origin_mm": list(origin),
        "axis": list(axis),
        "radius_mm": radius,
    }


def test_assembly_path_walks_up_and_flags_ambiguity():
    instances = [
        _instance("NAUO1", "root", "module"),
        _instance("NAUO2", "module", "motor"),
        _instance("NAUO3", "second parent", "motor"),
    ]
    path = _assembly_path(instances, "motor")
    assert path["ambiguous"] is True
    assert path["path"][0] in {"module", "second parent"}
    single = _assembly_path([instances[0], instances[1]], "motor")
    assert single == {"path": ["module", "root"], "ambiguous": False}


def test_installed_count_multiplies_the_parent_chain():
    """A motor inside a twice-instantiated gearbox is installed twice."""
    instances = [
        _instance("NAUO1", "root", "hip"),
        _instance("NAUO2", "hip", "gearbox"),
        _instance("NAUO3", "hip", "gearbox"),
        _instance("NAUO4", "gearbox", "motor"),
    ]
    result = _path_multiplicity(instances, "motor")
    assert result["count"] == 2
    assert result["ambiguous"] is False


def test_drive_train_counts_only_motors_inside_the_wheel_leg_module():
    instances = [
        _instance("NAUO1", "root", "英雄髋关节总装"),
        *[_instance(f"NAUO{i}", "英雄髋关节总装", "【电机】DM-8009") for i in range(2, 6)],
        _instance("NAUO6", "英雄髋关节总装", "#NW_GEARBOX"),
        _instance("NAUO7", "英雄髋关节总装", "#NW_GEARBOX"),
        _instance("NAUO8", "#NW_GEARBOX", "M3508_WithGear"),
        _instance("NAUO9", "M3508_WithGear", "MOTOR_M3508_Without Gearbox"),
        _instance("NAUO10", "M3508_WithGear", "STD_M3508_Gear_MKIV_III_corrected"),
        _instance("NAUO11", "26英雄云台Pitch轴", "RoboMaster M3508直流无刷电机_80版本"),
    ]
    by_rep = {
        1: [_cylinder(1, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 49.0)],
        2: [_cylinder(2, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 21.0)],
    }
    names = {1: ["【电机】DM-8009"], 2: ["MOTOR_M3508_Without Gearbox"]}
    report = _drive_train(instances, by_rep, names)
    assert report["motor_count"] == 6
    counts = {motor["product"]: motor["count"] for motor in report["motors"]}
    assert counts == {"【电机】DM-8009": 4, "MOTOR_M3508_Without Gearbox": 2}
    reasons = {entry["product"]: entry["rejected_because"] for entry in report["excluded_motors"]}
    assert reasons["M3508_WithGear"] == "sub-assembly"
    assert reasons["STD_M3508_Gear_MKIV_III_corrected"] == "drivetrain part"
    assert reasons["RoboMaster M3508直流无刷电机_80版本"] == "outside the wheel-leg module"
    assert report["gaps"], "the section must state what STEP cannot answer"


def test_gas_spring_pin_distance_is_decomposed():
    instances = [_instance("NAUO1", "root", "hip")]
    axis = (0.0, 0.75724918, -0.653126082)
    by_rep = {
        1: [_cylinder(1, (155.43, -105.72, 89.90), axis, 6.0)],
        2: [_cylinder(2, (155.43, 15.92, -15.01), (-axis[0], -axis[1], -axis[2]), 3.0)],
        3: [_cylinder(3, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 8.0)],
        4: [
            _cylinder(
                4,
                (11.0, 100.0 * axis[1], 100.0 * axis[2]),
                (-1.0, 0.0, 0.0),
                8.0,
            )
        ],
    }
    names = {
        1: ["1BKB0.45-75氮气弹簧 缸体"],
        2: ["1BKB0.45-75氮气弹簧 连杆"],
        3: ["【复用交C件】上部弹簧套"],
        4: ["【复用交C件】下部弹簧套"],
    }
    spring = _drive_train(instances, by_rep, names)["gas_spring"]
    assert spring["body_rod_line"]["angle_deg"] == pytest.approx(0.0, abs=1e-6)
    pins = spring["pin_to_pin"]
    assert pins["along_spring_axis_mm"] == pytest.approx(100.0, abs=1e-6)
    assert pins["perpendicular_mm"] == pytest.approx(11.0, abs=1e-6)
    assert pins["center_distance_mm"] == pytest.approx(math.hypot(100.0, 11.0), abs=1e-6)
    assert spring["status"] == "candidate_same_datum"


def test_audit_report_always_carries_the_drive_section(step_file):
    report = audit_step(step_file)
    drive = report["drive_train"]
    assert drive["motors"] == []
    assert drive["motor_count"] == 0
    assert drive["gas_spring"]["status"] == "no_geometry"
    assert drive["gearbox"]["status"] in {"no_gear_names", "inconclusive"}
    assert audit_step(step_file, with_geometry=False).get("drive_train") is None


# ---------------------------------------------------------------- pin carriers
def _axle_part(name: str, rep: int, y: float, z: float, radius: float, x: float = 0.0) -> tuple:
    return (
        rep,
        [
            {
                "cylinder": rep * 10 + index,
                "origin_mm": [x + index * 0.5, y, z],
                "axis": [-1.0, 0.0, 0.0],
                "radius_mm": radius,
            }
            for index in range(2)
        ],
        [name],
    )


def test_body_classification_puts_the_drivetrain_before_the_shin():
    assert classify_body("【碳板】15mm-小腿_默认") == "shin"
    assert classify_body("【碳板】18mm-大腿-V2-镜像_默认") == "thigh"
    # "小腿传动中部连杆" contains 小腿 but is a link, not the shin body
    assert classify_body("【碳板】8mm-小腿传动中部连杆_默认") == "drivetrain"
    assert classify_body("【复用交C件】节点轴承座_默认") == "drivetrain"
    assert classify_body("【深沟球轴承】F688_默认") == "drivetrain"
    assert classify_body("轮腿镜像_默认") == "wheel"
    assert classify_body("装甲板_默认") == "other"


def test_pin_carriers_identify_the_variant_from_a_full_scan():
    """A Ø24 seat on a thigh plate must not hide behind the hip/knee bosses."""
    link_a_y, link_a_z = VARIANT_PINS["LINK_PIN_A"]
    end_a_y, end_a_z = VARIANT_PINS["END_PIN_A"]
    parts = [
        _axle_part("【碳板】8mm-小腿传动中部连杆_默认", 1, link_a_y, link_a_z, 12.0),
        _axle_part("【碳板】18mm-大腿-V2-镜像_默认", 2, link_a_y, link_a_z, 17.5, x=-210.9),
        _axle_part("【复用交C件】节点轴承座_默认", 3, link_a_y, link_a_z, 11.5, x=158.9),
        _axle_part("【碳板】15mm-小腿_默认", 4, end_a_y, end_a_z, 12.0, x=177.9),
        _axle_part("【6.8】小腿传动末端连杆_默认", 5, end_a_y, end_a_z, 12.0, x=165.4),
    ]
    by_rep = {rep: cyl for rep, cyl, _ in parts}
    names = {rep: name for rep, _, name in parts}
    report = _pin_carriers(by_rep, names)
    assert report["variant_supported"] == "link_a_on_thigh"
    link_a = report["pins"]["LINK_PIN_A"]
    assert "thigh" in link_a["by_body"] and "shin" not in link_a["by_body"]
    assert link_a["parts"][0]["max_radius_mm"] == pytest.approx(17.5)  # largest first
    end_a = report["pins"]["END_PIN_A"]
    assert "shin" in end_a["by_body"] and "thigh" not in end_a["by_body"]


def test_pin_carriers_do_not_guess_when_both_plates_pin_the_same_node():
    link_a_y, link_a_z = VARIANT_PINS["LINK_PIN_A"]
    end_a_y, end_a_z = VARIANT_PINS["END_PIN_A"]
    parts = [
        _axle_part("【碳板】18mm-大腿-V2-镜像_默认", 1, link_a_y, link_a_z, 17.5),
        _axle_part("【碳板】15mm-小腿_默认", 2, link_a_y, link_a_z, 12.0),
        _axle_part("【碳板】15mm-小腿_默认", 3, end_a_y, end_a_z, 12.0),
    ]
    report = _pin_carriers(
        {rep: cyl for rep, cyl, _ in parts}, {rep: name for rep, _, name in parts}
    )
    assert report["variant_supported"] is None
    assert report["pins"]["LINK_PIN_A"]["status"] == "shared"


def test_pin_carriers_ignore_bores_that_are_not_parallel_to_x():
    link_a_y, link_a_z = VARIANT_PINS["LINK_PIN_A"]
    by_rep = {
        1: [{"cylinder": 11, "origin_mm": [0.0, link_a_y, link_a_z],
             "axis": [0.0, 0.0, 1.0], "radius_mm": 20.0}],
    }
    report = _pin_carriers(by_rep, {1: ["【碳板】18mm-大腿-V2-镜像_默认"]})
    assert report["pins"]["LINK_PIN_A"]["status"] == "not_found"


# ------------------------------------------------------------------ gear train
def _gear_part(rep: int, name: str, radii: list[float]) -> tuple[int, list[dict], list[str]]:
    cylinders = [
        {"cylinder": rep * 10 + index, "origin_mm": [0.0, 0.0, 0.0],
         "axis": [0.0, 1.0, 0.0], "radius_mm": radius}
        for index, radius in enumerate(radii)
    ]
    return rep, cylinders, [name]


def test_gear_teeth_are_read_from_the_part_name():
    assert _gear_teeth(["STD_DualSpurGear_M0.6xT14_M0.6xT46_MKIV_III_开源_默认"]) == [
        {"module_mm": 0.6, "teeth": 14, "internal": False},
        {"module_mm": 0.6, "teeth": 46, "internal": False},
    ]
    assert _gear_teeth(["STD_InternalSpurGear_M0.6xT77_MKIV_III_开源_默认"]) == [
        {"module_mm": 0.6, "teeth": 77, "internal": True}
    ]
    assert _gear_teeth(["STD_M3508_Gear_MKIV_III_corrected_开源_默认"]) == []


def test_predicted_radii_match_the_published_gear_geometry():
    assert _predicted_radii(0.6, 46, False)["tip_mm"] == pytest.approx(14.4)
    assert _predicted_radii(0.6, 77, True)["root_mm"] == pytest.approx(23.85)
    assert _predicted_radii(0.6, 17, False)["tip_mm"] == pytest.approx(5.7)


def test_compound_planetary_derives_the_sun_and_the_ratio():
    """Mk4: sun 17T (not in its name), planet 14/46, ring 77 -> ~15.9:1."""
    parts = [
        _gear_part(1, "STD_DualSpurGear_M0.6xT14_M0.6xT46_MKIV_III_开源", [5.52, 14.4]),
        _gear_part(2, "STD_InternalSpurGear_M0.6xT77_MKIV_III_开源", [25.0, 23.851]),
        _gear_part(3, "STD_M3508_Gear_MKIV_III_corrected_开源", [5.79]),
    ]
    by_rep = {rep: cyl for rep, cyl, _ in parts}
    names = {rep: name for rep, _, name in parts}
    report = _gear_train(by_rep, names)
    assert report["status"] == "candidate"
    assert report["ring_teeth"] == 77
    assert report["planet_teeth"] == [14, 46]
    assert report["sun_teeth_derived"] == 17
    assert report["centre_distance"]["equal"] is True
    assert report["centre_distance"]["planet_ring_mm"] == pytest.approx(18.9)
    assert report["sun_radius_check"]["relative_error"] < 0.02
    first, second = report["ratio"]["arrangements"]
    assert first["sun_meshes"] == 46
    assert first["ring_fixed_carrier_output"] == pytest.approx(15.8824, abs=1e-4)
    assert second["ring_fixed_carrier_output"] == pytest.approx(2.3785, abs=1e-4)


def test_a_ring_radius_mismatch_is_reported_not_smoothed():
    parts = [
        _gear_part(1, "STD_DualSpurGear_M0.6xT14_M0.6xT46_MKIV_III_开源", [14.4]),
        # a 60-tooth ring labelled 77 does not match the predicted root radius
        _gear_part(2, "STD_InternalSpurGear_M0.6xT77_MKIV_III_开源", [18.0]),
    ]
    by_rep = {rep: cyl for rep, cyl, _ in parts}
    names = {rep: name for rep, _, name in parts}
    report = _gear_train(by_rep, names)
    ring = next(gear for gear in report["gears"] if gear["internal"])
    assert ring["radius_confirmed"] is False
    assert ring["radius_error_mm"] > 0.05


def test_gear_train_reports_missing_inputs():
    assert _gear_train({}, {})["status"] == "no_gear_names"
    only_ring = _gear_part(1, "STD_InternalSpurGear_M0.6xT77_MKIV_III_开源", [23.851])
    report = _gear_train({only_ring[0]: only_ring[1]}, {only_ring[0]: only_ring[2]})
    assert report["status"] == "inconclusive"


def test_the_selected_gear_arrangement_is_the_carrier_fixed_reduction():
    """2026-10-31: carrier fixed, ring output -> 14.88:1, not 15.88 and not 1.38."""
    parts = [
        _gear_part(1, "STD_DualSpurGear_M0.6xT14_M0.6xT46_MKIV_III_开源", [5.52, 14.4]),
        _gear_part(2, "STD_InternalSpurGear_M0.6xT77_MKIV_III_开源", [25.0, 23.851]),
        _gear_part(3, "STD_M3508_Gear_MKIV_III_corrected_开源", [5.79]),
    ]
    report = _gear_train(
        {rep: cyl for rep, cyl, _ in parts}, {rep: name for rep, _, name in parts}
    )
    configuration = report["ratio"]["configuration"]
    assert configuration["fixed"] == "carrier"
    assert configuration["output"] == "ring"
    assert configuration["reduction"] == pytest.approx(14.8824, abs=1e-4)
    # the gearbox is modelled but the operator says it is not in the wheel path
    assert configuration["in_wheel_power_path"] is False
    assert report["sun_teeth_matches_cad"] is True


# ------------------------------------------------------------------- sprockets
def _sprocket_part(rep: int, name: str, centre: tuple[float, float],
                   pitch_radius: float, teeth: int,
                   inner_radius: float = 21.5, inner_count: int = 12) -> tuple:
    cylinders = []
    for index in range(teeth):
        angle = 2.0 * math.pi * index / teeth
        cylinders.append(
            {
                "cylinder": rep * 100 + index,
                "origin_mm": [
                    0.0,
                    centre[0] + pitch_radius * math.cos(angle),
                    centre[1] + pitch_radius * math.sin(angle),
                ],
                "axis": [1.0, 0.0, 0.0],
                "radius_mm": 4.0,
            }
        )
    # the lightening/bolt holes that used to be mistaken for teeth
    for index in range(inner_count):
        angle = 2.0 * math.pi * index / inner_count
        cylinders.append(
            {
                "cylinder": rep * 100 + 50 + index,
                "origin_mm": [
                    0.0,
                    centre[0] + inner_radius * math.cos(angle),
                    centre[1] + inner_radius * math.sin(angle),
                ],
                "axis": [-1.0, 0.0, 0.0],  # the same bore on the other face
                "radius_mm": 1.25,
            }
        )
    # the boss itself, so _dominant_axes finds the centre
    cylinders.append(
        {
            "cylinder": rep * 100 + 99,
            "origin_mm": [0.0, centre[0], centre[1]],
            "axis": [1.0, 0.0, 0.0],
            "radius_mm": pitch_radius + 4.0,
        }
    )
    return rep, cylinders, [name]


def test_sprocket_teeth_are_counted_and_the_chain_pitch_confirms_them():
    """1/2 in chain: 16 T -> 32.55 mm, 19 T -> 38.58 mm."""
    pitch = 12.7
    for teeth in (16, 19):
        radius = pitch / (2.0 * math.sin(math.pi / teeth))
        rep, cylinders, names = _sprocket_part(
            teeth, f"主动链轮_T{teeth}", (35.05, 97.159), radius, teeth
        )
        counted = sprocket_teeth(cylinders)
        assert counted["teeth"] == teeth
        assert counted["status"] == "confirmed"
        assert counted["standard_pitch_mm"] == pytest.approx(pitch)
        assert counted["implied_chain_pitch_mm"] == pytest.approx(pitch, abs=0.01)


def test_bolt_holes_are_not_counted_as_teeth():
    """12 holes at 30 deg look like a tooth ring but imply a 11.1 mm pitch."""
    rep, cylinders, names = _sprocket_part(
        1, "主动链轮", (35.05, 97.159), 38.58, 19, inner_radius=21.5, inner_count=12
    )
    counted = sprocket_teeth(cylinders)
    assert counted["teeth"] == 19  # the real ring, not the 12 holes
    rejected = {item["teeth"] for item in counted.get("rejected_rings", [])}
    assert 12 in rejected


def test_anti_parallel_bore_axes_are_still_counted():
    """The same bore written as -X must not be dropped; that hid a whole ring."""
    rep, cylinders, names = _sprocket_part(1, "链轮", (0.0, 0.0), 32.55, 16)
    flipped = [dict(entry, axis=[-1.0, 0.0, 0.0]) for entry in cylinders]
    assert sprocket_teeth(flipped)["teeth"] == 16
    assert sprocket_teeth(cylinders)["teeth"] == 16


def test_the_chain_ratio_uses_the_driving_and_driven_tooth_counts():
    parts = [
        _sprocket_part(1, "【复用交C件】-7075-大腿（内输出）主动链轮_默认", (35.05, 97.159), 32.549, 16),
        _sprocket_part(2, "【复用交C件】-7075-大腿从动链轮_默认", (70.05, 2.417), 38.583, 19),
    ]
    report = _sprocket_train(
        {rep: cyl for rep, cyl, _ in parts}, {rep: name for rep, _, name in parts}
    )
    drive = next(entry for entry in report["drives"] if entry["body"] == "thigh")
    assert drive["driving_teeth"] == 16
    assert drive["driven_teeth"] == 19
    assert drive["ratio"] == pytest.approx(19.0 / 16.0)
    assert drive["status"] == "confirmed"


def test_a_sprocket_without_a_driven_counterpart_is_reported_incomplete():
    rep, cylinders, names = _sprocket_part(1, "主动链轮", (0.0, 0.0), 32.549, 16)
    report = _sprocket_train({rep: cylinders}, {rep: names})
    assert report["drives"][0]["status"] == "incomplete"


def test_the_counted_sprocket_can_be_serialised():
    """The winning ring is also inside ``candidates``; attaching the list to it
    used to make the record contain itself, so ``--json`` blew up with
    'Circular reference detected' after a full 7 s audit."""
    rep, cylinders, names = _sprocket_part(1, "主动链轮", (0.0, 0.0), 32.549, 16)
    counted = sprocket_teeth(cylinders)
    assert counted["teeth"] == 16
    text = json.dumps(_sprocket_train({rep: cylinders}, {rep: names}))
    assert '"teeth": 16' in text
