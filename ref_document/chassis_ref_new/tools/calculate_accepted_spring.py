"""Offline catalogue calculation; never connects to the board or changes policy assets."""
from pathlib import Path
import bisect
import hashlib
import json
import math
import re

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def main():
    source = ROOT / "real_data_collection/identification/manufacturer_bg450/spring_curve_reference.json"
    decisions_path = ROOT / "real_data_collection/model_input_decisions_20261117.json"
    decisions = read(decisions_path)["spring"]
    assert decisions["nominal_pressure_mpa"] == 15
    assert decisions["horizontal_axis_unit_for_modeling"] == "mm"
    curve = next(c for c in read(source)["curves"] if c["catalogue_pressure_MPa"] == 15)
    # Preserve digitization noise; do not silently refit a gas law or manufacture a return loop.
    points = [(float(p["x_axis_value"]), float(p["force_n"])) for p in curve["points"]]
    xs = [p[0] for p in points]
    assert all(a < b for a, b in zip(xs, xs[1:]))
    stroke_mm = 80.0

    def force(x):
        if not 0 <= x <= stroke_mm:
            raise ValueError("compression outside the accepted 0..80 mm stroke")
        i = max(0, min(len(points) - 2, bisect.bisect_right(xs, x) - 1))
        x0, f0 = points[i]
        x1, f1 = points[i + 1]
        return f0 + (f1 - f0) * (x - x0) / (x1 - x0)

    for x, f in points:
        if x <= stroke_mm:
            assert abs(force(x) - f) < 1e-9
    for x in (-1, 81):
        try:
            force(x)
        except ValueError:
            pass
        else:
            raise AssertionError("stroke bounds not enforced")

    # Read only specific numeric CAD fields, without requiring a simulator/YAML installation.
    geometry_path = ROOT.parent / "fifty_mujoco/configs/mechanism.yaml"
    text = geometry_path.read_text(encoding="utf-8-sig")

    def node(name):
        match = re.search(r"^  " + name + r":\s*\n\s+yz_mm:\s*(\[[^\]]+\])", text, re.M)
        if not match:
            raise ValueError("missing CAD node " + name)
        return json.loads(match[1])

    hip, knee, wheel = (node(n) for n in ("HIP", "KNEE", "WHEEL"))
    block = text.split("  gas_spring:", 1)[1].split("  free_ends:", 1)[0]
    upper, lower = [json.loads(v) for v in re.findall(r"point_yz_mm:\s*(\[[^\]]+\])", block)]
    offsets = [float(v) for v in re.findall(r"pin_axis_offset_mm:\s*([0-9.]+)", block)]
    assert offsets and max(offsets) == min(offsets)
    axial = offsets[0]
    thigh = math.dist(hip, knee)
    angle_h = math.atan2(knee[1] - hip[1], knee[0] - hip[0])
    angle_s = math.atan2(wheel[1] - knee[1], wheel[0] - knee[0])

    def rotate(a, p):
        c, s = math.cos(a), math.sin(a)
        return c*p[0] - s*p[1], s*p[0] + c*p[1]

    u = rotate(-angle_h, (upper[0] - hip[0], upper[1] - hip[1]))
    v = rotate(-angle_s, (lower[0] - knee[0], lower[1] - knee[1]))

    def geometry(delta):
        vx, vy = rotate(delta, v)
        a, b = thigh - u[0], -u[1]
        length = math.sqrt((a + vx)**2 + (b + vy)**2 + axial**2)
        derivative = (-a*vy + b*vx) / length
        return length, derivative

    training_path = ROOT.parent / "fifty_mujoco/runs/realdata_20261117/evidence_and_spring_calculation.json"
    training = read(training_path)
    old = training["spring_candidate"]
    free_mm = old["free_pin_distance_m"] * 1000
    assembly_delta = (angle_s - angle_h) % (2*math.pi)
    assembly_length = geometry(assembly_delta)[0]
    assert abs(assembly_length - training["geometry"]["cad_pin_at_assembly_m"]*1000) < 1e-5
    pose_rows = []
    for delta in (assembly_delta, *[math.radians(v) for v in (30, 60, 90, 120, 150)]):
        length, derivative = geometry(delta)
        eps = 1e-6
        numerical = (geometry(delta + eps)[0] - geometry(delta - eps)[0]) / (2*eps)
        assert abs(derivative - numerical) < 1e-6
        compression = free_mm - length
        valid = 0 <= compression <= stroke_mm
        f = force(compression) if valid else None
        pose_rows.append({"knee_flexion_deg": math.degrees(delta),
                          "knee_included_deg": 180 - math.degrees(delta),
                          "pin_distance_mm": length, "compression_mm_if_free_pin_250mm": compression,
                          "within_stroke_under_provisional_free_pin": valid,
                          "force_n": f, "geometric_lever_mm": abs(derivative),
                          "signed_torque_flexion_nm": f*derivative/1000 if valid else None})

    dead = old["stroke_m"] / (1 - old["progression_at_full_stroke"]**(-1/old["polytropic_exponent"]))
    rows = []
    for x in range(81):
        old_f = old["initial_force_n"] * (dead / (dead-x/1000))**old["polytropic_exponent"]
        rows.append({"compression_mm": x, "force_n": force(x),
                     "pin_distance_mm_if_free_pin_250mm": free_mm-x,
                     "old_frozen_training_elastic_force_n_at_same_compression": old_f})
    work = sum((rows[i]["force_n"]+rows[i+1]["force_n"])*.5*.001 for i in range(80))
    sampled_knots = [(x, f) for x, f in points if x <= stroke_mm]
    knot_work = sum((a[1]+b[1])*.5*(b[0]-a[0])*.001 for a, b in zip(sampled_knots, sampled_knots[1:]))
    assert abs(work - knot_work) < 1e-10

    report = {
        "kind": "offline_calculation_under_explicit_user_assumptions",
        "nominal_pressure_mpa": 15, "nominal_pressure_bar": 150,
        "actual_charge_independently_measured": False,
        "horizontal_axis_unit": "mm", "stroke_mm": stroke_mm,
        "catalogue_curve_accepted_as_installed_by_user": True,
        "curve_source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "decisions_source_sha256": hashlib.sha256(decisions_path.read_bytes()).hexdigest(),
        "CAD_geometry_source_sha256": hashlib.sha256(geometry_path.read_bytes()).hexdigest(),
        "source_digitization_uncertainty_n_approx": read(source)["digitization_uncertainty_n_approx"],
        "method": "piecewise linear interpolation of archived digitized 15 MPa curve; no extrapolation beyond 80 mm",
        "source_points_beyond_80mm_not_used": [p for p in points if p[0] > stroke_mm],
        "source_digitization_noise_preserved": True,
        "force_0mm_n": force(0), "force_80mm_n": force(80),
        "force_progression_80_over_0": force(80)/force(0),
        "compression_work_0_to_80mm_j_per_spring": work,
        "comparison_to_frozen_training_at_80mm": {
            "training_force_n": rows[-1]["old_frozen_training_elastic_force_n_at_same_compression"],
            "catalogue_minus_training_fraction": force(80)/rows[-1]["old_frozen_training_elastic_force_n_at_same_compression"]-1},
        "conditional_pin_mapping": {"free_pin_distance_mm": free_mm,
            "status": "retained provisional training assumption, not proven by bare spring L=205mm",
            "equation": "compression_mm = free_pin_distance_mm - installed_pin_distance_mm",
            "torque_equation": "tau_flexion = force_n * d(pin_distance_m)/d(flexion_rad)",
            "pose_rows": pose_rows},
        "rows": rows,
        "return_hysteresis_and_damping_identified": False,
        "frozen_model_weights_or_results_changed": False,
        "firmware_output_or_calibration_changed": False,
        "hardware_accessed": False,
        "checks": {"catalogue_knots": "pass", "stroke_bounds": "pass", "CAD_assembly_distance": "pass",
                   "analytic_lever_vs_finite_difference": "pass", "work_integral": "pass"}}
    target = ROOT / "real_data_collection/spring_nominal_15mpa_mm_20261117.json"
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"report": str(target), "F0_N": force(0), "F80_N": force(80),
                      "work_J": work, "CAD_assembly": pose_rows[0],
                      "old_force_at_80_N": report["comparison_to_frozen_training_at_80mm"]["training_force_n"]},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
