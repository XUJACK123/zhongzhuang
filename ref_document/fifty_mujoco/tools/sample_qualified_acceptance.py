#!/usr/bin/env python3
"""Score frozen, disjoint balance heldout reports without changing thresholds."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from sample_qualified_evaluate import CLASSES, wilson_interval, PROJECT


def score(report, protocol, split, frozen):
    spec = protocol[split]
    expected = {spec["seed_base"]+i*protocol["class_stride"]+j
                for i in range(len(CLASSES)) for j in range(spec["seeds_per_class"])}
    rows = report["rows"]
    seeds = [r["seed"] for r in rows]
    provenance = (report["checkpoint_sha256"] == frozen["checkpoint_sha256"] and
                  report["firmware_sha256"] == frozen["firmware_sha256"] and
                  report["domain"] == split and report["duration_s"] == spec["duration_s"])
    complete = report["complete"] and len(seeds) == len(set(seeds)) and set(seeds) == expected
    classes = {}
    limits = json.loads((PROJECT/protocol["metric_thresholds_source"]).read_text())["thresholds"]
    wins = failures = 0
    combined = {"vx_squared": 0., "yaw_squared": 0., "clearance": 0., "samples": 0}
    for class_index, name in enumerate(CLASSES):
        group = [r for r in rows if r["scenario"] == name]
        expected_class = {spec["seed_base"]+class_index*protocol["class_stride"]+j
                          for j in range(spec["seeds_per_class"])}
        complete = complete and {r["seed"] for r in group} == expected_class
        passed = 0
        for r in group:
            params = r["parameters"]
            gains = params["motor_gain"]
            nominal = gains == [1.]*6 and params["effective_friction"] == .75 and params["spring_scale"] == 1.
            inside = (len(gains) == 6 and gains[2] == gains[5] == 1. and
                      all(.8 <= gains[i] <= 1. for i in (0, 1, 3, 4)) and
                      .6 <= params["effective_friction"] <= .9 and .9 <= params["spring_scale"] <= 1.1)
            expected_nominal = split == "nominal" or r["seed"] % 5 == 0
            if r["domain"] != split or not inside or nominal != expected_nominal:
                raise ValueError("episode parameters differ from the frozen domain")
            m = r["metrics"]
            finite = all(isinstance(m.get(k), (float, int)) and math.isfinite(m[k]) for k in (
                "point_vx_rmse_m_s", "yaw_rate_rmse_rad_s", "clearance_mae_m", "roll_abs_p95_deg", "pitch_abs_p95_deg"))
            saturation = r["motor_saturation_fractions"]
            finite = finite and len(saturation) == 6 and all(math.isfinite(x) and 0 <= x <= 1 for x in saturation)
            ok = bool(finite and r["completed"] and r["first_failure"] is None and
                      abs(r["elapsed_s"]-spec["duration_s"]) < 1e-6 and m["samples"] > 0 and
                      m["point_vx_rmse_m_s"] <= limits["vx_rmse_max_m_s"] and
                      m["yaw_rate_rmse_rad_s"] <= limits["wz_rmse_max_rad_s"] and
                      m["clearance_mae_m"] <= limits["clearance_mae_max_m"] and
                      max(m["roll_abs_p95_deg"], m["pitch_abs_p95_deg"]) <= limits["tilt_p95_max_deg"] and
                      max(saturation) <= limits["motor_saturation_max_per_motor"])
            if ok != r["thresholds_met"]:
                raise ValueError("independent rescoring disagrees with episode result")
            passed += ok
            failures += not r["completed"]
            if finite:
                n = m["samples"]
                combined["vx_squared"] += n*m["point_vx_rmse_m_s"]**2
                combined["yaw_squared"] += n*m["yaw_rate_rmse_rad_s"]**2
                combined["clearance"] += n*m["clearance_mae_m"]
                combined["samples"] += n
        wins += passed
        classes[name] = {"trials": len(group), "passed": passed, "rate": passed/len(group) if group else 0,
                         "wilson_95": wilson_interval(passed, len(group))}
    rate = wins/len(rows) if rows else 0.
    total = combined["samples"]
    metrics = {"point_vx_rmse_m_s": math.sqrt(combined["vx_squared"]/total),
               "yaw_rate_rmse_rad_s": math.sqrt(combined["yaw_squared"]/total),
               "clearance_mae_m": combined["clearance"]/total} if total else None
    passed = bool(provenance and complete and failures <= protocol["physical_failures_max"] and
                  rate >= protocol["rate_min"] and all(g["rate"] >= protocol["per_class_rate_min"] for g in classes.values()))
    return {"passed": passed, "complete": complete, "provenance_matches": provenance,
            "trials": len(rows), "successes": wins, "rate": rate, "physical_failures": failures,
            "wilson_95": wilson_interval(wins, len(rows)), "classes": classes, "metrics": metrics}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("protocol", "frozen", "nominal", "combined", "out"):
        p.add_argument("--"+name, type=Path, required=True)
    a = p.parse_args()
    if a.out.exists():
        p.error("fresh acceptance report required")
    protocol, frozen = [json.loads(x.read_text()) for x in (a.protocol, a.frozen)]
    if hashlib.sha256(a.protocol.read_bytes()).hexdigest() != frozen["protocol_sha256"]:
        raise ValueError("heldout protocol changed after candidate freeze")
    if hashlib.sha256((PROJECT/"tools/sample_qualified_evaluate.py").read_bytes()).hexdigest() != frozen["evaluator_sha256"]:
        raise ValueError("heldout evaluator changed after candidate freeze")
    if hashlib.sha256((PROJECT/protocol["metric_thresholds_source"]).read_bytes()).hexdigest() != frozen["thresholds_sha256"]:
        raise ValueError("heldout metric thresholds changed after candidate freeze")
    reports = {k: json.loads(getattr(a, k).read_text()) for k in ("nominal", "combined")}
    checks = {k: score(v, protocol, k, frozen) for k, v in reports.items()}
    result = {"schema": "qualified_balance_final_acceptance_v1", "checks": checks,
              "passed_limited_simulation_scope": all(c["passed"] for c in checks.values()),
              "checkpoint_sha256": frozen["checkpoint_sha256"], "scope": protocol["scope"],
              "getup_accepted": False, "real_robot_accepted": False,
              "inputs_sha256": {str(x): hashlib.sha256(x.read_bytes()).hexdigest()
                                for x in (a.protocol, a.frozen, a.nominal, a.combined)}}
    a.out.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
