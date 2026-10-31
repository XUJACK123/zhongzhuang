#!/usr/bin/env python3
"""Standalone STEP audit: standard library only.

The project's normal entry point is ``fifty-mujoco step-audit``, but the CLI
module imports numpy at import time.  This script exists so the STEP route can
be reproduced on any machine with a bare CPython, without installing MuJoCo,
torch or numpy - which is exactly the situation on a CAD workstation.

Usage::

    python tools/step_audit_standalone.py                       # search the workspace
    python tools/step_audit_standalone.py --step D:\\path\\a.STEP --json reports/x.json
    python tools/step_audit_standalone.py --no-geometry         # structure only, faster
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC = PROJECT_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from fifty_mujoco.assets.step_audit import audit_step, summarize  # noqa: E402


def candidates() -> list[Path]:
    workspace = PROJECT_ROOT.parents[1]  # <workspace>/ref_document/fifty_mujoco
    names = ("26英雄整车总装2.0.STEP", "26英雄整车总装.STEP")
    roots = [workspace / "solidwork_graph", workspace.parents[1] / "solidwork_graph"]
    return [root / name for root in roots for name in names]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="STEP assembly + cylinder-axis audit (stdlib only)")
    parser.add_argument("--step", default=None, help="STEP file (default: search known locations)")
    parser.add_argument("--json", default=None, help="output path (default: reports/step_geometry_audit.json)")
    parser.add_argument("--no-geometry", action="store_true", help="structure only")
    args = parser.parse_args(argv)

    path = Path(args.step) if args.step else next(
        (candidate for candidate in candidates() if candidate.is_file()), None
    )
    if path is None or not path.is_file():
        print("STEP not found. Tried:")
        for candidate in candidates():
            print(f"  {candidate}")
        return 2

    report = audit_step(path, with_geometry=not args.no_geometry)
    report["generated_at"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
    report["summary"] = summarize(report)
    out = Path(args.json) if args.json else PROJECT_ROOT / "reports" / "step_geometry_audit.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    counts = report["counts"]
    print(f"file      : {path.name} ({report['source']['bytes']:,} bytes)")
    print(f"sha256    : {report['source']['sha256']}")
    print(f"products  : {counts.get('PRODUCT')}")
    print(f"instances : {counts.get('NEXT_ASSEMBLY_USAGE_OCCURRENCE')} "
          f"(transforms parsed: {sum(1 for i in report['instances'] if 'transform_mm' in i)})")
    geometry = report.get("geometry") or {}
    if geometry:
        diagnostics = geometry["diagnostics"]
        print(f"cylinders : {diagnostics['cylinders']} "
              f"(attributed: {diagnostics['attributed_cylinders']})")
        for entry in geometry["measurements"]:
            distance = entry.get("distance_mm")
            value = f"{distance:10.3f} mm" if distance is not None else "  unmeasured"
            parts = ", ".join(entry.get("candidate_parts") or []) or "-"
            print(f"  {value}  {entry['label']}  [{entry['status']}]  {parts}")
        for entry in geometry.get("cross_part", []):
            print(f"  {'unmeasured':>13}  {entry['label']}  [{entry['status']}]")
        shared = geometry.get("shared_axis_nodes") or {}
        nodes = shared.get("shared_nodes") or []
        print(f"shared    : {len(nodes)} axis nodes appear in >= "
              f"{shared.get('min_parts', 2)} parts (tolerance {shared.get('tolerance_mm')} mm)")
        for node in nodes[:8]:
            y, z = node["point_mm"]
            parts = ", ".join(part["part"] for part in node["parts"][:4])
            print(f"  YZ=({y:9.3f}, {z:9.3f})  parts={node['part_count']:<2}  {parts}")
    drive = report.get("drive_train") or {}
    if drive:
        print(f"drive     : {drive['motor_count']} drive motors inside a "
              f"{drive['module_assembly']} module "
              f"({len(drive['excluded_motors'])} more name matches elsewhere)")
        for motor in drive["motors"]:
            print(f"  {motor['count']:2d} x {motor['product'][:44]}")
            for placement in motor["placements"]:
                moment = placement.get("translation_mm") or [0.0, 0.0, 0.0]
                print(f"       {placement['instance']:8s} parent={placement['parent'][:26]:26s} "
                      f"t=({moment[0]:8.2f},{moment[1]:8.2f},{moment[2]:8.2f})")
        spring = drive.get("gas_spring") or {}
        pins = spring.get("pin_to_pin") or {}
        if pins:
            print(f"spring    : pin-to-pin {pins['center_distance_mm']:.3f} mm "
                  f"(along axis {pins['along_spring_axis_mm']:.3f}, "
                  f"off axis {pins['perpendicular_mm']:.3f}) [{spring['status']}]")
        else:
            print(f"spring    : {spring.get('status')} (no pin pair measured)")
        gearbox = drive.get("gearbox") or {}
        ratio = gearbox.get("ratio") or {}
        arrangements = ratio.get("arrangements") or []
        if arrangements:
            best = arrangements[0]
            print(f"gearbox   : Z_sun {ratio['sun_teeth']} (derived), planets "
                  f"{gearbox['planet_teeth']}, ring {gearbox['ring_teeth']} -> "
                  f"{best['ring_fixed_carrier_output']:.2f}:1 ring fixed / "
                  f"{best['carrier_fixed_ring_output']:.2f}:1 carrier fixed "
                  f"[{ratio['status']}]")
        else:
            print(f"gearbox   : {gearbox.get('status')} (ratio not derived)")
        sprockets = drive.get("sprocket_train") or {}
        for entry in sprockets.get("drives") or []:
            if entry.get("ratio") is None:
                print(f"chain     : {entry['body']:5s} incomplete "
                      f"(driving {entry.get('driving_teeth')}, "
                      f"driven {entry.get('driven_teeth')})")
                continue
            print(f"chain     : {entry['body']:5s} {entry['driving_teeth']}T -> "
                  f"{entry['driven_teeth']}T  ratio {entry['ratio']:.4f} "
                  f"[{entry['status']}]")
        wheel = drive.get("wheel_drive") or {}
        if wheel:
            print(f"wheel     : {wheel.get('motor')} "
                  f"[gearbox in path: {wheel.get('gearbox_in_path')}, "
                  f"{wheel.get('status')}]")
            print(f"crank ref : {wheel.get('crank_reference')} "
                  f"(evidence in the JSON report)")
    print(f"status    : {report['status']}")
    print(f"report    : {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
