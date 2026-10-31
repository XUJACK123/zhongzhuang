"""Assemble the ``mechanism-check`` report.

The report separates three things that are easy to conflate:

1. **arithmetic** - does the declared length table match the CAD coordinates?
2. **serial leg** - does the two-link model, which the MJCF already uses,
   reproduce the CAD hip/knee/wheel points and round-trip through IK?
3. **closed chain** - does the two-loop planar chain close, and only when the
   caller has accepted the candidate body assignment?

A failure in 1 or 2 is a defect. A refusal to attempt 3 is not.
"""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Iterable

from .leg import body_constraints, serial_leg_report, static_closure
from .loader import Mechanism, load_mechanism
from .planar import NoSolutionError, TopologyNotConfirmedError, distance
from .pose import MechanismPose
from .resolve import describe as describe_resolution, resolve_nodes
from .spring import describe, load_spring_link
from .transmission import ThighCrankTransmission

from ..config import load_yaml
from ..paths import configs_dir

#: Half-open sweep around the assembly pose. Small on purpose: the point is to
#: prove the loop closes away from the pose it was measured at, not to explore a
#: workspace whose joint stops are still unknown.
DEFAULT_SWEEP: tuple[tuple[float, float], ...] = (
    (0.0, 0.0),
    (0.05, 0.0),
    (-0.05, 0.0),
    (0.0, 0.05),
    (0.0, -0.05),
    (0.03, 0.03),
    (-0.03, -0.03),
)

#: Grid used to compare the two candidate body assignments.
SWEEP_SPAN_RAD = 0.15
SWEEP_STEP_RAD = 0.01

#: Numerical threshold from progress.md §20.4 item 2.
RESIDUAL_LIMIT_M = 1e-9


def _confirmations(mechanism: Mechanism) -> list[str]:
    items: list[str] = []
    for body in mechanism.bodies.values():
        if body.confirm_in_cad:
            items.append(f"{body.name}: {body.confirm_in_cad}")
    chain = mechanism.topology.get("chain") or {}
    for entry in chain.get("confirm_in_cad") or []:
        items.append(str(entry))
    return items


def _spring_report(mechanism: Mechanism) -> dict[str, Any]:
    """Pin travel the linkage asks for, and what the real spring allows."""
    link = load_spring_link(mechanism)
    if link is None:
        return {"status": "no_anchors"}
    report: dict[str, Any] = {"status": "computed_from_points", **describe(link)}
    spring = load_yaml(configs_dir() / "gas_spring.yaml")
    real = spring.get("real_spring") or {}
    datasheet = real.get("datasheet") or {}
    free = datasheet.get("free_length_mm")
    stroke = datasheet.get("max_working_stroke_mm")
    if free is None or stroke is None:
        report["status"] = "no_datasheet"
        return report
    window = link.window(float(free), float(stroke), float(report["wheel_radius_mm"]))
    report["as_stated"] = window
    conventions = load_yaml(configs_dir() / "joint_conventions.yaml")
    target = conventions["reference_pose"]["target"]
    chord_mm = float(target["chord_m"]) * 1000.0
    nominal_height = float(target["base_height_m"])
    delta = link.delta_for_chord(chord_mm)
    pin = link.pin_distance_mm(delta)
    report["nominal_pin_distance_3d_mm"] = pin
    report["nominal_pin_distance_planar_mm"] = link.planar_pin_distance_mm(delta)
    # the smallest free length that still reaches the nominal height: a gas spring
    # can never be longer than its free length
    report["free_length_required_for_nominal_height_mm"] = pin
    report["compression_at_nominal_height_mm"] = float(free) - pin
    report["remaining_compression_at_nominal_height_mm"] = pin - (float(free) - float(stroke))
    report["reaches_nominal_height"] = bool(
        window["feasible"]
        and window["min_height_m"] - 1e-9 <= nominal_height <= window["max_height_m"] + 1e-9
    )
    report["runtime_profile"] = spring["gas_springs"]["profile"]
    report["real_spring_integrated"] = False
    return report


def topology_gate(mechanism: Mechanism, allow_inferred: bool = False) -> None:
    """Raise unless the candidate body assignment may be used.

    This is the single place that decides whether the planar chain may be turned
    into kinematics. Everything that would feed a controller, a Jacobian or a
    MJCF passes through here.
    """
    if allow_inferred or mechanism.inferred_topology_allowed:
        return
    candidates = [
        body.name for body in mechanism.bodies.values() if body.status == "candidate_inferred"
    ]
    if candidates:
        raise TopologyNotConfirmedError(
            "the planar body assignment is candidate_inferred for "
            + ", ".join(candidates)
            + "; set topology.chain.use_inferred_topology: true or pass"
            " allow_inferred=True after reading the CAD mates"
        )


def _variant_report(mechanism: Mechanism, name: str, pose, constraints) -> dict[str, Any]:
    """Close one candidate assignment over a grid and measure it against every length."""
    nodes = mechanism.assembly_pose_m()
    thigh_cad, crank_cad = pose.cad_angles()
    best_branch = None
    best_error = None
    for branch in range(8):
        try:
            produced = pose.forward(thigh_cad, crank_cad, branch)
        except NoSolutionError:
            continue
        error = max(distance(produced[node], nodes[node]) for node in nodes)
        if best_error is None or error < best_error:
            best_branch, best_error = branch, error
    if best_branch is None:
        return {
            "variant": name,
            "status": "cad_pose_not_reproducible",
            "cad_reproduction_error_m": None,
            "reason": "no branch of this assignment closes at the assembly pose",
        }

    steps = int(round(SWEEP_SPAN_RAD / SWEEP_STEP_RAD))
    grid = [index * SWEEP_STEP_RAD for index in range(-steps, steps + 1)]
    worst_residual = 0.0
    worst_pose = None
    closed = 0
    total = 0
    for d_thigh in grid:
        for d_crank in grid:
            total += 1
            try:
                produced = pose.forward(thigh_cad + d_thigh, crank_cad + d_crank, best_branch)
            except NoSolutionError:
                continue
            closed += 1
            residual = max(
                abs(distance(produced[first], produced[second]) - length)
                for first, second, length in constraints
            )
            if residual > worst_residual:
                worst_residual = residual
                worst_pose = [round(d_thigh, 6), round(d_crank, 6)]
    passed = (
        best_error is not None
        and best_error <= RESIDUAL_LIMIT_M
        and worst_residual <= RESIDUAL_LIMIT_M
    )
    return {
        "variant": name,
        "status": "closes" if passed else "does_not_close",
        "branch": best_branch,
        "cad_reproduction_error_m": best_error,
        "grid": {
            "span_rad": SWEEP_SPAN_RAD,
            "step_rad": SWEEP_STEP_RAD,
            "poses": total,
            "closed": closed,
            "closed_fraction": round(closed / total, 6) if total else 0.0,
        },
        "worst_residual_m": worst_residual,
        "worst_pose_delta": worst_pose,
    }


def mechanism_report(
    mechanism: Mechanism | None = None,
    allow_inferred: bool = False,
    sweep: Iterable[tuple[float, float]] = DEFAULT_SWEEP,
) -> dict[str, Any]:
    """Full report. ``allow_inferred`` is the explicit topology opt-in."""
    mech = mechanism if mechanism is not None else load_mechanism()
    source = (mech.meta.get("source") or {}) if isinstance(mech.meta, dict) else {}
    chain = mech.topology.get("chain") or {}
    del sweep  # the grid below is fixed so two runs can be compared

    gate_message: str | None = None
    try:
        topology_gate(mech, allow_inferred)
        use_inferred = True
    except TopologyNotConfirmedError as error:
        use_inferred = False
        gate_message = str(error)

    closure = static_closure(mech)
    leg = serial_leg_report(mech)

    forwarders = {
        "link_a_on_shin": lambda: MechanismPose.from_mechanism(mech),
        "link_a_on_thigh": lambda: ThighCrankTransmission.from_mechanism(mech),
    }
    variants: list[dict[str, Any]] = []
    if use_inferred:
        for spec in chain.get("variants") or []:
            name = str(spec.get("name"))
            factory = forwarders.get(name)
            if factory is None:
                variants.append(
                    {
                        "variant": name,
                        "status": "not_implemented",
                        "reason": "no kinematics implementation for this body assignment",
                    }
                )
                continue
            constraints = body_constraints(mech, spec.get("bodies"))
            variants.append(_variant_report(mech, name, factory(), constraints))
        closing = [entry for entry in variants if entry["status"] == "closes"]
        closed_chain: dict[str, Any] = {
            "status": "solved" if closing else "does_not_close",
            "attempted": True,
            "variants": variants,
            "closing_variants": [entry["variant"] for entry in closing],
            "selected_variant": chain.get("selected_variant"),
            "chain_status": chain.get("status"),
            "note": (
                "both assignments use the same eight pivots and the same fourteen lengths;"
                " they differ only in which part carries LINK_PIN_A and END_PIN_A"
            ),
        }
    else:
        closed_chain = {
            "status": "not_instantiated",
            "attempted": False,
            "reason": gate_message,
            "chain_status": chain.get("status"),
            "confirm_in_cad": list(chain.get("confirm_in_cad") or []),
            # Evidence available without instantiating anything: the STEP audit
            # names the plate that carries each pin (see docs §6.8).
            "selected_variant": chain.get("selected_variant"),
            "geometry_supports": chain.get("geometry_supports"),
        }

    failures: list[str] = []
    if closure["status"] != "consistent":
        failures.append(
            f"static closure worst error {closure['worst_error_m']:.3e} m exceeds 1e-9 m"
        )
    cad_error = float(leg["assembly_pose"]["cad_reproduction_error_m"])
    if cad_error > RESIDUAL_LIMIT_M:
        failures.append(f"serial leg CAD reproduction error {cad_error:.3e} m exceeds 1e-9 m")
    round_trip = float(leg["round_trip"]["worst_error_m"])
    if round_trip > RESIDUAL_LIMIT_M:
        failures.append(f"serial leg IK/FK round trip {round_trip:.3e} m exceeds 1e-9 m")
    if closed_chain["attempted"]:
        for entry in closed_chain["variants"]:
            if entry["status"] != "closes":
                continue
            if entry["worst_residual_m"] > RESIDUAL_LIMIT_M:
                failures.append(
                    f"{entry['variant']}: residual {entry['worst_residual_m']:.3e} m"
                    " exceeds 1e-9 m"
                )
        if not closed_chain["closing_variants"]:
            failures.append("no candidate body assignment closes the chain")

    return {
        "tool": "mechanism-check",
        "schema": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "config": str(mech.path),
        "source": {
            "step_sha256": source.get("step_sha256"),
            "audit_report": source.get("audit_report"),
            "assembly_pose": mech.meta.get("assembly_pose"),
            "sign_convention": mech.meta.get("sign_convention"),
        },
        "criteria": {
            "residual_limit_m": RESIDUAL_LIMIT_M,
            "source": "progress.md §20.4 item 2",
        },
        "topology": {
            "use_inferred_topology": use_inferred,
            "chain_status": chain.get("status"),
            "dof": chain.get("dof"),
            "loops": chain.get("loops"),
            "confirm_in_cad": _confirmations(mech),
        },
        "static_closure": closure,
        "serial_leg": leg,
        "closed_chain": closed_chain,
        "gas_spring": _spring_report(mech),
        "resolved_nodes": describe_resolution(resolve_nodes(mech)),
        "unknowns": sorted(str(name) for name in mech.unknowns),
        "passed": not failures,
        "failures": failures,
    }


def summarize(report: dict[str, Any]) -> str:
    """One-screen text summary for the CLI."""
    lines: list[str] = []
    closure = report["static_closure"]
    lines.append(
        f"static closure : {closure['status']}  worst {closure['worst_error_m'] * 1000.0:.3e} mm"
        f"  ({len(closure['links'])} lengths)"
    )
    leg = report["serial_leg"]
    pose = leg["assembly_pose"]
    lines.append(
        f"serial leg     : thigh {leg['thigh_m']:.6f} m  shin {leg['shin_m']:.6f} m"
        f"  reach {leg['reach_annulus_m'][0]:.3f}-{leg['reach_annulus_m'][1]:.3f} m"
    )
    lines.append(
        f"  CAD pose     : chord {pose['hip_to_wheel_chord_m']:.6f} m"
        f"  reproduction error {pose['cad_reproduction_error_m']:.3e} m"
        f"  inside annulus {pose['inside_annulus']}"
    )
    lines.append(
        f"  IK/FK        : {leg['round_trip']['samples']} targets"
        f"  worst {leg['round_trip']['worst_error_m']:.3e} m"
        f"  unreachable {leg['round_trip']['unreachable_targets']}"
    )
    chain = report["closed_chain"]
    if chain["attempted"]:
        lines.append(f"closed chain   : {chain['status']}")
        for entry in chain["variants"]:
            if entry.get("cad_reproduction_error_m") is None:
                lines.append(f"  {entry['variant']:<16} {entry['status']}: {entry.get('reason')}")
                continue
            grid = entry["grid"]
            lines.append(
                f"  {entry['variant']:<16} CAD {entry['cad_reproduction_error_m']:.2e} m"
                f"  closes {grid['closed']}/{grid['poses']}"
                f"  worst residual {entry['worst_residual_m']:.2e} m"
                f"  [{entry['status']}]"
            )
        selected = chain.get("selected_variant")
        if selected:
            lines.append(f"  selected     : {selected}")
    else:
        lines.append(f"closed chain   : {chain['status']} - {chain['reason']}")
        if chain.get("geometry_supports"):
            lines.append(
                f"  variant      : {chain.get('selected_variant')} (selected) / "
                f"geometry supports {chain['geometry_supports']}"
            )
    spring = report.get("gas_spring") or {}
    if spring.get("status") == "computed_from_points":
        lines.append(
            f"gas spring     : pin {spring['pin_min_mm']:.3f}-{spring['pin_max_mm']:.3f} mm"
            f"  travel {spring['pin_travel_mm']:.3f} mm"
            f"  (assembly {spring['pin_at_assembly_mm']:.3f} mm at"
            f" {spring['delta_at_assembly_deg']:.2f} deg flexion)"
        )
        stated = spring.get("as_stated") or {}
        if stated:
            lines.append(
                f"  as stated    : free {stated['free_length_mm']:.1f} mm /"
                f" stroke {stated['stroke_mm']:.1f} mm ->"
                f" height {stated['min_height_m']:.4f}-{stated['max_height_m']:.4f} m"
                f"  reaches nominal {spring.get('reaches_nominal_height')}"
            )
            lines.append(
                f"  at nominal   : 3D pin {spring['nominal_pin_distance_3d_mm']:.6f} mm"
                f"  compressed {spring['compression_at_nominal_height_mm']:.6f} mm"
                f"  remaining {spring['remaining_compression_at_nominal_height_mm']:.6f} mm"
            )
            for problem in stated.get("problems") or []:
                lines.append(f"  spring limit : {problem}")
    resolved = report.get("resolved_nodes") or {}
    if resolved:
        lateral = ((resolved.get("per_side") or {}).get("right") or {}).get("lateral") or {}
        lines.append(
            f"link length    : planar thigh {resolved['cad_planar_mm']['thigh']:.6f} mm"
            f" / shin {resolved['cad_planar_mm']['shin']:.6f} mm"
            f"  (projected from the URDF offsets, error"
            f" {resolved['worst_planar_error_mm']:.1e} mm)"
        )
        lines.append(
            f"  sideways     : knee {lateral.get('knee_from_hip_mm', 0.0):+.3f} mm"
            f"  wheel {lateral.get('wheel_from_hip_mm', 0.0):+.3f} mm (right, from the hip)"
            f"  end link {resolved['end_link_mm']:.3f} mm"
        )
    if report["failures"]:
        for failure in report["failures"]:
            lines.append(f"FAIL           : {failure}")
    else:
        lines.append("verdict        : all attempted checks within 1e-9 m")
    return "\n".join(lines)
