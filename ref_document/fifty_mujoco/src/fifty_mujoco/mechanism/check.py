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
from .transmission import ThighCrankTransmission

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
    else:
        lines.append(f"closed chain   : {chain['status']} - {chain['reason']}")
    if report["failures"]:
        for failure in report["failures"]:
            lines.append(f"FAIL           : {failure}")
    else:
        lines.append("verdict        : all attempted checks within 1e-9 m")
    return "\n".join(lines)
