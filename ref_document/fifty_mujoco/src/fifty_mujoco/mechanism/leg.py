"""Leg kinematics and the CAD cross-check.

Two models live here, and they answer different questions:

``TwoLinkLeg``
    The two-link (hip-knee-wheel) chain, solved analytically. It is the model
    ``configs/robot.yaml`` already describes, so it can be compared against the
    serial MJCF without any topology assumption.

``assembly_system`` / ``check_assembly``
    The two-loop closed chain from ``configs/mechanism.yaml``. Its *static*
    closure is an arithmetic consistency check on the CAD coordinates and runs
    always; the *driven* solve (joint angles in, node positions out) is a
    kinematic model of the real mechanism and therefore needs the explicit
    opt-in described in :class:`~fifty_mujoco.mechanism.planar.TopologyNotConfirmedError`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Mapping

from .loader import Mechanism
from .planar import NoSolutionError, circle_intersection, distance

Vec2 = tuple[float, float]


@dataclass(frozen=True)
class TwoLinkLeg:
    """Hip-knee-wheel chain in the plane, lengths in metres."""

    thigh_m: float
    shin_m: float

    @property
    def reach_min_m(self) -> float:
        return abs(self.thigh_m - self.shin_m)

    @property
    def reach_max_m(self) -> float:
        return self.thigh_m + self.shin_m

    def forward(self, hip: Vec2, thigh_angle: float, shin_angle: float) -> dict[str, Vec2]:
        """Absolute link angles (radians, from +y towards +z) to node positions."""
        knee = (
            hip[0] + self.thigh_m * math.cos(thigh_angle),
            hip[1] + self.thigh_m * math.sin(thigh_angle),
        )
        wheel = (
            knee[0] + self.shin_m * math.cos(shin_angle),
            knee[1] + self.shin_m * math.sin(shin_angle),
        )
        return {"hip": hip, "knee": knee, "wheel": wheel}

    def inverse(self, hip: Vec2, target: Vec2, branch: int = 0) -> tuple[float, float]:
        """Absolute (thigh, shin) angles that put the wheel on ``target``.

        ``branch`` selects which of the two elbow solutions is returned. The
        solution is analytic (circle-circle intersection), so a residual that is
        not ~1e-16 means the caller asked for an unreachable target.
        """
        if not self.reachable(hip, target):
            raise NoSolutionError(
                f"target is outside the reachable annulus "
                f"[{self.reach_min_m}, {self.reach_max_m}] m"
            )
        knee_options = circle_intersection(hip, self.thigh_m, target, self.shin_m)
        if not knee_options:
            raise NoSolutionError("no elbow solution: circles do not intersect")
        knee = knee_options[branch % len(knee_options)]
        thigh_angle = math.atan2(knee[1] - hip[1], knee[0] - hip[0])
        shin_angle = math.atan2(target[1] - knee[1], target[0] - knee[0])
        return thigh_angle, shin_angle

    def reachable(self, hip: Vec2, target: Vec2, tolerance: float = 1e-12) -> bool:
        length = distance(hip, target)
        return self.reach_min_m - tolerance <= length <= self.reach_max_m + tolerance

    def closure_residual(self, hip: Vec2, target: Vec2, branch: int = 0) -> float:
        """|FK(IK(target)) - target|, the round-trip error in metres."""
        thigh_angle, shin_angle = self.inverse(hip, target, branch)
        wheel = self.forward(hip, thigh_angle, shin_angle)["wheel"]
        return distance(wheel, target)


def _pair_key(first: str, second: str) -> frozenset[str]:
    return frozenset((first, second))


def declared_lengths(mechanism: Mechanism) -> dict[frozenset[str], float]:
    """Every declared length keyed by its unordered node pair, in metres."""
    lengths: dict[frozenset[str], float] = {}
    for link in mechanism.all_links.values():
        key = _pair_key(link.a, link.b)
        if key in lengths and not math.isclose(lengths[key], link.length_m, abs_tol=1e-12):
            raise ValueError(
                f"two links declare different lengths for {link.a}-{link.b}: "
                f"{lengths[key]} vs {link.length_m}"
            )
        lengths[key] = link.length_m
    return lengths


def body_constraints(
    mechanism: Mechanism,
    bodies: Mapping[str, Iterable[str]] | None = None,
) -> list[tuple[str, str, float]]:
    """All pairwise distances inside every candidate body, deduplicated.

    ``bodies`` defaults to the primary body map in ``mechanism.yaml``. The two
    candidate assignments differ exactly here, so the caller passes its own map
    when it evaluates one of the alternatives.
    """
    declared = declared_lengths(mechanism)
    groups: Mapping[str, Iterable[str]]
    if bodies is None:
        groups = {body.name: body.nodes for body in mechanism.bodies.values()}
    else:
        groups = bodies
    constraints: dict[frozenset[str], tuple[str, str, float]] = {}
    for body_name, members in groups.items():
        nodes = list(members)
        for index, first in enumerate(nodes):
            for second in nodes[index + 1 :]:
                key = _pair_key(first, second)
                if key in constraints:
                    continue
                if key not in declared:
                    raise ValueError(
                        f"body {body_name}: nodes {first}-{second} have no declared length;"
                        " add it to link_lengths or derived_lengths"
                    )
                constraints[key] = (first, second, declared[key])
    return list(constraints.values())


def static_closure(mechanism: Mechanism) -> dict[str, object]:
    """Closure of the declared length set at the CAD coordinates.

    No solver is involved: every declared length is compared against the
    distance between the two CAD nodes. A non-zero error means the coordinates
    and the length table disagree, which is the failure this check exists for.

    The comparison budget is ``1e-6 mm``: that is the precision the lengths are
    recorded with, so demanding better would only measure rounding.
    """
    nodes = mechanism.assembly_pose_m()
    detail: list[dict[str, object]] = []
    worst = 0.0
    for name, link in mechanism.all_links.items():
        measured = distance(nodes[link.a], nodes[link.b])
        error = measured - link.length_m
        worst = max(worst, abs(error))
        detail.append(
            {
                "link": name,
                "between": [link.a, link.b],
                "declared_mm": round(link.length_m * 1000.0, 6),
                "from_points_mm": round(measured * 1000.0, 6),
                "error_mm": round(error * 1000.0, 9),
                "status": link.status,
                "reproduces": link.reproduces,
            }
        )
    tolerance_mm = 1e-6
    return {
        "status": "consistent" if worst * 1000.0 <= tolerance_mm else "inconsistent",
        "worst_error_m": worst,
        "tolerance_mm": tolerance_mm,
        "links": detail,
        "note": (
            "static closure compares the length table with the CAD node coordinates;"
            " it is an arithmetic consistency check, not evidence that the body"
            " assignment is the real one"
        ),
    }


def serial_leg_report(mechanism: Mechanism, samples: int = 24) -> dict[str, object]:
    """Two-link leg: reproduce the CAD points and bound the IK/FK round trip."""
    thigh = mechanism.length_m("thigh_hip_to_knee")
    shin = mechanism.length_m("shin_knee_to_wheel")
    leg = TwoLinkLeg(thigh_m=thigh, shin_m=shin)
    hip = mechanism.node_m("HIP")
    knee = mechanism.node_m("KNEE")
    wheel = mechanism.node_m("WHEEL")

    thigh_angle = math.atan2(knee[1] - hip[1], knee[0] - hip[0])
    shin_angle = math.atan2(wheel[1] - knee[1], wheel[0] - knee[0])
    reproduced = leg.forward(hip, thigh_angle, shin_angle)
    cad_error = max(
        distance(reproduced["knee"], knee),
        distance(reproduced["wheel"], wheel),
    )

    # Round trip on a polar grid inside the reachable annulus. The grid is
    # built from the reachable interval, not from the CAD pose, so a wrong
    # length would show up as a residual instead of being cancelled out.
    worst_round_trip = 0.0
    unreachable = 0
    for radius_index in range(1, samples):
        radius = leg.reach_min_m + (leg.reach_max_m - leg.reach_min_m) * radius_index / samples
        for angle_index in range(samples):
            angle = 2.0 * math.pi * angle_index / samples
            target = (hip[0] + radius * math.cos(angle), hip[1] + radius * math.sin(angle))
            try:
                worst_round_trip = max(worst_round_trip, leg.closure_residual(hip, target))
            except NoSolutionError:
                unreachable += 1

    chord = distance(hip, wheel)
    return {
        "thigh_m": round(thigh, 9),
        "shin_m": round(shin, 9),
        "reach_annulus_m": [round(leg.reach_min_m, 9), round(leg.reach_max_m, 9)],
        "assembly_pose": {
            "thigh_angle_rad": round(thigh_angle, 12),
            "shin_angle_rad": round(shin_angle, 12),
            "hip_to_wheel_chord_m": round(chord, 9),
            "cad_reproduction_error_m": cad_error,
            "inside_annulus": leg.reachable(hip, wheel),
            "note": "angles are in the STEP planar frame; the mapping to controller zero is unknown",
        },
        "round_trip": {
            "samples": (samples - 1) * samples,
            "worst_error_m": worst_round_trip,
            "unreachable_targets": unreachable,
        },
    }
