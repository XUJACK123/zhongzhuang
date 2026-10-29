"""Exact forward kinematics of the planar two-loop chain.

Why not solve the whole chain numerically: the ternary link's three bores are
almost collinear (135 + 67 = 202 mm against a 200.51 mm span), so a distance
residual formulation is badly conditioned and a Levenberg-Marquardt step stalls
long before the loop is infeasible. The chain itself, however, is a chain of
circle intersections:

    thigh angle  -> KNEE, END_PIN_A                      (rigid thigh)
    crank angle  -> CRANK_PIN                            (rigid crank)
    parallelogram-> ternary orientation = thigh + const   (the link between
                                                           KNEE-END_PIN_A and
                                                           LINK_PIN_A-LINK_PIN_C)
    push rod     -> LINK_PIN_A lies on a circle around a point fixed by the
                    ternary orientation and the crank pin, radius 135

so the closed form is one circle-circle intersection per pose. The constants are
read from the CAD pose, never typed in, and the resulting pose is then checked
against *every* declared length by :class:`~fifty_mujoco.mechanism.planar.PlanarSystem`
- that check is what makes the analytic solution trustworthy rather than
self-fulfilling.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from .loader import Mechanism
from .planar import NoSolutionError, angle_of, circle_intersection, distance

Vec2 = tuple[float, float]


@dataclass(frozen=True)
class MechanismPose:
    """Constants of the chain, all derived from one CAD pose."""

    mechanism: Mechanism
    thigh_angle_cad: float
    crank_angle_cad: float
    #: KNEE -> WHEEL direction minus KNEE -> LINK_PIN_A direction. The shin body
    #: is referenced to LINK_PIN_A because that is the hole the transmission uses.
    wheel_from_link_a: float
    #: Ternary orientation minus the thigh angle (the parallelogram).
    ternary_offset: float
    #: LINK_PIN_A -> ROD_PIN direction minus the ternary orientation.
    ternary_from_link_a: float
    #: Which circle solution reproduces the CAD pose.
    cad_branch: int = 0

    @classmethod
    def from_mechanism(cls, mechanism: Mechanism) -> "MechanismPose":
        nodes = mechanism.assembly_pose_m()
        hip, knee, wheel = nodes["HIP"], nodes["KNEE"], nodes["WHEEL"]
        link_a, link_c = nodes["LINK_PIN_A"], nodes["LINK_PIN_C"]
        rod, crank_pin = nodes["ROD_PIN"], nodes["CRANK_PIN"]

        thigh_angle = angle_of((knee[0] - hip[0], knee[1] - hip[1]))
        crank_angle = angle_of((crank_pin[0] - hip[0], crank_pin[1] - hip[1]))
        link_a_angle = angle_of((link_a[0] - knee[0], link_a[1] - knee[1]))
        ternary_angle = angle_of((link_c[0] - link_a[0], link_c[1] - link_a[1]))
        pose = cls(
            mechanism=mechanism,
            thigh_angle_cad=thigh_angle,
            crank_angle_cad=crank_angle,
            wheel_from_link_a=_wrap(
                angle_of((wheel[0] - knee[0], wheel[1] - knee[1])) - link_a_angle
            ),
            ternary_offset=_wrap(ternary_angle - thigh_angle),
            ternary_from_link_a=_wrap(
                angle_of((rod[0] - link_a[0], rod[1] - link_a[1])) - ternary_angle
            ),
        )
        best_branch = 0
        best_error: float | None = None
        for branch in (0, 1):
            try:
                produced = pose.forward(thigh_angle, crank_angle, branch)
            except NoSolutionError:
                continue
            error = max(
                distance(produced[name], nodes[name]) for name in nodes
            )
            if best_error is None or error < best_error:
                best_branch, best_error = branch, error
        return replace(pose, cad_branch=best_branch)

    # -- geometry ---------------------------------------------------------
    @property
    def thigh_m(self) -> float:
        return self.mechanism.length_m("thigh_hip_to_knee")

    @property
    def shin_m(self) -> float:
        return self.mechanism.length_m("shin_knee_to_wheel")

    @property
    def knee_to_link_a_m(self) -> float:
        return self.mechanism.length_m("knee_to_link_pin_a")

    @property
    def crank_m(self) -> float:
        return self.mechanism.length_m("hip_to_crank_pin")

    @property
    def push_rod_m(self) -> float:
        return self.mechanism.length_m("push_rod")

    @property
    def ternary_arm_m(self) -> float:
        return self.mechanism.length_m("ternary_a_b")

    @property
    def ternary_side_m(self) -> float:
        return self.mechanism.length_m("ternary_a_c")

    def cad_angles(self) -> tuple[float, float]:
        return (self.thigh_angle_cad, self.crank_angle_cad)

    # -- kinematics -------------------------------------------------------
    def _ternary_angle(self, thigh_angle: float) -> float:
        return _wrap(thigh_angle + self.ternary_offset)

    def link_pin_a(self, thigh_angle: float, crank_angle: float, branch: int = 0) -> Vec2:
        """Solve the push-rod constraint for LINK_PIN_A (one circle intersection)."""
        nodes = self.mechanism.assembly_pose_m()
        hip = nodes["HIP"]
        knee = (
            hip[0] + self.thigh_m * math.cos(thigh_angle),
            hip[1] + self.thigh_m * math.sin(thigh_angle),
        )
        crank_pin = (
            hip[0] + self.crank_m * math.cos(crank_angle),
            hip[1] + self.crank_m * math.sin(crank_angle),
        )
        ternary_angle = self._ternary_angle(thigh_angle)
        # ROD_PIN = KNEE + knee_to_link_a * u(link_a) + arm * u(ternary + offset),
        # so ROD_PIN lies on a circle of radius knee_to_link_a about a point that
        # is already known, and at push_rod distance from the crank pin.
        pivot = (
            knee[0] + self.ternary_arm_m * math.cos(ternary_angle + self.ternary_from_link_a),
            knee[1] + self.ternary_arm_m * math.sin(ternary_angle + self.ternary_from_link_a),
        )
        intersections = circle_intersection(
            pivot, self.knee_to_link_a_m, crank_pin, self.push_rod_m
        )
        if not intersections:
            raise NoSolutionError(
                "the push rod cannot reach the crank pin at this pose: the loop is outside"
                " its feasible range"
            )
        rod_pin = intersections[branch % len(intersections)]
        arm_angle = ternary_angle + self.ternary_from_link_a
        return (
            rod_pin[0] - self.ternary_arm_m * math.cos(arm_angle),
            rod_pin[1] - self.ternary_arm_m * math.sin(arm_angle),
        )

    def forward(
        self,
        thigh_angle: float,
        crank_angle: float,
        branch: int | None = None,
    ) -> dict[str, Vec2]:
        """Node positions for a driven pose. Raises when the loop cannot close."""
        if branch is None:
            branch = self.cad_branch
        nodes = self.mechanism.assembly_pose_m()
        hip = nodes["HIP"]
        knee = (
            hip[0] + self.thigh_m * math.cos(thigh_angle),
            hip[1] + self.thigh_m * math.sin(thigh_angle),
        )
        crank_pin = (
            hip[0] + self.crank_m * math.cos(crank_angle),
            hip[1] + self.crank_m * math.sin(crank_angle),
        )

        # Thigh is rigid: END_PIN_A follows KNEE by the CAD offset.
        end_a_local = _rotate(_sub(nodes["END_PIN_A"], nodes["KNEE"]), -self.thigh_angle_cad)
        end_a = _add(knee, _rotate(end_a_local, thigh_angle))

        ternary_angle = self._ternary_angle(thigh_angle)
        arm_angle = ternary_angle + self.ternary_from_link_a
        link_a = self.link_pin_a(thigh_angle, crank_angle, branch)
        rod = _add(link_a, _polar(self.ternary_arm_m, arm_angle))
        # Ternary link is rigid: its third hole sits at a fixed arm and angle from
        # LINK_PIN_A, and the whole body turns with the thigh.
        link_c = _add(link_a, _polar(self.ternary_side_m, ternary_angle))
        link_a_angle = _wrap(angle_of(_sub(link_a, knee)))
        wheel = _polar_from(knee, self.shin_m, link_a_angle + self.wheel_from_link_a)
        return {
            "HIP": hip,
            "KNEE": knee,
            "END_PIN_A": end_a,
            "CRANK_PIN": crank_pin,
            "LINK_PIN_A": link_a,
            "LINK_PIN_C": link_c,
            "ROD_PIN": rod,
            "WHEEL": wheel,
        }


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _sub(a: Vec2, b: Vec2) -> Vec2:
    return (a[0] - b[0], a[1] - b[1])


def _add(a: Vec2, b: Vec2) -> Vec2:
    return (a[0] + b[0], a[1] + b[1])


def _rotate(vector: Vec2, angle: float) -> Vec2:
    cos, sin = math.cos(angle), math.sin(angle)
    return (vector[0] * cos - vector[1] * sin, vector[0] * sin + vector[1] * cos)


def _polar(length: float, angle: float) -> Vec2:
    return (length * math.cos(angle), length * math.sin(angle))


def _polar_from(origin: Vec2, length: float, angle: float) -> Vec2:
    return (origin[0] + length * math.cos(angle), origin[1] + length * math.sin(angle))
