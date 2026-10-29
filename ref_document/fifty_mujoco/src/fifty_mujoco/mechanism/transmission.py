"""Second candidate body assignment for the same geometry.

The STEP audit fixes where the eight pivots are, but not which part is bolted to
which. Two assignments are consistent with the distances:

``link_a_on_shin`` (implemented in :mod:`fifty_mujoco.mechanism.pose`)
    the ternary link pivots on the shin, so the parallelogram slaves the ternary
    angle to the thigh angle.

``link_a_on_thigh`` (this module)
    the ternary link pivots on the thigh, so the parallelogram slaves the ternary
    angle to the shin angle and the chain is
    ``crank -> push rod -> ternary -> end link -> shin pin``.

They are not equivalent: the first closes only in a narrow band around the CAD
pose, the second closes over a wide range. That difference is evidence, not
proof, so both are reported and the CAD mates decide. Nothing here may be fed to
a controller until that happens.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .loader import Mechanism
from .planar import NoSolutionError, angle_of, circle_intersection

Vec2 = tuple[float, float]

VARIANT = "link_a_on_thigh"


@dataclass(frozen=True)
class ThighCrankTransmission:
    """Ternary link pivoting on the thigh: crank -> rod -> ternary -> end link."""

    mechanism: Mechanism
    thigh_angle_cad: float
    crank_angle_cad: float
    link_a_from_thigh: float  # KNEE -> LINK_PIN_A direction minus the thigh angle
    wheel_from_end_pin: float  # KNEE -> WHEEL minus KNEE -> END_PIN_A
    #: Which of the two circle solutions reproduces the CAD pose, as
    #: ``(rod, link_c, end_pin)``.
    cad_branches: tuple[int, int, int] = (0, 0, 0)

    @classmethod
    def from_mechanism(cls, mechanism: Mechanism) -> "ThighCrankTransmission":
        nodes = mechanism.assembly_pose_m()
        hip, knee, wheel = nodes["HIP"], nodes["KNEE"], nodes["WHEEL"]
        link_a = nodes["LINK_PIN_A"]
        end_a, crank_pin = nodes["END_PIN_A"], nodes["CRANK_PIN"]
        thigh_angle = angle_of((knee[0] - hip[0], knee[1] - hip[1]))
        crank_angle = angle_of((crank_pin[0] - hip[0], crank_pin[1] - hip[1]))
        pose = cls(
            mechanism=mechanism,
            thigh_angle_cad=thigh_angle,
            crank_angle_cad=crank_angle,
            link_a_from_thigh=_wrap(
                angle_of((link_a[0] - knee[0], link_a[1] - knee[1])) - thigh_angle
            ),
            wheel_from_end_pin=_wrap(
                angle_of((wheel[0] - knee[0], wheel[1] - knee[1]))
                - angle_of((end_a[0] - knee[0], end_a[1] - knee[1]))
            ),
        )
        # Pick the branch combination that reproduces the CAD pose. Doing this
        # once, from the data, is what keeps the two assignments comparable.
        best = (0, 0, 0)
        best_error = None
        for rod_branch in (0, 1):
            for link_c_branch in (0, 1):
                for end_branch in (0, 1):
                    try:
                        produced = pose.forward(
                            thigh_angle,
                            crank_angle,
                            branch=(rod_branch, link_c_branch, end_branch),
                        )
                    except NoSolutionError:
                        continue
                    error = max(
                        _dist(produced[name], nodes[name]) for name in nodes
                    )
                    if best_error is None or error < best_error:
                        best = (rod_branch, link_c_branch, end_branch)
                        best_error = error
        return cls(
            mechanism=pose.mechanism,
            thigh_angle_cad=pose.thigh_angle_cad,
            crank_angle_cad=pose.crank_angle_cad,
            link_a_from_thigh=pose.link_a_from_thigh,
            wheel_from_end_pin=pose.wheel_from_end_pin,
            cad_branches=best,
        )

    @property
    def thigh_m(self) -> float:
        return self.mechanism.length_m("thigh_hip_to_knee")

    @property
    def shin_m(self) -> float:
        return self.mechanism.length_m("shin_knee_to_wheel")

    @property
    def crank_m(self) -> float:
        return self.mechanism.length_m("hip_to_crank_pin")

    @property
    def link_a_m(self) -> float:
        return self.mechanism.length_m("knee_to_link_pin_a")

    @property
    def push_rod_m(self) -> float:
        return self.mechanism.length_m("push_rod")

    @property
    def ternary_arm_m(self) -> float:
        return self.mechanism.length_m("ternary_a_b")

    @property
    def ternary_side_m(self) -> float:
        return self.mechanism.length_m("ternary_a_c")

    @property
    def end_link_m(self) -> float:
        return self.mechanism.length_m("end_link")

    @property
    def ternary_span_m(self) -> float:
        return self.mechanism.length_m("ternary_b_c")

    @property
    def shin_pin_m(self) -> float:
        return self.mechanism.length_m("knee_to_end_pin_a")

    def cad_angles(self) -> tuple[float, float]:
        return (self.thigh_angle_cad, self.crank_angle_cad)

    def forward(
        self,
        thigh_angle: float,
        crank_angle: float,
        branch: tuple[int, int, int] | int | None = None,
    ) -> dict[str, Vec2]:
        """``branch`` is ``(rod, link_c, end_pin)``; ``None`` uses the CAD choice."""
        if branch is None:
            rod_branch, link_c_branch, end_branch = self.cad_branches
        elif isinstance(branch, tuple):
            rod_branch, link_c_branch, end_branch = branch
        else:
            rod_branch, link_c_branch, end_branch = (
                branch & 1,
                (branch >> 1) & 1,
                (branch >> 2) & 1,
            )
        nodes = self.mechanism.assembly_pose_m()
        hip = nodes["HIP"]
        knee_now = _polar_from(hip, self.thigh_m, thigh_angle)
        crank_pin = _polar_from(hip, self.crank_m, crank_angle)
        link_a = _polar_from(
            knee_now, self.link_a_m, thigh_angle + self.link_a_from_thigh
        )
        rod_options = circle_intersection(link_a, self.ternary_arm_m, crank_pin, self.push_rod_m)
        if not rod_options:
            raise NoSolutionError(
                "the push rod cannot reach both the ternary link and the crank pin at this pose"
            )
        rod = rod_options[rod_branch % len(rod_options)]
        # The ternary link is a rigid triangle 135 / 67 / 200.51: its third hole
        # follows from the other two, it is not a free direction.
        link_c_options = circle_intersection(
            link_a, self.ternary_side_m, rod, self.ternary_span_m
        )
        if not link_c_options:
            raise NoSolutionError("the ternary triangle cannot be closed at this pose")
        link_c = link_c_options[link_c_branch % len(link_c_options)]
        end_options = circle_intersection(knee_now, self.shin_pin_m, link_c, self.end_link_m)
        if not end_options:
            raise NoSolutionError(
                "the end link cannot reach the shin pin at this pose: the loop is outside"
                " its feasible range"
            )
        end_a = end_options[end_branch % len(end_options)]
        end_angle = angle_of((end_a[0] - knee_now[0], end_a[1] - knee_now[1]))
        wheel = _polar_from(knee_now, self.shin_m, end_angle + self.wheel_from_end_pin)
        return {
            "HIP": hip,
            "KNEE": knee_now,
            "END_PIN_A": end_a,
            "CRANK_PIN": crank_pin,
            "LINK_PIN_A": link_a,
            "LINK_PIN_C": link_c,
            "ROD_PIN": rod,
            "WHEEL": wheel,
        }


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _dist(a: Vec2, b: Vec2) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _polar_from(origin: Vec2, length: float, angle: float) -> Vec2:
    return (origin[0] + length * math.cos(angle), origin[1] + length * math.sin(angle))
