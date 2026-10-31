"""Map between the equivalent serial leg and the confirmed closed chain.

The MJCF in this repository is an *equivalent serial* two-link leg (hip joint +
knee joint, thigh 0.210 m, shin 0.250 m). The real machine is a two-DOF five-bar:
the second motor turns a crank about the hip axis and the knee angle follows from
the closed chain, so it is not a free coordinate.

Both parameterisations describe the same wheel position, which is what makes them
comparable without guessing:

``shin_angle`` / ``mechanism_forward``
    ``(thigh, crank) -> shin``: run the confirmed ``link_a_on_thigh`` chain and
    read the shin direction off KNEE -> WHEEL.

``crank_roots`` / ``mechanism_from_serial``
    ``(thigh, shin) -> crank``: a one-dimensional root find. The thigh angle is
    shared (both use HIP-KNEE, G01 = 210 mm), so only the crank is unknown.

A five-bar can reach the same shin direction at more than one crank angle, and
some of those roots are **singular** (the Jacobian loses rank, so the pose cannot
be controlled). The selection rule is therefore explicit: keep the roots where
the chain closes *and* the Jacobian is non-singular, then take the one closest to
the reference crank angle. Nothing is clamped silently - a pose outside the
feasible set raises
:class:`~fifty_mujoco.mechanism.planar.NoSolutionError`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .planar import NoSolutionError, angle_of
from .transmission import ThighCrankTransmission

Vec2 = tuple[float, float]

#: Scan resolution of the crank search, in radians. Half a degree is 17x finer
#: than the 0.15 rad grid the feasibility sweep used, so no root can hide.
CRANK_SCAN_STEP_RAD = math.radians(0.5)
#: Polish target on the shin-direction residual.
ROOT_TOLERANCE_RAD = 1e-13
#: Below this |det J| a pose is treated as singular and not selected.
MIN_JACOBIAN_DET = 1e-6


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def shin_angle(transmission: ThighCrankTransmission, thigh: float, crank: float) -> float:
    """Absolute KNEE -> WHEEL direction for a given ``(thigh, crank)``."""
    nodes = transmission.forward(thigh, crank)
    knee, wheel = nodes["KNEE"], nodes["WHEEL"]
    return angle_of((wheel[0] - knee[0], wheel[1] - knee[1]))


def mechanism_forward(
    transmission: ThighCrankTransmission, thigh: float, crank: float
) -> tuple[float, float]:
    """``(thigh, crank) -> (thigh, shin)`` in absolute link angles."""
    return thigh, shin_angle(transmission, thigh, crank)


def wheel_jacobian(
    transmission: ThighCrankTransmission, thigh: float, crank: float, step: float = 1e-6
) -> tuple[tuple[float, float], tuple[float, float]]:
    """d(wheel position relative to hip)/d(thigh, crank).

    Column 0 is the thigh derivative, column 1 the crank derivative; rows are the
    planar Y and Z components. Central differences are used because the analytic
    derivative of the ternary-link circle intersection costs more error risk than
    it saves: the geometry is exact to 1e-12 and the truncation error is O(step^2).
    """
    plus_t = transmission.forward(thigh + step, crank)
    minus_t = transmission.forward(thigh - step, crank)
    plus_c = transmission.forward(thigh, crank + step)
    minus_c = transmission.forward(thigh, crank - step)

    def column(plus: dict[str, Vec2], minus: dict[str, Vec2]) -> tuple[float, float]:
        return (
            (plus["WHEEL"][0] - minus["WHEEL"][0]) / (2.0 * step),
            (plus["WHEEL"][1] - minus["WHEEL"][1]) / (2.0 * step),
        )

    return column(plus_t, minus_t), column(plus_c, minus_c)


def jacobian_determinant(
    transmission: ThighCrankTransmission, thigh: float, crank: float, step: float = 1e-6
) -> float:
    first, second = wheel_jacobian(transmission, thigh, crank, step)
    return first[0] * second[1] - first[1] * second[0]


@dataclass(frozen=True)
class CrankRoot:
    crank_angle: float
    closes: bool
    jacobian_det: float | None

    @property
    def usable(self) -> bool:
        return (
            self.closes
            and self.jacobian_det is not None
            and abs(self.jacobian_det) >= MIN_JACOBIAN_DET
        )


def crank_roots(
    transmission: ThighCrankTransmission,
    thigh: float,
    shin: float,
    scan_step: float = CRANK_SCAN_STEP_RAD,
    tolerance: float = ROOT_TOLERANCE_RAD,
) -> list[CrankRoot]:
    """Every crank angle in ``[-pi, pi)`` that reproduces ``shin`` at ``thigh``.

    Two failure modes of a naive scan are filtered here, because both produce a
    *fake* root and a fake root is worse than a missing one:

    * **wrap**: the residual is an angle difference, so a crossing through +-pi
      looks like a sign change although nothing passes through zero;
    * **feasibility edge**: where the push rod stops reaching, the shin direction
      jumps, which also flips the sign. The bisection then converges to the edge
      instead of to a solution.

    A candidate is kept only if the residual is actually ~0 there *and* the chain
    still closes on both sides of it.
    """
    target = _wrap(shin)
    wrap_margin = math.pi - 0.2

    def residual(crank: float) -> float | None:
        try:
            return _wrap(shin_angle(transmission, thigh, crank) - target)
        except NoSolutionError:
            return None

    def continuous_at(crank: float) -> bool:
        for offset in (-3.0 * scan_step, 3.0 * scan_step):
            if residual(crank + offset) is None:
                return False
        return True

    steps = int(round(2.0 * math.pi / scan_step))
    found: list[float] = []
    previous: tuple[float, float] | None = None
    for index in range(steps + 1):
        crank = -math.pi + index * scan_step
        value = residual(crank)
        if value is None:
            previous = None
            continue
        if (
            previous is not None
            and previous[1] * value < 0.0
            and abs(previous[1]) < wrap_margin
            and abs(value) < wrap_margin
        ):
            low, low_value = previous
            high = crank
            for _ in range(200):
                middle = 0.5 * (low + high)
                middle_value = residual(middle)
                if middle_value is None:
                    break
                if abs(middle_value) <= tolerance or (high - low) <= tolerance:
                    low = high = middle
                    break
                if low_value * middle_value <= 0.0:
                    high = middle
                else:
                    low, low_value = middle, middle_value
            found.append(0.5 * (low + high))
        previous = (crank, value)

    roots: list[CrankRoot] = []
    seen: list[float] = []
    for crank in found:
        if any(abs(_wrap(crank - other)) < 1e-6 for other in seen):
            continue
        seen.append(crank)
        value = residual(crank)
        if value is None or abs(value) > 1e-9 or not continuous_at(crank):
            # keep it visible, but unusable: it is an artefact, not a pose
            roots.append(CrankRoot(crank_angle=crank, closes=False, jacobian_det=None))
            continue
        try:
            det = jacobian_determinant(transmission, thigh, crank)
        except NoSolutionError:
            det = None
        roots.append(CrankRoot(crank_angle=crank, closes=True, jacobian_det=det))
    return roots


def mechanism_from_serial(
    transmission: ThighCrankTransmission,
    thigh: float,
    shin: float,
    crank_reference: float | None = None,
) -> float:
    """Crank angle that makes the closed chain produce ``(thigh, shin)``.

    Among the roots that close with a non-singular Jacobian, the one closest to
    ``crank_reference`` (default: the CAD crank angle) wins. If the chain only
    closes through a singular root the call raises: that pose is one the linkage
    can occupy but not one a controller can hold.
    """
    reference = (
        transmission.crank_angle_cad if crank_reference is None else crank_reference
    )
    roots = crank_roots(transmission, thigh, shin)
    usable = [root for root in roots if root.usable]
    if not usable:
        if roots:
            summary = ", ".join(
                f"{math.degrees(root.crank_angle):+.3f} deg"
                f" det="
                + ("none" if root.jacobian_det is None else f"{root.jacobian_det:+.3e}")
                for root in roots
            )
            raise NoSolutionError(
                "the chain closes at this pose but no solution is controllable"
                f" ({summary}); |det J| must exceed {MIN_JACOBIAN_DET}"
            )
        raise NoSolutionError(
            "no crank angle closes the chain for this shin direction: the pose is"
            " outside the feasible set of the confirmed link_a_on_thigh chain"
        )
    return min(
        usable, key=lambda root: abs(_wrap(root.crank_angle - reference))
    ).crank_angle


@dataclass(frozen=True)
class LegPose:
    """One leg pose in every coordinate system the project needs."""

    thigh_angle: float
    shin_angle: float
    crank_angle: float
    wheel_from_hip: Vec2
    jacobian_det: float

    @property
    def chord_m(self) -> float:
        return math.hypot(*self.wheel_from_hip)

    def as_dict(self) -> dict[str, float]:
        return {
            "thigh_rad": self.thigh_angle,
            "shin_rad": self.shin_angle,
            "crank_rad": self.crank_angle,
            "wheel_from_hip_y_m": self.wheel_from_hip[0],
            "wheel_from_hip_z_m": self.wheel_from_hip[1],
            "chord_m": self.chord_m,
            "jacobian_det": self.jacobian_det,
        }


def leg_pose(
    transmission: ThighCrankTransmission, thigh: float, crank: float
) -> LegPose:
    """Full pose of one leg, wheel position and Jacobian determinant included."""
    nodes = transmission.forward(thigh, crank)
    hip, wheel = nodes["HIP"], nodes["WHEEL"]
    return LegPose(
        thigh_angle=thigh,
        shin_angle=shin_angle(transmission, thigh, crank),
        crank_angle=crank,
        wheel_from_hip=(wheel[0] - hip[0], wheel[1] - hip[1]),
        jacobian_det=jacobian_determinant(transmission, thigh, crank),
    )


def leg_pose_from_serial(
    transmission: ThighCrankTransmission, thigh: float, shin: float
) -> LegPose:
    crank = mechanism_from_serial(transmission, thigh, shin)
    return leg_pose(transmission, thigh, crank)
