"""Dependency-free mathematical contract for the archived sample actor.

This module does not read a simulator or advance a history buffer. Callers must
provide time-aligned body-frame measurements and the PREVIOUS clipped action.
The original base-environment revision is still missing; this is a mathematical
reference, not a declaration of simulator or observation-timing parity.
"""

from __future__ import annotations

import math
from typing import Sequence

JOINT_ORDER = (
    "rf0_Joint", "rf1_Joint", "r_wheel_Joint",
    "lf0_Joint", "lf1_Joint", "l_wheel_Joint",
)
LEG_INDICES = (0, 1, 3, 4)
WHEEL_INDICES = (2, 5)
ACTOR_SIZES = (31, 128, 64, 32, 6)
SOURCE_DEFAULT_ANGLES = (
    3.1063109024878943, -3.6606010515197696, 0.0,
    -3.1063109024878943, 3.6606010515197696, 0.0,
)
COMMAND_RATES = (1.5, 2 * math.pi, 0.12)
REFERENCE_OFFSET = (-0.07828434314112345, 0.003702250000000018)


def finite_vector(values: Sequence[float], size: int, name: str) -> list[float]:
    if len(values) != size:
        raise ValueError(f"{name}: expected {size} values, got {len(values)}")
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name}: non-finite value")
    return result


def wrap_angle(value: float) -> float:
    if not math.isfinite(value):
        raise ValueError("angle must be finite")
    return math.atan2(math.sin(value), math.cos(value))


def clip_actions(actions: Sequence[float]) -> list[float]:
    result = finite_vector(actions, 6, "actions")
    return [max(-2.0 if i in WHEEL_INDICES else -1.0,
                min(2.0 if i in WHEEL_INDICES else 1.0, value))
            for i, value in enumerate(result)]


def ramp_commands(applied: Sequence[float], requested: Sequence[float], dt: float,
                  rates: Sequence[float] = COMMAND_RATES) -> list[float]:
    old = finite_vector(applied, 3, "applied commands")
    target = finite_vector(requested, 3, "requested commands")
    limits = finite_vector(rates, 3, "command rates")
    if not math.isfinite(dt) or dt <= 0 or min(limits) <= 0:
        raise ValueError("dt and command rates must be positive")
    return [a + max(-r * dt, min(r * dt, b - a))
            for a, b, r in zip(old, target, limits)]


def advance_reference(position_xy: Sequence[float], heading: float, forward: float,
                      yaw_rate: float, dt: float,
                      offset_xy: Sequence[float] = REFERENCE_OFFSET,
                      ) -> tuple[list[float], float]:
    position = finite_vector(position_xy, 2, "reference position")
    offset = finite_vector(offset_xy, 2, "command point offset")
    finite_vector((heading, forward, yaw_rate, dt), 4, "reference integration")
    if dt <= 0:
        raise ValueError("dt must be positive")
    half = yaw_rate * dt / 2
    # sin(x)/x, equivalent to the archive's normalized numpy/torch sinc(x/pi).
    sinc = 1.0 if abs(half) < 1e-12 else math.sin(half) / half
    distance = forward * dt * sinc
    new_heading = wrap_angle(heading + 2 * half)
    dc = math.cos(heading) - math.cos(new_heading)
    ds = math.sin(heading) - math.sin(new_heading)
    return [position[0] + distance * math.cos(heading + half)
            + dc * offset[0] - ds * offset[1],
            position[1] + distance * math.sin(heading + half)
            + ds * offset[0] + dc * offset[1]], new_heading


def reference_features(position_xy: Sequence[float], reference_xy: Sequence[float],
                       yaw: float, reference_yaw: float,
                       body_velocity_xy: Sequence[float]) -> list[float]:
    p = finite_vector(position_xy, 2, "position")
    ref = finite_vector(reference_xy, 2, "reference position")
    vel = finite_vector(body_velocity_xy, 2, "body velocity")
    finite_vector((yaw, reference_yaw), 2, "headings")
    dx, dy = ref[0] - p[0], ref[1] - p[1]
    c, s = math.cos(yaw), math.sin(yaw)
    error = reference_yaw - yaw
    return [max(-2.0, min(2.0, 5 * (c * dx + s * dy))),
            max(-2.0, min(2.0, 5 * (-s * dx + c * dy))),
            math.sin(error), math.cos(error) - 1, 2 * vel[0], 2 * vel[1]]


def build_observation(*, angular_velocity_body: Sequence[float],
                      projected_gravity_body: Sequence[float],
                      applied_commands: Sequence[float],
                      joint_position: Sequence[float],
                      joint_velocity: Sequence[float],
                      previous_clipped_action: Sequence[float],
                      position_xy: Sequence[float], reference_xy: Sequence[float],
                      yaw: float, reference_yaw: float,
                      body_velocity_xy: Sequence[float],
                      default_angles: Sequence[float] = SOURCE_DEFAULT_ANGLES,
                      ) -> list[float]:
    angular = finite_vector(angular_velocity_body, 3, "body angular velocity")
    gravity = finite_vector(projected_gravity_body, 3, "projected gravity")
    command = finite_vector(applied_commands, 3, "applied commands")
    q = finite_vector(joint_position, 6, "joint position")
    qd = finite_vector(joint_velocity, 6, "joint velocity")
    default = finite_vector(default_angles, 6, "default angles")
    last = finite_vector(previous_clipped_action, 6, "previous clipped action")
    if last != clip_actions(last):
        raise ValueError("previous action must already use per-channel clipping")
    offsets = [wrap_angle(q[i] - default[i]) if i in (0, 3)
               else q[i] - default[i] for i in LEG_INDICES]
    obs = ([x * 0.25 for x in angular] + gravity
           + [x * scale for x, scale in zip(command, (2.0, 0.25, 5.0))]
           + offsets + [x * 0.05 for x in qd] + last
           + reference_features(position_xy, reference_xy, yaw, reference_yaw,
                                body_velocity_xy))
    # Archive normalization.clip_observations; source noise is disabled.
    return [max(-100.0, min(100.0, value)) for value in obs]


def bottom_clearance(root_height: float, projected_gravity: Sequence[float],
                     box_center: Sequence[float], box_halfsize: Sequence[float]) -> float:
    gravity = finite_vector(projected_gravity, 3, "projected gravity")
    center = finite_vector(box_center, 3, "box center")
    half = finite_vector(box_halfsize, 3, "box halfsize")
    if not math.isfinite(root_height) or min(half) < 0:
        raise ValueError("invalid root height or box halfsize")
    up = [-x for x in gravity]
    return root_height + sum(a * b for a, b in zip(up, center)) - sum(
        abs(a) * b for a, b in zip(up, half))


def action_targets(actions: Sequence[float], knee_limits: Sequence[Sequence[float]],
                   default_angles: Sequence[float] = SOURCE_DEFAULT_ANGLES,
                   ) -> dict[str, list[float]]:
    """Decode targets only. No PD, motor torque mapping, or physics is applied."""
    clipped = clip_actions(actions)
    default = finite_vector(default_angles, 6, "default angles")
    if len(knee_limits) != 2:
        raise ValueError("right and left knee limits are required")
    position = list(default)
    for i in LEG_INDICES:
        position[i] += 0.5 * clipped[i]
    for i, limits in zip((1, 4), knee_limits):
        lo, hi = finite_vector(limits, 2, "knee limits")
        if lo >= hi:
            raise ValueError("invalid knee limits")
        position[i] = max(lo, min(hi, position[i]))
    velocity = [20.0 * clipped[i] if i in WHEEL_INDICES else 0.0
                for i in range(6)]
    return {"clipped_action": clipped, "position": position, "velocity": velocity}
