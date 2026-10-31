"""Gas-spring pin distance, derived from the confirmed anchors.

The two clevis pins sit on the thigh and the shin, so the pin-to-pin distance is
a function of the knee angle **alone**: rotate the whole leg and nothing changes,
because the pin vector is expressed in the rotating thigh frame.  That turns the
spring into a one-dimensional constraint on the chord, which is what decides

* how much pin travel the linkage asks for over its whole reachable range, and
* whether a spring with a given free length and stroke can reach a given
  standing height at all.

Both questions are pure geometry, so they are answered here instead of being
guessed from the assembly pose. A first force model may use an explicitly
approximate 450 N constant force; a measured curve is a later refinement.
The running old profile remains 300 N until the real anchors are integrated.

The knee angle is ``delta = shin - thigh`` unwrapped into ``[0, pi]``: that is
the branch the mechanism actually uses (assembly delta = 99 deg) and the one
``mechanism/bridge.py`` accepts.  The chord decreases monotonically. Pin spacing has a shallow interior minimum,
so its inverse uses only the branch from extension to that minimum.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Mapping

from .resolve import resolve_nodes

if TYPE_CHECKING:
    from .loader import Mechanism

Vec2 = tuple[float, float]

#: The assembly pose sits at 99 deg of knee flexion; the folded limit is 180 deg.
FLEXION_MIN_RAD = 0.0
FLEXION_MAX_RAD = math.pi


def _rotate(angle: float, vector: Vec2) -> Vec2:
    cos, sin = math.cos(angle), math.sin(angle)
    return (cos * vector[0] - sin * vector[1], sin * vector[0] + cos * vector[1])


def _angle(from_point: Vec2, to_point: Vec2) -> float:
    return math.atan2(to_point[1] - from_point[1], to_point[0] - from_point[0])


def wrap_flexion(delta: float) -> float:
    """Unwrap an angle difference into ``[0, 2*pi)``."""
    return delta % (2.0 * math.pi)


@dataclass(frozen=True)
class SpringLink:
    """One gas spring between a thigh anchor and a shin anchor.

    All lengths are millimetres and all angles radians, matching the planar
    ``(y, z)`` frame used by ``configs/mechanism.yaml``.
    """

    proximal: Vec2
    thigh_mm: float
    shin_mm: float
    upper_local: Vec2
    lower_local: Vec2
    delta_at_assembly: float
    pin_axis_offset_mm: float = 0.0

    def __post_init__(self):
        if not math.isfinite(self.pin_axis_offset_mm) or self.pin_axis_offset_mm < 0.0:
            raise ValueError("pin_axis_offset_mm must be finite and non-negative")

    # ------------------------------------------------------------------ state
    def planar_pin_distance_mm(self, delta: float | None = None) -> float:
        """YZ projection only; do not compare this distance with physical stroke."""
        moment = wrap_flexion(self.delta_at_assembly if delta is None else delta)
        lower = _rotate(moment, self.lower_local)
        return math.hypot(
            self.thigh_mm + lower[0] - self.upper_local[0],
            lower[1] - self.upper_local[1],
        )

    def pin_distance_mm(self, delta: float | None = None) -> float:
        """Physical 3D pin-centre distance, including the axial installation offset."""
        return math.hypot(self.planar_pin_distance_mm(delta), self.pin_axis_offset_mm)

    def chord_mm(self, delta: float | None = None) -> float:
        """Distal-node distance (hip to wheel) at a knee flexion angle."""
        moment = wrap_flexion(self.delta_at_assembly if delta is None else delta)
        return math.hypot(
            self.thigh_mm + self.shin_mm * math.cos(moment),
            self.shin_mm * math.sin(moment),
        )

    # ------------------------------------------------------------- inversion
    def minimum_pin(self, samples: int = 361) -> tuple[float, float]:
        """``(pin_min, delta_at_min)`` over the reachable flexion range.

        The pin distance is *not* monotone all the way to the folded limit: it
        bottoms out at 175.3 deg and then rises 0.22 mm by 180 deg, because the
        two anchors are offset on opposite sides of their links.  The dip is tiny
        but it matters, because it is what makes ``pin_min`` well defined and
        what keeps :meth:`delta_for_pin` single-valued.

        A coarse scan brackets the dip, then a ternary search refines it, so the
        answer does not depend on the scan resolution.
        """
        step = (FLEXION_MAX_RAD - FLEXION_MIN_RAD) / (samples - 1)
        best_index = 0
        best_pin = math.inf
        for index in range(samples):
            pin = self.pin_distance_mm(FLEXION_MIN_RAD + step * index)
            if pin < best_pin:
                best_pin, best_index = pin, index
        low = FLEXION_MIN_RAD + step * max(best_index - 1, 0)
        high = FLEXION_MIN_RAD + step * min(best_index + 1, samples - 1)
        for _ in range(200):
            left = low + (high - low) / 3.0
            right = high - (high - low) / 3.0
            if self.pin_distance_mm(left) < self.pin_distance_mm(right):
                high = right
            else:
                low = left
        delta = 0.5 * (low + high)
        return self.pin_distance_mm(delta), delta

    def delta_for_chord(self, chord_mm: float, iterations: int = 200) -> float:
        """Knee flexion that puts the distal node at ``chord_mm`` from the proximal one."""
        low, high = FLEXION_MIN_RAD, FLEXION_MAX_RAD
        if not self.chord_mm(high) - 1e-9 <= chord_mm <= self.chord_mm(low) + 1e-9:
            raise ValueError(
                f"chord {chord_mm:.3f} mm is outside the reachable "
                f"[{self.chord_mm(high):.3f}, {self.chord_mm(low):.3f}] mm"
            )
        for _ in range(iterations):
            middle = 0.5 * (low + high)
            if self.chord_mm(middle) > chord_mm:
                low = middle
            else:
                high = middle
        return 0.5 * (low + high)

    def delta_for_pin(self, pin_mm: float, iterations: int = 200) -> float:
        """Knee flexion that puts the two pins ``pin_mm`` apart.

        Only the monotone part of the range is used - from full extension to the
        interior minimum - so the answer is unique.
        """
        pin_min, delta_min = self.minimum_pin()
        extended = self.pin_distance_mm(FLEXION_MIN_RAD)
        if not pin_min - 1e-9 <= pin_mm <= extended + 1e-9:
            raise ValueError(
                f"pin distance {pin_mm:.3f} mm is outside the linkage's "
                f"[{pin_min:.3f}, {extended:.3f}] mm"
            )
        low, high = FLEXION_MIN_RAD, delta_min
        for _ in range(iterations):
            middle = 0.5 * (low + high)
            if self.pin_distance_mm(middle) > pin_mm:
                low = middle
            else:
                high = middle
        return 0.5 * (low + high)

    # ---------------------------------------------------------------- reports
    def travel(self) -> dict[str, object]:
        """Pin travel the linkage asks for over its whole reachable range."""
        pin_min, delta_min = self.minimum_pin()
        return {
            "pin_min_mm": pin_min,
            "delta_at_min_pin_deg": math.degrees(delta_min),
            "pin_max_mm": self.pin_distance_mm(FLEXION_MIN_RAD),
            "pin_travel_mm": self.pin_distance_mm(FLEXION_MIN_RAD) - pin_min,
            "pin_at_folded_limit_mm": self.pin_distance_mm(FLEXION_MAX_RAD),
            "chord_min_mm": self.chord_mm(FLEXION_MAX_RAD),
            "chord_max_mm": self.chord_mm(FLEXION_MIN_RAD),
            "pin_at_assembly_mm": self.pin_distance_mm(),
            "planar_pin_at_assembly_mm": self.planar_pin_distance_mm(),
            "pin_axis_offset_mm": self.pin_axis_offset_mm,
            "distance_metric": "spatial_pin_center_distance",
            "chord_at_assembly_mm": self.chord_mm(),
        }

    def window(
        self, free_length_mm: float, stroke_mm: float, wheel_radius_mm: float
    ) -> dict[str, object]:
        """Chord / height window a spring of this free length and stroke allows.

        A gas spring cannot be longer than its free length and cannot be
        compressed by more than its stroke, so the pin distance is confined to
        ``[free - stroke, free]``.  Because the pin distance grows with the chord,
        the free length clamps the *straightest* pose the leg can reach and the
        stroke clamps the most folded one.  Whichever end is clamped is reported
        as a problem: a spring that limits the leg is a design decision, not an
        accident, and it has to be visible.
        """
        if not all(math.isfinite(v) for v in (free_length_mm, stroke_mm, wheel_radius_mm)):
            raise ValueError("spring dimensions must be finite")
        if free_length_mm <= 0.0 or not 0.0 < stroke_mm <= free_length_mm or wheel_radius_mm < 0.0:
            raise ValueError("invalid free length, stroke or wheel radius")
        travel = self.travel()
        pin_min = travel["pin_min_mm"]
        pin_max = travel["pin_max_mm"]
        pin_low = free_length_mm - stroke_mm
        problems: list[str] = []

        if free_length_mm >= pin_max:
            max_chord = travel["chord_max_mm"]
            clamps_extension = False
        elif free_length_mm > pin_min:
            max_chord = self.chord_mm(self.delta_for_pin(free_length_mm))
            clamps_extension = True
        else:
            max_chord = travel["chord_min_mm"]
            clamps_extension = True
            problems.append(
                f"free length {free_length_mm:.1f} mm is shorter than the most "
                f"folded pin distance {pin_min:.1f} mm: the spring cannot be "
                "installed at full flexion"
            )

        if pin_low <= pin_min:
            min_chord = travel["chord_min_mm"]
            clamps_flexion = False
        elif pin_low < pin_max:
            min_chord = self.chord_mm(self.delta_for_pin(pin_low))
            clamps_flexion = True
            problems.append(
                f"at the {stroke_mm:.1f} mm stroke limit the pin sits at "
                f"{pin_low:.1f} mm, so the leg cannot fold past chord "
                f"{min_chord:.1f} mm"
            )
        else:
            min_chord = travel["chord_max_mm"]
            clamps_flexion = True
            problems.append(
                f"free length minus stroke ({pin_low:.1f} mm) exceeds the "
                f"linkage's longest pin distance ({pin_max:.1f} mm)"
            )

        feasible = max(pin_low, pin_min) <= min(free_length_mm, pin_max)
        return {
            "feasible": feasible,
            "scope": "geometry_only; ignores collision, other joint limits and dynamics",
            "distance_metric": "spatial_pin_center_distance",
            "free_length_mm": free_length_mm,
            "stroke_mm": stroke_mm,
            "usable_pin_mm": (max(pin_low, pin_min), min(free_length_mm, pin_max)),
            "compression_at_assembly_mm": free_length_mm - travel["pin_at_assembly_mm"],
            "compression_at_full_extension_mm": free_length_mm - pin_max,
            "compression_at_full_flexion_mm": free_length_mm - travel["pin_at_folded_limit_mm"],
            "compression_at_minimum_pin_mm": free_length_mm - pin_min,
            "covers_full_travel": not clamps_extension and not clamps_flexion,
            "clamps_extension": clamps_extension,
            "clamps_flexion": clamps_flexion,
            "min_chord_mm": min_chord,
            "max_chord_mm": max_chord,
            "min_height_m": (min_chord + wheel_radius_mm) / 1000.0,
            "max_height_m": (max_chord + wheel_radius_mm) / 1000.0,
            "problems": problems,
        }


def load_spring_link(mechanism: Mechanism) -> SpringLink | None:
    """Build the :class:`SpringLink` from ``configs/mechanism.yaml``.

    Returns ``None`` when the file carries no ``topology.gas_spring.endpoints``
    block, so older configs keep loading.
    """
    block = (mechanism.topology.get("gas_spring") or {}) if mechanism.topology else {}
    endpoints = block.get("endpoints") or {}
    upper = endpoints.get("upper") or {}
    lower = endpoints.get("lower") or {}
    if not upper.get("point_yz_mm") or not lower.get("point_yz_mm"):
        return None

    resolved = resolve_nodes(mechanism)
    hip = resolved.nodes_mm["HIP"]
    knee = resolved.nodes_mm["KNEE"]
    wheel = resolved.nodes_mm["WHEEL"]
    thigh_angle = _angle(hip, knee)
    shin_angle = _angle(knee, wheel)

    upper_point = (float(upper["point_yz_mm"][0]), float(upper["point_yz_mm"][1]))
    lower_point = (float(lower["point_yz_mm"][0]), float(lower["point_yz_mm"][1]))
    upper_local = _rotate(-thigh_angle, (upper_point[0] - hip[0], upper_point[1] - hip[1]))
    lower_local = _rotate(-shin_angle, (lower_point[0] - knee[0], lower_point[1] - knee[1]))

    return SpringLink(
        proximal=hip,
        thigh_mm=resolved.thigh_mm,
        shin_mm=resolved.shin_mm,
        upper_local=upper_local,
        lower_local=lower_local,
        delta_at_assembly=wrap_flexion(shin_angle - thigh_angle),
        pin_axis_offset_mm=float(block["pin_axis_offset_mm"]),
    )


def describe(link: SpringLink, wheel_radius_mm: float = 60.0) -> Mapping[str, object]:
    """Everything the reports and the docs need, in one JSON-able mapping."""
    travel = link.travel()
    return {
        "delta_at_assembly_deg": math.degrees(link.delta_at_assembly),
        **travel,
        "upper_offset_from_proximal_mm": round(math.hypot(*link.upper_local), 6),
        "lower_offset_from_distal_mm": round(math.hypot(*link.lower_local), 6),
        "wheel_radius_mm": wheel_radius_mm,
    }
