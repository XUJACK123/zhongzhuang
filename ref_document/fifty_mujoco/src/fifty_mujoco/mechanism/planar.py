"""Planar distance-constraint solver for closed chains.

Why a solver instead of closed-form kinematics: the mechanism in
``configs/mechanism.yaml`` is a two-loop, two-DOF planar chain. Writing a
bespoke formula per loop would hard-code the very body assignment that is still
unconfirmed. A generic solver over distance constraints keeps the geometry data
in charge:

* nodes are 2D points, variables are the free ones;
* a rigid body is a set of distance constraints between its nodes;
* a revolute joint is a node shared by two bodies;
* a driven degree of freedom pins a node.

Everything is standard library (``math`` only), so the geometry stays verifiable
on a machine without numpy/MuJoCo.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Mapping, Sequence

Vec2 = tuple[float, float]


class NoSolutionError(ValueError):
    """Two circles do not intersect, or a loop cannot be closed."""


class TopologyNotConfirmedError(RuntimeError):
    """A candidate body assignment was about to be used without opt-in.

    The body assignment in ``mechanism.yaml`` is inferred from coincident bore
    coordinates and a parallelogram identity. That is strong evidence, not a
    confirmed topology, so building a controller or a model from it is refused
    unless the caller says so explicitly.
    """


def distance(a: Vec2, b: Vec2) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def angle_of(vector: Vec2) -> float:
    return math.atan2(vector[1], vector[0])


def unit(theta: float) -> Vec2:
    return (math.cos(theta), math.sin(theta))


def circle_intersection(
    centre_a: Vec2,
    radius_a: float,
    centre_b: Vec2,
    radius_b: float,
    tolerance: float = 1e-12,
) -> list[Vec2]:
    """Intersection points of two circles, in a deterministic order.

    Returns 0, 1 or 2 points. Tangent circles return one point; concentric
    circles, or circles that cannot reach, return an empty list - the caller
    must decide whether that is an infeasible pose or a bug.
    """
    delta = (centre_b[0] - centre_a[0], centre_b[1] - centre_a[1])
    separation = math.hypot(*delta)
    if separation < tolerance:
        return []
    if separation > radius_a + radius_b + tolerance:
        return []
    if separation < abs(radius_a - radius_b) - tolerance:
        return []
    along = (radius_a * radius_a - radius_b * radius_b + separation * separation) / (
        2.0 * separation
    )
    height_sq = radius_a * radius_a - along * along
    height = math.sqrt(max(height_sq, 0.0))
    base = (
        centre_a[0] + along * delta[0] / separation,
        centre_a[1] + along * delta[1] / separation,
    )
    if height <= tolerance:
        return [base]
    offset = (-delta[1] / separation * height, delta[0] / separation * height)
    return [(base[0] + offset[0], base[1] + offset[1]), (base[0] - offset[0], base[1] - offset[1])]


@dataclass
class SolveResult:
    """Outcome of one closure solve. ``converged`` is a claim about residuals."""

    positions: dict[str, Vec2]
    iterations: int
    max_residual: float
    residual_norm: float
    converged: bool
    history: list[float] = field(default_factory=list)

    def as_dict(self) -> dict[str, object]:
        return {
            "iterations": self.iterations,
            "max_residual_m": self.max_residual,
            "residual_norm_m": self.residual_norm,
            "converged": self.converged,
            "node_positions_m": {
                name: [round(value[0], 12), round(value[1], 12)]
                for name, value in self.positions.items()
            },
        }


def _solve_linear(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    """Gaussian elimination with partial pivoting (small dense systems only)."""
    size = len(rhs)
    rows = [row[:] + [rhs[index]] for index, row in enumerate(matrix)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(rows[row][column]))
        if abs(rows[pivot][column]) < 1e-18:
            raise NoSolutionError("singular normal equations: the constraint set is degenerate")
        rows[column], rows[pivot] = rows[pivot], rows[column]
        divisor = rows[column][column]
        rows[column] = [value / divisor for value in rows[column]]
        for row in range(size):
            if row == column:
                continue
            factor = rows[row][column]
            if factor == 0.0:
                continue
            rows[row] = [
                value - factor * pivot_value
                for value, pivot_value in zip(rows[row], rows[column], strict=True)
            ]
    return [rows[row][size] for row in range(size)]


class PlanarSystem:
    """A planar chain described purely by node coordinates and distances.

    ``fixed`` nodes are not variables (the ground, and any driven node whose
    position already follows from the input angles). ``constraints`` are
    ``(node_a, node_b, length)`` triples in metres.
    """

    def __init__(
        self,
        nodes: Mapping[str, Vec2],
        constraints: Iterable[tuple[str, str, float]],
        fixed: Iterable[str] = (),
    ) -> None:
        self.nodes = dict(nodes)
        self.constraints: list[tuple[str, str, float]] = []
        self.fixed = tuple(fixed)
        for node_a, node_b, length in constraints:
            for node in (node_a, node_b):
                if node not in self.nodes:
                    raise KeyError(f"constraint references unknown node {node!r}")
            if not length > 0.0:
                raise ValueError(f"constraint {node_a}-{node_b}: length must be positive")
            self.constraints.append((node_a, node_b, float(length)))
        missing = [node for node in self.fixed if node not in self.nodes]
        if missing:
            raise KeyError(f"fixed nodes not in the system: {missing}")
        self.variables: list[str] = [
            name for name in self.nodes if name not in set(self.fixed)
        ]

    # -- description ------------------------------------------------------
    @property
    def degrees_of_freedom(self) -> int:
        return len(self.variables) * 2 - len(self.constraints)

    # -- residuals --------------------------------------------------------
    def _positions(self, values: Sequence[float]) -> dict[str, Vec2]:
        positions = {name: self.nodes[name] for name in self.fixed}
        for index, name in enumerate(self.variables):
            positions[name] = (values[2 * index], values[2 * index + 1])
        return positions

    def _flatten(self, positions: Mapping[str, Vec2]) -> list[float]:
        values: list[float] = []
        for name in self.variables:
            point = positions[name]
            values.extend((float(point[0]), float(point[1])))
        return values

    def residuals(self, values: Sequence[float]) -> list[float]:
        positions = self._positions(values)
        residuals: list[float] = []
        for node_a, node_b, length in self.constraints:
            residuals.append(distance(positions[node_a], positions[node_b]) - length)
        return residuals

    def jacobian(self, values: Sequence[float]) -> list[list[float]]:
        positions = self._positions(values)
        index = {name: position for position, name in enumerate(self.variables)}
        matrix = [[0.0] * (2 * len(self.variables)) for _ in self.constraints]
        for row, (node_a, node_b, _length) in enumerate(self.constraints):
            pa, pb = positions[node_a], positions[node_b]
            delta = (pb[0] - pa[0], pb[1] - pa[1])
            separation = math.hypot(*delta)
            if separation <= 0.0:
                continue
            direction = (delta[0] / separation, delta[1] / separation)
            if node_a in index:
                column = 2 * index[node_a]
                matrix[row][column] -= direction[0]
                matrix[row][column + 1] -= direction[1]
            if node_b in index:
                column = 2 * index[node_b]
                matrix[row][column] += direction[0]
                matrix[row][column + 1] += direction[1]
        return matrix

    # -- solving ----------------------------------------------------------
    def closure_error(self, positions: Mapping[str, Vec2]) -> float:
        """Largest |distance - target| over all constraints, metres."""
        worst = 0.0
        for node_a, node_b, length in self.constraints:
            worst = max(worst, abs(distance(positions[node_a], positions[node_b]) - length))
        return worst

    def solve(
        self,
        initial: Mapping[str, Vec2] | None = None,
        max_iterations: int = 80,
        tolerance: float = 1e-12,
    ) -> SolveResult:
        """Levenberg-Marquardt on the distance residuals.

        Damped least squares rather than plain Newton: a trial pose can put the
        chain at a singularity or on the wrong branch, and a bare Newton step
        then diverges silently instead of reporting failure.
        """
        start = dict(self.nodes) if initial is None else dict(initial)
        values = self._flatten(start)
        residuals = self.residuals(values)
        cost = sum(value * value for value in residuals)
        damping = 1e-6
        history: list[float] = [max((abs(value) for value in residuals), default=0.0)]
        iterations = 0
        for iterations in range(1, max_iterations + 1):
            if max((abs(value) for value in residuals), default=0.0) <= tolerance:
                iterations -= 1
                break
            matrix = self.jacobian(values)
            size = len(values)
            normal = [[0.0] * size for _ in range(size)]
            gradient = [0.0] * size
            for row, residual in zip(matrix, residuals, strict=True):
                for i in range(size):
                    if row[i] == 0.0:
                        continue
                    gradient[i] += row[i] * residual
                    for j in range(i, size):
                        if row[j] != 0.0:
                            normal[i][j] += row[i] * row[j]
            for i in range(size):
                for j in range(i):
                    normal[i][j] = normal[j][i]
                normal[i][i] += damping * (1.0 + abs(normal[i][i]))
            try:
                step = _solve_linear(normal, [-value for value in gradient])
            except NoSolutionError:
                break
            candidate = [value + delta for value, delta in zip(values, step, strict=True)]
            candidate_residuals = self.residuals(candidate)
            candidate_cost = sum(value * value for value in candidate_residuals)
            if candidate_cost < cost:
                values, residuals, cost = candidate, candidate_residuals, candidate_cost
                damping = max(damping / 3.0, 1e-12)
            else:
                damping *= 10.0
                if damping > 1e12:
                    break
            history.append(max((abs(value) for value in residuals), default=0.0))
        max_residual = max((abs(value) for value in residuals), default=0.0)
        return SolveResult(
            positions=self._positions(values),
            iterations=iterations,
            max_residual=max_residual,
            residual_norm=math.sqrt(sum(value * value for value in residuals)),
            converged=max_residual <= max(tolerance, 1e-12),
            history=[round(value, 15) for value in history],
        )
