"""Planar solver, serial leg and the two candidate closed chains.

The solver tests are synthetic on purpose: a four-bar with a known solution is
the only way to prove the residual maths, before any CAD number is involved.
"""

from __future__ import annotations

import math

import pytest

from fifty_mujoco.mechanism import load_mechanism
from fifty_mujoco.mechanism.check import mechanism_report, summarize
from fifty_mujoco.mechanism.leg import TwoLinkLeg, body_constraints
from fifty_mujoco.mechanism.planar import (
    NoSolutionError,
    PlanarSystem,
    circle_intersection,
    distance,
)
from fifty_mujoco.mechanism.pose import MechanismPose
from fifty_mujoco.mechanism.transmission import ThighCrankTransmission


@pytest.fixture(scope="module")
def mechanism():
    return load_mechanism()


# ------------------------------------------------------------------ circles
def test_circle_intersection_returns_two_points_in_a_stable_order():
    points = circle_intersection((0.0, 0.0), 5.0, (8.0, 0.0), 5.0)
    assert len(points) == 2
    assert points[0] == pytest.approx((4.0, 3.0), abs=1e-12)
    assert points[1] == pytest.approx((4.0, -3.0), abs=1e-12)
    assert circle_intersection((0.0, 0.0), 5.0, (8.0, 0.0), 5.0) == points


def test_circle_intersection_handles_tangent_and_disjoint_cases():
    tangent = circle_intersection((0.0, 0.0), 5.0, (10.0, 0.0), 5.0)
    assert len(tangent) == 1
    assert tangent[0] == pytest.approx((5.0, 0.0), abs=1e-9)
    assert circle_intersection((0.0, 0.0), 1.0, (10.0, 0.0), 1.0) == []
    assert circle_intersection((0.0, 0.0), 1.0, (0.0, 0.0), 1.0) == []
    # one circle inside the other, never touching
    assert circle_intersection((0.0, 0.0), 10.0, (1.0, 0.0), 1.0) == []


# ------------------------------------------------------------------ solver
def test_planar_solver_closes_a_four_bar_from_a_wrong_start():
    ground = {"A": (0.0, 0.0), "D": (1.0, 0.0)}
    nodes = {**ground, "B": (0.1, 0.9), "C": (0.9, 1.0)}  # deliberately wrong
    system = PlanarSystem(
        nodes=nodes,
        constraints=[("A", "B", 0.5), ("B", "C", 1.0), ("C", "D", 0.6)],
        fixed=("A", "D"),
    )
    assert system.degrees_of_freedom == 1
    result = system.solve(max_iterations=200)
    assert result.converged
    assert result.max_residual <= 1e-12
    assert distance(result.positions["A"], result.positions["B"]) == pytest.approx(0.5, abs=1e-12)
    assert distance(result.positions["B"], result.positions["C"]) == pytest.approx(1.0, abs=1e-12)
    assert distance(result.positions["C"], result.positions["D"]) == pytest.approx(0.6, abs=1e-12)


def test_planar_solver_reports_an_impossible_loop_instead_of_inventing_one():
    system = PlanarSystem(
        nodes={"A": (0.0, 0.0), "B": (0.1, 0.0)},
        constraints=[("A", "B", 1.0)],
        fixed=("A",),
    )
    # A chain of one distance is trivially satisfiable; make it impossible by
    # fixing both ends too far apart for the middle link.
    impossible = PlanarSystem(
        nodes={"A": (0.0, 0.0), "B": (5.0, 0.0), "C": (0.5, 0.1)},
        constraints=[("A", "C", 1.0), ("C", "B", 1.0)],
        fixed=("A", "B"),
    )
    assert system.solve().converged
    result = impossible.solve(max_iterations=50)
    assert not result.converged
    assert result.max_residual > 1e-3


# --------------------------------------------------------------------- leg
def test_two_link_leg_round_trip_is_exact_inside_the_annulus():
    leg = TwoLinkLeg(thigh_m=0.210, shin_m=0.250)
    hip = (0.0, 0.0)
    for radius in (0.05, 0.2, 0.35, 0.459):
        for index in range(8):
            angle = 2.0 * math.pi * index / 8
            target = (radius * math.cos(angle), radius * math.sin(angle))
            assert leg.closure_residual(hip, target) <= 1e-15


def test_two_link_leg_refuses_an_unreachable_target():
    leg = TwoLinkLeg(thigh_m=0.210, shin_m=0.250)
    with pytest.raises(NoSolutionError):
        leg.inverse((0.0, 0.0), (0.9, 0.0))
    with pytest.raises(NoSolutionError):
        leg.inverse((0.0, 0.0), (0.001, 0.0))  # inside the dead zone


def test_two_link_leg_branches_are_distinct_but_reach_the_same_target():
    leg = TwoLinkLeg(thigh_m=0.210, shin_m=0.250)
    hip = (0.0, 0.0)
    target = (-0.30, 0.05)
    first = leg.forward(hip, *leg.inverse(hip, target, branch=0))
    second = leg.forward(hip, *leg.inverse(hip, target, branch=1))
    assert first["wheel"] == pytest.approx(target, abs=1e-12)
    assert second["wheel"] == pytest.approx(target, abs=1e-12)
    assert distance(first["knee"], second["knee"]) > 0.1


# ---------------------------------------------------------- closed chains
def test_both_assignments_reproduce_the_cad_pose(mechanism):
    nodes = mechanism.assembly_pose_m()
    variants = {
        "link_a_on_shin": MechanismPose.from_mechanism(mechanism),
        "link_a_on_thigh": ThighCrankTransmission.from_mechanism(mechanism),
    }
    for name, pose in variants.items():
        thigh, crank = pose.cad_angles()
        produced = pose.forward(thigh, crank)
        error = max(distance(produced[node], nodes[node]) for node in nodes)
        assert error <= 1e-9, name


def test_each_assignment_closes_against_its_own_body_map(mechanism):
    nodes = mechanism.assembly_pose_m()
    variants = {
        "link_a_on_shin": MechanismPose.from_mechanism(mechanism),
        "link_a_on_thigh": ThighCrankTransmission.from_mechanism(mechanism),
    }
    bodies = {entry["name"]: entry["bodies"] for entry in mechanism.topology["chain"]["variants"]}
    for name, pose in variants.items():
        constraints = body_constraints(mechanism, bodies[name])
        thigh, crank = pose.cad_angles()
        for d_thigh, d_crank in ((0.0, 0.0), (0.05, 0.0), (0.0, -0.05), (0.04, 0.04)):
            produced = pose.forward(thigh + d_thigh, crank + d_crank)
            worst = max(
                abs(distance(produced[a], produced[b]) - length)
                for a, b, length in constraints
            )
            assert worst <= 1e-9, (name, d_thigh, d_crank, worst)


def test_the_two_assignments_are_not_equivalent(mechanism):
    """A discrimination that the CAD mates have to settle.

    Both assignments fit the pivots, but they differ in how much of a ±0.15 rad
    grid still closes: the ternary-on-thigh chain closes everywhere, the
    ternary-on-shin chain locks up outside a band. This test records the
    difference so nobody silently "simplifies" one of them away.
    """
    report = mechanism_report(mechanism, allow_inferred=True)
    by_name = {entry["variant"]: entry for entry in report["closed_chain"]["variants"]}
    lifting = by_name["link_a_on_thigh"]["grid"]["closed_fraction"]
    ternary_on_shin = by_name["link_a_on_shin"]["grid"]["closed_fraction"]
    assert lifting == pytest.approx(1.0)
    assert ternary_on_shin < 0.9
    assert set(report["closed_chain"]["closing_variants"]) == {
        "link_a_on_thigh",
        "link_a_on_shin",
    }


def test_report_refuses_the_chain_by_default_and_says_why(mechanism):
    report = mechanism_report(mechanism)
    closed_chain = report["closed_chain"]
    assert closed_chain["attempted"] is False
    assert closed_chain["status"] == "not_instantiated"
    assert "candidate_inferred" in closed_chain["reason"]
    assert report["passed"] is True
    text = summarize(report)
    assert "not_instantiated" in text
