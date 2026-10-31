"""The gas spring as a one-dimensional constraint on the leg.

These tests pin down the geometry that decides whether a given spring can reach
a given standing height.  They are the reason the documentation can state the
"free length must be at least 227.760939 mm" requirement as arithmetic instead of an
opinion.
"""

from __future__ import annotations

import math

import pytest

from fifty_mujoco.mechanism.loader import load_mechanism
from fifty_mujoco.mechanism.spring import (
    FLEXION_MAX_RAD,
    FLEXION_MIN_RAD,
    SpringLink,
    load_spring_link,
    wrap_flexion,
)


@pytest.fixture(scope="module")
def link() -> SpringLink:
    loaded = load_spring_link(load_mechanism())
    assert loaded is not None
    return loaded


def test_the_anchors_reproduce_the_derived_assembly_distance(link: SpringLink):
    """194.949622 mm spatial pin distance; planar projection is 194.639038 mm."""
    assert link.pin_distance_mm() == pytest.approx(194.949622, abs=5e-3)
    assert math.degrees(link.delta_at_assembly) == pytest.approx(99.016, abs=5e-3)
    assert link.chord_mm() == pytest.approx(300.243, abs=5e-3)


def test_the_anchor_offsets_match_the_step_audit(link: SpringLink):
    assert math.hypot(*link.upper_local) == pytest.approx(51.000, abs=1e-3)
    assert math.hypot(*link.lower_local) == pytest.approx(48.632, abs=1e-3)


def test_the_pin_distance_grows_as_the_leg_straightens(link: SpringLink):
    """Strictly decreasing from full extension down to the interior minimum."""
    pin_min, delta_min = link.minimum_pin()
    previous = math.inf
    for step in range(37):
        current = link.pin_distance_mm(delta_min * step / 36.0)
        assert current < previous
        previous = current
    assert previous == pytest.approx(pin_min, abs=1e-6)


def test_the_pin_distance_has_a_shallow_interior_minimum(link: SpringLink):
    """175.3 deg, 0.22 mm below the folded limit - the reason pin_min is scanned."""
    pin_min, delta_min = link.minimum_pin()
    assert math.degrees(delta_min) == pytest.approx(175.26, abs=0.02)
    assert pin_min == pytest.approx(152.152021, abs=1e-3)
    assert link.pin_distance_mm(FLEXION_MAX_RAD) > pin_min


def test_the_whole_leg_asks_for_97_mm_of_pin_travel(link: SpringLink):
    travel = link.travel()
    assert travel["pin_min_mm"] == pytest.approx(152.152021, abs=1e-3)
    assert travel["pin_max_mm"] == pytest.approx(249.126126, abs=5e-3)
    assert travel["pin_travel_mm"] == pytest.approx(96.974105, abs=5e-3)
    assert travel["pin_at_folded_limit_mm"] == pytest.approx(152.371079, abs=5e-3)
    assert travel["chord_min_mm"] == pytest.approx(40.0, abs=1e-6)
    assert travel["chord_max_mm"] == pytest.approx(460.0, abs=1e-6)


def test_a_75_mm_stroke_cannot_cover_the_whole_leg(link: SpringLink):
    """The accepted stroke restricts geometry; no mechanical-stop validation implied."""
    assert link.travel()["pin_travel_mm"] > 75.0
    window = link.window(250.0, 75.0, 60.0)
    assert window["covers_full_travel"] is False
    assert window["clamps_flexion"] is True
    assert window["clamps_extension"] is False

def test_chord_and_pin_distance_invert_each_other(link: SpringLink):
    for chord in (100.0, 200.0, 300.243, 404.751, 450.0):
        delta = link.delta_for_chord(chord)
        assert link.chord_mm(delta) == pytest.approx(chord, abs=1e-9)
    for pin in (160.0, 194.949622, 227.760939, 240.0):
        delta = link.delta_for_pin(pin)
        assert link.pin_distance_mm(delta) == pytest.approx(pin, abs=1e-9)


def test_the_stated_250_mm_spring_reaches_the_nominal_height(link: SpringLink):
    """At 250 mm the stance side is free and the stop is at the folded end."""
    window = link.window(250.0, 75.0, 60.0)
    assert window["max_height_m"] == pytest.approx(0.520, abs=5e-4)
    assert window["min_height_m"] == pytest.approx(0.28118965, abs=5e-5)
    assert window["clamps_extension"] is False
    assert window["clamps_flexion"] is True
    assert window["compression_at_assembly_mm"] == pytest.approx(55.050378, abs=5e-3)
    assert link.window(250.0, 75.0, 60.0)["max_height_m"] > 0.4647513721


def test_the_withdrawn_195_mm_reading_is_still_reproducible(link: SpringLink):
    """Kept as a regression: it is why the CAD length was mistaken for the free length."""
    window = link.window(195.0, 75.0, 60.0)
    assert window["max_height_m"] == pytest.approx(0.36042213, abs=5e-5)
    assert window["clamps_extension"] is True
    # 195 mm is essentially the pin spacing the CAD draws at the stowed pose
    assert link.pin_distance_mm() == pytest.approx(194.949622, abs=5e-3)


def test_the_nominal_height_needs_a_free_length_of_at_least_227_761_mm(link: SpringLink):
    """0.4648 m = chord 0.4048 m + 0.06 m wheel, and that chord is spatial pin 227.760939 mm."""
    assert link.window(227.7609, 75.0, 60.0)["max_height_m"] == pytest.approx(0.46475, abs=5e-5)
    assert link.window(227.7, 75.0, 60.0)["max_height_m"] < 0.46475


def test_a_free_length_below_the_folded_pin_distance_is_reported(link: SpringLink):
    window = link.window(150.0, 75.0, 60.0)
    assert window["problems"]
    assert "cannot be installed" in window["problems"][0]


def test_wrap_flexion_accepts_the_negative_assembly_difference():
    """The assembly is 99 deg, but ``shin - thigh`` is -261 deg before wrapping."""
    assert math.degrees(wrap_flexion(math.radians(-261.0))) == pytest.approx(99.0, abs=1e-9)


def test_an_unreachable_chord_is_rejected(link: SpringLink):
    with pytest.raises(ValueError):
        link.delta_for_chord(500.0)


def test_load_spring_link_returns_none_without_endpoints():
    mechanism = load_mechanism()
    stripped = type(mechanism)(
        path=mechanism.path,
        meta=mechanism.meta,
        nodes=mechanism.nodes,
        links=mechanism.links,
        derived=mechanism.derived,
        bodies=mechanism.bodies,
        topology={"chain": mechanism.topology["chain"]},
        unknowns=mechanism.unknowns,
    )
    assert load_spring_link(stripped) is None


def test_the_config_envelope_matches_the_computation(link: SpringLink):
    """`configs/gas_spring.yaml: real_spring.envelope` must not drift from the code."""
    from fifty_mujoco.config import load_yaml
    from fifty_mujoco.paths import configs_dir

    block = load_yaml(configs_dir() / "gas_spring.yaml")["real_spring"]
    envelope = block["envelope"]
    travel = link.travel()
    assert envelope["pin_min_mm"] == pytest.approx(travel["pin_min_mm"], abs=5e-4)
    assert envelope["pin_max_mm"] == pytest.approx(travel["pin_max_mm"], abs=5e-4)
    assert envelope["pin_travel_mm"] == pytest.approx(travel["pin_travel_mm"], abs=5e-4)
    assert envelope["pin_at_assembly_mm"] == pytest.approx(
        travel["pin_at_assembly_mm"], abs=5e-4
    )
    assert envelope["delta_at_assembly_deg"] == pytest.approx(
        math.degrees(link.delta_at_assembly), abs=5e-4
    )
    assert envelope["chord_min_mm"] == pytest.approx(travel["chord_min_mm"], abs=5e-4)
    assert envelope["chord_max_mm"] == pytest.approx(travel["chord_max_mm"], abs=5e-4)


def test_the_config_window_matches_the_computation(link: SpringLink):
    from fifty_mujoco.config import load_yaml
    from fifty_mujoco.paths import configs_dir

    block = load_yaml(configs_dir() / "gas_spring.yaml")["real_spring"]
    datasheet = block["datasheet"]
    window = link.window(
        datasheet["free_length_mm"], datasheet["max_working_stroke_mm"], 60.0
    )
    stated = block["window_as_stated"]
    assert stated["max_chord_mm"] == pytest.approx(window["max_chord_mm"], abs=5e-4)
    assert stated["min_chord_mm"] == pytest.approx(window["min_chord_mm"], abs=5e-4)
    assert stated["max_height_m"] == pytest.approx(window["max_height_m"], abs=5e-6)
    assert stated["min_height_m"] == pytest.approx(window["min_height_m"], abs=5e-6)
    assert stated["clamps_extension"] is window["clamps_extension"]
    assert stated["clamps_flexion"] is window["clamps_flexion"]
    assert stated["compression_at_nominal_height_mm"] == pytest.approx(
        250.0 - link.pin_distance_mm(link.delta_for_chord(404.7513721)), abs=5e-3
    )
    assert datasheet["free_length_mm"] == pytest.approx(250.0)


def test_spatial_distance_includes_axial_offset_and_preserves_planar_evidence(link):
    from dataclasses import replace
    assert link.pin_axis_offset_mm == 11.0
    assert link.planar_pin_distance_mm() == pytest.approx(194.639037525, abs=1e-6)
    planar = replace(link, pin_axis_offset_mm=0.0)
    for delta in (0.0, 0.5, 1.0, 2.0, math.pi):
        assert link.pin_distance_mm(delta) ** 2 == pytest.approx(
            planar.pin_distance_mm(delta) ** 2 + 11.0 ** 2, abs=1e-8
        )
    with pytest.raises(ValueError):
        replace(link, pin_axis_offset_mm=-11.0)


def test_remaining_compression_is_not_compression_already_used(link):
    pin = link.pin_distance_mm(link.delta_for_chord(404.7513721610853))
    used, remaining = 250.0 - pin, pin - 175.0
    assert pin == pytest.approx(227.760939473, abs=1e-6)
    assert used == pytest.approx(22.239060527, abs=1e-6)
    assert remaining == pytest.approx(52.760939473, abs=1e-6)
    assert used + remaining == pytest.approx(75.0, abs=1e-9)


def test_invalid_and_disjoint_stroke_windows_are_not_feasible(link):
    with pytest.raises(ValueError):
        link.window(250.0, -1.0, 60.0)
    with pytest.raises(ValueError):
        link.window(math.nan, 75.0, 60.0)
    assert link.window(100.0, 75.0, 60.0)["feasible"] is False
    assert link.window(400.0, 75.0, 60.0)["feasible"] is False
    assert link.window(250.0, 75.0, 60.0)["feasible"] is True
