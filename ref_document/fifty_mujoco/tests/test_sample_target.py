import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from fifty_mujoco.contracts.sample_actor31 import bottom_clearance
from fifty_mujoco.envs.sample_env31_target import SampleTargetEnv31
from fifty_mujoco.transfer.sample_source import SampleSource, source_module
from fifty_mujoco.transfer.sample_target import (SampleTargetSource, knee_stops, load_profile,
                                                 target_hardware, target_spring_config,
                                                 target_xml, total_wheel_ratio)


def test_target_profile_matches_recorded_values():
    profile = load_profile()
    assert profile["wheel"]["radius_m"] == 0.06
    assert abs(total_wheel_ratio(profile) - 68229 / 2992) < 1e-12
    assert abs(total_wheel_ratio(profile) - 3591 / 187 * 19 / 16) < 1e-12
    assert profile["spring"]["reference_force_n"] == 450.
    assert profile["spring"]["max_length_m"] - profile["spring"]["min_length_m"] == pytest.approx(.075)


def test_source_model_stays_frozen():
    source = SampleSource()
    assert float(source.model.geom("right_wheel_collision_0").size[0]) == .055
    assert source.spring_config["springs"][0]["min_length_m"] == pytest.approx(.14801022999909108)
    assert source.spring_config["springs"][0]["stroke_m"] == pytest.approx(.08)


def test_target_xml_changes_only_radius_spawn_and_knee_stop():
    profile = load_profile()
    xml, env = target_xml(profile)
    root = ET.fromstring(xml)
    radii = sorted({float(g.get("size").split()[0]) for b in root.iter("body")
                    if b.get("name") in ("right_wheel", "left_wheel") for g in b.findall("geom")})
    assert radii == [profile["wheel"]["radius_m"]]
    source_clearance = bottom_clearance(.2749762, [0, 0, -1], env["env"]["clearance_box_center"],
                                        env["env"]["clearance_box_halfsize"])
    raised = bottom_clearance(env["init_state"]["pos"][2], [0, 0, -1],
                              env["env"]["clearance_box_center"], env["env"]["clearance_box_halfsize"])
    assert source_clearance == pytest.approx(.10000010658856573)
    assert raised == pytest.approx(profile["environment"]["expected_initial_clearance_m"])
    module = source_module("gas_spring")
    stops = knee_stops(profile)
    spring = {s["joint"]: s for s in target_spring_config(profile)["springs"]}
    for joint, value in stops.items():
        assert module.geometry(value, spring[joint])[0] == pytest.approx(.175, abs=1e-9)
    ranges = {j.get("name"): [float(v) for v in j.get("range").split()]
              for j in root.iter("joint") if j.get("name") in stops}
    assert ranges["rf1_Joint"][0] == pytest.approx(stops["rf1_Joint"])
    assert ranges["lf1_Joint"][1] == pytest.approx(stops["lf1_Joint"])


def test_target_hardware_scales_the_p19_proxy():
    profile = load_profile()
    hardware = target_hardware(profile)
    module = source_module("actuator_model")
    ratings = module.motor_ratings(hardware)
    assert hardware["wheel_diameter_m"] == pytest.approx(.12)
    assert hardware["wheel_motor"]["total_reduction_ratio"] == pytest.approx(68229 / 2992)
    assert ratings["wheel_torque_nm"] == pytest.approx(3.0 * 19 / 16)
    assert ratings["wheel_rated_rad_s"] == pytest.approx(469 * 16 / 19 * math.pi / 30)
    assert ratings["wheel_no_load_rad_s"] == pytest.approx(482 * 16 / 19 * math.pi / 30)
    assert ratings["leg_torque_nm"] == pytest.approx(20.)


def test_target_spring_config_passes_archive_loader():
    profile = load_profile()
    cfg = target_spring_config(profile)
    assert cfg["enabled"] is True
    assert [s["min_length_m"] for s in cfg["springs"]] == [0.175, 0.175]
    assert [s["max_length_m"] for s in cfg["springs"]] == [0.25, 0.25]
    assert all(s["force_model"] == "nominal_constant" for s in cfg["springs"])
    assert all(s["reference_force_n"] == 450. for s in cfg["springs"])


def test_target_env_observations_and_stroke_are_finite():
    env = SampleTargetEnv31(episode_seconds=1.)
    obs = env.reset(1234, (0, 0, .1))
    assert obs.shape == (31,)
    for _ in range(10):
        obs, reward, terminated, truncated, info = env.step(np.zeros(6))
        assert np.isfinite(obs).all() and math.isfinite(reward)
        assert not terminated
        # The allowed numerical penetration is the configured 2 mm tolerance.
        assert all(margin >= -.002 - 1e-9 for margin in info["spring_margins_m"])
    assert abs(env.source.model.geom("left_wheel_collision_0").size[0] - .06) < 1e-12
