"""Target-machine profile layered on the frozen reconstructed source model.

The archived files under ``sample_weight_file/`` and the frozen source modules
(``transfer/sample_source.py`` in particular, whose hash is part of the running
31-D training contract) are never edited. This module post-processes the MJCF
string produced by :func:`sample_source.source_xml` and supplies target
hardware/spring dictionaries instead.

Scope: V5 target-profile construction and diagnostics only. No randomization,
no target acceptance and no training entry are provided here.
"""

from __future__ import annotations

import copy
import json
import math
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from ..contracts.sample_actor31 import JOINT_ORDER
from ..paths import configs_dir
from .sample_source import SampleSource, source_module, source_xml, verified_artifact, vec

PROFILE_PATH = configs_dir() / "sample_target_profile.json"


def load_profile(path=PROFILE_PATH):
    profile = json.loads(Path(path).read_text(encoding="utf-8"))
    wheel = profile["wheel"]
    builtin = wheel["builtin_reduction"]["numerator"] / wheel["builtin_reduction"]["denominator"]
    chain = wheel["chain"]["driven_teeth"] / wheel["chain"]["driver_teeth"]
    total = wheel["total_rotor_to_wheel_fraction"]["numerator"] / wheel["total_rotor_to_wheel_fraction"]["denominator"]
    if abs(total - builtin * chain) > 1e-12:
        raise ValueError("target wheel ratio fraction disagrees with built-in x chain")
    if not 0 < wheel["radius_m"]:
        raise ValueError("target wheel radius must be positive")
    spring = profile["spring"]
    if abs(spring["max_length_m"] - spring["min_length_m"] - spring["stroke_m"]) > 1e-9:
        raise ValueError("target spring installed interval must equal the specified stroke")
    if spring["free_length_m"] - spring["stroke_m"] < spring["min_length_m"] - 1e-12:
        raise ValueError("target spring compression stop must be free length minus stroke")
    return profile


def total_wheel_ratio(profile):
    wheel = profile["wheel"]
    return (wheel["total_rotor_to_wheel_fraction"]["numerator"]
            / wheel["total_rotor_to_wheel_fraction"]["denominator"])


def _archive_spring_config():
    return json.loads(verified_artifact("gas_spring_resolved.json").read_text(encoding="utf-8"))


def target_spring_config(profile):
    """Target 450 N / 250 mm free / 75 mm stroke variant of the archived strut."""
    cfg = copy.deepcopy(_archive_spring_config())
    spring = profile["spring"]
    cfg["model_label"] = spring["model"]
    cfg["reported_dimensions"] = {
        "nominal_force_n": spring["reference_force_n"],
        "stroke_m": spring["stroke_m"],
        "free_length_m": spring["free_length_m"],
        "compressed_length_m": spring["compressed_length_m"],
        "nominal_force_reference_length_m": spring["reference_length_m"],
        "note": "Target 1BKB0.45-75 values from progress.md §2.4; anchors retained from the archived resolved geometry.",
        "length_definition": "bare nitrogen spring, excludes robot mounting adapters",
    }
    cfg["installation_measurements"]["note"] = (
        "Target profile reuses the archived URDF link-frame anchors; local CAD pin distances "
        "agree within 0.25 mm but lateral offsets remain unverified.")
    cfg["simulation_fidelity"] = {
        "force_model": "nominal_constant",
        "description": "450 N nominal constant push force with posture-dependent lever arm; target 75 mm stroke.",
        "physical_stroke_enforced": True,
        "reason": "Compression stop at 175 mm is enforced as a knee joint stop in the target MJCF. "
                  "The 250 mm extension end is unreachable (geometric maximum about 249.3 mm). "
                  "Force progression remains uncalibrated.",
    }
    for s in cfg["springs"]:
        s["reference_force_n"] = spring["reference_force_n"]
        s["stroke_m"] = spring["stroke_m"]
        s["min_length_m"] = spring["min_length_m"]
        s["max_length_m"] = spring["max_length_m"]
        s["reference_length_m"] = spring["reference_length_m"]
        s["force_model"] = spring["force_model"]
        s["viscous_damping_ns_m"] = 0.0
        s["length_limit_tolerance_m"] = spring.get("length_limit_tolerance_m", 0.002)
        for stale in ("pre_stroke_joint_limits_rad", "joint_stop_limits_rad", "stroke_limit_analysis"):
            s.pop(stale, None)
    module = source_module("gas_spring")
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, encoding="utf-8") as handle:
        json.dump(cfg, handle)
        path = Path(handle.name)
    try:
        return module.load_config(path, verified_artifact("fifty_training.urdf"))
    finally:
        path.unlink(missing_ok=True)


def knee_stops(profile, spring_config=None):
    """Pin-length 0.175 m knee angles, found on the archived geometry branch."""
    cfg = spring_config or target_spring_config(profile)
    module = source_module("gas_spring")
    minimum = profile["spring"]["min_length_m"]
    result = {}
    for spring in cfg["springs"]:
        name = spring["joint"]
        joint = verified_artifact("fifty_training.urdf")
        root = ET.parse(joint).getroot()
        limit = root.find(f"joint[@name='{name}']/limit")
        lower, upper = float(limit.get("lower")), float(limit.get("upper"))
        f = lambda q: module.geometry(q, spring)[0] - minimum
        a, b = lower, upper
        fa, fb = f(a), f(b)
        if fa * fb > 0:
            raise ValueError(f"{name}: 175 mm stop is not bracketed by the source knee range")
        for _ in range(200):
            middle = (a + b) / 2
            fm = f(middle)
            if fa * fm <= 0:
                b, fb = middle, fm
            else:
                a, fa = middle, fm
        result[name] = (a + b) / 2
    return result


def target_hardware(profile):
    """Archived hardware profile with the target wheel radius and chain ratio."""
    cfg = copy.deepcopy(json.loads(verified_artifact("actuator_hardware.json").read_text(encoding="utf-8")))
    wheel = profile["wheel"]
    total = total_wheel_ratio(profile)
    cfg["wheel_diameter_m"] = 2 * wheel["radius_m"]
    cfg["wheel_motor"]["model"] = (
        f"DJI M3508 with built-in {wheel['builtin_reduction']['numerator']}/"
        f"{wheel['builtin_reduction']['denominator']} and "
        f"{wheel['chain']['driver_teeth']}T->{wheel['chain']['driven_teeth']}T chain")
    cfg["wheel_motor"]["total_reduction_ratio"] = total
    cfg["user_confirmed"]["wheel_total_rotor_to_wheel_ratio"] = total
    cfg["user_confirmed"]["target_profile_note"] = "2026-11-12 target profile derived from progress.md §2.5; archive unchanged"
    return cfg


def target_xml(profile, effective_friction=.75):
    """Source MJCF with target wheel radius, spawn height and knee compression stop."""
    xml, env = source_xml(effective_friction)
    root = ET.fromstring(xml)
    radius = profile["wheel"]["radius_m"]
    seen = []
    for body in root.iter("body"):
        if body.get("name") in ("right_wheel", "left_wheel"):
            for geom in body.findall("geom"):
                if geom.get("type") != "cylinder":
                    continue
                sizes = geom.get("size").split()
                seen.append(float(sizes[0]))
                if abs(float(sizes[0]) - profile["source_wheel_radius_m"]) > 1e-9:
                    raise ValueError("source MJCF wheel radius no longer matches the frozen profile")
                sizes[0] = repr(radius)
                geom.set("size", " ".join(sizes))
    if len(seen) != 2:
        raise ValueError(f"expected two wheel collision cylinders, found {len(seen)}")
    stops = knee_stops(profile)
    for body in root.iter("joint"):
        name = body.get("name")
        if name not in stops:
            continue
        lower, upper = (float(value) for value in body.get("range").split())
        if name == "rf1_Joint":
            lower = max(lower, stops[name])
        else:
            upper = min(upper, stops[name])
        body.set("range", vec([lower, upper]))
    env = copy.deepcopy(env)
    env["init_state"]["pos"][2] += profile["environment"]["spawn_height_delta_m"]
    env["target_profile"] = {"schema": profile["schema"], "wheel_radius_m": radius,
                             "wheel_total_ratio": total_wheel_ratio(profile),
                             "knee_stop_rad": {name: value for name, value in stops.items()}}
    return ET.tostring(root, encoding="unicode"), env


class SampleTargetSource(SampleSource):
    """Mirror of the frozen SampleSource initialisation with target parameters.

    The frozen ``sample_source.py`` hash is part of the active training contract,
    so the target profile is a subclass instead of an edit.
    """

    def __init__(self, effective_friction=.75):
        self.profile = load_profile()
        self.xml, self.env = target_xml(self.profile, effective_friction)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.joint_ids = np.array([self.model.joint(name).id for name in JOINT_ORDER])
        self.qpos_ids = self.model.jnt_qposadr[self.joint_ids]
        self.qvel_ids = self.model.jnt_dofadr[self.joint_ids]
        self.defaults = np.array([self.env["init_state"]["default_joint_angles"][name]
                                  for name in JOINT_ORDER])
        self.actuator_module = source_module("actuator_model")
        self.hardware = target_hardware(self.profile)
        self.spring_module = source_module("gas_spring")
        self.spring_config = target_spring_config(self.profile)
        self.physics_substeps = self.env["sim"]["substeps"]
        self.external_dt = self.model.opt.timestep * self.physics_substeps
        self.policy_dt = self.external_dt * self.env["control"]["decimation"]
        self.reset()
