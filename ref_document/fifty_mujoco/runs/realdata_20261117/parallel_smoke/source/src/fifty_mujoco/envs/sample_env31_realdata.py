"""Isolated 17 November data candidate; never patches production assets.

Catalogue force and rotor inertia remain hypotheses. Conservative inherited
knee limits and the 20/2 Nm motor command caps are retained.
"""
from __future__ import annotations

import ctypes as c
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import tempfile

import mujoco
import numpy as np

from .sample_env31_qualified import QualifiedHardwareEnv31
from .sample_env31_hardware import rpy_from_wxyz
from ..contracts.sample_actor31 import (action_targets, advance_reference, bottom_clearance,
                                        clip_actions, ramp_commands)
from ..paths import PROJECT_ROOT
from ..transfer.firmware_runtime import load_runtime


CONFIG = PROJECT_ROOT / "configs/realdata_20261117.json"


def configuration():
    return json.loads(CONFIG.read_text())


def spring_force(length, rate, cfg):
    """Polytropic proxy; seal/viscous terms always dissipate mechanical energy."""
    stroke, exponent = cfg["stroke_m"], cfg["polytropic_exponent"]
    dead_length = stroke / (1 - cfg["progression_at_full_stroke"] ** (-1 / exponent))
    compression = cfg["free_pin_distance_m"] - length
    if dead_length <= compression:
        raise ValueError("nonpositive candidate gas volume")
    elastic = cfg["initial_force_n"] * (dead_length / (dead_length - compression)) ** exponent
    dissipative = (-cfg["seal_friction_n"] * math.tanh(rate / cfg["friction_smoothing_m_s"])
                   - cfg["viscous_damping_ns_m"] * rate)
    return max(0., elastic + dissipative)


def candidate_runtime(radius):
    """Build actual C estimator with the candidate wheel scale in a temp copy."""
    firmware = PROJECT_ROOT.parent / "chassis_ref_new"
    paths = ["user/state_estimator.c", "user/state_estimator.h", "user/robot_config.h",
             "user/algorithm/rl.c", "user/algorithm/rl.h", "user/algorithm/model/model.c",
             "user/algorithm/model/model.h"]
    raw = {name: (firmware/name).read_bytes() for name in paths}
    text = raw["user/robot_config.h"].decode()
    text, count = re.subn(r"(#define ROBOT_WHEEL_RADIUS_M) [^\n]+",
                         rf"\g<1> {radius:.12f}f /* offline candidate */", text)
    if count != 1:
        raise ValueError("expected one wheel radius macro")
    raw["user/robot_config.h"] = text.encode()
    digest = hashlib.sha256(b"".join(raw.values())).hexdigest()[:24]
    root = Path(tempfile.gettempdir()) / "fifty_realdata_runtime" / digest
    target = root / "runtime.so"
    if not target.exists():
        for name, data in raw.items():
            path = root/name
            path.parent.mkdir(exist_ok=True, parents=True)
            path.write_bytes(data)
        temporary = root / f"runtime.{os.getpid()}.so"
        subprocess.run(["cc", "-std=c11", "-O2", "-shared", "-fPIC", "-I"+str(root/"user"),
                        str(root/"user/state_estimator.c"), str(root/"user/algorithm/rl.c"),
                        str(root/"user/algorithm/model/model.c"), "-lm", "-o", str(temporary)], check=True)
        os.replace(temporary, target)
    # Reuse the authoritative ctypes function signatures, not its wheel scale.
    reference, result = load_runtime(), c.CDLL(str(target))
    for name in ("estimator_update_qualified", "rl_manage_reference", "rl_motor_to_joint", "rl_policy_step"):
        getattr(result, name).argtypes = getattr(reference, name).argtypes
        getattr(result, name).restype = getattr(reference, name).restype
    return result


class RealDataHardwareEnv31(QualifiedHardwareEnv31):
    def __init__(self, *args, config=None, **kwargs):
        self.candidate = config or configuration()
        self.candidate_ready = False
        kwargs.setdefault("wheel_kd", self.candidate["wheel_kd"])
        super().__init__(*args, **kwargs)
        cfg, model = self.candidate, self.model
        old_mass = float(model.body_mass[self.base])
        model.body_mass[self.base] += cfg["mass_kg"] - model.body_mass.sum()
        model.body_inertia[self.base] *= model.body_mass[self.base] / old_mass
        for name in ("right_wheel_collision_0", "left_wheel_collision_0"):
            model.geom_size[model.geom(name).id, 0] = cfg["wheel_radius_m"]
        self.source.profile = dict(self.source.profile, wheel_radius_m=cfg["wheel_radius_m"])
        self.source.defaults = np.asarray(cfg["standing_default_rad"])
        for name, angle in zip(self.source.env["init_state"]["default_joint_angles"], self.source.defaults):
            self.source.env["init_state"]["default_joint_angles"][name] = float(angle)
        model.opt.timestep = cfg["physics_dt_s"]
        self.source.physics_substeps = round(cfg["pd_dt_s"] / cfg["physics_dt_s"])
        self.source.external_dt = cfg["pd_dt_s"]
        ratio = self.fast_actuators.ratio
        armature = np.zeros(6)
        j = cfg["reflected_inertia"]["leg_motor_output_kg_m2"]
        for h, k, sign in ((0, 1, 1), (3, 4, -1)):
            beta = sign*self.source.defaults[k] + math.pi/2
            alpha = 2*math.atan2(math.sin(beta), ratio+math.cos(beta))
            derivative = .5 + ratio*math.cos(alpha/2)/(2*math.sqrt(1-ratio**2*math.sin(alpha/2)**2))
            armature[h], armature[k], armature[h+2] = 2*j, j/derivative**2, cfg["reflected_inertia"]["wheel_output_kg_m2"]
        model.dof_armature[self.source.qvel_ids] = armature
        stops = cfg["joint_stop_solver"]
        ids = self.source.joint_ids[[1, 4]]
        model.jnt_solref[ids] = [stops["time_constant_s"], stops["damping_ratio"]]
        model.jnt_solimp[ids] = stops["impedance"]
        model.jnt_margin[ids] = stops["inward_contact_margin_rad"]
        model.dof_damping[self.source.qvel_ids] = cfg["passive_drive"]["damping_nms_rad"]
        model.dof_frictionloss[self.source.qvel_ids] = cfg["passive_drive"]["frictionloss_nm"]
        self.spring_scale = 1.
        self.source.spring_torque = self._spring_torque
        self.runtime = candidate_runtime(cfg["wheel_radius_m"])
        mujoco.mj_setConst(model, self.data)
        self.candidate_ready = True
        self.reset()

    def _spring_torque(self, q, qd):
        result, lengths, forces = np.zeros(6), [], []
        cfg = self.candidate["spring"]
        for k, spring in zip((1, 4), self.source.spring_config["springs"]):
            length, derivative = self.source.spring_module.geometry(float(q[k]), spring)
            if not cfg["free_pin_distance_m"] - cfg["stroke_m"] - .0001 <= length <= cfg["free_pin_distance_m"] + .0001:
                raise ValueError("candidate spring stroke exceeded")
            force = spring_force(length, derivative*float(qd[k]), cfg) * self.spring_scale
            result[k] = derivative * force
            lengths.append(length)
            forces.append(force)
        self.last_spring_forces = forces
        return result, lengths

    def reset(self, seed=0, command=(0., 0., .1)):
        if self.candidate_ready and command == (0., 0., .1):
            command = (0., 0., self.candidate["standing_clearance_m"])
        result = super().reset(seed, command)
        if self.candidate_ready:
            self.data.xfrc_applied[:] = 0
            self.applied[2] = self.requested[2]
            result = self.observation()
        return result

    def observation(self):
        result = super().observation()
        if self.candidate_ready:
            q = self.data.qpos[self.source.qpos_ids]
            offset = q - self.source.defaults
            offset[[0, 3]] = np.arctan2(np.sin(offset[[0, 3]]), np.cos(offset[[0, 3]]))
            result[9:13] = offset[[0, 1, 3, 4]]
        return result

    def _plant_step(self, action):
        if self.finished:
            raise RuntimeError("reset required after terminal/timeout observation")
        self.applied = ramp_commands(self.applied, self.requested, self.source.policy_dt)
        action = clip_actions(action)
        previous_action = self.last_action.copy()
        targets = action_targets(action, self.target_limits, self.source.defaults)
        previous_root = self.data.qpos[:3].copy()
        previous_point = self._point().copy()
        previous_yaw = rpy_from_wxyz(self.data.qpos[3:7])[2]
        command = self.applied.copy()
        saturation = np.zeros(6)
        maximum_motor_speed = np.zeros(6)
        failure = None
        completed_controls = 0
        active, passive, lengths = np.zeros(6), np.zeros(6), []
        for _ in range(self.source.env["control"]["decimation"]):
            q = self.data.qpos[self.source.qpos_ids].copy()
            error = np.array(targets["position"]) - q
            error[[0, 3]] = np.arctan2(np.sin(error[[0, 3]]), np.cos(error[[0, 3]]))
            requested = np.array([40, 60, 0, 40, 60, 0])*error + np.array([2, 2, self.wheel_kd, 2, 2, self.wheel_kd])*(
                np.array(targets["velocity"])-self.joint_velocity)
            active = self.fast_actuators.apply(requested, q, self.joint_velocity)
            saturation += np.abs(self.fast_actuators.requested-self.fast_actuators.command) > 1e-6
            maximum_motor_speed = np.maximum(maximum_motor_speed, np.abs(self.fast_actuators.velocity))
            try:
                passive, lengths = self.source.spring_torque(q, self.joint_velocity)
            except ValueError as exc:
                failure = "spring_stroke:" + str(exc)
                break
            self.data.qfrc_applied[:] = 0
            self.data.qfrc_applied[self.source.qvel_ids] = active + passive
            for _ in range(self.source.physics_substeps):
                mujoco.mj_step(self.model, self.data)
                mujoco.mj_forward(self.model, self.data)
                if not self._update_estimator():
                    failure = "firmware_estimator_invalid"
                if not np.isfinite(np.r_[self.data.qpos, self.data.qvel]).all():
                    raise FloatingPointError("nonfinite MuJoCo state; abort experiment")
            mujoco.mj_forward(self.model, self.data)
            completed_controls += 1
            if failure is not None:
                break
            knees = self.data.qpos[self.source.qpos_ids[[1, 4]]]
            if np.any(knees < self.limits[:, 0]-1e-4) or np.any(knees > self.limits[:, 1]+1e-4):
                failure = "source_knee_stop_violation"
                break
        self.tick += 1
        elapsed = completed_controls*self.source.external_dt
        rotation = self._rotation()
        gravity = rotation.T @ [0., 0., -1.]
        rpy = rpy_from_wxyz(self.data.qpos[3:7])
        self.body_velocity = rotation.T @ ((self.data.qpos[:3]-previous_root)/max(elapsed, 1e-12))
        point_world_velocity = (self._point()-previous_point)/max(elapsed, 1e-12)
        c, s = math.cos(rpy[2]), math.sin(rpy[2])
        point_velocity = np.array([c*point_world_velocity[0]+s*point_world_velocity[1],
                                  -s*point_world_velocity[0]+c*point_world_velocity[1]])
        yaw_delta = rpy[2]-previous_yaw
        yaw_rate = math.atan2(math.sin(yaw_delta), math.cos(yaw_delta))/max(elapsed, 1e-12)
        clearance = bottom_clearance(self.data.qpos[2], gravity,
            self.source.env["env"]["clearance_box_center"], self.source.env["env"]["clearance_box_halfsize"])
        tilt = math.degrees(math.acos(float(np.clip(-gravity[2], -1, 1))))
        if failure is None and (tilt > math.degrees(.70) or self.data.qpos[2] < .15):
            failure = "tilt_or_low_root"
        force_world = np.zeros(3)
        for i, contact in enumerate(self.data.contact):
            names = [self.model.geom(n).name for n in (contact.geom1, contact.geom2)]
            if "floor" in names and "base_link_collision_0" in names:
                force = np.empty(6)
                mujoco.mj_contactForce(self.model, self.data, i, force)
                force_world += np.array(contact.frame).reshape(3, 3).T @ force[:3]
        if failure is None and np.linalg.norm(force_world) > 10:
            failure = "base_ground_contact"
        terminated = failure is not None
        truncated = self.tick >= self.max_steps and not terminated
        self.finished = terminated or truncated
        # Integrate the reference with this step's applied command, then advance
        # the ramp used in the NEXT observation. Terminal state is never reset.
        self.reference, self.heading = advance_reference(self.reference, self.heading,
            command[0], command[1], self.source.policy_dt)
        self.last_action = action
        error_vx, error_wz = point_velocity[0]-command[0], yaw_rate-command[1]
        reward = self.source.policy_dt * (math.exp(-(error_vx/.1)**2)
            + math.exp(-(error_wz/.3)**2) + math.exp(-((clearance-command[2])/.02)**2)
            - .01*sum((a-b)**2 for a, b in zip(action, previous_action))
            - .01*np.dot(rpy[:2], rpy[:2])) - float(terminated)
        saturation_fraction = float(np.mean(saturation[[2, 5]] / max(1, completed_controls)))
        reward -= self.source.policy_dt * .5 * saturation_fraction
        margins = [min(length-spring["min_length_m"], spring["max_length_m"]-length)
            for length, spring in zip(lengths, self.source.spring_config["springs"])]
        info = {"time_s": float(self.data.time), "failure": failure, "applied_command": command,
            "point_velocity_xy": point_velocity.tolist(), "root_velocity_body": self.body_velocity.tolist(),
            "yaw_rate": yaw_rate, "roll_pitch_deg": np.degrees(rpy[:2]).tolist(), "clearance_m": clearance,
            "saturation_counts": saturation.tolist(), "controls": completed_controls,
            "max_motor_speed_rad_s": maximum_motor_speed.tolist(), "spring_margins_m": margins,
            "reference_error_xy_m": (np.array(self.reference)-self.data.qpos[:2]).tolist(),
            "reference_yaw_error_rad": math.atan2(math.sin(self.heading-rpy[2]), math.cos(self.heading-rpy[2])),
            "active_torque_nm": active.tolist(), "passive_torque_nm": passive.tolist()}
        info["wheel_saturation_penalty"] = self.source.policy_dt * .5 * saturation_fraction
        info["estimated_xy_m"] = self.estimated_xy.tolist()
        info["estimated_velocity_body_m_s"] = self.estimated_velocity.tolist()
        info["odometry_error_xy_m"] = (self.estimated_xy-self.data.qpos[:2]).tolist()
        self.last_info = info
        return self.observation(), float(reward), terminated, truncated, info

    def step(self, action):
        result = self._plant_step(action)
        result[4].update(odometry_confidence=float(self.c_quality.confidence),
                         spring_force_n=self.last_spring_forces)
        return result

    def snapshot(self):
        return {"configuration": self.candidate, "mass_kg": float(self.model.body_mass.sum()),
                "armature_kg_m2": self.model.dof_armature[self.source.qvel_ids].tolist(),
                "knee_limits_rad": self.limits.tolist(), "action_limits_rad": self.target_limits,
                "spring_geometry": self.source.spring_config["springs"],
                "physics_backend": "CPU MuJoCo; policy/optimizer can use CUDA"}
