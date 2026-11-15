"""Commissioned hardware variant, isolated from all previous training contracts.

Matches current firmware command ramp/reference ordering and 2 ms PD, 10 ms
policy. Gains are delivered MOTOR torque gains, after firmware envelope clamps.
Real-robot acceptance is outside the scope of this reconstructed environment.
"""
from __future__ import annotations

import copy
import json
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import torch

from .sample_env31 import SampleEnv31, rpy_from_wxyz
from ..contracts.sample_actor31 import JOINT_ORDER, advance_reference, ramp_commands, build_observation, REFERENCE_OFFSET, bottom_clearance, action_targets, clip_actions
from ..paths import configs_dir
from ..transfer.sample_source import SampleSource, source_module
from ..transfer.sample_target import load_profile, target_hardware, target_spring_config, target_xml

ENV_SCHEMA = "sample_commissioned_cpu31_v2"
REWARD_SCHEMA = "sample_transfer_tracking_reward_v1_firmware_timing"


def hardware_profile():
    return json.loads((configs_dir() / "sample_hardware_profile.json").read_text())


class FirmwareActuators:
    def __init__(self, module, hardware, profile, gain):
        self.inner = module.CoupledActuators(torch, hardware, JOINT_ORDER)
        self.inner.ratings["leg_torque_nm"] = profile["leg_torque_cap_nm"]
        self.inner.ratings["wheel_torque_nm"] = profile["wheel_torque_cap_nm"]
        self.effort = torch.tensor(profile["joint_effort_nm"], dtype=torch.float64)
        self.gain = torch.tensor(gain, dtype=torch.float64)[None]

    def apply(self, requested, q, qd):
        requested = torch.clamp(requested, -self.effort, self.effort)
        result = self.inner.apply(requested, q, qd, output_gain=self.gain)
        torque, rated, free = self.inner._limits[(requested.device, requested.dtype)]
        speed = self.inner.motor_velocity
        positive = torque * torch.clamp((free-speed)/(free-rated), 0, 1)
        negative = -torque * torch.clamp((free+speed)/(free-rated), 0, 1)
        command_limited = torch.maximum(torch.minimum(self.inner.motor_torque_requested, positive), negative)
        for name in ("motor_torque_requested", "motor_torque_applied", "motor_velocity"):
            setattr(self, name, getattr(self.inner, name))
        self.delivered_motor_torque = self.motor_torque_applied
        # Saturation is requested vs envelope-limited COMMAND, not calibrated
        # delivery error. A gain below one must not count as 100% saturation.
        self.motor_torque_applied = command_limited
        return result


class HardwareSource(SampleSource):
    def __init__(self, effective_friction=.75):
        self.profile = hardware_profile()
        old = copy.deepcopy(load_profile())
        old["wheel"]["radius_m"] = self.profile["wheel_radius_m"]
        old["wheel"]["total_rotor_to_wheel_fraction"] = {"numerator": 1588, "denominator": 100}
        old["environment"]["spawn_height_delta_m"] = 0.
        self.xml, self.env = target_xml(old, effective_friction)
        root = ET.fromstring(self.xml)
        root.find("option").set("timestep", str(self.profile["external_pd_dt_s"] / 2))
        self.xml = ET.tostring(root, encoding="unicode")
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.joint_ids = np.array([self.model.joint(n).id for n in JOINT_ORDER])
        self.qpos_ids = self.model.jnt_qposadr[self.joint_ids]
        self.qvel_ids = self.model.jnt_dofadr[self.joint_ids]
        self.defaults = np.array([self.env["init_state"]["default_joint_angles"][n] for n in JOINT_ORDER])
        self.actuator_module = source_module("actuator_model")
        self.hardware = target_hardware(old)
        self.spring_module = source_module("gas_spring")
        self.spring_config = target_spring_config(old)
        self.physics_substeps = 2
        self.external_dt = self.model.opt.timestep * self.physics_substeps
        self.policy_dt = self.profile["policy_dt_s"]
        self.env["control"]["decimation"] = round(self.policy_dt / self.external_dt)
        assert abs(self.env["control"]["decimation"] * self.external_dt - self.policy_dt) < 1e-12
        self.reset()


class SampleHardwareEnv31(SampleEnv31):
    def __init__(self, episode_seconds=20., effective_friction=.75, reset_mode="default",
                 motor_gain=1., spring_scale=1.):
        if reset_mode not in ("default", "natural", "mixed"):
            raise ValueError("unknown reset mode")
        if not math.isfinite(episode_seconds) or episode_seconds <= 0:
            raise ValueError("positive duration required")
        self.reset_mode = reset_mode
        self.motor_gain = np.broadcast_to(motor_gain, (6,)).copy()
        if not np.isfinite(self.motor_gain).all() or np.any(self.motor_gain <= 0):
            raise ValueError("positive finite motor gain required")
        if not math.isfinite(spring_scale) or spring_scale <= 0:
            raise ValueError("positive finite spring scale required")
        self.source = HardwareSource(effective_friction)
        self.model, self.data = self.source.model, self.source.data
        self.base = self.model.body("base_link").id
        self.max_steps = math.ceil(episode_seconds / self.source.policy_dt)
        self.limits = self.model.jnt_range[self.source.joint_ids[[1, 4]]].copy()
        self.actuators = FirmwareActuators(self.source.actuator_module, self.source.hardware,
                                          self.source.profile, self.motor_gain)
        for spring in self.source.spring_config["springs"]:
            spring["reference_force_n"] *= spring_scale
        self.spring_scale = spring_scale
        self.reset()

    def reset(self, seed=0, command=(0., 0., .1)):
        super().reset(seed, command)
        rng = np.random.default_rng(seed)
        fraction = {"default": 0., "natural": 1.}.get(self.reset_mode)
        if fraction is None:
            # Include both endpoints and interpolation, hold this pose for reset only.
            fraction = float(rng.choice([0., 1., rng.uniform()]))
        self.reset_fraction = fraction
        natural = np.asarray(self.source.profile["natural_joint_pose_rad"])
        self.data.qpos[self.source.qpos_ids] = self.source.defaults + fraction * (natural - self.source.defaults)
        mujoco.mj_forward(self.model, self.data)
        # Place both wheels on the plane, respecting cylinder orientation/width.
        bottoms = []
        for name in ("right_wheel_collision_0", "left_wheel_collision_0"):
            g = self.model.geom(name).id
            axis_z = float(self.data.geom_xmat[g].reshape(3, 3)[2, 2])
            extent = self.model.geom_size[g, 0] * math.sqrt(max(0., 1-axis_z*axis_z))
            extent += self.model.geom_size[g, 1] * abs(axis_z)
            bottoms.append(self.data.geom_xpos[g, 2] - extent)
        self.data.qpos[2] -= min(bottoms)
        mujoco.mj_forward(self.model, self.data)
        self.reference = self.data.qpos[:2].tolist()
        self.heading = float(rpy_from_wxyz(self.data.qpos[3:7])[2])
        self.last_info = None
        self.estimated_xy = np.zeros(2)
        self.estimated_velocity = np.zeros(3)
        self.velocity_sample_q = self.data.qpos[self.source.qpos_ids].copy()
        self.velocity_sample_time = 0.
        return self.observation()

    def _update_estimator(self):
        q = self.data.qpos[self.source.qpos_ids]
        raw_qd = self.data.qvel[self.source.qvel_ids]
        elapsed = self.data.time - self.velocity_sample_time
        # chassis_service samples at 1 ms, estimator before the 2 ms PD gate.
        if elapsed < .005 - 1e-12:
            return True
        if elapsed >= .005 - 1e-12:
            delta = q - self.velocity_sample_q
            delta[[0, 3]] = np.arctan2(np.sin(delta[[0, 3]]), np.cos(delta[[0, 3]]))
            self.joint_velocity = delta / elapsed
            self.velocity_sample_q = q.copy()
            self.velocity_sample_time = self.data.time
        rotation = self._rotation()
        omega = self._velocity6()[:3]
        axle = rotation[:, 1]
        norm = math.sqrt(max(0., 1-axle[2]**2))
        if norm <= .5:
            return False
        radius = self.source.profile['wheel_radius_m']
        contact = radius * np.array([axle[2]*axle[0], axle[2]*axle[1], -(1-axle[2]**2)]) / norm
        world_velocity = np.zeros(3)
        for h, k, sign in ((0, 1, 1), (3, 4, -1)):
            a, b = sign*q[h], sign*(q[h]+q[k])+math.pi/2
            ad, bd = sign*raw_qd[h], sign*(raw_qd[h]+raw_qd[k])
            point = np.array([.21*math.cos(a)+.25*math.cos(b), -.260797767 if h==0 else .268202233,
                              -.21*math.sin(a)-.25*math.sin(b)])
            derivative = np.array([-.21*math.sin(a)*ad-.25*math.sin(b)*bd, 0.,
                                    -.21*math.cos(a)*ad-.25*math.cos(b)*bd])
            relative = rotation @ (np.cross(omega, point)+derivative)
            spin = rotation @ (omega + [0., bd+raw_qd[h+2], 0.])
            world_velocity += .5 * (-np.cross(spin, contact)-relative)
        if not np.isfinite(world_velocity).all() or np.abs(world_velocity).max() > 5:
            return False
        self.estimated_xy += world_velocity[:2]*elapsed
        self.estimated_velocity = rotation.T @ world_velocity
        return True

    def observation(self):
        upcoming = ramp_commands(self.applied, self.requested, self.source.policy_dt)
        return np.asarray(build_observation(angular_velocity_body=self._velocity6()[:3],
            projected_gravity_body=self._rotation().T @ [0.,0.,-1.],
            applied_commands=upcoming, joint_position=self.data.qpos[self.source.qpos_ids],
            joint_velocity=self.joint_velocity, previous_clipped_action=self.last_action,
            position_xy=getattr(self, 'estimated_xy', np.zeros(2)), reference_xy=self.reference,
            yaw=rpy_from_wxyz(self.data.qpos[3:7])[2], reference_yaw=self.heading,
            body_velocity_xy=getattr(self, 'estimated_velocity', np.zeros(3))[:2]), dtype=np.float32)

    def step(self, action):
        if self.finished:
            raise RuntimeError("reset required after terminal/timeout observation")
        self.applied = ramp_commands(self.applied, self.requested, self.source.policy_dt)
        action = clip_actions(action)
        previous_action = self.last_action.copy()
        targets = action_targets(action, self.limits.tolist())
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
            requested = np.array([40, 60, 0, 40, 60, 0])*error + np.array([2, 2, .3, 2, 2, .3])*(
                np.array(targets["velocity"])-self.joint_velocity)
            with torch.no_grad():
                active = self.actuators.apply(torch.tensor(requested)[None], torch.tensor(q)[None],
                    torch.tensor(self.joint_velocity)[None])[0].numpy()
            saturation += (self.actuators.motor_torque_requested-self.actuators.motor_torque_applied).abs()[0].numpy() > 1e-6
            maximum_motor_speed = np.maximum(maximum_motor_speed, self.actuators.motor_velocity.abs()[0].numpy())
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
        info["estimated_xy_m"] = self.estimated_xy.tolist()
        info["estimated_velocity_body_m_s"] = self.estimated_velocity.tolist()
        info["odometry_error_xy_m"] = (self.estimated_xy-self.data.qpos[:2]).tolist()
        self.last_info = info
        return self.observation(), float(reward), terminated, truncated, info
