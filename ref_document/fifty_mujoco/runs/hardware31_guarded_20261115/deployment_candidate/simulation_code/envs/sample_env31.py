"""Versioned, reconstructed CPU environment for sample actor transfer.

One actor step is 10 ms, external PD 5 ms and integration 2.5 ms. No internal
reset: callers must retain the terminal observation before resetting. This
profile reproduces the archived controls, not the missing Isaac runtime.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np
import torch

from ..contracts.sample_actor31 import (
    JOINT_ORDER, REFERENCE_OFFSET, action_targets, advance_reference,
    bottom_clearance, build_observation, clip_actions, ramp_commands,
)
from ..transfer.sample_source import SampleSource

ENV_SCHEMA = "sample_reconstructed_cpu31_v1"
REWARD_SCHEMA = "sample_transfer_tracking_reward_v1"


def rpy_from_wxyz(quat):
    w, x, y, z = quat
    return np.array([math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                     math.asin(float(np.clip(2*(w*y-z*x), -1, 1))),
                     math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))])


def wilson_interval(successes, total):
    if not total:
        return [None, None]
    z = 1.959963984540054
    p = successes/total
    denominator = 1 + z*z/total
    center = (p+z*z/(2*total))/denominator
    half = z*math.sqrt(p*(1-p)/total+z*z/(4*total*total))/denominator
    return [max(0., center-half), min(1., center+half)]


class SampleEnv31:
    def __init__(self, episode_seconds=10., effective_friction=.75):
        self.source = SampleSource(effective_friction)
        self.model, self.data = self.source.model, self.source.data
        if not math.isfinite(episode_seconds) or episode_seconds <= 0:
            raise ValueError("positive finite episode duration required")
        self.max_steps = max(1, math.ceil(episode_seconds/self.source.policy_dt))
        self.base = self.model.body("base_link").id
        self.limits = self.model.jnt_range[self.source.joint_ids[[1, 4]]].copy()
        self.actuators = self.source.actuator_module.CoupledActuators(torch, self.source.hardware, JOINT_ORDER)
        self.reset()

    def _rotation(self):
        return self.data.xmat[self.base].reshape(3, 3)

    def _point(self):
        return self.data.qpos[:3] + self._rotation() @ np.array([*REFERENCE_OFFSET, 0.])

    def _velocity6(self):
        velocity = np.empty(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.base, velocity, 1)
        return velocity

    def observation(self):
        return np.array(build_observation(angular_velocity_body=self._velocity6()[:3],
            projected_gravity_body=self._rotation().T @ [0., 0., -1.],
            applied_commands=self.applied, joint_position=self.data.qpos[self.source.qpos_ids],
            joint_velocity=self.joint_velocity, previous_clipped_action=self.last_action,
            position_xy=self.data.qpos[:2], reference_xy=self.reference,
            yaw=rpy_from_wxyz(self.data.qpos[3:7])[2], reference_yaw=self.heading,
            body_velocity_xy=self.body_velocity[:2]), dtype=np.float32)

    def reset(self, seed=0, command=(0., 0., .1)):
        self.source.reset()
        self.request(command)
        rng = np.random.default_rng(seed)
        noise = self.source.env["env"].get("initial_velocity_noise", 0)
        self.data.qvel[:2] = rng.uniform(-noise, noise, 2)
        self.data.qvel[3:6] = rng.uniform(-noise, noise, 3)
        mujoco.mj_forward(self.model, self.data)
        self.reference, self.heading = self.data.qpos[:2].tolist(), 0.
        self.applied = [0., 0., .1]
        self.last_action = [0.] * 6
        self.joint_velocity = np.zeros(6)
        self.body_velocity = np.zeros(3)
        self.tick, self.finished = 0, False
        return self.observation()

    def request(self, command):
        values = np.asarray(command, dtype=float)
        if values.shape != (3,) or not np.isfinite(values).all() or values[2] <= 0:
            raise ValueError("command must be finite [vx,wz,positive clearance]")
        self.requested = values.tolist()

    def step(self, action):
        if self.finished:
            raise RuntimeError("reset required after terminal/timeout observation")
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
                if not np.isfinite(np.r_[self.data.qpos, self.data.qvel]).all():
                    raise FloatingPointError("nonfinite MuJoCo state; abort experiment")
            mujoco.mj_forward(self.model, self.data)
            completed_controls += 1
            self.joint_velocity = np.arctan2(np.sin(self.data.qpos[self.source.qpos_ids]-q),
                np.cos(self.data.qpos[self.source.qpos_ids]-q)) / self.source.external_dt
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
        if failure is None and (tilt > 60 or self.data.qpos[2] < .15):
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
        self.applied = ramp_commands(self.applied, self.requested, self.source.policy_dt)
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
        return self.observation(), float(reward), terminated, truncated, info
