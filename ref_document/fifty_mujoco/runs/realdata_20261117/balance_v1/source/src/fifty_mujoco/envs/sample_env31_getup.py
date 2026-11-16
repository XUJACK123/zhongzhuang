"""Get-up task adapter reusing the guarded hardware plant and scalar actuators.

This module requires the existing CPU MuJoCo/Torch runtime. The offline protocol
suite validates lifecycle logic separately. Contact configuration and force
feasibility require dynamic validation before training or deployment.
"""

import math
import random

import mujoco
import numpy as np
import torch

from .sample_env31_hardware_guarded import GuardedHardwareEnv31
from .sample_env31_hardware import rpy_from_wxyz
from ..contracts.getup31 import GetupController, Phase, Sample, SCHEMA, blended_targets, reward_terms


class GetupEnv31:
    protocol_schema = SCHEMA
    autonomous_balance_validation = True

    def __init__(self, plan, profile, frozen_balance_actor, device="cpu"):
        self.plan, self.profile, self.balance_actor, self.device = plan, profile, frozen_balance_actor, device
        self.spec = plan["getup"]
        duration = (max(self.spec["settle_zero_torque_s"]) + self.spec["getup_timeout_s"]
                    + self.spec["handoff_blend_s"] + self.spec["post_handoff_balance_s"] + 1.)
        self.env = GuardedHardwareEnv31(episode_seconds=duration, wheel_kd=plan["baseline"]["wheel_kd"],
                                       knee_margin_rad=plan["baseline"]["target_knee_margin_rad"])
        if self.env.source.profile != profile:
            raise ValueError("plant profile differs from supplied profile")
        self.base = self.env.base
        self.bodyweight = float(self.env.model.body_mass.sum()) * abs(float(self.env.model.opt.gravity[2]))
        if self.bodyweight <= 0:
            raise ValueError("positive body weight required")
        self.original_apply = self.env.fast_actuators.apply
        self.env.fast_actuators.apply = self._apply_scaled
        self.active_scale = 0.
        self.parameters = None
        self.finished = True

    def _apply_scaled(self, requested, q, qd):
        active = self.original_apply(requested, q, qd)
        self.env.fast_actuators.command *= self.active_scale
        self.env.fast_actuators.delivered *= self.active_scale
        return active * self.active_scale

    def _support(self):
        if self.parameters["stage"] not in ("supported_survey", "support_release"):
            return 0.
        elapsed = 0. if self.settling else max(0., float(self.env.data.time) - self.enabled_at)
        fraction = max(0., 1 - max(0., elapsed - self.release_after_s) / self.spec["assist_fade_s"])
        demand = (self.spec["assist_vertical_kp_n_m"] * (self.support_height - self.env.data.qpos[2])
                  - self.spec["assist_vertical_kd_ns_m"] * self.env.data.qvel[2])
        return fraction * min(self.spec["assist_force_cap_bodyweights"] * self.bodyweight, max(0., demand))

    def _contacts(self):
        allowed = set(self.spec["allowed_rest_floor_geoms"])
        supported = set()
        body_contact, forbidden, impulse = False, False, 0.
        for index, contact in enumerate(self.env.data.contact):
            force = np.zeros(6)
            mujoco.mj_contactForce(self.env.model, self.env.data, index, force)
            magnitude = float(np.linalg.norm(force[:3]))
            if magnitude <= 1e-3:
                continue
            names = {self.env.model.geom(int(i)).name for i in (contact.geom1, contact.geom2)}
            if "floor" in names:
                other = next(name for name in names if name != "floor")
                forbidden |= other not in allowed
                body_contact |= other == "base_link_collision_0" and magnitude > 10.
                if abs(force[0]) >= self.bodyweight * self.spec["wheel_support_min_bodyweight_fraction"]:
                    supported.add(other)
            else:
                forbidden = True
            if "floor" not in names or not names & {"right_wheel_collision_0", "left_wheel_collision_0"}:
                impulse += magnitude * self.env.source.policy_dt
        return {"right_wheel_collision_0", "left_wheel_collision_0"} <= supported, body_contact, forbidden, impulse

    def _sample(self, info):
        e = self.env
        both, body, forbidden, impulse = self._contacts()
        self.contact_impulse_proxy = impulse
        failure = info.get("failure")
        tilt_failure = max(abs(x) for x in info["roll_pitch_deg"]) > self.spec["getup_tilt_limit_deg"]
        balance_failure = failure if self.controller.phase in (Phase.HANDOFF, Phase.BALANCE) else None
        if tilt_failure:
            balance_failure = "getup_tilt_limit"
            forbidden = True
        knees = e.data.qpos[e.source.qpos_ids[[1, 4]]]
        return Sample(time_s=float(e.data.time), joint_position_rad=tuple(e.data.qpos[e.source.qpos_ids]),
                      roll_pitch_deg=tuple(info["roll_pitch_deg"]), angular_velocity_rad_s=tuple(e._velocity6()[:3]),
                      forward_velocity_m_s=float(e.estimated_velocity[0]), clearance_m=info["clearance_m"],
                      displacement_m=float(np.linalg.norm(e.data.qpos[:2] - self.origin)),
                      both_wheels_supported=both, body_contact=body, assistance_force_n=self.assistance_n,
                      telemetry_valid=failure != "firmware_estimator_invalid",
                      joint_limits_valid=bool(np.all(knees >= e.limits[:, 0] - 1e-4) and np.all(knees <= e.limits[:, 1] + 1e-4)),
                      spring_stroke_valid=not (failure or "").startswith("spring_stroke:"),
                      linkage_branch_valid=True, forbidden_contact=forbidden, balance_failure=balance_failure)

    def _advance_plant(self, action, scale):
        e = self.env
        self.active_scale = scale
        self.assistance_n = self._support()
        e.data.xfrc_applied[self.base, :] = 0
        e.data.xfrc_applied[self.base, 2] = self.assistance_n
        observation, _, terminated, truncated, info = e.step(action)
        # Only these old standing-task conditions become phase-specific. Their
        # truth is retained in info and evaluated again by the get-up controller.
        if terminated and info["failure"] in ("base_ground_contact", "tilt_or_low_root"):
            e.finished = False
        if truncated:
            info["failure"] = "episode_time_limit"
        sample = self._sample(info)
        if info["failure"] == "episode_time_limit":
            raise RuntimeError("get-up plant duration exhausted before a task outcome")
        return observation, info, sample

    def reset(self, seed, parameters):
        self.parameters = parameters
        self.settling = True
        self.controller = GetupController(self.plan)
        self.controller.ramp_s = parameters["enable_ramp_s"]
        e, rng = self.env, random.Random(seed)
        gains = np.asarray(parameters["motor_gain"], dtype=float)
        e.motor_gain[:] = gains
        e.fast_actuators.gain[:] = gains
        e.source.model.geom_friction[:, 0] = parameters["effective_friction"]
        for spring in e.source.spring_config["springs"]:
            spring["reference_force_n"] = self.profile["spring_force_n"] * parameters["spring_scale"]
        stage = parameters["stage"]
        reset_class = "near_stand" if stage == "near_stand" else "disabled_rest"
        if stage == "joint_randomization":
            reset_class = self.spec["final_reset_classes"][seed % len(self.spec["final_reset_classes"])]
        e.reset_mode = "default" if reset_class == "near_stand" else "natural"
        original_pose = e.source.profile["natural_joint_pose_rad"]
        if reset_class == "asymmetric_rest":
            pose = list(original_pose)
            pose[3] += rng.uniform(-.03, .03)
            pose[4] += rng.uniform(-.03, .03)
            e.source.profile["natural_joint_pose_rad"] = pose
        try:
            e.reset(seed, tuple(self.spec["actor_commands"]))
        finally:
            e.source.profile["natural_joint_pose_rad"] = original_pose
        self.origin = e.data.qpos[:2].copy()
        self.support_height = float(e.data.qpos[2])
        self.enabled_at = float(e.data.time)
        self.release_after_s = .5 if stage == "supported_survey" else rng.uniform(.5, 1.5)
        settle = parameters["settle_zero_torque_s"]
        for _ in range(math.ceil(settle / e.source.policy_dt)):
            _, _, state = self._advance_plant([0.] * 6, 0.)
            if not all((state.telemetry_valid, state.joint_limits_valid, state.spring_stroke_valid,
                        state.linkage_branch_valid, not state.forbidden_contact)):
                raise ValueError("disabled reset settles outside the configured mechanical/contact domain")
        # One disabled step supplies a timestamp before the enable transition.
        _, _, state = self._advance_plant([0.] * 6, 0.)
        self.controller.update(state, False)
        self.enabled_at = float(e.data.time)
        self.settling = False
        self.origin = e.data.qpos[:2].copy()
        _, info, state = self._advance_plant([0.] * 6, 0.)
        self.decision = self.controller.update(state, True)
        if self.decision.failure:
            raise ValueError(f"invalid initial enabled state: {self.decision.failure}")
        e.reference = e.estimated_xy.tolist()
        e.heading = float(rpy_from_wxyz(e.data.qpos[3:7])[2])
        e.velocity_sample_q = e.data.qpos[e.source.qpos_ids].copy()
        e.velocity_sample_time = float(e.data.time)
        e.last_action = np.zeros(6)
        self.previous_sample = state
        self.finished = False
        self.reset_class = reset_class
        return e.observation()

    def _balance_action(self):
        with torch.no_grad():
            observation = torch.tensor(self.env.observation(), device=self.device)[None]
            return self.balance_actor(observation)[0].cpu().tolist()

    def step(self, action):
        if self.finished:
            raise RuntimeError("reset required after get-up outcome")
        phase_before = self.decision.phase
        if phase_before not in (Phase.GETUP, Phase.HANDOFF):
            raise RuntimeError("learner transition begins outside get-up/handoff")
        total_reward = 0.
        reward_log = {}
        while True:
            before = self.decision
            targets = blended_targets(before, action, self._balance_action(), self.env.target_limits)
            previous_action = self.env.last_action.copy()
            observation, info, state = self._advance_plant(targets["clipped_action"], before.torque_scale)
            self.decision = self.controller.update(state, True)
            saturation = sum(info["saturation_counts"]) / (6 * max(1, info["controls"]))
            slip = float(np.linalg.norm(np.asarray(info["point_velocity_xy"]) - self.env.estimated_velocity[:2]))
            terms = reward_terms(state, self.previous_sample, self.decision, targets["clipped_action"], previous_action,
                                 self.env.fast_actuators.delivered, self.env.fast_actuators.velocity, saturation,
                                 self.contact_impulse_proxy, slip, self.spec["reward_weights"], self.decision.success)
            total_reward += terms["total"]
            for key, value in terms["weighted"].items():
                reward_log[key] = reward_log.get(key, 0.) + value
            self.previous_sample = state
            self.finished = self.decision.success or self.decision.failure is not None
            if self.finished or self.decision.phase != Phase.BALANCE:
                break
        info.update(policy_phase_before=phase_before.value, phase_after=self.decision.phase.value,
                    success=self.decision.success, failure=self.decision.failure,
                    balance_validation_completed=self.decision.success, reset_class=self.reset_class,
                    reward_terms=reward_log, episode_parameters=self.parameters)
        return observation, total_reward, bool(self.finished), False, info


def make_env(plan, profile, frozen_balance_actor, device="cpu"):
    return GetupEnv31(plan, profile, frozen_balance_actor, device)
