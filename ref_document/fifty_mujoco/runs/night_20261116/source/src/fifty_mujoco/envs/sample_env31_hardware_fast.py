"""Scalar NumPy actuator implementation of the v3 hardware control contract.

No physics/reward/timing changes. Small CPU tensor operations are replaced by
float64 scalar algebra, checked against both the frozen Torch actuator and the
firmware C torque path. Keep the reference implementation available for parity.
"""
import math
import mujoco
import numpy as np
from .sample_env31_hardware import SampleHardwareEnv31, rpy_from_wxyz
from ..contracts.sample_actor31 import clip_actions, action_targets, bottom_clearance, advance_reference, ramp_commands


class ScalarActuators:
    def __init__(self, env):
        ratings=env.actuators.inner.ratings
        self.limit=np.array([ratings['leg_torque_nm'],ratings['leg_torque_nm'],ratings['wheel_torque_nm']]*2)
        self.rated=np.array([ratings['leg_rated_rad_s'],ratings['leg_rated_rad_s'],ratings['wheel_rated_rad_s']]*2)
        self.free=np.array([ratings['leg_no_load_rad_s'],ratings['leg_no_load_rad_s'],ratings['wheel_no_load_rad_s']]*2)
        self.effort=np.array(env.source.profile['joint_effort_nm'])
        self.gain=env.motor_gain.copy()
        cfg=env.source.hardware['linkage']
        self.ratio=cfg['active_arm_m']/cfg['passive_arm_m']

    def apply(self, requested, q, qd):
        requested=np.clip(requested,-self.effort,self.effort)
        self.requested=requested.copy();self.velocity=qd.copy();ratios=[]
        for h,k,sign in ((0,1,1),(3,4,-1)):
            beta=sign*q[k]+math.pi/2
            alpha=2*math.atan2(math.sin(beta),self.ratio+math.cos(beta))
            ratio=.5+self.ratio*math.cos(alpha/2)/(2*math.sqrt(1-self.ratio**2*math.sin(alpha/2)**2))
            ratios.append(ratio)
            self.requested[h]=sign*(requested[h]-ratio*requested[k])
            self.requested[k]=sign*ratio*requested[k]
            self.velocity[h]=sign*qd[h]
            self.velocity[k]=sign*(qd[h]+qd[k]/ratio)
        positive=self.limit*np.clip((self.free-self.velocity)/(self.free-self.rated),0,1)
        negative=-self.limit*np.clip((self.free+self.velocity)/(self.free-self.rated),0,1)
        self.command=np.maximum(np.minimum(self.requested,positive),negative)
        self.delivered=np.maximum(np.minimum(self.command*self.gain,positive),negative)
        joint=self.delivered.copy()
        for (h,k,sign),ratio in zip(((0,1,1),(3,4,-1)),ratios):
            joint[h]=sign*(self.delivered[h]+self.delivered[k])
            joint[k]=sign*self.delivered[k]/ratio
        return joint


class FastHardwareEnv31(SampleHardwareEnv31):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.fast_actuators=ScalarActuators(self)

    def step(self, action):
        if self.finished:
            raise RuntimeError("reset required after terminal/timeout observation")
        self.applied = ramp_commands(self.applied, self.requested, self.source.policy_dt)
        action = clip_actions(action)
        previous_action = self.last_action.copy()
        targets = action_targets(action, self.target_limits)
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
