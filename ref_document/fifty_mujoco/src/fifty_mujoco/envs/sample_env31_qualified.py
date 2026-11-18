"""Guarded plant with the current C odometry quality and reference manager.

The controller begins already balancing at the existing reset. Startup/get-up,
IMU filtering and real CAN timing are explicitly outside this environment.
"""
import ctypes as c
import math
import numpy as np

from .sample_env31_hardware_guarded import GuardedHardwareEnv31
from .sample_env31_hardware import rpy_from_wxyz
from ..transfer.firmware_runtime import Estimator, Quality, Measurement, Controller, load_runtime, floats

ENV_SCHEMA = "sample_qualified_odometry_cpu31_v1"


class QualifiedHardwareEnv31(GuardedHardwareEnv31):
    def __init__(self, *args, **kwargs):
        self.runtime = load_runtime()
        self.c_estimator, self.c_quality = Estimator(), Quality()
        self.c_measurement, self.c_controller = Measurement(), Controller()
        self.last_estimator_time = 0.
        self.reference_managed_tick = None
        self.initializing = True
        super().__init__(*args, **kwargs)
        self.initializing = False
        self.reset()

    def _motor_coordinates(self, q, qd):
        arms, speeds = q.copy(), qd.copy()
        cfg = self.source.hardware["linkage"]
        link_ratio = cfg["active_arm_m"] / cfg["passive_arm_m"]
        for h, k, sign in ((0, 1, 1), (3, 4, -1)):
            beta = sign*q[k] + math.pi/2
            alpha = 2*math.atan2(math.sin(beta), link_ratio+math.cos(beta))
            ratio = .5 + link_ratio*math.cos(alpha/2)/(2*math.sqrt(1-link_ratio**2*math.sin(alpha/2)**2))
            arms[h], arms[k] = sign*q[h], sign*q[h]+alpha
            speeds[h], speeds[k] = sign*qd[h], sign*(qd[h]+qd[k]/ratio)
        return arms, speeds

    def _update_estimator(self):
        elapsed = float(self.data.time) - self.last_estimator_time
        if elapsed < .005-1e-12:
            return True
        q, qd = self.data.qpos[self.source.qpos_ids], self.data.qvel[self.source.qvel_ids]
        arms, speeds = self._motor_coordinates(q, qd)
        rotation, omega = self._rotation(), self._gyro_sample()
        # Ideal specific force at the free-joint root, with no sensor noise.
        acceleration = rotation.T @ (self.data.qacc[:3] - self.model.opt.gravity)
        now = round(float(self.data.time)*1000)
        ticks = (c.c_uint32*6)(*[now]*6)
        valid = self.runtime.estimator_update_qualified(c.byref(self.c_estimator), c.byref(self.c_quality),
            c.byref(self.c_measurement), floats(self.data.qpos[3:7]), floats(arms), floats(speeds),
            floats(omega), floats(acceleration), ticks, now, elapsed)
        self.last_estimator_time = float(self.data.time)
        if valid:
            self.joint_velocity = np.array(self.c_measurement.qd)
            self.estimated_xy = np.array(self.c_measurement.xy)
            self.estimated_velocity = np.r_[np.array(self.c_measurement.velocity), 0.]
        return bool(valid)

    def _gyro_sample(self):
        return self._velocity6()[:3]

    def reset(self, seed=0, command=(0., 0., .1)):
        self.c_estimator, self.c_quality = Estimator(), Quality()
        self.c_measurement, self.c_controller = Measurement(), Controller()
        self.reference_managed_tick = None
        self.last_estimator_time = 0.
        self.reset_in_progress = True
        super().reset(seed, command)
        self.reset_in_progress = False
        self.c_estimator.last_q[:] = self.data.qpos[self.source.qpos_ids]
        self.c_estimator.held_qd[:] = self.joint_velocity
        self.c_estimator.velocity_valid = True
        # The reset represents an already enabled balance state, not startup.
        self.c_quality.confidence = 1.
        self.c_quality.initialized = True
        self.c_measurement.q[:] = self.data.qpos[self.source.qpos_ids]
        return self.observation()

    def observation(self):
        if not getattr(self, "reset_in_progress", True) and self.reference_managed_tick != self.tick:
            m, ctrl = self.c_measurement, self.c_controller
            m.xy[:] = self.estimated_xy
            m.yaw = rpy_from_wxyz(self.data.qpos[3:7])[2]
            ctrl.reference_xy[:], ctrl.reference_yaw = self.reference, self.heading
            ctrl.command[:] = self.applied
            trusted = self.c_quality.confidence >= .25
            self.runtime.rl_manage_reference(c.byref(ctrl), c.byref(m), trusted, self.c_quality.reference_realign)
            self.c_quality.reference_realign = False
            self.reference, self.heading, self.applied = list(ctrl.reference_xy), float(ctrl.reference_yaw), list(ctrl.command)
            self.reference_managed_tick = self.tick
        requested = self.requested
        if not getattr(self, "reset_in_progress", True) and self.c_quality.confidence < .25:
            self.requested = [0., 0., requested[2]]
        result = super().observation()
        self.requested = requested
        return result

    def step(self, action):
        requested = self.requested
        if self.c_quality.confidence < .25:
            self.requested = [0., 0., requested[2]]
        result = super().step(action)
        self.requested = requested
        # Recompute after restoring the external request for the next policy tick.
        observation = self.observation()
        result[4].update(odometry_confidence=float(self.c_quality.confidence),
                         odometry_residual_m_s=float(self.c_quality.residual),
                         odometry_degraded_ms=int(self.c_quality.degraded_ms),
                         external_requested_command=list(requested))
        return observation, *result[1:]
