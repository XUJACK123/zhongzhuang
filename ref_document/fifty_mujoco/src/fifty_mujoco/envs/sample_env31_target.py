"""Target-profile 31-D environment layered on the frozen SampleEnv31.

``SampleEnv31`` is part of the active v1 training contract (its hash is checked
on resume), so the target profile swaps in :class:`SampleTargetSource` through a
subclass instead of editing the frozen module. Observation/action/reward
semantics are identical; only the physics profile differs.
"""

from __future__ import annotations

import torch

from ..contracts.sample_actor31 import JOINT_ORDER
from ..transfer.sample_target import SampleTargetSource
from .sample_env31 import REWARD_SCHEMA, SampleEnv31

TARGET_ENV_SCHEMA = "sample_target_cpu31_v1"
TARGET_REWARD_SCHEMA = REWARD_SCHEMA


class SampleTargetEnv31(SampleEnv31):
    def __init__(self, episode_seconds=10., effective_friction=.75):
        super().__init__(episode_seconds, effective_friction)
        self.source = SampleTargetSource(effective_friction)
        self.model, self.data = self.source.model, self.source.data
        self.base = self.model.body("base_link").id
        self.limits = self.model.jnt_range[self.source.joint_ids[[1, 4]]].copy()
        self.actuators = self.source.actuator_module.CoupledActuators(
            torch, self.source.hardware, JOINT_ORDER)
        self.reset()
