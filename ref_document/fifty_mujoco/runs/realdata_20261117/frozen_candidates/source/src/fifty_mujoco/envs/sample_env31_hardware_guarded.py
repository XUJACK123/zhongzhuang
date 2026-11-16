"""Guard action targets inside the measured stroke; retain physical failures.

The actuator/PD/timing/reward and physical joint stops are identical to v3.
Only target position clipping changes, and it must be exported to firmware.
"""
import math
from .sample_env31_hardware_fast import FastHardwareEnv31
from .sample_env31_hardware import REWARD_SCHEMA
ENV_SCHEMA='sample_commissioned_guarded_cpu31_v4'


class GuardedHardwareEnv31(FastHardwareEnv31):
    def __init__(self,*args,knee_margin_rad=.03,**kwargs):
        if not math.isfinite(knee_margin_rad) or not 0<=knee_margin_rad<=.1:
            raise ValueError('knee margin must be finite within 0..0.1 rad')
        super().__init__(*args,**kwargs)
        self.knee_margin_rad=knee_margin_rad
        self.target_limits=[list(v) for v in self.target_limits]
        self.target_limits[0][0]=max(self.target_limits[0][0],float(self.limits[0,0])+knee_margin_rad)
        self.target_limits[1][1]=min(self.target_limits[1][1],float(self.limits[1,1])-knee_margin_rad)
