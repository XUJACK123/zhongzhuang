"""fifty_mujoco: MuJoCo migration of the fifty wheel-legged robot.

Scope of this package (phase P0-P2):
    P0  audit source assets, quantify the known G1 transform defects
    P1  rebuild visual/COM/inertia with a single rigid transform, emit MJCF
    P2  CPU spawn audit and visual inspection

Nothing in this package trains a policy yet.
"""

from __future__ import annotations

__version__ = "0.1.0"

PACKAGE_ROOT: str = "fifty_mujoco"
