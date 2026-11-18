"""Host-build the actual firmware estimator/reference code for simulation.

No hardware interfaces are linked. Input sensors are supplied by the simulator;
this does not reproduce the IMU filter, transport latency or encoder hardware.
"""
import ctypes as c
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile

from ..paths import PROJECT_ROOT


class Estimator(c.Structure):
    _fields_ = [("xy", c.c_float*2), ("last_q", c.c_float*6), ("held_qd", c.c_float*6),
                ("velocity_valid", c.c_bool), ("velocity_tick", c.c_uint32)]


class Quality(c.Structure):
    _fields_ = [("wheel_velocity_world", (c.c_float*3)*2), ("weight", c.c_float*2),
                ("residual", c.c_float), ("confidence", c.c_float), ("predicted_world", c.c_float*3),
                ("age_ms", c.c_uint32*6), ("degraded_ms", c.c_uint32),
                ("initialized", c.c_bool), ("reference_realign", c.c_bool)]


class Measurement(c.Structure):
    _fields_ = [("q", c.c_float*6), ("qd", c.c_float*6), ("omega", c.c_float*3),
                ("gravity", c.c_float*3), ("xy", c.c_float*2), ("yaw", c.c_float),
                ("velocity", c.c_float*2)]


class Controller(c.Structure):
    _fields_ = [("network", c.c_float*224), ("obs", c.c_float*31), ("action", c.c_float*6),
                ("command", c.c_float*3), ("reference_xy", c.c_float*2), ("reference_yaw", c.c_float),
                ("target_q", c.c_float*6), ("target_qd", c.c_float*6),
                ("joint_torque", c.c_float*6), ("motor_torque", c.c_float*6)]


RUNTIME_SOURCES = (
    "user/state_estimator.c", "user/state_estimator.h", "user/robot_config.h",
    "user/algorithm/rl.c", "user/algorithm/rl.h", "user/algorithm/model/model.c",
    "user/algorithm/model/model.h", "user/algorithm/model/profile.h",
)


def source_fingerprint():
    firmware = PROJECT_ROOT.parent / "chassis_ref_new"
    return {name: hashlib.sha256((firmware/name).read_bytes()).hexdigest() for name in RUNTIME_SOURCES}


def load_runtime():
    fingerprint = source_fingerprint()
    digest = hashlib.sha256(repr(sorted(fingerprint.items())).encode()).hexdigest()[:20]
    directory = Path(tempfile.gettempdir()) / "fifty_firmware_runtime"
    directory.mkdir(exist_ok=True)
    target = directory / f"qualified_{digest}.so"
    firmware = PROJECT_ROOT.parent / "chassis_ref_new"
    if not target.exists():
        temporary = target.with_suffix(f".{os.getpid()}.so")
        subprocess.run(["cc", "-std=c11", "-O2", "-shared", "-fPIC", "-I"+str(firmware/"user"),
                        str(firmware/"user/state_estimator.c"), str(firmware/"user/algorithm/rl.c"),
                        str(firmware/"user/algorithm/model/model.c"), "-lm", "-o", str(temporary)], check=True)
        os.replace(temporary, target)
    lib = c.CDLL(str(target))
    fp = c.POINTER(c.c_float)
    lib.estimator_update_qualified.restype = c.c_bool
    lib.estimator_update_qualified.argtypes = [c.POINTER(Estimator), c.POINTER(Quality), c.POINTER(Measurement),
                                              fp, fp, fp, fp, fp, c.POINTER(c.c_uint32), c.c_uint32, c.c_float]
    lib.rl_manage_reference.argtypes = [c.POINTER(Controller), c.POINTER(Measurement), c.c_bool, c.c_bool]
    lib.rl_motor_to_joint.restype = c.c_bool
    lib.rl_motor_to_joint.argtypes = [fp, fp, fp, fp]
    lib.rl_policy_step.restype = c.c_bool
    lib.rl_policy_step.argtypes = [c.POINTER(Controller), c.POINTER(Measurement), fp, c.c_float]
    return lib


def floats(values):
    return (c.c_float*len(values))(*values)
