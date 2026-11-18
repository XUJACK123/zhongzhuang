"""Replay early motor snapshots and construct traceable seated reset candidates."""

from collections import Counter
from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
import torch

RUN = Path(__file__).resolve().parent
PROJECT = RUN.parents[1]
REFERENCE = PROJECT.parent
FROZEN = RUN.parent / "getup_seated_20261118_035341" / "frozen"
sys.path.insert(0, str(PROJECT / "tools"))

from fifty_mujoco.envs.sample_env31_supported import SupportedGetupEnv31
from realdata_seated_state_teacher import load_policy


IDS = (0x304, 0x303, 0x301, 0x302)
JOINTS = (0, 1, 3, 4)
SOURCES = {
    "173944": REFERENCE / "DM8009J_test/diagnostics/debug_recovery_20261113/check_20261113_173944_result.json",
    "205441": REFERENCE / "DM8009J_test/diagnostics/debug_recovery_20261113/wheels_20261113_205441_result.json",
}
CALIBRATION = REFERENCE / "chassis_ref_new/diagnostics/commission_20261113_230207/commission.json"


def read(path):
    return json.loads(path.read_text())


def write(name, payload):
    (RUN / name).write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n")


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def decode(path, calibration):
    frames = read(path)["b"]["frames"]
    observed = {}
    legs = []
    for identifier, idx in zip(IDS, JOINTS):
        counts = Counter(
            int.from_bytes(bytes.fromhex(frame["data"])[:2], "big")
            for frame in frames
            if frame["dlc"] == 8 and int(frame["id"], 16) == identifier
        )
        observed[hex(identifier)] = counts.most_common()
        count = counts.most_common(1)[0][0]
        delta = (count - calibration["counts"][idx] + 4096) % 8192 - 4096
        legs.append(calibration["ROBOT_ARM_REFERENCE_RAD"][idx]
                    + calibration["signs"][idx] * delta * np.pi / 4096)
    angles = []
    for idx, sign in ((0, 1), (2, -1)):
        hip, knee = legs[idx:idx + 2]
        half_alpha = (knee - hip) / 2
        beta = half_alpha + np.arcsin((113.4 / 135) * np.sin(half_alpha))
        angles.extend((sign * hip, sign * (beta - np.pi / 2)))
    return observed, np.array(angles)


def tilt_quaternion(roll_deg, pitch_deg):
    roll, pitch = np.deg2rad([roll_deg, pitch_deg]) / 2
    return [np.cos(roll) * np.cos(pitch), np.sin(roll) * np.cos(pitch),
            np.cos(roll) * np.sin(pitch), -np.sin(roll) * np.sin(pitch)]


class Replay:
    def __init__(self, config):
        self.actor = load_policy(FROZEN / "balance.pt", "cpu")
        self.env = SupportedGetupEnv31(self.actor, config=config, settle_seconds=3.)
        self.model, self.data = self.env.model, self.env.data
        self.leg = self.env.source.qpos_ids[list(JOINTS)]
        self.body = self.model.geom("base_link_collision_0").id
        self.wheels = [self.model.geom(f"{side}_wheel_collision_0").id for side in ("right", "left")]
        self.zero = np.zeros(self.model.nv)
        self.bank = self.env.seated["reset_bank"]

    def bottom(self, geom):
        z = self.data.geom_xmat[geom].reshape(3, 3)[2]
        size = self.model.geom_size[geom]
        if geom == self.body:
            extent = np.abs(z) @ size
        else:
            extent = size[0] * np.linalg.norm(z[:2]) + size[1] * abs(z[2])
        return self.data.geom_xpos[geom, 2] - extent

    def offset(self, geom):
        forward = self.data.xmat[self.model.geom_bodyid[self.body]].reshape(3, 3)[:2, 0]
        return float((self.data.geom_xpos[geom, :2] - self.data.qpos[:2]) @ forward)

    def shape(self, qpos):
        self.data.qpos[:] = qpos
        mujoco.mj_kinematics(self.model, self.data)
        return [self.bottom(g) for g in (self.body, *self.wheels)]

    def pose(self, angles, tilt):
        qpos = np.zeros(self.model.nq)
        qpos[3:7] = tilt_quaternion(*tilt)
        qpos[self.leg] = angles
        return qpos

    def project(self, angles, tilt, nominal_hips):
        qpos = self.pose(angles, tilt)
        for j, wheel in enumerate(self.wheels):
            hip = self.leg[2*j]

            def height(delta):
                qpos[hip] = nominal_hips[j] + delta
                bottom = self.shape(qpos)
                return bottom[j+1] - bottom[0]

            scan = np.linspace(-1.3, 1.3, 45)
            residual = [height(delta) for delta in scan]
            roots = []
            for low, high, prev, following in zip(scan[:-1], scan[1:], residual[:-1], residual[1:]):
                if prev * following > 0:
                    continue
                for _ in range(28):
                    mid = (low + high) / 2
                    if height(mid) * prev <= 0:
                        high = mid
                    else:
                        low = mid
                        prev = height(low)
                height((low + high) / 2)
                if self.offset(wheel) < -.01:
                    roots.append(qpos[hip])
            if not roots:
                return None
            qpos[hip] = min(roots, key=lambda root: abs(root - angles[2*j]))
        return qpos

    def run(self, qpos, spawn_gap, seed):
        qpos = qpos.copy()
        bottoms = self.shape(qpos)
        qpos[2] += spawn_gap - min(bottoms)
        entry = {"qpos": qpos.tolist(), "qvel": self.zero.tolist(), "class": "seated"}
        self.bank[:] = [entry]
        self.env.reset(seed)
        sample = self.env.last_sample
        state = self.data.qpos.copy()
        speed = float(np.max(np.abs(self.data.qvel)))
        self.shape(state)
        state[:2] = 0.
        detail = {
            "qpos_initial": entry["qpos"], "initial_bottom_m": (np.array(bottoms) + qpos[2]).tolist(),
            "qpos_final": state.tolist(), "time_s": sample.time_s, "failure": self.env.failed_reset,
            "observer_failure": self.env.env.last_info.get("failure"),
            "body_contact": sample.body_contact, "clearance_m": sample.clearance_m,
            "maximum_dof_speed": speed, "wheel_offset_m": [self.offset(g) for g in self.wheels],
            "wheel_load_n": list(self.env.wheel_load_n.values()),
            "assistance_n": self.env.support_peak_n, "assistance_nm": self.env.support_peak_nm,
            "active_motor_nm": float(np.max(np.abs(self.env.env.fast_actuators.command))),
        }
        return detail, entry


def main():
    torch.set_num_threads(1)
    config = read(FROZEN / "configuration.json")
    old_bank = deepcopy(config["seated_back"]["reset_bank"])
    replay = Replay(config)
    nominal = np.asarray(old_bank[0]["qpos"])[replay.leg]
    cal = read(CALIBRATION)["applied"][0]
    report = {"base_config_sha256": digest(FROZEN / "configuration.json"),
              "calibration_sha256": digest(CALIBRATION), "passive_settle_seconds": 3,
              "measured_contact": False, "sources": {}}
    train, evaluation = [], []
    for label, path in SOURCES.items():
        counts, angles = decode(path, cal)
        record = {"source": str(path.relative_to(REFERENCE)), "sha256": digest(path),
                  "counts": counts, "leg_angles_rad": angles.tolist(), "direct": [],
                  "projected": []}
        report["sources"][label] = record
        for roll, pitch in [(0, 0), (-8, 0), (8, 0), (0, -12), (0, 12)]:
            detail, _ = replay.run(replay.pose(angles, (roll, pitch)), .002, 850010)
            record["direct"].append(dict(detail, tilt_deg=[roll, pitch]))
        for measured_knee_fraction in (1., .75, .5, .32, .25, .18, .125, .0625):
            adjusted = angles.copy()
            adjusted[1::2] = nominal[1::2] + measured_knee_fraction * (angles[1::2] - nominal[1::2])
            for split, poses in [
                ("train", [(0, 0), (-2, 0), (2, 0), (0, -3), (0, 3)]),
                ("evaluation", [(-1, -2), (1, 2), (-3, 1), (3, -1)]),
            ]:
                for tilt in poses:
                    qpos = replay.project(adjusted, tilt, nominal[::2])
                    if qpos is None:
                        detail, entry = {"failure": "no_rear_wheel_tangent_solution"}, None
                    else:
                        detail, entry = replay.run(qpos, .0002, 850010)
                    detail.update(split=split, tilt_deg=list(tilt), measured_knee_fraction=measured_knee_fraction)
                    record["projected"].append(detail)
                    if detail["failure"] is None:
                        entry.update(source_snapshot=label, measured_knee_fraction=measured_knee_fraction,
                                     projected_tilt_deg=list(tilt))
                        (train if split == "train" else evaluation).append(entry)
        print(label, "direct", [x["failure"] for x in record["direct"]],
              "valid projected", sum(x["failure"] is None for x in record["projected"]), flush=True)
    report["train_candidates"] = len(train)
    report["evaluation_candidates"] = len(evaluation)
    write("mujoco_replay.json", report)
    if not train or not evaluation:
        return
    for name, entries in (("train", train + old_bank), ("evaluation", evaluation), ("nominal", old_bank)):
        saved = deepcopy(config)
        saved["seated_back"]["reset_bank"] = entries
        # A mild physics cohort tests initial-pose sensitivity alongside small hardware variation.
        saved["domain_ranges"] = {"nominal_fraction": .4, "motor_gain": [.985, 1.],
                                  "spring_scale": [.99, 1.01], "friction": [.71, .79],
                                  "armature_scale": [.93, 1.08], "drag_scale": [.91, 1.09]}
        write(f"{name}_configuration.json", saved)


if __name__ == "__main__":
    main()
