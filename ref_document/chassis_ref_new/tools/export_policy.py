#!/usr/bin/env python3
"""Export the archived float32 actor without Torch/CubeAI dependencies.

Only inert tensor descriptors are read; unknown pickle globals are rejected.
Use --torch-check when a local CPU PyTorch installation is available.
"""
from __future__ import annotations
import argparse
import collections
import dataclasses
import hashlib
import io
import json
import math
import os
from pathlib import Path
import pickle
import re
import struct
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SAMPLE = ROOT.parents[1] / "sample_weight_file" / "artifacts"
SIZES = (31, 128, 64, 32, 6)
TRAINING = ROOT.parent / "fifty_mujoco"
REALDATA = TRAINING / "runs/realdata_20261117"
PROFILES = ("realdata_full", "realdata_near", "qualified_u400", "legacy_u100")

def active_checkpoint():
    manifest = json.loads((ROOT / "user/algorithm/model/manifest.json").read_text())
    return ROOT / manifest.get("checkpoint_path", "../../sample_weight_file/artifacts/model_3050.pt")

def checked_export(folder):
    """Check every frozen export asset before installing any file."""
    manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8-sig"))
    normalized = []
    for name, digest in manifest["generated_sha256"].items():
        path = folder / name
        if not path.resolve().is_relative_to(folder.resolve()):
            raise ValueError("Export asset escapes its folder")
        raw = path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != digest:
            if path.suffix not in (".c", ".h", ".patch", ".py", ".json", ".md", ".xml") or hashlib.sha256(raw.replace(b"\r\n", b"\n")).hexdigest() != digest:
                raise ValueError(f"Candidate asset changed: {name}")
            normalized.append(name)
    if sha(folder / "actor31.pt") != manifest["sha256"]:
        raise ValueError("Actor checkpoint hash mismatch")
    load_actor(folder / "actor31.pt")
    return manifest, normalized


def install_candidate(output, profile="qualified_u400"):
    """Install matching balance/get-up exports; never register or flash them."""
    getup = None
    if profile in ("realdata_near", "realdata_full"):
        role = profile.removeprefix("realdata_")
        folder = REALDATA / ("final_export_balance_" + role)
        getup = REALDATA / ("final_export_" + role)
        training = REALDATA / "frozen_candidates" / role / "balance.pt"
        getup_training = training.with_name("getup.pt")
        source_update = 25 if role == "near" else 20
        version = 2026111700 + source_update
        acceptance = json.loads((REALDATA / "final_summary.json").read_text(encoding="utf-8-sig"))
        frozen = json.loads((REALDATA / "frozen_candidates/manifest.json").read_text(encoding="utf-8-sig"))
        if sha(training) != frozen["candidates"][role]["balance_sha256"] or sha(getup_training) != frozen["candidates"][role]["getup_sha256"]:
            raise ValueError("Frozen real-data checkpoint hash mismatch")
        qualified = False
    else:
        current = profile == "qualified_u400"
        run = TRAINING / ("runs/qualified_combined_v1_20261116" if current else "runs/hardware31_guarded_20261115")
        folder = run / ("candidate_u400" if current else "deployment_candidate")
        source_update = 400 if current else 100
        training = run / f"update_{source_update:06d}.pt"
        version = source_update
        acceptance = {"scope": "limited standing locomotion simulation; hardware pending"}
        qualified = current
    manifest, normalized = checked_export(folder)
    expected_training = manifest.get("source_training_checkpoint_sha256", manifest.get("source_checkpoint_sha256"))
    if sha(training) != expected_training:
        raise ValueError("Source training checkpoint hash mismatch")
    assets = {name: (folder / name).read_bytes().replace(b"\r\n", b"\n") for name in ("model.c", "model.h")}
    assets["actor31.pt"] = (folder / "actor31.pt").read_bytes()
    assets["model.h"] = assets["model.h"].replace(b"#endif", b'#include "profile.h"\n#endif')
    getup_metadata = None
    if getup is not None:
        gm, gn = checked_export(getup)
        normalized += ["getup/" + name for name in gn]
        physical_keys = ("wheel_radius_m", "spring", "reflected_inertia", "passive_drive",
                         "standing_default_rad", "standing_clearance_m", "wheel_kd")
        if gm["default_q"] != manifest["default_q"] or any(gm["configuration"][k] != manifest["configuration"][k] for k in physical_keys):
            raise ValueError("Get-up and balance physical/control contracts differ")
        manifest["balance_source_configuration"] = manifest["configuration"]
        manifest["configuration"] = gm["configuration"]
        if gm["source_checkpoint_sha256"] != sha(getup_training):
            raise ValueError("Get-up training source mismatch")
        for name in ("model.c", "model.h"):
            text = (getup / name).read_text(encoding="utf-8-sig")
            text = re.sub(r"\brl_", "rlg_", text)
            text = text.replace('"model.h"', '"getup_model.h"').replace("RL_MODEL_H", "RL_GETUP_MODEL_H")
            assets["getup_" + name] = text.encode("utf-8")
        assets["getup_actor31.pt"] = (getup / "actor31.pt").read_bytes()
        getup_metadata = {"sha256": gm["sha256"], "source_checkpoint_sha256": sha(getup_training),
                          "source_training_update": source_update, "registered": False,
                          "source_export_manifest_sha256": sha(getup / "manifest.json"),
                          "source_export_host_c_parity": gm["host_c_parity"]}
    else:
        assets["getup_model.h"] = b"/* Generated: selected profile has no get-up actor. */\n#ifndef RL_GETUP_MODEL_H\n#define RL_GETUP_MODEL_H\n#endif\n"
        assets["getup_model.c"] = b'#include "getup_model.h"\n'
        # Keep the installed file set deterministic when switching profiles.
        assets["getup_actor31.pt"] = b""
    cfg = manifest.get("configuration", {})
    radius = cfg.get("wheel_radius_m", 0.055)
    height = cfg.get("standing_clearance_m", 0.10)
    caps = 0.0 if getup else 0.15
    yaw = 0.0 if getup else 0.30
    constants = {"RL_PROFILE_MODEL_VERSION": str(version) + "U",
                 "RL_PROFILE_HAS_GETUP": str(int(getup is not None)),
                 "RL_PROFILE_ALLOW_REVERSE": str(int(qualified)),
                 "RL_PROFILE_WHEEL_RADIUS_M": literal(radius),
                 "RL_PROFILE_HEIGHT_COMMAND_M": literal(height),
                 "RL_PROFILE_FORWARD_CAP_M_S": literal(caps),
                 "RL_PROFILE_YAW_CAP_RAD_S": literal(yaw),
                 "RL_PROFILE_GETUP_TIMEOUT_MS": str(round(cfg.get("success", {}).get("getup_timeout_s", 5.0)*1000)) + "U",
                 "RL_PROFILE_HANDOFF_REANCHOR": str(int(cfg.get("handoff_reanchor_reference", False))),
                 "RL_PROFILE_ENTRY_SPEED_M_S": literal(cfg.get("handoff_velocity_limit_m_s", 0.08)),
                 "RL_PROFILE_BLEND_SPEED_M_S": literal(cfg.get("blend_velocity_limit_m_s", cfg.get("handoff_velocity_limit_m_s", 0.08)))}
    header = ["/* Generated by make export-policy; do not edit. */", "#ifndef RL_PROFILE_H", "#define RL_PROFILE_H"]
    header += [f"#define {key} {value}" for key, value in constants.items()]
    header += ["#endif", ""]
    assets["profile.h"] = "\n".join(header).encode("ascii")
    manifest.update(schema="installed_actor_bundle_v2", profile=profile,
                    checkpoint_path=Path(os.path.relpath(output / "actor31.pt", ROOT)).as_posix(),
                    deployment_version=profile + "_20261117_source_only",
                    source_newline_normalized_files=normalized,
                    source_export_manifest_sha256=sha(folder / "manifest.json"),
                    source_training_checkpoint=Path(os.path.relpath(training, ROOT)).as_posix(),
                    source_training_checkpoint_sha256=sha(training),
                    source_training_update=source_update if getup is None else None,
                    firmware_control_version="model_profiles_20261117_hardware_pending",
                    hardware_parity="source integration only; calibration and physical validation pending",
                    runtime_parameters={"wheel_radius_m": radius, "height_command_m": height,
                                        "forward_cap_m_s": caps, "yaw_cap_rad_s": yaw,
                                        "model_version": version},
                    getup=getup_metadata, startup_actor_auto_registered=False,
                    source_evaluation={"model_status": acceptance.get("model_status", acceptance.get("scope")),
                                       "all_scopes_passed": acceptance.get("all_scopes_passed"),
                                       "heldout_groups": acceptance.get("heldout_groups", [])},
                    real_robot_accepted=False, installed=True, flashed=False)
    if getup_metadata:
        getup_metadata["checkpoint_path"] = Path(os.path.relpath(output / "getup_actor31.pt", ROOT)).as_posix()
    output.mkdir(parents=True, exist_ok=True)
    changed = any(not (output / name).is_file() or (output / name).read_bytes() != raw
                  for name, raw in assets.items())
    for name, raw in assets.items():
        (output / name).write_bytes(raw)
    if changed and output.resolve() == (ROOT / "user/algorithm/model").resolve():
        # Some Windows make builds compare timestamps at one-second resolution.
        # A quick profile switch can otherwise reuse objects from another model.
        # Only disposable object files under this checkout's build root are removed.
        build = (ROOT / "build").resolve()
        for obj in build.rglob("*.o"):
            if not obj.resolve().is_relative_to(build):
                raise ValueError("Build object escapes workspace")
            obj.unlink()
    manifest["generated_sha256"] = {name: sha(output / name) for name in assets}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest

@dataclasses.dataclass
class Storage:
    key: str
    count: int

@dataclasses.dataclass
class Tensor:
    storage: Storage
    offset: int
    shape: tuple
    stride: tuple

def rebuild(storage, offset, shape, stride, *_):
    if not isinstance(storage, Storage) or type(offset) is not int or offset < 0:
        raise ValueError("Invalid tensor")
    if not isinstance(shape, tuple) or not isinstance(stride, tuple) or len(shape) != len(stride):
        raise ValueError("Invalid dimensions")
    if any(type(n) is not int or n < 0 for n in (*shape, *stride)):
        raise ValueError("Invalid extent")
    return Tensor(storage, offset, shape, stride)

class Reader(pickle.Unpickler):
    def find_class(self, module, name):
        allowed = {("collections", "OrderedDict"): collections.OrderedDict,
                   ("torch", "FloatStorage"): "f32",
                   ("torch._utils", "_rebuild_tensor_v2"): rebuild}
        if (module, name) not in allowed:
            raise ValueError(f"Unsupported pickle global {module}.{name}")
        return allowed[module, name]

    def persistent_load(self, pid):
        if not isinstance(pid, tuple) or len(pid) != 5:
            raise ValueError("Invalid persistent ID")
        kind, dtype, key, location, count = pid
        if kind != "storage" or dtype != "f32" or not isinstance(key, str) or not key.isdigit():
            raise ValueError("Only float32 storage is supported")
        if type(count) is not int or not 0 < count <= 8_000_000 or not isinstance(location, str):
            raise ValueError("Invalid storage bounds")
        return Storage(key, count)

def load_actor(path):
    def read(z, name):
        if z.getinfo(name).file_size > 32_000_000:
            raise ValueError("Oversized archive member")
        return z.read(name)
    with zipfile.ZipFile(path) as z:
        names = z.namelist()
        if len(names) != len(set(names)):
            raise ValueError("Duplicate ZIP member")
        meta = [n for n in names if n.endswith("/data.pkl")]
        if len(meta) != 1:
            raise ValueError("Expected one data.pkl")
        prefix = meta[0][:-8]
        if read(z, prefix + "byteorder").strip() != b"little":
            raise ValueError("Expected little endian")
        payload = Reader(io.BytesIO(read(z, meta[0]))).load()
        state = payload["model_state_dict"]
        actor = []
        expected = {f"actor.{i*2}.{kind}" for i in range(4) for kind in ("weight", "bias")}
        if {k for k in state if k.startswith("actor.")} != expected or any(k.startswith("encoder.") for k in state):
            raise ValueError("Unsupported actor/encoder schema")
        for i, (before, after) in enumerate(zip(SIZES, SIZES[1:])):
            layer = []
            for kind, shape in (("weight", (after, before)), ("bias", (after,))):
                t = state[f"actor.{2*i}.{kind}"]
                if not isinstance(t, Tensor) or t.shape != shape:
                    raise ValueError("Wrong tensor shape")
                stride = 1
                for n, s in zip(reversed(t.shape), reversed(t.stride)):
                    if s != stride:
                        raise ValueError("Noncontiguous tensor")
                    stride *= n
                raw = read(z, prefix + "data/" + t.storage.key)
                count = math.prod(shape)
                if len(raw) != 4*t.storage.count or t.offset + count > t.storage.count:
                    raise ValueError("Out-of-bounds tensor")
                values = list(struct.unpack_from(f"<{count}f", raw, t.offset*4))
                if not all(map(math.isfinite, values)):
                    raise ValueError("Nonfinite parameter")
                layer.append(values)
            actor.append(layer)
        return actor

def reference(actor, obs):
    x = list(obs)
    for i, (w, b) in enumerate(actor):
        n = len(x)
        y = [bias + sum(a*v for a, v in zip(w[o*n:(o+1)*n], x)) for o, bias in enumerate(b)]
        x = [math.expm1(v) if v < 0 else v for v in y] if i < 3 else y
    return x

def literal(value):
    # C99 hexadecimal literals preserve the checkpoint's EXACT float32 bits.
    return float(value).hex() + "f"

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def export(checkpoint, output):
    actor = load_actor(checkpoint)
    env = json.loads((SAMPLE / "env_cfg.json").read_text())
    train = json.loads((SAMPLE / "train_cfg.json").read_text())
    hw = json.loads((SAMPLE / "actuator_hardware.json").read_text())
    params = json.loads((SAMPLE / "model_parameters.json").read_text())
    if train["policy"]["activation"] != "elu" or env["env"]["num_observations"] != 31:
        raise ValueError("Unsupported model contract")
    output.mkdir(parents=True, exist_ok=True)
    header = ["/* Generated by tools/export_policy.py; do not edit. */", "#ifndef RL_MODEL_H", "#define RL_MODEL_H"]
    source = ['#include "model.h"']
    for i, layer in enumerate(actor):
        for kind, values in zip(("w", "b"), layer):
            name = f"rl_{kind}{i}"
            header.append(f"extern const float {name}[{len(values)}];")
            source.append(f"const float {name}[{len(values)}] = {{")
            source.extend("    " + ", ".join(map(literal, values[j:j+6])) + "," for j in range(0, len(values), 6))
            source.append("};")
    order = params["expected_runtime_dof_order"]
    arrays = {"rl_default_q": [env["init_state"]["default_joint_angles"][n] for n in order],
              "rl_kp": [env["control"]["stiffness"]["wheel" if "wheel" in n else n[1:3]] for n in order],
              "rl_kd": [env["control"]["damping"]["wheel" if "wheel" in n else n[1:3]] for n in order],
              "rl_knee_limits": sum([params["joint_limits_rad"][n] for n in (order[1], order[4])], []),
              "rl_joint_effort": [params["actuator_envelope"]["effort_nm"][n] for n in order],
              "rl_selftest_obs": [0.0]*31,
              "rl_selftest_output": reference(actor, [0.0]*31)}
    arrays["rl_physical_knee_limits"] = list(arrays["rl_knee_limits"])
    for name, values in arrays.items():
        header.append(f"extern const float {name}[{len(values)}];")
        source.append(f"const float {name}[{len(values)}] = {{" + ", ".join(map(literal, values)) + "};")
    leg, wheel = hw["leg_motor"], hw["wheel_motor"]
    ratio = wheel["total_reduction_ratio"] / (3591/187)
    constants = {"RL_LINK_RATIO": hw["linkage"]["active_arm_m"]/hw["linkage"]["passive_arm_m"],
                 "RL_REFERENCE_X": env["env"]["motion_reference_offset_xy_m"][0],
                 "RL_REFERENCE_Y": env["env"]["motion_reference_offset_xy_m"][1],
                 "RL_LEG_RATED_TORQUE": leg["rated_torque_nm"],
                 "RL_LEG_RATED_SPEED": leg["rated_speed_rpm_at_24v"]*math.pi/30,
                 "RL_LEG_FREE_SPEED": leg["no_load_speed_rpm_at_24v"]*math.pi/30,
                 "RL_WHEEL_RATED_TORQUE": 3*ratio,
                 "RL_WHEEL_RATED_SPEED": 469/ratio*math.pi/30,
                 "RL_WHEEL_FREE_SPEED": 482/ratio*math.pi/30}
    header.extend(f"#define {k} {literal(v)}" for k, v in constants.items())
    header.append("#endif")
    (output / "model.h").write_text("\n".join(header)+"\n", encoding="utf-8", newline="\n")
    (output / "model.c").write_text("\n".join(source)+"\n", encoding="utf-8", newline="\n")
    manifest = {"checkpoint": checkpoint.name, "sha256": sha(checkpoint), "sizes": SIZES,
                "checkpoint_path": Path(os.path.relpath(checkpoint.resolve(), ROOT)).as_posix(),
                "target_knee_limits_rad": [arrays["rl_knee_limits"][:2], arrays["rl_knee_limits"][2:]],
                "activation": "ELU alpha=1, linear output", "parameter_count": sum(len(v) for l in actor for v in l),
                "weights_are_exact_float32": True,
                "observation_timing_source_parity": "pending: original legged_robot.py revision unavailable",
                "hardware_parity": "pending calibration and source/golden-trace review",
                "sources": {p: sha(SAMPLE/p) for p in ("env_cfg.json", "train_cfg.json", "actuator_hardware.json", "model_parameters.json")},
                "joint_order": order, "default_q": arrays["rl_default_q"],
                "policy_dt": env["sim"]["dt"]*env["control"]["decimation"],
                "generated_sha256": {n: sha(output/n) for n in ("model.h", "model.c")}}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n", encoding="utf-8", newline="\n")
    return actor, manifest

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", type=Path, help="Explicit legacy archive export")
    p.add_argument("--profile", choices=PROFILES, default="qualified_u400",
                   help="Matched model bundle; real-data profiles remain unqualified")
    p.add_argument("--output", type=Path, default=ROOT/"user/algorithm/model")
    p.add_argument("--torch-check", action="store_true")
    args = p.parse_args()
    if args.checkpoint is None:
        manifest = install_candidate(args.output, args.profile)
        print(json.dumps({key: manifest[key] for key in ("checkpoint", "sha256", "deployment_version")}))
        return
    actor, manifest = export(args.checkpoint, args.output)
    if args.torch_check:
        import torch
        data = torch.load(args.checkpoint, map_location="cpu", weights_only=True)["model_state_dict"]
        x = torch.zeros(1, 31)
        for i in range(4):
            x = torch.nn.functional.linear(x, data[f"actor.{2*i}.weight"], data[f"actor.{2*i}.bias"])
            if i < 3:
                x = torch.nn.functional.elu(x)
        error = max(abs(a-b) for a, b in zip(x[0].tolist(), reference(actor, [0]*31)))
        if error > 1e-3:
            raise ValueError(f"PyTorch mismatch {error}")
        manifest["torch_check_zero_vector_max_error"] = error
        (args.output/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    print(json.dumps({k: manifest[k] for k in ("checkpoint", "sha256", "parameter_count", "policy_dt")}))

if __name__ == "__main__":
    main()
