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
from pathlib import Path
import pickle
import struct
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SAMPLE = ROOT.parents[1] / "sample_weight_file" / "artifacts"
SIZES = (31, 128, 64, 32, 6)

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
    p.add_argument("--checkpoint", type=Path, default=SAMPLE/"model_3050.pt")
    p.add_argument("--output", type=Path, default=ROOT/"user/algorithm/model")
    p.add_argument("--torch-check", action="store_true")
    args = p.parse_args()
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
