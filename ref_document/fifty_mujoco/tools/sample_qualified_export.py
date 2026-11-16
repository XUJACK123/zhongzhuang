#!/usr/bin/env python3
"""Export an isolated qualified-balance candidate and verify host C inference."""
import argparse
import collections
import ctypes as c
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import numpy as np
import torch

PROJECT = Path(__file__).resolve().parents[1]
FIRMWARE = PROJECT.parent/"chassis_ref_new"
sys.path.insert(0, str(PROJECT/"src"))
sys.path.insert(0, str(FIRMWARE/"tools"))
import export_policy
from fifty_mujoco.envs.sample_env31_qualified import QualifiedHardwareEnv31, ENV_SCHEMA
from fifty_mujoco.learning.sample_ppo import SamplePolicy31
from fifty_mujoco.transfer.sample_source import verified_artifact
from fifty_mujoco.transfer.firmware_runtime import source_fingerprint


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if FIRMWARE.resolve() in (args.out.resolve(), *args.out.resolve().parents):
        parser.error("output must be outside active firmware")
    saved = torch.load(args.checkpoint, map_location="cpu", weights_only=True)
    contract = saved["contract"]
    if contract["environment"] != ENV_SCHEMA or contract["firmware_source"] != source_fingerprint():
        raise ValueError("qualified environment/firmware contract mismatch")
    for name, digest in contract["code"].items():
        path = PROJECT/name if name.startswith(("tools/", "src/")) else PROJECT/"src/fifty_mujoco"/name
        if sha(path) != digest:
            raise ValueError(f"source changed since training: {name}")
    torch.set_num_threads(1)
    policy = SamplePolicy31(verified_artifact("model_3050.pt"))
    policy.load_state_dict(saved["model"])
    env = QualifiedHardwareEnv31(wheel_kd=contract["wheel_kd"], knee_margin_rad=contract["knee_margin_rad"])
    if env.target_limits != contract["action_knee_limits"]:
        raise ValueError("target limits changed")
    args.out.mkdir(parents=True, exist_ok=False)
    actor_path = args.out/"actor31.pt"
    state = collections.OrderedDict(("actor."+k, v.detach().cpu().contiguous()) for k, v in policy.actor.state_dict().items())
    torch.save({"model_state_dict": state}, actor_path)
    _, manifest = export_policy.export(actor_path, args.out)
    generated = args.out/"model.c"
    source = generated.read_text()
    arrays = {"rl_kd": [2., 2., contract["wheel_kd"]]*2,
              "rl_knee_limits": sum(contract["action_knee_limits"], []),
              "rl_physical_knee_limits": env.limits.reshape(-1).tolist()}
    for name, values in arrays.items():
        replacement = f"const float {name}[{len(values)}] = {{"+", ".join(map(export_policy.literal, values))+"};"
        source, count = re.subn(r"const float "+name+r"\[\d+\] = \{[^}]*\};", replacement, source)
        if count != 1:
            raise ValueError(f"unexpected generated array {name}")
    generated.write_text(source)
    runtime = args.out/"c_runtime"
    for name in ("state_estimator.c", "state_estimator.h", "robot_config.h", "algorithm/rl.c", "algorithm/rl.h"):
        target = runtime/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(FIRMWARE/"user"/name, target)
    model = runtime/"algorithm/model"
    model.mkdir()
    for name in ("model.c", "model.h"):
        shutil.copyfile(args.out/name, model/name)
    wrapper = args.out/"check_actor.c"
    wrapper.write_text('#include "algorithm/rl.h"\nint candidate_actor(const float *o, float *a) { RlNetwork n; return rl_actor(&n,o,a); }\n')
    library = args.out/"host_actor.so"
    subprocess.run(["cc", "-std=c11", "-O2", "-shared", "-fPIC", "-I"+str(runtime), str(wrapper),
                    str(runtime/"algorithm/rl.c"), str(runtime/"state_estimator.c"), str(model/"model.c"),
                    "-lm", "-o", str(library)], check=True)
    lib = c.CDLL(str(library.resolve()))
    fp = c.POINTER(c.c_float)
    lib.candidate_actor.argtypes, lib.candidate_actor.restype = [fp, fp], c.c_int
    rng = np.random.default_rng(20261116)
    observations = np.r_[np.zeros((1, 31)), rng.uniform(-2, 2, (511, 31))].astype(np.float32)
    with torch.no_grad():
        expected = policy.actor(torch.tensor(observations)).numpy()
    outputs = np.empty((len(observations), 6), dtype=np.float32)
    for obs, out in zip(observations, outputs):
        if not lib.candidate_actor(obs.ctypes.data_as(fp), out.ctypes.data_as(fp)):
            raise ValueError("host C actor failed")
    if not np.allclose(outputs, expected, atol=1e-5, rtol=1e-4):
        raise ValueError("host C/Torch parity failed")
    (args.out/"simulation_snapshot.xml").write_text(env.source.xml)
    (args.out/"simulation_physics.json").write_text(json.dumps({"profile": env.source.profile,
        "spring": env.source.spring_config, "motor_ratings": env.actuators.inner.ratings,
        "joint_limits": env.limits.tolist(), "target_limits": env.target_limits,
        "physics_dt_s": float(env.model.opt.timestep), "policy_dt_s": env.source.policy_dt}, indent=2))
    manifest["archived_source_knee_limits_rad"] = manifest["target_knee_limits_rad"]
    manifest.update(target_knee_limits_rad=contract["action_knee_limits"],
                    physical_knee_limits_rad=env.limits.tolist(),
                    environment_contract=contract, source_training_checkpoint=str(args.checkpoint.resolve()),
                    source_training_checkpoint_sha256=sha(args.checkpoint), source_training_update=saved["update"],
                    wheel_kd=contract["wheel_kd"], physical_deployment_executed=False, real_robot_accepted=False,
                    getup_actor=False, host_c_parity={"vectors": len(observations), "passed": True,
                    "max_abs_error": float(np.abs(outputs-expected).max()), "atol": 1e-5, "rtol": 1e-4})
    manifest["generated_sha256"] = {str(p.relative_to(args.out)): sha(p) for p in args.out.rglob("*") if p.is_file() and p.name != "manifest.json"}
    (args.out/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    (args.out/"README.md").write_text(f"# 平衡仿真候选（未烧录）\n\n来源：{args.checkpoint.name}，update {saved['update']}。这是平衡actor，不是起身actor。\n\n轮Kd={contract['wheel_kd']}，膝目标余量={contract['knee_margin_rad']} rad；31维输入，PD/估计器/actor为2/5/10 ms。\n仿真使用当前固件C里程计质量/参考管理，传感器仍为理想输入；不包含实机IMU滤波、CAN时序、失能后使能或起身。\n\n`model.c/.h`及`actor31.pt`可复核，512组主机C/Torch输入对拍通过。`c_runtime/`保存本次编译源码，`simulation_physics.json`保存物理参数。\n不得把本包的验证视作实车通过；原固件未修改。\n")
    print(json.dumps({"out": str(args.out), "host_c_parity": manifest["host_c_parity"]}))


if __name__ == "__main__":
    main()
