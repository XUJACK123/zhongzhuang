#!/usr/bin/env python3
"""Audit the sample archive with Python's standard library ONLY.

No torch/Isaac/MuJoCo import, tensor inference, ZIP extraction or network access.
Only a restricted set of inert storage/tensor descriptors is reconstructed;
unrecognized pickle globals are rejected. Run directly, not through the physics
CLI: python -I -S tools/sample_weight_offline.py --out reports/sample_weight/offline_audit.json
"""

from __future__ import annotations

import argparse
import ast
import collections
import dataclasses
import hashlib
import io
import json
import math
import pickle
import struct
import sys
import xml.etree.ElementTree as ET
import zipfile
from datetime import datetime, timezone
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
WORKSPACE = PROJECT.parents[1]
MAX_ENTRY_BYTES = 32 * 1024 * 1024
FLOAT_STORAGE = "symbolic_torch_float32_storage"


@dataclasses.dataclass(frozen=True)
class Storage:
    key: str
    location: str
    count: int


@dataclasses.dataclass
class Tensor:
    storage: Storage
    offset: int
    shape: tuple[int, ...]
    stride: tuple[int, ...]


def rebuild_tensor(storage, offset, shape, stride, requires_grad, hooks, *extra):
    if not isinstance(storage, Storage):
        raise ValueError("unsupported tensor storage")
    if type(offset) is not int or offset < 0:
        raise ValueError("invalid tensor offset")
    if not isinstance(shape, tuple) or not isinstance(stride, tuple):
        raise ValueError("tensor dimensions must be tuples")
    if len(shape) != len(stride) or len(shape) > 8:
        raise ValueError("invalid tensor dimensions")
    if any(type(x) is not int or x < 0 for x in (*shape, *stride)):
        raise ValueError("invalid shape/stride")
    return Tensor(storage, offset, shape, stride)


class DescriptorReader(pickle.Unpickler):
    """Never import classes requested by the checkpoint."""

    def find_class(self, module, name):
        allowed = {
            ("collections", "OrderedDict"): collections.OrderedDict,
            ("torch", "FloatStorage"): FLOAT_STORAGE,
            ("torch._utils", "_rebuild_tensor_v2"): rebuild_tensor,
        }
        try:
            return allowed[module, name]
        except KeyError:
            raise ValueError(f"unsupported checkpoint global: {module}.{name}") from None

    def persistent_load(self, pid):
        if not isinstance(pid, tuple) or len(pid) != 5:
            raise ValueError("invalid storage descriptor")
        kind, dtype, key, location, count = pid
        if kind != "storage" or dtype != FLOAT_STORAGE:
            raise ValueError("only float32 tensor descriptors are supported")
        if not isinstance(key, str) or not key.isdigit():
            raise ValueError("invalid storage key")
        if type(count) is not int or not 0 <= count <= MAX_ENTRY_BYTES // 4:
            raise ValueError("storage size outside audit limits")
        if not isinstance(location, str):
            raise ValueError("invalid source device")
        return Storage(key, location, count)


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def inside(base: Path, relative: str) -> Path:
    candidate = (base / relative).resolve()
    if not candidate.is_relative_to(base.resolve()):
        raise ValueError(f"path escapes audit root: {relative}")
    return candidate


def compare_file(path: Path, expected: dict) -> dict:
    if not path.is_file():
        return {"status": "missing"}
    raw = path.read_bytes()
    digest = sha256(raw)
    lf_digest = sha256(raw.replace(b"\r\n", b"\n"))
    status = ("raw_match" if digest == expected["sha256"] and len(raw) == expected["bytes"]
              else "lf_match" if lf_digest == expected["sha256"] else "different")
    return {"status": status, "bytes": len(raw), "sha256": digest,
            "sha256_lf": lf_digest, "expected_sha256": expected["sha256"]}


def read_entry(archive: zipfile.ZipFile, name: str) -> bytes:
    info = archive.getinfo(name)
    if info.file_size > MAX_ENTRY_BYTES:
        raise ValueError(f"ZIP entry exceeds audit limits: {name}")
    return archive.read(info)  # zipfile checks CRC; never extract paths.


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(a, v):
    return [sum(a[i][j] * v[j] for j in range(3)) for i in range(3)]


def rotation(axis, angle):
    norm = math.sqrt(sum(x*x for x in axis))
    if norm == 0:
        raise ValueError("zero joint axis")
    x, y, z = [v/norm for v in axis]
    c, s, t = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    return [[t*x*x+c, t*x*y-s*z, t*x*z+s*y],
            [t*x*y+s*z, t*y*y+c, t*y*z-s*x],
            [t*x*z-s*y, t*y*z+s*x, t*z*z+c]]


def origin(element):
    identity = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
    if element is None:
        return identity, [0., 0., 0.]
    xyz = [float(v) for v in element.get("xyz", "0 0 0").split()]
    r, p, y = [float(v) for v in element.get("rpy", "0 0 0").split()]
    return matmul(matmul(rotation([0, 0, 1], y), rotation([0, 1, 0], p)),
                  rotation([1, 0, 0], r)), xyz


def compose(a, b):
    ra, pa = a
    rb, pb = b
    delta = matvec(ra, pb)
    return matmul(ra, rb), [x+y for x, y in zip(pa, delta)]


def inertia_is_physical(xx, yy, zz, xy, xz, yz):
    values = (xx, yy, zz, xy, xz, yz)
    if not all(math.isfinite(x) for x in values):
        return False, False
    det = xx*yy*zz + 2*xy*xz*yz - xx*yz*yz - yy*xz*xz - zz*xy*xy
    positive = xx > 0 and xx*yy-xy*xy > 0 and det > 0
    trace = xx+yy+zz
    if trace <= 0:
        return positive, False
    # C = trace(I)/2 * identity - I must be positive semidefinite. This
    # expresses the principal-inertia triangle inequalities in any frame.
    a, b, c = [.5-v/trace for v in (xx, yy, zz)]
    d, e, f = [-v/trace for v in (xy, xz, yz)]
    minors = (a, b, c, a*b-d*d, a*c-e*e, b*c-f*f,
              a*b*c+2*d*e*f-a*f*f-b*e*e-c*d*d)
    return positive, min(minors) >= -1e-12


def source_geometry_audit(urdf, env, params):
    """URDF FK and convex primitive support bounds; no contact solver."""
    if env["init_state"]["rot"] != [0., 0., 0., 1.]:
        raise ValueError("source static audit requires the archived identity root orientation")
    frames = {"base_link": (origin(None)[0], env["init_state"]["pos"])}
    pending = list(urdf.findall("joint"))
    angles = env["init_state"]["default_joint_angles"]
    while pending:
        progressed = False
        for joint in list(pending):
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            if parent not in frames:
                continue
            frame = compose(frames[parent], origin(joint.find("origin")))
            if joint.get("type") in ("continuous", "revolute"):
                axis_node = joint.find("axis")
                axis = [float(v) for v in axis_node.get("xyz").split()]
                frame = compose(frame, (rotation(axis, angles[joint.get("name")]), [0., 0., 0.]))
            elif joint.get("type") != "fixed":
                raise ValueError("unsupported joint type in source audit")
            frames[child] = frame
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError("source URDF has disconnected/cyclic joint tree")
    collision_bounds = []
    inertia_checks = []
    total_mass = 0.
    for link in urdf.findall("link"):
        name = link.get("name")
        inertia = link.find("inertial/inertia")
        mass = float(link.find("inertial/mass").get("value"))
        values = {k: float(v) for k, v in inertia.attrib.items()}
        xx, yy, zz = [values[k] for k in ("ixx", "iyy", "izz")]
        xy, xz, yz = [values[k] for k in ("ixy", "ixz", "iyz")]
        positive, triangle = inertia_is_physical(xx, yy, zz, xy, xz, yz)
        inertia_checks.append({"body": name, "mass_kg": mass,
                               "mass_positive": math.isfinite(mass) and mass > 0,
                               "inertia_positive_definite": positive,
                               "principal_triangle_inequalities": triangle})
        total_mass += mass
        for collision in link.findall("collision"):
            r, p = compose(frames[name], origin(collision.find("origin")))
            box, cylinder = collision.find("geometry/box"), collision.find("geometry/cylinder")
            if box is not None:
                half = [float(v)/2 for v in box.get("size").split()]
                extent = sum(abs(r[2][i])*half[i] for i in range(3))
                kind = "box"
            elif cylinder is not None:
                radius, length = float(cylinder.get("radius")), float(cylinder.get("length"))
                vertical_axis = max(-1., min(1., r[2][2]))
                extent = radius*math.sqrt(max(0., 1-vertical_axis**2)) + length/2*abs(vertical_axis)
                kind = "cylinder"
            else:
                raise ValueError("source collision is not a supported primitive")
            collision_bounds.append({"body": name, "kind": kind,
                                     "center_world_m": p, "lowest_world_z_m": p[2]-extent})
    wheel_bottoms = [r["lowest_world_z_m"] for r in collision_bounds if "wheel" in r["body"]]
    base_bottoms = [r["lowest_world_z_m"] for r in collision_bounds if r["body"] == "base_link"]
    clearance_command = env["commands"]["ranges"]["height"][0]
    checks = {"mass_matches_source": abs(total_mass-params["modified_mass_kg"]) < 1e-9,
              "physical_inertias": all(r["mass_positive"] and r["inertia_positive_definite"]
                                        and r["principal_triangle_inequalities"] for r in inertia_checks),
              "both_wheels_at_floor": len(wheel_bottoms) == 2 and max(map(abs, wheel_bottoms)) < 1e-6,
              "base_clearance_matches_command": len(base_bottoms) == 1
              and abs(base_bottoms[0]-clearance_command) < 1e-6,
              "no_primitive_below_floor": all(r["lowest_world_z_m"] >= -1e-6 for r in collision_bounds)}
    return {"validation_level": "static_URDF_kinematics_only",
            "total_mass_kg": total_mass,
            "checks": checks,
            "inertias": inertia_checks, "collision_vertical_bounds": collision_bounds,
            "wheel_bottom_z_m": wheel_bottoms, "base_bottom_z_m": base_bottoms,
            "nominal_clearance_command_m": clearance_command,
            "not_checked": ["self_collision", "contact_dynamics", "spring_force_direction",
                            "closed_chain_feasibility", "standing_stability"]}


def checkpoint_audit(path: Path) -> dict:
    with zipfile.ZipFile(path) as archive:
        names = archive.namelist()
        if len(names) != len(set(names)):
            raise ValueError("duplicate ZIP entry")
        metadata = [name for name in names if name.endswith("/data.pkl")]
        if len(metadata) != 1:
            raise ValueError("expected one checkpoint data.pkl")
        prefix = metadata[0][:-len("data.pkl")]
        payload = DescriptorReader(io.BytesIO(read_entry(archive, metadata[0]))).load()
        if not isinstance(payload, dict) or not isinstance(payload.get("model_state_dict"), dict):
            raise ValueError("missing model_state_dict")
        if read_entry(archive, prefix + "byteorder").strip() != b"little":
            raise ValueError("only little-endian float32 archives are supported")
        state = payload["model_state_dict"]
        shapes = {}
        tensors = {}
        for key, tensor in state.items():
            if not isinstance(tensor, Tensor):
                raise ValueError(f"non-tensor model entry: {key}")
            stride = 1
            for extent, actual_stride in zip(reversed(tensor.shape), reversed(tensor.stride)):
                if actual_stride != stride:
                    raise ValueError(f"non-contiguous model tensor: {key}")
                stride *= extent
            count = math.prod(tensor.shape)
            data = read_entry(archive, prefix + "data/" + tensor.storage.key)
            if len(data) != 4 * tensor.storage.count or tensor.offset + count > tensor.storage.count:
                raise ValueError(f"storage bounds mismatch: {key}")
            values = [value[0] for value in struct.iter_unpack(
                "<f", data[tensor.offset * 4:(tensor.offset + count) * 4])]
            if not values or not all(math.isfinite(v) for v in values):
                raise ValueError(f"empty or non-finite model tensor: {key}")
            if key == "std" and min(values) <= 0:
                raise ValueError("policy standard deviations must be positive")
            shapes[key] = list(tensor.shape)
            tensors[key] = {"count": count, "min": min(values), "max": max(values),
                            "all_finite": True, "source_device": tensor.storage.location}
        expected = {"std": [6]}
        for group, sizes in (("actor", (31, 128, 64, 32, 6)),
                             ("critic", (147, 256, 128, 64, 1))):
            for i, (before, after) in enumerate(zip(sizes, sizes[1:])):
                expected[f"{group}.{2*i}.weight"] = [after, before]
                expected[f"{group}.{2*i}.bias"] = [after]
        if shapes != expected:
            raise ValueError(f"unexpected model schema: {shapes}")
        return {"path": path.name, "sha256": sha256(path.read_bytes()),
                "status": "static_pass", "iteration": payload.get("iter"),
                "payload_keys": list(payload), "shapes": shapes, "tensors": tensors,
                "encoder_present": any(k.startswith("encoder.") for k in state),
                "inference_executed": False}


def audit(workspace: Path) -> dict:
    workspace = workspace.resolve()
    source = workspace / "sample_weight_file"
    manifest = read_json(source / "manifest.json")["files"]
    archive_rows = {name: compare_file(inside(source, name), expected)
                    for name, expected in manifest.items()}
    source_manifest = read_json(source / "source-files.json")["files"]
    source_rows = {name: compare_file(inside(workspace, name), expected)
                   for name, expected in source_manifest.items()}
    # Resolve archived snapshots without replacing the user's current sources.
    resolved_sources = {}
    for name, expected in source_manifest.items():
        if source_rows[name]["status"] in ("raw_match", "lf_match"):
            resolved_sources[name] = {"path": name, "origin": "workspace",
                                      "status": source_rows[name]["status"]}
            continue
        archived = source / "artifacts" / Path(name).name
        comparison = compare_file(archived, expected)
        if comparison["status"] in ("raw_match", "lf_match"):
            resolved_sources[name] = {"path": archived.relative_to(workspace).as_posix(),
                                      "origin": "archive_snapshot", "status": comparison["status"]}
    critical_names = {
        "legged_robot.py", "on_policy_runner.py", "task_registry.py", "ppo.py",
        "moving_yaw_rewards.py", "evaluate_tracking.py",
    }
    critical_unresolved = [name for name in source_manifest
                           if Path(name).name in critical_names and name not in resolved_sources]
    candidate_search = []
    candidate_root = workspace / "ref_document/fudan_rl_wheel_leg-main"
    for name in critical_unresolved:
        matches = []
        scanned = 0
        for candidate in sorted(candidate_root.rglob(Path(name).name)):
            if not candidate.is_file():
                continue
            scanned += 1
            comparison = compare_file(candidate, source_manifest[name])
            if comparison["status"] in ("raw_match", "lf_match"):
                matches.append(candidate.relative_to(workspace).as_posix())
        candidate_search.append({"source": name, "candidates_checked": scanned, "matches": matches})
        if matches:
            resolved_sources[name] = {"path": matches[0], "origin": "local_matching_copy",
                                      "status": "hash_verified"}
    critical_unresolved = [name for name in critical_unresolved if name not in resolved_sources]
    checkpoints = [checkpoint_audit(source / "artifacts" / name)
                   for name in sorted(p.name for p in (source / "artifacts").glob("model_*.pt"))]
    run = read_json(source / "run.json")
    env = read_json(source / "artifacts/env_cfg.json")
    params = read_json(source / "artifacts/model_parameters.json")
    # Compare archived URDF asset names with the original package location too;
    # loading it directly from artifacts/ would resolve ../meshes incorrectly.
    urdf = ET.parse(source / "artifacts/fifty_training.urdf").getroot()
    geometry = source_geometry_audit(urdf, env, params)
    mesh_rows = []
    for mesh in urdf.findall(".//mesh"):
        name = mesh.attrib["filename"]
        intended = workspace / "solidwork_graph/fifty_version_training/urdf" / name
        local = source / "artifacts" / name
        intended = intended.resolve()
        source_name = intended.relative_to(workspace).as_posix()
        verified = source_rows.get(source_name, {}).get("status") == "raw_match"
        mesh_rows.append({"uri": name, "archive_relative_exists": local.is_file(),
                          "workspace_package_exists": intended.is_file(),
                          "workspace_sha256": sha256(intended.read_bytes()) if intended.is_file() else None,
                          "source_hash_verified": verified})
    imports = []
    tree = ast.parse((source / "artifacts/train_fifty.py").read_text(encoding="utf-8"))
    for stmt in tree.body:
        module = stmt.module if isinstance(stmt, ast.ImportFrom) else None
        if module and module not in sys.stdlib_module_names:
            local = source / "artifacts" / (module + ".py")
            imports.append({"module": module, "archived_local_module": local.is_file()})
    pose_issues = []
    for name, angle in env["init_state"]["default_joint_angles"].items():
        limits = params["joint_limits_rad"][name]
        if limits is not None and not limits[0] <= angle <= limits[1]:
            pose_issues.append(name)
    config = workspace / "ref_document/fifty_mujoco/configs/sample_actor31.json"
    contract = read_json(config)
    source_profile = read_json(config.with_name("sample_source_nominal.json"))
    pinned_inputs = source_profile["source_inputs"]
    profile_checks = {
        "runtime_disabled": source_profile["runtime_enabled"] is False,
        "inputs_frozen": bool(pinned_inputs) and all(
            inside(workspace, row["path"]).is_file()
            and sha256(inside(workspace, row["path"]).read_bytes()) == row["sha256"]
            for row in pinned_inputs.values()),
        "mesh_map_frozen": len(source_profile["mesh_resolution"]) == 7 and all(
            inside(workspace, row["path"]).is_file()
            and sha256(inside(workspace, row["path"]).read_bytes()) == row["sha256"]
            for row in source_profile["mesh_resolution"]),
        "effective_reset_matches_archive": source_profile["effective_reset"] == env["init_state"],
    }
    contract_checks = {
        "observation_dim": contract["network"]["actor_sizes"] == [31, 128, 64, 32, 6],
        "commands": contract["observations"]["command_scales"] == [2, 0.25, 5],
        "source_pose": contract["source"]["default_angles"] == [
            env["init_state"]["default_joint_angles"][n] for n in contract["joint_order"]],
        "wheel_scale": contract["actions"]["wheel_velocity_scale"] == env["control"]["vel_action_scale"],
        "source_height_is_clearance": env["env"]["clearance_control"] is True,
        "runtime_disabled": contract["runtime_enabled"] is False,
    }
    passed = (all(r["status"] == "raw_match" for r in archive_rows.values())
              and bool(checkpoints) and not pose_issues and all(contract_checks.values())
              and all(geometry["checks"].values()) and all(profile_checks.values()))
    return {"tool": "sample-weight-offline", "schema": 1,
            "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "status": "static_pass_runtime_pending" if passed else "failed",
            "validation_level": "static_only", "source_commit": run["source_commit"],
            "final_checkpoint": run["final_checkpoint"],
            "archive_integrity": {"checked": len(archive_rows),
                                  "matched": sum(r["status"] == "raw_match" for r in archive_rows.values()),
                                  "files": archive_rows},
            "source_comparison": {"counts": dict(collections.Counter(
                r["status"] for r in source_rows.values())), "files": source_rows},
            "source_resolution": {"resolved_count": len(resolved_sources),
                                  "files": resolved_sources,
                                  "critical_unresolved": critical_unresolved,
                                  "candidate_search": candidate_search},
            "checkpoints": checkpoints, "mesh_references": mesh_rows,
            "source_geometry": geometry,
            "top_level_local_imports": imports,
            "source_pose_outside_archived_limits": pose_issues,
            "contract_checks": contract_checks,
            "source_profile_checks": profile_checks,
            "gates": {"V0": "partial_source_revision_and_assets_pending",
                      "V1": "static_shapes_and_finiteness_only",
                      "V2_V8": "not_run"},
            "not_executed": ["torch_import", "network_inference", "mujoco_import",
                             "isaac_gym", "GPU", "simulation", "training"],
            "note": "Static pass is not source restoration, numerical inference parity or motion success."}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)
    report = audit(args.workspace)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"],
                      "archive_matched": report["archive_integrity"]["matched"],
                      "checkpoints": len(report["checkpoints"]),
                      "source_counts": report["source_comparison"]["counts"],
                      "out": str(args.out) if args.out else None}, ensure_ascii=False))
    assert not any(name in sys.modules for name in ("torch", "mujoco", "isaacgym"))
    return 0 if report["status"] == "static_pass_runtime_pending" else 1


if __name__ == "__main__":
    raise SystemExit(main())
