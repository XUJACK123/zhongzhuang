#!/usr/bin/env python3
"""Dependency-free validation of the generated `fifty_training` URDF.

Checks (all must pass before any Isaac Gym run):

1. XML parses, 7 links / 6 joints, one root, every link reachable.
2. Joint order equals ``expected_runtime_dof_order`` (right leg first, because
   ``legged_robot.py`` uses hard coded indices [0,1,3,4] and [2,5]).
3. 4 revolute + 2 continuous, every axis is a unit vector and the wheel axis
   is the lateral axis.
4. Mass equals the SW export mass; every inertia is positive definite and its
   principal moments satisfy the triangle inequality.
5. Every visual mesh exists and its SHA256 matches the recorded source hash.
6. Default joint angles lie inside the URDF limits.
7. 200 random poses: the URDF forward kinematics equals the Fudan planar
   formula (thigh horizontal forward = 0, shin straight down = pi/2).
8. The stance height implied by the default angles is consistent with the
   recorded spawn height.

Usage: python integration/validate_model.py  (exit code 1 on failure)
"""

from __future__ import annotations

import hashlib
import json
import math
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
PARAMS = json.loads((PACKAGE / "config" / "model_parameters.json").read_text(encoding="utf-8"))
URDF = PACKAGE / "urdf" / "fifty_training.urdf"


# ---------------------------------------------------------------------------
# minimal 3x3 linear algebra
# ---------------------------------------------------------------------------
IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def mat_vec(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def transpose(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def norm(a):
    return math.sqrt(sum(x * x for x in a))


def unit(a):
    n = norm(a)
    return [x / n for x in a]


def rpy_to_mat(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def rot_axis(axis, angle):
    x, y, z = unit(axis)
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def eigenvalues_sym(m):
    """Jacobi eigenvalues of a symmetric 3x3 matrix (ascending is not needed)."""
    a = [row[:] for row in m]
    for _ in range(60):
        off = abs(a[0][1]) + abs(a[0][2]) + abs(a[1][2])
        if off < 1e-18:
            break
        for p, q in ((0, 1), (0, 2), (1, 2)):
            if abs(a[p][q]) < 1e-20:
                continue
            theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
            t = (1.0 if theta >= 0 else -1.0) / (abs(theta) + math.sqrt(theta * theta + 1.0))
            c = 1.0 / math.sqrt(t * t + 1.0)
            s = t * c
            for k in range(3):
                akp, akq = a[k][p], a[k][q]
                a[k][p] = c * akp - s * akq
                a[k][q] = s * akp + c * akq
            for k in range(3):
                apk, aqk = a[p][k], a[q][k]
                a[p][k] = c * apk - s * aqk
                a[q][k] = s * apk + c * aqk
    return [a[0][0], a[1][1], a[2][2]]


# ---------------------------------------------------------------------------
# parsing / kinematics
# ---------------------------------------------------------------------------
def parse(path):
    root = ET.parse(path).getroot()
    links, joints, order = {}, {}, []
    for el in root:
        if el.tag == "link":
            name = el.get("name")
            inertial = el.find("inertial")
            i = inertial.find("inertia")
            vis = el.find("visual")
            col = el.find("collision")
            mesh = vis.find("geometry/mesh") if vis is not None else None
            geom = col.find("geometry") if col is not None else None
            prim = list(geom)[0] if geom is not None and len(geom) else None
            links[name] = {
                "mass": float(inertial.find("mass").get("value")),
                "com": [float(x) for x in inertial.find("origin").get("xyz").split()],
                "inertia": [float(i.get(k)) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")],
                "mesh": mesh.get("filename").split("/")[-1] if mesh is not None else None,
                "visual_origin": ([float(x) for x in vis.find("origin").get("xyz").split()]
                                  if vis is not None else None),
                "primitive": (prim.tag, dict(prim.attrib)) if prim is not None else None,
            }
        elif el.tag == "joint":
            origin = el.find("origin")
            limit = el.find("limit")
            order.append(el.get("name"))
            joints[el.get("name")] = {
                "type": el.get("type"),
                "parent": el.find("parent").get("link"),
                "child": el.find("child").get("link"),
                "xyz": [float(x) for x in origin.get("xyz", "0 0 0").split()],
                "rpy": [float(x) for x in origin.get("rpy", "0 0 0").split()],
                "axis": [float(x) for x in el.find("axis").get("xyz", "0 0 1").split()],
                "lower": float(limit.get("lower")) if limit is not None and limit.get("lower") else None,
                "upper": float(limit.get("upper")) if limit is not None and limit.get("upper") else None,
                "effort": float(limit.get("effort")) if limit is not None else None,
            }
    return links, joints, order


def fk(links, joints, q, root_link="base_link"):
    children = {}
    for name, j in joints.items():
        children.setdefault(j["parent"], []).append(name)
    poses = {root_link: (IDENTITY, [0.0, 0.0, 0.0])}
    stack = [root_link]
    seen = {root_link}
    while stack:
        parent = stack.pop()
        rp, pp = poses[parent]
        for jname in children.get(parent, []):
            j = joints[jname]
            rc = rpy_to_mat(j["rpy"])
            rc = mat_mul(rc, rot_axis(j["axis"], q.get(jname, 0.0)) if j["type"] != "fixed" else IDENTITY)
            r = mat_mul(rp, rc)
            p = [pp[k] + mat_vec(rp, j["xyz"])[k] for k in range(3)]
            poses[j["child"]] = (r, p)
            if j["child"] in seen:
                raise ValueError(f"link {j['child']} has two parents")
            seen.add(j["child"])
            stack.append(j["child"])
    return poses, seen


def main():
    checks = {}
    details = {}

    links, joints, order = parse(URDF)
    poses, reachable = fk(links, joints, {})
    checks["xml_parses"] = True
    checks["seven_links"] = len(links) == 7
    checks["six_joints"] = len(joints) == 6
    checks["all_links_reachable"] = reachable == set(links)
    checks["dof_order"] = order == PARAMS["expected_runtime_dof_order"]
    details["dof_order"] = order

    types = sorted(j["type"] for j in joints.values())
    checks["joint_types"] = types == ["continuous", "continuous", "revolute", "revolute", "revolute", "revolute"]
    checks["axes_unit_and_lateral"] = all(
        abs(norm(j["axis"]) - 1.0) < 1e-12 and abs(j["axis"][2] - 1.0) < 1e-12
        for j in joints.values())

    mass = sum(l["mass"] for l in links.values())
    checks["mass_preserved"] = abs(mass - PARAMS["source_mass_kg"]) < 1e-9
    details["mass_kg"] = mass

    inertia_ok, triangle_ok = True, True
    for name, l in links.items():
        ixx, ixy, ixz, iyy, iyz, izz = l["inertia"]
        M = [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]
        d1 = M[0][0]
        d2 = M[0][0] * M[1][1] - M[0][1] ** 2
        d3 = (M[0][0] * (M[1][1] * M[2][2] - M[1][2] ** 2)
              - M[0][1] * (M[0][1] * M[2][2] - M[1][2] * M[0][2])
              + M[0][2] * (M[0][1] * M[1][2] - M[1][1] * M[0][2]))
        if not (d1 > 0 and d2 > 0 and d3 > 0):
            inertia_ok = False
        ev = sorted(eigenvalues_sym(M))
        if not (ev[0] + ev[1] >= ev[2] - 1e-12 and ev[0] > 0):
            triangle_ok = False
    checks["inertia_positive_definite"] = inertia_ok
    checks["inertia_triangle_inequality"] = triangle_ok

    mesh_ok, hash_ok = True, True
    for name, l in links.items():
        dst = PACKAGE / "meshes" / l["mesh"]
        if not dst.is_file():
            mesh_ok = False
            continue
        recorded = PARAMS["mesh_transforms"][name]["sha256"]
        if recorded and hashlib.sha256(dst.read_bytes()).hexdigest() != recorded:
            hash_ok = False
    checks["mesh_files_exist"] = mesh_ok
    checks["mesh_sha256_matches_source"] = hash_ok

    limits_ok = True
    for name, j in joints.items():
        if j["type"] == "continuous":
            if j["lower"] is not None or j["upper"] is not None:
                limits_ok = False
        else:
            q = PARAMS["default_joint_angles_rad"][name]
            if not (j["lower"] < q < j["upper"]):
                limits_ok = False
    checks["default_angles_inside_limits"] = limits_ok

    # collision primitives
    prim_ok = True
    for name, l in links.items():
        if l["primitive"] is None:
            prim_ok = False
            continue
        tag, attrs = l["primitive"]
        if tag == "box":
            vals = [float(x) for x in attrs["size"].split()]
            prim_ok &= all(v > 0 for v in vals)
        elif tag == "cylinder":
            prim_ok &= float(attrs["radius"]) > 0 and float(attrs["length"]) > 0
        else:
            prim_ok = False
    checks["collision_primitives_positive"] = prim_ok
    wheel_r = float(links["right_wheel"]["primitive"][1]["radius"])
    checks["wheel_radius_matches_parameter"] = abs(wheel_r - PARAMS["wheel_radius_m"]) < 1e-12

    # 200 random poses: URDF FK vs Fudan planar formula
    l1 = PARAMS["leg_dimensions"]["l"]["thigh_planar_m"]
    l2 = PARAMS["leg_dimensions"]["l"]["shin_planar_m"]
    pairs = {"r": ("rf0_Joint", "rf1_Joint", "r0_link", "right_wheel"),
             "l": ("lf0_Joint", "lf1_Joint", "l0_link", "left_wheel")}
    random.seed(12)
    max_err = 0.0
    for _ in range(200):
        q = {n: random.uniform(-0.9, 0.9) + PARAMS["default_joint_angles_rad"][n] for n in joints}
        p, _ = fk(links, joints, q)
        for side, (hj, kj, thigh, wheel) in pairs.items():
            hip = p[thigh][1]
            wh = p[wheel][1]
            dx, dz = wh[0] - hip[0], wh[2] - hip[2]
            if side == "r":
                t1, t2 = q[hj], q[kj] + math.pi / 2.0
            else:
                t1, t2 = -q[hj], -q[kj] + math.pi / 2.0
            ex = l1 * math.cos(t1) + l2 * math.cos(t1 + t2)
            ey = l1 * math.sin(t1) + l2 * math.sin(t1 + t2)
            max_err = max(max_err, abs(dx - ex), abs(-dz - ey))
    checks["fudan_formula_match"] = max_err < 1e-9
    details["max_fk_error_m"] = max_err

    # stance height consistency
    p_default, _ = fk(links, joints, PARAMS["default_joint_angles_rad"])
    wheel_z = min(p_default["right_wheel"][1][2], p_default["left_wheel"][1][2])
    stance_height = -wheel_z + PARAMS["wheel_radius_m"]
    details["stance_height_m"] = stance_height
    details["spawn_height_m"] = PARAMS["spawn_height_m"]
    checks["spawn_height_matches_stance"] = abs(
        PARAMS["spawn_height_m"] - stance_height - 0.010) < 1e-6

    ok = all(checks.values())
    print(json.dumps({"ok": ok, "checks": checks, "details": details},
                     ensure_ascii=False, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
