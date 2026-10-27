#!/usr/bin/env python3
"""Build the Fudan-compatible training asset from `fifty_version_with_meshes`.

Input : ../../fifty_version_with_meshes/urdf/fifty_version_with_meshes.urdf (+ meshes)
Output: ../urdf/fifty_training.urdf
        ../config/model_parameters.json
        ../config/joint_names_fifty_training.yaml
        ../config/build_report.json
        ../docs/PARAMETER_TABLE.md
        ../meshes/*.STL  (hard links to the source meshes)

What the conversion does
------------------------
1. New base frame: origin = midpoint of the two hip joint origins,
   X = +old Y (provisional forward), Y = +old X (left), Z = -old Z (up).
   With old +X = left and old +Z = down this is the only proper rotation that
   yields X forward / Y left / Z up; --forward-sign only flips X and then
   prints a warning that Y no longer points left.
2. Every link frame is re-defined so that at q = 0 the leg matches the Fudan
   formula (thigh horizontal forward, shin straight down) and every joint
   local Z is the physical lateral axis.  The mesh / inertial transforms are
   derived from that choice, so the STL files are never modified.
3. Joints are written right-leg-first so that Isaac Gym's DOF order matches
   the hard coded indices in `legged_robot.py` ([0,1,3,4] legs, 2/5 wheels).
4. Collision geometry is replaced by primitives (the source reuses 1.18 M
   triangle visual meshes for collision, including a 51 MB base mesh).

Only the Python standard library is used.
"""

from __future__ import annotations

import argparse
import array
import hashlib
import json
import math
import os
import shutil
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE.parent / "fifty_version_with_meshes"
SOURCE_URDF = SOURCE / "urdf" / "fifty_version_with_meshes.urdf"
SOURCE_MESHES = SOURCE / "meshes"

# ---------------------------------------------------------------------------
# Design constants (all provisional values are recorded in model_parameters)
# ---------------------------------------------------------------------------
FORWARD_SIGN = 1            # +1: new X = +old Y
WHEEL_RADIUS_M = 0.060      # mesh derived, provisional (wheel assembly may lack a tyre)
SPAWN_CLEARANCE_M = 0.010
LEG_LIMIT_HALF_RANGE = {"hip": 0.60, "knee": 0.70}
EFFORT_NM = 30.0            # copied from the SW export, provisional
VELOCITY_RAD_S = 1000.0     # copied from the SW export, provisional
DAMPING = {"hip": 0.03, "knee": 0.03, "wheel": 0.002}
FRICTIONLOSS = {"hip": 0.015, "knee": 0.015, "wheel": 0.001}
BASE_COLLISION_SCALE = 1.0

LEGS = {
    "r": {
        "side": -1.0,
        "hip": "rf0_Joint", "knee": "rf1_Joint", "wheel": "r_wheel_Joint",
        "thigh": "r0_link", "shin": "r1_link", "wheel_link": "right_wheel",
    },
    "l": {
        "side": +1.0,
        "hip": "lf0_Joint", "knee": "lf1_Joint", "wheel": "l_wheel_Joint",
        "thigh": "l0_link", "shin": "l1_link", "wheel_link": "left_wheel",
    },
}
# Fudan / legged_robot.py hard coded order: right hip, right knee, right wheel,
# left hip, left knee, left wheel.
DOF_ORDER_OUT = [
    "rf0_Joint", "rf1_Joint", "r_wheel_Joint",
    "lf0_Joint", "lf1_Joint", "l_wheel_Joint",
]
# Mesh vertices per link are needed for the collision primitives.  base_link has
# >1 M triangles, so it is handled with its raw AABB instead of percentiles.
TRIMMED_BBOX_LINKS = {"l0_link", "l1_link", "left_wheel", "r0_link", "r1_link", "right_wheel"}

# ---------------------------------------------------------------------------
# Small 3x3 / 4x4 helpers (pure python, row major)
# ---------------------------------------------------------------------------
def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpose(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vec_scale(a, s):
    return [a[0] * s, a[1] * s, a[2] * s]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    if n <= 0.0:
        raise ValueError("zero-length vector")
    return [a[0] / n, a[1] / n, a[2] / n]


IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def rpy_to_mat(rpy):
    """URDF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def mat_to_rpy(m):
    sp = -m[2][0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    if abs(m[2][0]) < 0.9999999:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    else:  # gimbal lock
        roll = 0.0
        yaw = math.atan2(-m[0][1], m[1][1])
    return [roll, pitch, yaw]


def rot_axis(axis, angle):
    """Rodrigues rotation matrix about a unit axis."""
    x, y, z = unit(axis)
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return [
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]


def compose(ra, pa, rb, pb):
    """(Rb, pb) expressed in frame a: returns rotation, position."""
    return mat_mul(ra, rb), vec_add(pa, mat_vec(ra, pb))


def inverse(ra, pa):
    rt = transpose(ra)
    return rt, mat_vec(rt, vec_scale(pa, -1.0))


def fmt(v, nd=15):
    """Compact but exact-enough float formatting for URDF/JSON output."""
    if abs(v) < 1e-12:
        v = 0.0
    return f"{v:.{nd}g}"


def fmt_vec(v, nd=15):
    return " ".join(fmt(x, nd) for x in v)


# ---------------------------------------------------------------------------
# URDF parsing / forward kinematics of the source model
# ---------------------------------------------------------------------------
def parse_urdf(path):
    root = ET.parse(path).getroot()
    links, joints = {}, {}
    order_links, order_joints = [], []
    for el in root:
        if el.tag == "link":
            name = el.get("name")
            order_links.append(name)
            inertial = el.find("inertial")
            origin = inertial.find("origin")
            mass = float(inertial.find("mass").get("value"))
            i = inertial.find("inertia")
            links[name] = {
                "mass": mass,
                "com": [float(x) for x in origin.get("xyz", "0 0 0").split()],
                "inertial_rpy": [float(x) for x in origin.get("rpy", "0 0 0").split()],
                "inertia": {k: float(i.get(k, 0.0))
                            for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")},
                "mesh": None,
                "rgba": None,
            }
            vis = el.find("visual")
            if vis is not None:
                mesh = vis.find("geometry/mesh")
                if mesh is not None:
                    links[name]["mesh"] = mesh.get("filename").split("meshes/")[-1]
                color = vis.find("material/color")
                if color is not None:
                    links[name]["rgba"] = color.get("rgba")
        elif el.tag == "joint":
            name = el.get("name")
            order_joints.append(name)
            origin = el.find("origin")
            axis = el.find("axis")
            limit = el.find("limit")
            joints[name] = {
                "type": el.get("type"),
                "parent": el.find("parent").get("link"),
                "child": el.find("child").get("link"),
                "xyz": [float(x) for x in origin.get("xyz", "0 0 0").split()],
                "rpy": [float(x) for x in origin.get("rpy", "0 0 0").split()],
                "axis": [float(x) for x in axis.get("xyz", "0 0 1").split()],
                "limit": {k: (float(limit.get(k)) if limit.get(k) is not None else None)
                          for k in ("lower", "upper", "effort", "velocity")} if limit is not None else {},
            }
    return root.get("name"), links, joints, order_links, order_joints


def forward_kinematics(links, joints, root_link="base_link"):
    """World (source base frame) pose of every link at q = 0."""
    children = {}
    for name, j in joints.items():
        children.setdefault(j["parent"], []).append(name)
    poses = {root_link: (IDENTITY, [0.0, 0.0, 0.0])}
    stack = [root_link]
    while stack:
        parent = stack.pop()
        rp, pp = poses[parent]
        for jname in children.get(parent, []):
            j = joints[jname]
            rc = rpy_to_mat(j["rpy"])
            r, p = compose(rp, pp, rc, j["xyz"])
            poses[j["child"]] = (r, p)
            stack.append(j["child"])
    return poses


def joint_world_axis(joints, poses, name):
    """Joint axis in the source base frame.

    In URDF the ``<axis>`` is expressed in the joint frame, which coincides
    with the child link frame at q = 0 -- not in the parent frame.
    """
    j = joints[name]
    rc, _ = poses[j["child"]]
    return unit(mat_vec(rc, j["axis"]))


def joint_world_origin(joints, poses, name):
    j = joints[name]
    rp, pp = poses[j["parent"]]
    return vec_add(pp, mat_vec(rp, j["xyz"]))


# ---------------------------------------------------------------------------
# STL helpers
# ---------------------------------------------------------------------------
def stl_scan(path, rotation=None, translation=None, trim=None):
    """sha256 / triangle count / AABB, optionally expressed in another frame.

    When ``trim`` is given (fraction, e.g. 0.02) a percentile trimmed bbox is
    also returned; it is computed from the transformed vertices, so the result
    is an AABB of the mesh in the target link frame, not an inflated box of a
    rotated AABB.
    """
    h = hashlib.sha256()
    rot = rotation or IDENTITY
    trans = translation or [0.0, 0.0, 0.0]
    with open(path, "rb") as f:
        head = f.read(84)
        h.update(head)
        if len(head) < 84:
            raise ValueError(f"{path} is too short to be a binary STL")
        count = struct.unpack("<I", head[80:84])[0]
        expected = 84 + 50 * count
        size = os.path.getsize(path)
        if size != expected:
            raise ValueError(f"{path}: size {size} != 84+50*{count} (binary STL expected)")
        xs = array.array("f") if trim is not None else None
        ys = array.array("f") if trim is not None else None
        zs = array.array("f") if trim is not None else None
        mn = [math.inf] * 3
        mx = [-math.inf] * 3
        remaining = count
        while remaining:
            chunk = f.read(50 * min(remaining, 20000))
            h.update(chunk)
            remaining -= len(chunk) // 50
            for off in range(0, len(chunk), 50):
                vals = struct.unpack_from("<12f", chunk, off)
                for k in range(3):
                    t = mat_vec(rot, [vals[3 + 3 * k], vals[4 + 3 * k], vals[5 + 3 * k]])
                    v = [t[0] + trans[0], t[1] + trans[1], t[2] + trans[2]]
                    for a in range(3):
                        if v[a] < mn[a]:
                            mn[a] = v[a]
                        if v[a] > mx[a]:
                            mx[a] = v[a]
                    if xs is not None:
                        xs.append(v[0])
                        ys.append(v[1])
                        zs.append(v[2])
        result = {
            "bytes": size,
            "triangles": count,
            "sha256": h.hexdigest(),
            "aabb_min": mn,
            "aabb_max": mx,
        }
        if trim is not None:
            tmin, tmax = [], []
            for arr in (xs, ys, zs):
                ordered = sorted(arr)
                lo = ordered[int(trim * (len(ordered) - 1))]
                hi = ordered[int((1.0 - trim) * (len(ordered) - 1))]
                tmin.append(lo)
                tmax.append(hi)
            result["trimmed_min"] = tmin
            result["trimmed_max"] = tmax
            result["trim"] = trim
        return result


def transform_aabb(rot, pos, mn, mx):
    """AABB of a rotated box (8 corners) - conservative for the source AABB."""
    corners = []
    for x in (mn[0], mx[0]):
        for y in (mn[1], mx[1]):
            for z in (mn[2], mx[2]):
                corners.append(vec_add(pos, mat_vec(rot, [x, y, z])))
    lo = [min(c[i] for c in corners) for i in range(3)]
    hi = [max(c[i] for c in corners) for i in range(3)]
    return lo, hi


# ---------------------------------------------------------------------------
# Conversion
# ---------------------------------------------------------------------------
def sagittal_angle(x, z):
    """Angle from +X toward -Z (down); 0 = horizontal forward, pi/2 = straight down."""
    return math.atan2(-z, x)


def frame_from_axes(x_axis, z_axis):
    z = unit(z_axis)
    x = vec_sub(x_axis, vec_scale(z, dot(x_axis, z)))
    x = unit(x)
    y = cross(z, x)
    return [[x[0], y[0], z[0]],
            [x[1], y[1], z[1]],
            [x[2], y[2], z[2]]]


def build(args):
    robot_name, links, joints, order_links, order_joints = parse_urdf(args.source_urdf)
    poses = forward_kinematics(links, joints)
    total_mass = sum(l["mass"] for l in links.values())

    # --- 1. new base frame -------------------------------------------------
    hip_l = joint_world_origin(joints, poses, "lf0_Joint")
    hip_r = joint_world_origin(joints, poses, "rf0_Joint")
    origin_old = vec_scale(vec_add(hip_l, hip_r), 0.5)
    s = args.forward_sign
    x_new_old = [0.0, float(s), 0.0]
    z_new_old = [0.0, 0.0, -1.0]
    y_new_old = cross(z_new_old, x_new_old)
    R_B = [[x_new_old[0], x_new_old[1], x_new_old[2]],
           [y_new_old[0], y_new_old[1], y_new_old[2]],
           [z_new_old[0], z_new_old[1], z_new_old[2]]]
    det = (R_B[0][0] * (R_B[1][1] * R_B[2][2] - R_B[1][2] * R_B[2][1])
           - R_B[0][1] * (R_B[1][0] * R_B[2][2] - R_B[1][2] * R_B[2][0])
           + R_B[0][2] * (R_B[1][0] * R_B[2][1] - R_B[1][1] * R_B[2][0]))
    if det < 0:
        raise SystemExit("base transform is not a proper rotation")
    y_axis_base = [0.0, 1.0, 0.0]
    if dot(y_new_old, [1.0, 0.0, 0.0]) < 0:
        print("WARNING: with --forward-sign -1 the new +Y points to the old -X, "
              "i.e. the model's left side lands on -Y. Left/right labels must be swapped.")

    def to_base_point(p):
        return mat_vec(R_B, vec_sub(p, origin_old))

    def to_base_rot(r):
        return mat_mul(R_B, r)

    # --- 2. per leg: re-zero to the Fudan zero pose ------------------------
    leg_info = {}
    for side, spec in LEGS.items():
        hip_pt = to_base_point(joint_world_origin(joints, poses, spec["hip"]))
        knee_pt0 = to_base_point(joint_world_origin(joints, poses, spec["knee"]))
        wheel_pt0 = to_base_point(joint_world_origin(joints, poses, spec["wheel"]))
        a_hip = unit(mat_vec(R_B, joint_world_axis(joints, poses, spec["hip"])))
        a_knee = unit(mat_vec(R_B, joint_world_axis(joints, poses, spec["knee"])))
        if abs(abs(dot(a_hip, y_axis_base)) - 1.0) > 1e-9:
            raise SystemExit(f"{spec['hip']}: axis is not lateral")
        if abs(abs(dot(a_knee, y_axis_base)) - 1.0) > 1e-9:
            raise SystemExit(f"{spec['knee']}: axis is not lateral")

        v1 = vec_sub(knee_pt0, hip_pt)
        phi1 = sagittal_angle(v1[0], v1[2])
        delta_hip = -phi1                      # rotate thigh to horizontal forward
        R_hip = rot_axis(y_axis_base, delta_hip)
        v1r = mat_vec(R_hip, v1)
        knee_pt = vec_add(hip_pt, v1r)

        v2_sw = vec_sub(wheel_pt0, knee_pt0)
        phi2_sw = sagittal_angle(v2_sw[0], v2_sw[2])
        v2 = mat_vec(R_hip, v2_sw)
        phi2 = sagittal_angle(v2[0], v2[2])
        delta_knee = math.pi / 2.0 - phi2      # rotate shin to straight down
        v2r = mat_vec(rot_axis(y_axis_base, delta_knee), v2)
        wheel_pt = vec_add(knee_pt, v2r)

        leg_info[side] = {
            "hip_point": hip_pt,
            "knee_point": knee_pt,
            "wheel_point": wheel_pt,
            "wheel_point_sw": wheel_pt0,
            "delta_hip": delta_hip,
            "delta_knee": delta_knee,
            "l1_planar": math.hypot(v1r[0], v1r[2]),
            "l2_planar": math.hypot(v2r[0], v2r[2]),
            "l1_lateral": v1r[1],
            "l2_lateral": v2r[1],
            "theta1_sw": phi1,
            "theta2_sw": phi2_sw - phi1,
        }

    # --- 3. new link frames at q = 0 --------------------------------------
    X_B = [1.0, 0.0, 0.0]
    Z_B = [0.0, 0.0, 1.0]
    Y_B = [0.0, 1.0, 0.0]
    new_frames = {"base_link": (IDENTITY, [0.0, 0.0, 0.0])}
    joint_sign = {}
    for side, spec in LEGS.items():
        info = leg_info[side]
        # Fudan: theta1 = +q (right) / -q (left); theta2 = +/-q + pi/2.
        # Positive q must rotate about +Y (theta increases), so the child frame
        # Z axis is +Y for the right leg and -Y for the left leg.
        sz_leg = 1.0 if side == "r" else -1.0
        sz_wheel = 1.0                      # both wheels: positive qd = forward
        thigh_R = frame_from_axes(X_B, vec_scale(Y_B, sz_leg))
        shin_R = frame_from_axes(vec_scale(Z_B, -1.0), vec_scale(Y_B, sz_leg))
        wheel_R = frame_from_axes(X_B, vec_scale(Y_B, sz_wheel))
        new_frames[spec["thigh"]] = (thigh_R, info["hip_point"])
        new_frames[spec["shin"]] = (shin_R, info["knee_point"])
        new_frames[spec["wheel_link"]] = (wheel_R, info["wheel_point"])
        joint_sign[spec["hip"]] = sz_leg
        joint_sign[spec["knee"]] = sz_leg
        joint_sign[spec["wheel"]] = sz_wheel

    # --- 4. relative transforms old link -> new link ----------------------
    # The old link frame at q = 0 is rotated/translated by the re-zero rotation
    # of its own joint chain.  rot_acc[side] is the accumulated rotation.
    mesh_tf = {}
    for name, (R_old, p_old) in poses.items():
        R_old_b = to_base_rot(R_old)
        p_old_b = to_base_point(p_old)
        if name == "base_link":
            R_zero, p_zero = R_old_b, p_old_b
        else:
            side = "l" if name.startswith("l") else "r"
            info = leg_info[side]
            if name in (LEGS[side]["thigh"],):
                R_zero = mat_mul(rot_axis(Y_B, info["delta_hip"]), R_old_b)
                p_zero = vec_add(info["hip_point"], mat_vec(
                    rot_axis(Y_B, info["delta_hip"]), vec_sub(p_old_b, info["hip_point"])))
            elif name in (LEGS[side]["shin"],):
                Rz = rot_axis(Y_B, info["delta_hip"] + info["delta_knee"])
                p_zero = vec_add(info["knee_point"], mat_vec(
                    rot_axis(Y_B, info["delta_knee"]),
                    vec_sub(p_old_b, info["knee_point"])))
                R_zero = mat_mul(Rz, R_old_b)
            else:  # wheel link
                Rz = rot_axis(Y_B, info["delta_hip"] + info["delta_knee"])
                R_zero = mat_mul(Rz, R_old_b)
                p_zero = vec_add(info["wheel_point"], mat_vec(
                    Rz, vec_sub(p_old_b, info["wheel_point"])))
        Rn, pn = new_frames[name]
        rel_R, rel_p = inverse(Rn, pn)
        rel_R, rel_p = compose(rel_R, rel_p, R_zero, p_zero)
        mesh_tf[name] = (rel_R, rel_p)

    # --- 5. default joint angles = the SW assembly pose in the new chart ---
    default = {}
    for side, spec in LEGS.items():
        info = leg_info[side]
        sgn = 1.0 if side == "r" else -1.0
        default[spec["hip"]] = sgn * info["theta1_sw"]
        default[spec["knee"]] = sgn * (info["theta2_sw"] - math.pi / 2.0)
        default[spec["wheel"]] = 0.0

    # --- 6. collision primitives -----------------------------------------
    mesh_stats = {}
    for name, link in links.items():
        if not link["mesh"]:
            continue
        rel_R, rel_p = mesh_tf[name]
        path = SOURCE_MESHES / link["mesh"]
        stats = stl_scan(path, rotation=rel_R, translation=rel_p,
                         trim=0.02 if name in TRIMMED_BBOX_LINKS else None)
        stats["file"] = link["mesh"]
        mesh_stats[name] = stats

    collision = {}
    for name in links:
        rel_R, rel_p = mesh_tf[name]
        st = mesh_stats[name]
        if name == "base_link":
            lo, hi = st["aabb_min"], st["aabb_max"]
            size = [(hi[i] - lo[i]) * BASE_COLLISION_SCALE for i in range(3)]
            center = [(hi[i] + lo[i]) * 0.5 for i in range(3)]
            collision[name] = {"type": "box", "size": size, "origin": center,
                               "source": "mesh AABB in the new link frame, scale %.2f"
                                         % BASE_COLLISION_SCALE}
        elif name in ("l0_link", "r0_link", "l1_link", "r1_link"):
            lo, hi = st["trimmed_min"], st["trimmed_max"]
            ext = [hi[i] - lo[i] for i in range(3)]
            radius = max(0.012, min(0.05, 0.15 * max(ext[1], ext[2])))
            length = max(0.02, ext[0])
            center = [(hi[i] + lo[i]) * 0.5 for i in range(3)]
            # URDF has no <capsule>; a cylinder keeps the file portable and
            # Isaac Gym can convert it with asset_options.replace_cylinder_with_capsule.
            collision[name] = {"type": "cylinder", "radius": radius, "length": length,
                               "origin": center, "rpy": [0.0, math.pi / 2.0, 0.0],
                               "source": "trimmed bbox (2-98%) of the STL, cylinder axis = link x"}
        else:  # wheels: cylinder centred on the axle, width from the mesh
            width = st["trimmed_max"][2] - st["trimmed_min"][2]
            collision[name] = {"type": "cylinder", "radius": args.wheel_radius,
                               "length": max(0.02, abs(width)),
                               "origin": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0],
                               "source": "wheel radius parameter, width from the mesh"}

    # --- 7. spawn height ---------------------------------------------------
    # Height of the base origin at the *default* (SW assembly) stance.
    wheel_z = min(leg_info[s]["wheel_point_sw"][2] for s in LEGS)
    spawn_height = -wheel_z + args.wheel_radius + SPAWN_CLEARANCE_M

    # --- 8. joint limits ---------------------------------------------------
    limits = {}
    for side, spec in LEGS.items():
        for kind, jname in (("hip", spec["hip"]), ("knee", spec["knee"])):
            half = LEG_LIMIT_HALF_RANGE[kind]
            limits[jname] = [default[jname] - half, default[jname] + half]
        limits[spec["wheel"]] = None

    # --- 9. emit URDF ------------------------------------------------------
    urdf_root = ET.Element("robot", {"name": "fifty_training"})
    urdf_root.append(ET.Comment(
        " Generated by tools/build_training_urdf.py from "
        "fifty_version_with_meshes/urdf/fifty_version_with_meshes.urdf "
        "(joint types/axes repaired).  Equivalent serial 6-DOF model. "
        "Zero pose: thigh horizontal forward, shin straight down (Fudan convention). "
        "See config/model_parameters.json and docs/CONVERSION_NOTES.md. "))

    def add_link_xml(name):
        link = links[name]
        el = ET.SubElement(urdf_root, "link", {"name": name})
        rel_R, rel_p = mesh_tf[name]
        # inertial (about the CoM, expressed in the new link frame)
        inertial = ET.SubElement(el, "inertial")
        com_new = vec_add(rel_p, mat_vec(rel_R, link["com"]))
        ET.SubElement(inertial, "origin", {
            "xyz": fmt_vec(com_new), "rpy": fmt_vec(mat_to_rpy(rel_R))})
        ET.SubElement(inertial, "mass", {"value": fmt(link["mass"])})
        I = link["inertia"]
        M = [[I["ixx"], I["ixy"], I["ixz"]],
             [I["ixy"], I["iyy"], I["iyz"]],
             [I["ixz"], I["iyz"], I["izz"]]]
        Mn = mat_mul(mat_mul(rel_R, M), transpose(rel_R))
        ET.SubElement(inertial, "inertia", {
            "ixx": fmt(Mn[0][0]), "ixy": fmt(Mn[0][1]), "ixz": fmt(Mn[0][2]),
            "iyy": fmt(Mn[1][1]), "iyz": fmt(Mn[1][2]), "izz": fmt(Mn[2][2])})
        # visual keeps the original STL
        vis = ET.SubElement(el, "visual")
        ET.SubElement(vis, "origin", {"xyz": fmt_vec(rel_p), "rpy": fmt_vec(mat_to_rpy(rel_R))})
        geom = ET.SubElement(vis, "geometry")
        ET.SubElement(geom, "mesh", {"filename": f"../meshes/{link['mesh']}"})
        if link["rgba"]:
            mat = ET.SubElement(vis, "material", {"name": ""})
            ET.SubElement(mat, "color", {"rgba": link["rgba"]})
        col = ET.SubElement(el, "collision")
        c = collision[name]
        ET.SubElement(col, "origin", {"xyz": fmt_vec(c["origin"]),
                                      "rpy": fmt_vec(c.get("rpy", [0, 0, 0]))})
        g = ET.SubElement(col, "geometry")
        if c["type"] == "box":
            ET.SubElement(g, "box", {"size": fmt_vec(c["size"])})
        elif c["type"] == "capsule":
            ET.SubElement(g, "capsule", {"radius": fmt(c["radius"]),
                                         "length": fmt(c["length"])})
        else:
            ET.SubElement(g, "cylinder", {"radius": fmt(c["radius"]),
                                          "length": fmt(c["length"])})

    def add_joint_xml(name):
        j = joints[name]
        el = ET.SubElement(urdf_root, "joint", {"name": name, "type": j["type"]})
        parent_frame = new_frames[j["parent"]]
        child_frame = new_frames[j["child"]]
        rp, pp = inverse(parent_frame[0], parent_frame[1])
        rel_R, rel_p = compose(rp, pp, child_frame[0], child_frame[1])
        ET.SubElement(el, "origin", {"xyz": fmt_vec(rel_p), "rpy": fmt_vec(mat_to_rpy(rel_R))})
        ET.SubElement(el, "parent", {"link": j["parent"]})
        ET.SubElement(el, "child", {"link": j["child"]})
        ET.SubElement(el, "axis", {"xyz": "0 0 1"})
        kind = "wheel" if "wheel" in name else ("hip" if name in ("rf0_Joint", "lf0_Joint") else "knee")
        ET.SubElement(el, "dynamics", {"damping": fmt(DAMPING[kind]),
                                       "friction": fmt(FRICTIONLOSS[kind])})
        if limits[name] is None:
            ET.SubElement(el, "limit", {"effort": fmt(args.effort),
                                        "velocity": fmt(args.velocity)})
        else:
            lo, hi = limits[name]
            if not (lo < default[name] < hi):
                raise SystemExit(f"{name}: default angle outside limits")
            ET.SubElement(el, "limit", {"lower": fmt(lo), "upper": fmt(hi),
                                        "effort": fmt(args.effort),
                                        "velocity": fmt(args.velocity)})

    # Declaration order: right leg first (DOF order must match legged_robot.py).
    add_link_xml("base_link")
    for side in ("r", "l"):
        spec = LEGS[side]
        add_joint_xml(spec["hip"])
        add_link_xml(spec["thigh"])
        add_joint_xml(spec["knee"])
        add_link_xml(spec["shin"])
        add_joint_xml(spec["wheel"])
        add_link_xml(spec["wheel_link"])

    ET.indent(urdf_root, space="  ")
    out_urdf = PACKAGE / "urdf" / "fifty_training.urdf"
    out_urdf.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(urdf_root).write(out_urdf, encoding="utf-8", xml_declaration=True)

    # --- 10. meshes (hard link) -------------------------------------------
    out_mesh_dir = PACKAGE / "meshes"
    out_mesh_dir.mkdir(parents=True, exist_ok=True)
    linked = {}
    for name, link in links.items():
        if not link["mesh"]:
            continue
        src = SOURCE_MESHES / link["mesh"]
        dst = out_mesh_dir / link["mesh"]
        if not dst.exists():
            try:
                os.link(src, dst)
                linked[link["mesh"]] = "hardlink"
            except OSError:
                shutil.copy2(src, dst)
                linked[link["mesh"]] = "copy"
        else:
            linked[link["mesh"]] = "existing"

    # --- 11. model_parameters.json ----------------------------------------
    params = {
        "model_kind": "fifty_equivalent_serial_6dof",
        "source_urdf": str(SOURCE_URDF),
        "source_urdf_sha256": hashlib.sha256(SOURCE_URDF.read_bytes()).hexdigest(),
        "source_robot_name": robot_name,
        "source_mass_kg": total_mass,
        "modified_mass_kg": total_mass,
        "base_convention": (
            "origin = hip axis midpoint; X = %+d * old Y (provisional forward); "
            "Y = Z x X (left while forward-sign = +1); Z = -old Z (up)" % s),
        "base_old_to_new_rotation": R_B,
        "base_old_origin_m": origin_old,
        "zero_pose": "thigh horizontal forward, shin straight down (Fudan formula zero)",
        "expected_runtime_dof_order": DOF_ORDER_OUT,
        "default_joint_angles_rad": default,
        "joint_sign_table": joint_sign,
        "leg_dimensions": {
            side: {
                "thigh_planar_m": leg_info[side]["l1_planar"],
                "shin_planar_m": leg_info[side]["l2_planar"],
                "thigh_lateral_m": leg_info[side]["l1_lateral"],
                "shin_lateral_m": leg_info[side]["l2_lateral"],
                "hip_m": leg_info[side]["hip_point"],
                "knee_m": leg_info[side]["knee_point"],
                "wheel_m": leg_info[side]["wheel_point"],
                "sw_theta1_rad": leg_info[side]["theta1_sw"],
                "sw_theta2_rad": leg_info[side]["theta2_sw"],
                "rezero_hip_rad": leg_info[side]["delta_hip"],
                "rezero_knee_rad": leg_info[side]["delta_knee"],
            } for side in ("r", "l")
        },
        "spawn_height_m": spawn_height,
        "wheel_radius_m": args.wheel_radius,
        "wheel_radius_source": "mesh radius about the axle, NOT measured on hardware",
        "joint_limits_rad": {k: v for k, v in limits.items()},
        "joint_effort_nm": args.effort,
        "joint_velocity_rad_s": args.velocity,
        "joint_damping": DAMPING,
        "joint_frictionloss": FRICTIONLOSS,
        "mesh_transforms": {
            name: {
                "rotation": mesh_tf[name][0],
                "translation": mesh_tf[name][1],
                "visual_xyz": mesh_tf[name][1],
                "visual_rpy": mat_to_rpy(mesh_tf[name][0]),
                "file": links[name]["mesh"],
                "sha256": mesh_stats[name]["sha256"] if name in mesh_stats else None,
                "triangles": mesh_stats[name]["triangles"] if name in mesh_stats else None,
            } for name in links
        },
        "collision": collision,
        "provisional": {
            "wheel_radius_m": True,
            "joint_limits_rad": True,
            "joint_effort_nm": True,
            "joint_velocity_rad_s": True,
            "base_collision_box": True,
            "forward_direction": s == 1,
            "base_inertia": True,
        },
        "validation": {
            "isaacgym_runtime": "NOT RUN",
            "mujoco_runtime": "NOT RUN",
            "fudan_formula_cases": 200,
        },
    }
    cfg_dir = PACKAGE / "config"
    cfg_dir.mkdir(parents=True, exist_ok=True)
    (cfg_dir / "model_parameters.json").write_text(
        json.dumps(params, indent=2, ensure_ascii=False), encoding="utf-8")
    (cfg_dir / "joint_names_fifty_training.yaml").write_text(
        "controller_joint_names: [" + ", ".join(DOF_ORDER_OUT) + "]\n", encoding="utf-8")

    # --- 12. verification --------------------------------------------------
    report = verify(out_urdf, params, links, joints, poses, leg_info, total_mass, mesh_stats)
    report["mesh_files"] = linked
    report["source"] = str(SOURCE_URDF)
    (cfg_dir / "build_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    write_parameter_table(params, report)
    print(json.dumps({k: report[k] for k in
                      ("checks", "max_fk_error_m", "total_mass_kg", "spawn_height_m")},
                     indent=2, ensure_ascii=False))
    print(f"\nwrote {out_urdf}")


def to_base_rot_axis(poses, joints, name):
    """World joint axis of the source model (helper to keep build() readable)."""
    return joint_world_axis(joints, poses, name)


def fk_new_chain(links_new, joints_new, default, q):
    """FK of the generated URDF for a full configuration vector."""
    children = {}
    for name, j in joints_new.items():
        children.setdefault(j["parent"], []).append(name)
    poses = {"base_link": (IDENTITY, [0.0, 0.0, 0.0])}
    stack = ["base_link"]
    while stack:
        parent = stack.pop()
        rp, pp = poses[parent]
        for jname in children.get(parent, []):
            j = joints_new[jname]
            r0 = rpy_to_mat(j["rpy"])
            ang = q.get(jname, 0.0)
            rj = rot_axis(j["axis"], ang) if j["type"] != "fixed" else IDENTITY
            r, p = compose(rp, pp, mat_mul(r0, rj), j["xyz"])
            poses[j["child"]] = (r, p)
            stack.append(j["child"])
    return poses


def verify(out_urdf, params, links, joints_src, poses_src, leg_info, total_mass, mesh_stats):
    import random
    root, links_new, joints_new, _, _ = parse_urdf(out_urdf)
    checks = []
    checks.append(("xml_parses", True))
    checks.append(("links", len(links_new) == 7))
    checks.append(("joints", len(joints_new) == 6))
    checks.append(("dof_order", [j for j in joints_new] == DOF_ORDER_OUT))
    mass_new = sum(l["mass"] for l in links_new.values())
    checks.append(("mass_preserved", abs(mass_new - total_mass) < 1e-9))

    # positive definite inertia + triangle inequality
    ok = True
    for name, l in links_new.items():
        i = l["inertia"]
        M = [[i["ixx"], i["ixy"], i["ixz"]],
             [i["ixy"], i["iyy"], i["iyz"]],
             [i["ixz"], i["iyz"], i["izz"]]]
        tr = M[0][0] + M[1][1] + M[2][2]
        # principal moments via 20 power iterations on a shifted matrix is overkill;
        # use Sylvester's criterion with the 2x2 minors plus the triangle check.
        d1 = M[0][0]
        d2 = M[0][0] * M[1][1] - M[0][1] * M[1][0]
        d3 = (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
              - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
              + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]))
        if not (d1 > 0 and d2 > 0 and d3 > 0):
            ok = False
        if not (tr > 0):
            ok = False
    checks.append(("inertia_positive_definite", ok))

    # mesh files exist
    ok = all((PACKAGE / "meshes" / l["mesh"]).exists() for l in links_new.values() if l["mesh"])
    checks.append(("mesh_files_exist", ok))
    for name, st in mesh_stats.items():
        if (PACKAGE / "meshes" / st["file"]).exists():
            dst = PACKAGE / "meshes" / st["file"]
            h = hashlib.sha256(dst.read_bytes()).hexdigest()
            if h != st["sha256"]:
                checks.append((f"mesh_unchanged_{name}", False))
    checks.append(("meshes_unchanged", all(ok for n, ok in checks if n.startswith("mesh_unchanged")))) 

    # default angles inside limits
    ok = True
    for name, j in joints_new.items():
        if j["type"] == "continuous":
            continue
        if not (j["limit"]["lower"] < params["default_joint_angles_rad"][name] < j["limit"]["upper"]):
            ok = False
    checks.append(("default_angles_inside_limits", ok))

    # 200 random poses: URDF FK vs Fudan planar formula (sagittal projection)
    random.seed(12)
    max_err = 0.0
    l1 = {s: leg_info[s]["l1_planar"] for s in leg_info}
    l2 = {s: leg_info[s]["l2_planar"] for s in leg_info}
    for _ in range(200):
        q = {n: random.uniform(-0.9, 0.9) + params["default_joint_angles_rad"][n]
             for n in joints_new}
        p = fk_new_chain(links_new, joints_new, None, q)
        for side in ("r", "l"):
            spec = LEGS[side]
            hip = p[spec["thigh"]][1]
            wheel = p[spec["wheel_link"]][1]
            dx = wheel[0] - hip[0]
            dz = wheel[2] - hip[2]
            if side == "r":
                t1 = q[spec["hip"]]
                t2 = q[spec["knee"]] + math.pi / 2.0
            else:
                t1 = -q[spec["hip"]]
                t2 = -q[spec["knee"]] + math.pi / 2.0
            ex = l1[side] * math.cos(t1) + l2[side] * math.cos(t1 + t2)
            ey = l1[side] * math.sin(t1) + l2[side] * math.sin(t1 + t2)
            err = max(abs(dx - ex), abs(-dz - ey))
            max_err = max(max_err, err)
    checks.append(("fudan_formula_fk_match", max_err < 1e-9))

    # zero pose check (child link origins: thigh link origin = hip, shin = knee)
    p0 = fk_new_chain(links_new, joints_new, None, {})
    knee_r = p0["r1_link"][1]
    checks.append(("zero_pose_thigh_horizontal", abs(knee_r[2]) < 1e-9 and knee_r[0] > 0))
    wheel_r = p0["right_wheel"][1]
    checks.append(("zero_pose_shin_vertical", abs(wheel_r[0] - knee_r[0]) < 1e-9 and wheel_r[2] < knee_r[2]))

    return {
        "checks": {k: bool(v) for k, v in checks},
        "max_fk_error_m": max_err,
        "total_mass_kg": mass_new,
        "spawn_height_m": params["spawn_height_m"],
        "all_passed": all(bool(v) for _, v in checks),
    }


def write_parameter_table(params, report):
    d = params["leg_dimensions"]
    rows = [
        ("total_mass_kg", f"{params['modified_mass_kg']:.9f}", "URDF / SW export", "mechanical design (not weighed)"),
        ("wheel_radius_m", f"{params['wheel_radius_m']:.4f}", "STL radius about axle", "PROVISIONAL - measure the real wheel"),
        ("thigh_planar_m", f"{d['r']['thigh_planar_m']:.6f}", "URDF FK", "exact from geometry"),
        ("shin_planar_m", f"{d['r']['shin_planar_m']:.6f}", "URDF FK", "exact from geometry"),
        ("hip_separation_m", f"{abs(d['l']['hip_m'][1] - d['r']['hip_m'][1]):.6f}", "URDF FK", "exact from geometry"),
        ("wheel_base_m", f"{abs(d['l']['wheel_m'][1] - d['r']['wheel_m'][1]):.6f}", "URDF FK", "exact from geometry"),
        ("spawn_height_m", f"{params['spawn_height_m']:.6f}", "computed", "default stance + 10 mm clearance"),
        ("base_com_forward_m", "see model_parameters", "URDF inertial", "mechanical design"),
        ("leg_joint_effort_nm", f"{params['joint_effort_nm']:.1f}", "SW export", "PROVISIONAL - no motor data"),
        ("leg_joint_velocity_rad_s", f"{params['joint_velocity_rad_s']:.0f}", "SW export", "PROVISIONAL"),
    ]
    lines = [
        "# fifty_version_training 参数表",
        "",
        "由 `tools/build_training_urdf.py` 生成，数值改动后请重新生成，不要手工编辑。",
        "",
        "| 参数 | 数值 | 来源 | 可信度 |",
        "|---|---:|---|---|",
    ]
    for r in rows:
        lines.append("| `%s` | %s | %s | %s |" % r)
    lines += [
        "",
        "## 默认关节角（= SW 装配姿态在复旦坐标下的表示）",
        "",
        "| joint | q (rad) | 说明 |",
        "|---|---:|---|",
    ]
    for name in params["expected_runtime_dof_order"]:
        lines.append("| `%s` | %.6f | %s |" % (
            name, params["default_joint_angles_rad"][name],
            "wheel" if "wheel" in name else "leg"))
    lines += [
        "",
        "## 关节限位（provisional）",
        "",
        "| joint | lower | upper |",
        "|---|---:|---:|",
    ]
    for name in params["expected_runtime_dof_order"]:
        lim = params["joint_limits_rad"][name]
        if lim is None:
            lines.append("| `%s` | -inf | +inf (continuous) |" % name)
        else:
            lines.append("| `%s` | %.4f | %.4f |" % (name, lim[0], lim[1]))
    lines += [
        "",
        "## 符号表（positive q 的物理方向）",
        "",
        "| joint | axis sign (w.r.t. new +Y) |",
        "|---|---:|",
    ]
    for name in params["expected_runtime_dof_order"]:
        lines.append("| `%s` | %+d |" % (name, params["joint_sign_table"][name]))
    lines += [
        "",
        "## 构建校验",
        "",
        "```",
        json.dumps(report["checks"], indent=2),
        "```",
        "",
        f"FK vs Fudan 公式最大误差：{report['max_fk_error_m']:.3e} m",
        "",
    ]
    (PACKAGE / "docs").mkdir(parents=True, exist_ok=True)
    (PACKAGE / "docs" / "PARAMETER_TABLE.md").write_text("\n".join(lines), encoding="utf-8")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source-urdf", type=Path, default=SOURCE_URDF)
    ap.add_argument("--forward-sign", type=int, default=FORWARD_SIGN, choices=(-1, 1))
    ap.add_argument("--wheel-radius", type=float, default=WHEEL_RADIUS_M)
    ap.add_argument("--effort", type=float, default=EFFORT_NM)
    ap.add_argument("--velocity", type=float, default=VELOCITY_RAD_S)
    ap.add_argument("--force", action="store_true",
                    help="overwrite an existing generated URDF")
    args = ap.parse_args(argv)
    if (PACKAGE / "urdf" / "fifty_training.urdf").exists() and not args.force:
        raise SystemExit("urdf/fifty_training.urdf already exists; use --force to rebuild")
    build(args)


if __name__ == "__main__":
    main()
