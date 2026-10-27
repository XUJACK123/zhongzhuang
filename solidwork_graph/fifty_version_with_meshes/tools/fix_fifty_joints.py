"""Audit + repair tool for solidwork_graph/fifty_version_with_meshes.

Turns the SolidWorks export (six `fixed` joints, zero axes) into an articulated
model shaped like the fudan / fourth_version references:

* l0/l1/r0/r1 leg joints  -> revolute about the joint-local z axis
* left_wheel/right_wheel  -> continuous about the joint-local z axis
* joint names aligned with the fudan training convention
* controller_joint_names, URDF CSV, display.launch and RViz config synced
* manifest, mesh hashes and kinematic checks written to a JSON report

Link inertial/visual/collision blocks, joint origins, mesh files and the q=0
assembly pose are left untouched.  Run once on the pristine export; the script
refuses to re-run on a model that already contains moving joints.
"""

import csv
import datetime
import hashlib
import json
import math
import os
import shutil
import struct
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG_NAME = "fifty_version_with_meshes"

URDF_DIR = os.path.join(PKG, "urdf")
URDF_NAME = PKG_NAME + ".urdf"
CSV_NAME = PKG_NAME + ".csv"
YAML_NAME = "joint_names_" + PKG_NAME + ".yaml"

# joint name in the export -> (fudan-style name, type, axis, lower, upper)
JOINT_PLAN = [
    ("l0_joint", "lf0_Joint", "revolute", "0 0 1", -100.0, 100.0),
    ("l1_joint", "lf1_Joint", "revolute", "0 0 1", -100.0, 100.0),
    ("left_wheel", "l_wheel_Joint", "continuous", "0 0 1", None, None),
    ("r0_joint", "rf0_Joint", "revolute", "0 0 1", -100.0, 100.0),
    ("ri_joint", "rf1_Joint", "revolute", "0 0 1", -100.0, 100.0),
    ("right_wheel", "r_wheel_Joint", "continuous", "0 0 1", None, None),
]
EFFORT = "30"
VELOCITY = "1000"
# controller order used by the fudan training model (right side first)
CONTROLLER_ORDER = [
    "rf0_Joint",
    "rf1_Joint",
    "r_wheel_Joint",
    "lf0_Joint",
    "lf1_Joint",
    "l_wheel_Joint",
]


# --------------------------------------------------------------------------- #
# small math helpers (3x3 only, no numpy)
# --------------------------------------------------------------------------- #
def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def rpy_to_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    return matmul(rz, matmul(ry, rx))


def axis_angle_to_matrix(axis, angle):
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def norm(a):
    return math.sqrt(sum(c * c for c in a))


def unit(a):
    n = norm(a)
    return [c / n for c in a]


def eigen_sym(m):
    a = [row[:] for row in m]
    for _ in range(100):
        if max(abs(a[0][1]), abs(a[0][2]), abs(a[1][2])) < 1e-18:
            break
        for p, q in ((0, 1), (0, 2), (1, 2)):
            if abs(a[p][q]) < 1e-20:
                continue
            theta = (a[q][q] - a[p][p]) / (2 * a[p][q])
            t = (1 if theta >= 0 else -1) / (abs(theta) + math.sqrt(theta * theta + 1))
            c = 1 / math.sqrt(t * t + 1)
            s = t * c
            for k in range(3):
                akp, akq = a[k][p], a[k][q]
                a[k][p] = c * akp - s * akq
                a[k][q] = s * akp + c * akq
            for k in range(3):
                apk, aqk = a[p][k], a[q][k]
                a[p][k] = c * apk - s * aqk
                a[q][k] = s * apk + c * aqk
    return sorted([a[i][i] for i in range(3)], reverse=True)


# --------------------------------------------------------------------------- #
# STL helpers
# --------------------------------------------------------------------------- #
def stl_stats(path, collect_vertices=False, face_groups=False, max_vertices=20000):
    with open(path, "rb") as fh:
        data = fh.read()
    sha = hashlib.sha256(data).hexdigest()
    if len(data) < 84:
        raise ValueError("truncated STL: %s" % path)
    tris = struct.unpack_from("<I", data, 80)[0]
    if 84 + tris * 50 != len(data):
        raise ValueError("not a valid binary STL: %s" % path)
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    groups = defaultdict(float)
    verts = []
    stride = max(1, tris // max_vertices) if collect_vertices else 0
    off = 84
    for i in range(tris):
        values = struct.unpack_from("<12f", data, off)
        n, v1, v2, v3 = values[0:3], values[3:6], values[6:9], values[9:12]
        off += 50
        for v in (v1, v2, v3):
            for k in range(3):
                if not math.isfinite(v[k]):
                    raise ValueError("non-finite vertex in %s" % path)
                if v[k] < lo[k]:
                    lo[k] = v[k]
                if v[k] > hi[k]:
                    hi[k] = v[k]
        if face_groups:
            area = 0.5 * norm(cross(sub(v2, v1), sub(v3, v1)))
            if area > 0:
                u = unit(n)
                key = tuple(round(c, 2) for c in u)
                if key > (0.0, 0.0, 0.0):
                    key = tuple(-c for c in key)
                groups[key] += area
        if collect_vertices and i % stride == 0:
            verts.extend((v1, v2, v3))
    return {
        "sha256": sha,
        "triangles": tris,
        "bbox": [lo, hi],
        "vertices": verts,
        "face_groups": sorted(groups.items(), key=lambda kv: -kv[1])[:5],
    }


# --------------------------------------------------------------------------- #
# URDF parsing / writing
# --------------------------------------------------------------------------- #
def parse_model(urdf_text):
    root = ET.fromstring(urdf_text)
    links = {l.get("name"): l for l in root.findall("link")}
    joints = []
    for j in root.findall("joint"):
        o = j.find("origin")
        a = j.find("axis")
        lim = j.find("limit")
        joints.append(
            {
                "name": j.get("name"),
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "xyz": [float(v) for v in o.get("xyz", "0 0 0").split()],
                "rpy": [float(v) for v in o.get("rpy", "0 0 0").split()],
                "axis": [float(v) for v in a.get("xyz").split()] if a is not None else [0, 0, 0],
                "limit": dict(lim.attrib) if lim is not None else None,
            }
        )
    return root, links, joints


def mesh_path(link):
    node = link.find("visual/geometry/mesh")
    if node is None:
        node = link.find("collision/geometry/mesh")
    name = node.get("filename").split("meshes/")[-1]
    return os.path.join(PKG, "meshes", name), name


def world_frame(joints):
    """Map link -> (R, t) in the base link frame at q = 0."""
    children = {j["child"] for j in joints}
    parents = {j["parent"] for j in joints}
    frames = {}

    def walk(link, R, t):
        frames[link] = (R, t)
        for j in joints:
            if j["parent"] == link:
                Rj = rpy_to_matrix(*j["rpy"])
                walk(j["child"], matmul(R, Rj), [t[i] + matvec(R, j["xyz"])[i] for i in range(3)])

    for r in parents - children:
        walk(r, [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0])
    return frames


def joint_chain(joints, link):
    """Return the joints from the root down to `link`."""
    by_child = {j["child"]: j for j in joints}
    chain = []
    while link in by_child:
        j = by_child[link]
        chain.append(j)
        link = j["parent"]
    return list(reversed(chain))


def fk(joints, link, q):
    """World transform of `link` for joint values q (dict name -> angle)."""
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    t = [0.0, 0.0, 0.0]
    for j in joint_chain(joints, link):
        t = [t[i] + matvec(R, j["xyz"])[i] for i in range(3)]
        Rj = rpy_to_matrix(*j["rpy"])
        if q.get(j["name"]):
            Rj = matmul(Rj, axis_angle_to_matrix(unit(j["axis"]), q[j["name"]]))
        R = matmul(R, Rj)
    return R, t


def limit_block(kind, lower, upper):
    if kind == "continuous":
        return '<limit\n      effort="%s"\n      velocity="%s" />' % (EFFORT, VELOCITY)
    return '<limit\n      lower="%g"\n      upper="%g"\n      effort="%s"\n      velocity="%s" />' % (
        lower,
        upper,
        EFFORT,
        VELOCITY,
    )


def rewrite_urdf(text):
    out = text
    for old, new, kind, axis, lower, upper in JOINT_PLAN:
        marker = '<joint\n    name="%s"\n    type="fixed">' % old
        if marker not in out:
            raise SystemExit("joint block not found for %s" % old)
        start = out.index(marker)
        end = out.index("</joint>", start)
        block = out[start:end]
        block = block.replace(marker, '<joint\n    name="%s"\n    type="%s">' % (new, kind))
        if '<axis\n      xyz="0 0 0" />' not in block:
            raise SystemExit("unexpected axis block for %s" % old)
        block = block.replace(
            '<axis\n      xyz="0 0 0" />',
            '<axis\n      xyz="%s" />\n    %s' % (axis, limit_block(kind, lower, upper)),
        )
        out = out[:start] + block + out[end:]
    return out


# --------------------------------------------------------------------------- #
# CSV / YAML / launch / rviz
# --------------------------------------------------------------------------- #
def rewrite_csv(path, backup_path):
    with open(backup_path, newline="", encoding="utf-8") as fh:
        before = list(csv.reader(fh))
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.reader(fh))
    header = rows[0]
    idx = {name: i for i, name in enumerate(header)}

    def put(row, column, value):
        row[idx[column]] = value

    plan = {old: (new, kind, axis, lower, upper) for old, new, kind, axis, lower, upper in JOINT_PLAN}
    for row in rows[1:]:
        joint = row[idx["Joint Name"]]
        if joint not in plan:
            continue
        new, kind, axis, lower, upper = plan[joint]
        ax, ay, az = axis.split()
        put(row, "Joint Name", new)
        put(row, "Joint Type", kind)
        put(row, "Joint Axis X", ax)
        put(row, "Joint Axis Y", ay)
        put(row, "Joint Axis Z", az)
        put(row, "Limit Effort", EFFORT)
        put(row, "Limit Velocity", VELOCITY)
        put(row, "Limit Lower", "" if lower is None else "%g" % lower)
        put(row, "Limit Upper", "" if upper is None else "%g" % upper)

    with open(path, "w", newline="", encoding="utf-8") as fh:
        csv.writer(fh, lineterminator="\r\n").writerows(rows)

    # nothing but the six planned rows may differ
    changed = []
    for old_row, new_row in zip(before, rows):
        for column, old_value, new_value in zip(header, old_row, new_row):
            if old_value != new_value:
                changed.append((old_row[idx["Joint Name"]] or old_row[0], column, old_value, new_value))
    return changed


def write_yaml(path):
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("controller_joint_names: [" + ", ".join(CONTROLLER_ORDER) + "]\n")


def write_launch(path):
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(
            """<launch>
  <param
    name="robot_description"
    textfile="$(find %(pkg)s)/urdf/%(pkg)s.urdf" />
  <node
    name="joint_state_publisher_gui"
    pkg="joint_state_publisher_gui"
    type="joint_state_publisher_gui" />
  <node
    name="robot_state_publisher"
    pkg="robot_state_publisher"
    type="robot_state_publisher" />
  <node
    name="rviz"
    pkg="rviz"
    type="rviz"
    args="-d $(find %(pkg)s)/config/display.rviz" />
</launch>
"""
            % {"pkg": PKG_NAME}
        )


RVIZ = """Panels:
  - Class: rviz/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Global Options:
    Fixed Frame: base_link
    Background Color: 48; 48; 48
  Displays:
    - Class: rviz/Grid
      Name: Grid
      Enabled: true
      Cell Size: 0.1
    - Class: rviz/RobotModel
      Name: RobotModel
      Enabled: true
      Robot Description: robot_description
      Visual Enabled: true
      Collision Enabled: false
    - Class: rviz/TF
      Name: TF
      Enabled: true
      Show Axes: true
      Show Names: false
  Views:
    Current:
      Class: rviz/Orbit
      Distance: 1.8
      Pitch: 0.5
      Yaw: 0.8
"""


# --------------------------------------------------------------------------- #
def main():
    urdf_path = os.path.join(URDF_DIR, URDF_NAME)
    csv_path = os.path.join(URDF_DIR, CSV_NAME)
    yaml_path = os.path.join(PKG, "config", YAML_NAME)
    launch_path = os.path.join(PKG, "launch", "display.launch")
    rviz_path = os.path.join(PKG, "config", "display.rviz")

    with open(urdf_path, encoding="utf-8", newline="") as fh:
        original_text = fh.read()
    root, links, joints = parse_model(original_text)
    if any(j["type"] != "fixed" for j in joints):
        raise SystemExit("model already has moving joints; aborting to protect the current file")
    names = {j["name"] for j in joints}
    missing = [old for old, *_ in JOINT_PLAN if old not in names]
    if missing:
        raise SystemExit("expected export joints not found: %s" % missing)

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = os.path.join(PKG, "backups", "before_joint_fix_" + stamp)
    for rel in (os.path.join("urdf", URDF_NAME), os.path.join("urdf", CSV_NAME),
                os.path.join("config", YAML_NAME), os.path.join("launch", "display.launch")):
        dst = os.path.join(backup_dir, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(os.path.join(PKG, rel), dst)

    # --- geometry evidence before editing -----------------------------------
    stl = {}
    for link_name, link in links.items():
        path, file_name = mesh_path(link)
        stl[link_name] = dict(file_name=file_name, **stl_stats(path, collect_vertices=True))
    wheel_axes = {}
    for wheel in ("left_wheel", "right_wheel"):
        path, _ = mesh_path(links[wheel])
        groups = stl_stats(path, face_groups=True)["face_groups"]
        wheel_axes[wheel] = {
            "dominant_normal": [round(c, 3) for c in groups[0][0]],
            "area": groups[0][1],
            "groups": [[[round(c, 2) for c in g[0]], g[1]] for g in groups[:3]],
        }

    # --- rewrite -------------------------------------------------------------
    newline = "\r\n" if "\r\n" in original_text else "\n"
    new_text = rewrite_urdf(original_text.replace("\r\n", "\n")).replace("\n", newline)
    with open(urdf_path, "w", encoding="utf-8", newline="") as fh:
        fh.write(new_text)

    csv_changes = rewrite_csv(csv_path, os.path.join(backup_dir, "urdf", CSV_NAME))
    write_yaml(yaml_path)
    write_launch(launch_path)
    os.makedirs(os.path.dirname(rviz_path), exist_ok=True)
    with open(rviz_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(RVIZ)

    # --- validation ----------------------------------------------------------
    with open(urdf_path, encoding="utf-8") as fh:
        new_root, new_links, new_joints = parse_model(fh.read())
    problems = []
    if len(new_links) != len(links):
        problems.append("link count changed")
    if len(new_joints) != len(joints):
        problems.append("joint count changed")

    old_by_child = {j["child"]: j for j in joints}
    for j in new_joints:
        old = old_by_child.get(j["child"])
        if old is None:
            problems.append("unknown child link %s" % j["child"])
            continue
        if j["parent"] != old["parent"] or j["xyz"] != old["xyz"] or j["rpy"] != old["rpy"]:
            problems.append("joint frame changed for %s" % j["child"])
        if j["type"] == "fixed":
            problems.append("joint %s still fixed" % j["name"])
        if abs(norm(j["axis"]) - 1.0) > 1e-9:
            problems.append("axis not unit length for %s" % j["name"])

    for name, link in new_links.items():
        if ET.tostring(link) != ET.tostring(links[name]):
            problems.append("link definition changed: %s" % name)

    children = {j["child"] for j in new_joints}
    roots = {j["parent"] for j in new_joints} - children
    if len(roots) != 1:
        problems.append("unexpected root set %s" % roots)

    total_mass = 0.0
    for name, link in new_links.items():
        inert = link.find("inertial")
        mass = float(inert.find("mass").get("value"))
        total_mass += mass
        i = inert.find("inertia")
        I = [[float(i.get("ixx")), float(i.get("ixy")), float(i.get("ixz"))],
             [float(i.get("ixy")), float(i.get("iyy")), float(i.get("iyz"))],
             [float(i.get("ixz")), float(i.get("iyz")), float(i.get("izz"))]]
        ev = eigen_sym(I)
        if min(ev) <= 0:
            problems.append("inertia not positive definite: %s" % name)
        if ev[0] > ev[1] + ev[2] + 1e-12:
            problems.append("inertia violates triangle inequality: %s" % name)

    # joint motion: rotate 0.2 rad and require a vertex of the child mesh to move
    motions = {}
    for j in new_joints:
        if j["type"] == "fixed":
            continue
        verts = stl[j["child"]]["vertices"]
        R0, t0 = fk(new_joints, j["child"], {})
        R1, t1 = fk(new_joints, j["child"], {j["name"]: 0.2})
        best = 0.0
        for v in verts:
            p0 = [matvec(R0, v)[i] + t0[i] for i in range(3)]
            p1 = [matvec(R1, v)[i] + t1[i] for i in range(3)]
            best = max(best, norm(sub(p1, p0)))
        motions[j["name"]] = {"angle_rad": 0.2, "max_sample_vertex_motion_m": round(best, 6)}
        if best < 1e-4:
            problems.append("joint %s does not move its child mesh" % j["name"])

    frames = world_frame(new_joints)
    axis_in_base = {}
    for j in new_joints:
        R_par = frames[j["parent"]][0]
        Rj = rpy_to_matrix(*j["rpy"])
        axis_in_base[j["name"]] = [round(c, 6) for c in matvec(R_par, matvec(Rj, unit(j["axis"])))]

    report = {
        "package": PKG_NAME,
        "purpose": "convert SolidWorks fixed joints into the articulated structure used by fudan / fourth_version",
        "corrections": {
            new: {"export_name": old, "type": kind, "axis": axis,
                  "effort": float(EFFORT), "velocity": float(VELOCITY),
                  "lower": lower, "upper": upper}
            for old, new, kind, axis, lower, upper in JOINT_PLAN
        },
        "axis_in_base_frame_at_zero_pose": axis_in_base,
        "links": len(new_links),
        "moving_joints": len([j for j in new_joints if j["type"] != "fixed"]),
        "total_mass_kg": round(total_mass, 9),
        "mesh_files": {
            stl[n]["file_name"]: {
                "sha256": stl[n]["sha256"],
                "triangles": stl[n]["triangles"],
                "bbox_m": [[round(c, 6) for c in stl[n]["bbox"][0]], [round(c, 6) for c in stl[n]["bbox"][1]]],
            }
            for n in stl
        },
        "wheel_face_normal_evidence": wheel_axes,
        "joint_motion_tests": motions,
        "csv_changes": [{"row": r, "column": c, "before": b, "after": a} for r, c, b, a in csv_changes],
        "controller_joint_names": CONTROLLER_ORDER,
        "runtime_tested": False,
        "link_data_and_zero_pose_unchanged": not any(
            "link definition changed" in p or "frame changed" in p for p in problems
        ),
        "backup_directory": backup_dir,
        "problems": problems,
        "post_install_validation_passed": not problems,
    }
    report_path = os.path.join(PKG, "config", "joint_fix_report.json")
    with open(report_path, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(report, fh, indent=2, ensure_ascii=False)
        fh.write("\n")

    print("backup:", backup_dir)
    print("report:", report_path)
    print("total mass: %.9f kg" % total_mass)
    print("joint motion (0.2 rad):", json.dumps(motions, indent=2))
    print("axis in base:", json.dumps(axis_in_base, indent=2))
    print("csv changes:", len(csv_changes))
    print("problems:", problems if problems else "none")
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
