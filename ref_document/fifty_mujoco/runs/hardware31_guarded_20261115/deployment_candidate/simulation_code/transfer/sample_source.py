"""Isolated sample-source MJCF and archived control helpers.

This is a diagnostic reconstruction, not PhysX dynamics/timing certification.
It never selects or modifies the project's legacy robot/model configuration.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from ..assets.urdf_io import load_urdf
from ..contracts.sample_actor31 import JOINT_ORDER
from ..math3d import mat_to_quat_wxyz, rpy_to_mat
from ..paths import workspace_root


def verified_artifact(name):
    archive = workspace_root() / "sample_weight_file"
    expected = json.loads((archive / "manifest.json").read_text())["files"][f"artifacts/{name}"]
    path = archive / "artifacts" / name
    raw = path.read_bytes()
    if len(raw) != expected["bytes"] or hashlib.sha256(raw).hexdigest() != expected["sha256"]:
        raise ValueError(f"source artifact must match raw frozen bytes: {name}")
    return path


def source_module(name):
    path = verified_artifact(name + ".py")
    spec = importlib.util.spec_from_file_location("sample_archived_" + name, path)
    module = importlib.util.module_from_spec(spec)
    # Execute the verified text without creating __pycache__ in the frozen archive.
    exec(compile(path.read_bytes(), str(path), "exec"), module.__dict__)
    return module


def vec(values):
    return " ".join(repr(float(v)) for v in values)


def source_xml(effective_friction=.75):
    if not np.isfinite(effective_friction) or effective_friction <= 0:
        raise ValueError("positive finite effective friction required")
    robot = load_urdf(verified_artifact("fifty_training.urdf"))
    env = json.loads(verified_artifact("env_cfg.json").read_text())
    root = ET.Element("mujoco", model="sample_source_diagnostic")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true", balanceinertia="false")
    ET.SubElement(root, "option", timestep=str(env["sim"]["dt"] / env["sim"]["substeps"]),
                  gravity=vec(env["sim"]["gravity"]), integrator="implicitfast",
                  cone="elliptic", iterations="80", tolerance="1e-10")
    defaults = ET.SubElement(root, "default")
    ET.SubElement(defaults, "joint", damping="0", armature="0", frictionloss="0")
    ET.SubElement(defaults, "geom", friction=f"{effective_friction} 0 0", condim="3",
                  solref="0.01 1", solimp="0.9 0.95 0.001")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "geom", name="floor", type="plane", size="10 10 .1")
    children = robot.child_map()

    def body(parent, name, joint=None):
        attributes = {"name": name}
        if joint is not None:
            attributes.update(pos=vec(joint.xyz), quat=vec(mat_to_quat_wxyz(rpy_to_mat(joint.rpy))))
        else:
            attributes["pos"] = vec(env["init_state"]["pos"])
        element = ET.SubElement(parent, "body", attributes)
        link = robot.links[name]
        inertia = link.inertial
        matrix = inertia.inertia_link_frame
        ET.SubElement(element, "inertial", pos=vec(inertia.com), mass=str(inertia.mass),
                      fullinertia=vec([matrix[0, 0], matrix[1, 1], matrix[2, 2],
                                       matrix[0, 1], matrix[0, 2], matrix[1, 2]]))
        if joint is None:
            ET.SubElement(element, "freejoint", name="root")
        else:
            attrs = dict(name=joint.name, axis=vec(joint.axis), type="hinge")
            if joint.lower is not None:
                attrs["range"] = vec([joint.lower, joint.upper])
            else:
                attrs["limited"] = "false"
            ET.SubElement(element, "joint", attrs)
        for i, shape in enumerate(link.collisions):
            geometry = shape.geometry
            size = geometry.size.copy()
            if geometry.kind == "box":
                size /= 2
            elif geometry.kind == "cylinder":
                size[1] /= 2
            else:
                raise ValueError(f"unsupported source collision: {geometry.kind}")
            ET.SubElement(element, "geom", name=f"{name}_collision_{i}", type=geometry.kind,
                          size=vec(size), pos=vec(shape.xyz),
                          quat=vec(mat_to_quat_wxyz(rpy_to_mat(shape.rpy))))
        for child in children[name]:
            body(element, child.child, child)

    body(world, robot.root_link)
    contact = ET.SubElement(root, "contact")
    for joint in robot.joints.values():
        ET.SubElement(contact, "exclude", body1=joint.parent, body2=joint.child)
    return ET.tostring(root, encoding="unicode"), env


class SampleSource:
    def __init__(self, effective_friction=.75):
        self.xml, self.env = source_xml(effective_friction)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.joint_ids = np.array([self.model.joint(name).id for name in JOINT_ORDER])
        self.qpos_ids = self.model.jnt_qposadr[self.joint_ids]
        self.qvel_ids = self.model.jnt_dofadr[self.joint_ids]
        self.defaults = np.array([self.env["init_state"]["default_joint_angles"][n] for n in JOINT_ORDER])
        self.actuator_module = source_module("actuator_model")
        self.hardware = self.actuator_module.load_hardware(verified_artifact("actuator_hardware.json"))
        self.spring_module = source_module("gas_spring")
        self.spring_config = self.spring_module.load_config(
            verified_artifact("gas_spring_resolved.json"), verified_artifact("fifty_training.urdf"))
        self.physics_substeps = self.env["sim"]["substeps"]
        self.external_dt = self.model.opt.timestep * self.physics_substeps
        self.policy_dt = self.external_dt * self.env["control"]["decimation"]
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:3] = self.env["init_state"]["pos"]
        xyzw = self.env["init_state"]["rot"]
        self.data.qpos[3:7] = [xyzw[3], *xyzw[:3]]
        self.data.qpos[self.qpos_ids] = self.defaults
        self.data.qvel[:] = 0
        self.data.qfrc_applied[:] = 0
        mujoco.mj_forward(self.model, self.data)

    def spring_torque(self, q, qd):
        result = np.zeros(6)
        lengths = []
        for spring in self.spring_config["springs"]:
            index = JOINT_ORDER.index(spring["joint"])
            torque, length, _ = self.spring_module.evaluate(float(q[index]), float(qd[index]), spring)
            result[index] += torque
            lengths.append(length)
        return result, lengths

    def contacts(self):
        return [{"geom1": self.model.geom(c.geom1).name, "geom2": self.model.geom(c.geom2).name,
                 "distance_m": float(c.dist)} for c in self.data.contact]
