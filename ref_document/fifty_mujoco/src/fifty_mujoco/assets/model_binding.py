"""Read the passive-joint values a model needs at reset, without MuJoCo.

Why this exists
---------------
The closed-chain model has eight hinges the policy does not drive
(``r_knee_Joint`` and friends).  If a reset leaves them at zero the two loops do
not close: at the reference pose the start would be 0.24 m / 0.44 m out and the
equality solver would have to snap it shut.  The values are already in the model
as the ``<keyframe name="reference">`` that ``build_closed_mjcf`` writes, so they
are read from there instead of being retyped.

The parser is standard-library only and returns ``qpos`` addresses computed the
way MuJoCo lays them out: a free joint owns seven slots and a hinge owns one,
with joints ordered by depth-first body traversal and, inside a body, by
declaration order.  ``tests/test_model_binding.py`` checks that against a real
``MjModel`` so this assumption cannot rot.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable, Mapping

#: qpos width of the joint types this project uses.
_QPOS_WIDTH = {"free": 7, "ball": 4, "hinge": 1, "slide": 1}


def joint_qpos_addresses(xml_path: str | Path) -> dict[str, int]:
    """``joint name -> qpos address`` in MuJoCo's ordering."""
    root = ET.parse(Path(xml_path)).getroot()
    world = root.find("worldbody")
    if world is None:
        return {}
    addresses: dict[str, int] = {}
    address = 0

    def walk(body: ET.Element) -> None:
        nonlocal address
        for child in list(body):
            if child.tag == "freejoint":
                address += _QPOS_WIDTH["free"]
            elif child.tag == "joint":
                kind = child.get("type", "hinge")
                name = child.get("name")
                if name:
                    addresses[name] = address
                address += _QPOS_WIDTH.get(kind, 1)
            elif child.tag == "body":
                walk(child)

    walk(world)
    return addresses


def keyframe_values(
    xml_path: str | Path, name: str = "reference"
) -> list[float] | None:
    """``qpos`` stored in the keyframe called ``name``, or ``None``."""
    root = ET.parse(Path(xml_path)).getroot()
    keyframe = root.find("keyframe")
    if keyframe is None:
        return None
    for key in keyframe.findall("key"):
        if key.get("name") == name:
            raw = key.get("qpos")
            if raw is None:
                return None
            return [float(value) for value in raw.replace(",", " ").split()]
    return None


def passive_joint_values(
    xml_path: str | Path,
    policy_joints: Iterable[str],
    keyframe: str = "reference",
) -> dict[str, float]:
    """Values for every hinge the policy does not drive, from the keyframe.

    Returns an empty mapping when the model has no such keyframe - which is the
    serial model's case, and must stay harmless rather than raising.
    """
    values = keyframe_values(xml_path, keyframe)
    if values is None:
        return {}
    addresses = joint_qpos_addresses(xml_path)
    driven = set(policy_joints)
    result: dict[str, float] = {}
    for joint, address in addresses.items():
        if joint in driven:
            continue
        if 0 <= address < len(values):
            result[joint] = float(values[address])
    return result


def spawn_qpos(
    xml_path: str | Path,
    root_position: Iterable[float],
    root_quat_wxyz: Iterable[float],
    joint_angles: Mapping[str, float],
    passive: Mapping[str, float] | None = None,
) -> list[float]:
    """Full ``qpos`` for a spawn pose, including the passive joints.

    The root occupies the first seven slots, so the caller supplies that pose and
    every hinge value by name; anything not named stays zero.
    """
    addresses = joint_qpos_addresses(xml_path)
    values = [0.0] * (max(addresses.values(), default=-1) + 1)
    if not values:
        return values
    position = list(root_position)
    quaternion = list(root_quat_wxyz)
    values[0:3] = position[:3]
    values[3:7] = quaternion[:4]
    for joint, angle in joint_angles.items():
        if joint in addresses:
            values[addresses[joint]] = float(angle)
    for joint, angle in (passive or {}).items():
        if joint in addresses:
            values[addresses[joint]] = float(angle)
    return values
