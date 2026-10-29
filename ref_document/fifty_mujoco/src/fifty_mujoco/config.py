"""Configuration loading with explicit provenance checks."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .paths import configs_dir, workspace_root


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a mapping at the top level")
    return data


@dataclass(frozen=True)
class Config:
    robot: dict[str, Any]
    spawn: dict[str, Any]
    gas_spring: dict[str, Any]
    policy_contract: dict[str, Any]

    # ------------------------------------------------------------------ paths
    def workspace_path(self, relative: str) -> Path:
        path = Path(relative)
        return path if path.is_absolute() else workspace_root() / path

    @property
    def source_urdf(self) -> Path:
        return self.workspace_path(self.robot["robot"]["source_urdf"])

    @property
    def training_urdf(self) -> Path:
        return self.workspace_path(self.robot["robot"]["training_urdf"])

    @property
    def parameters_json(self) -> Path:
        return self.workspace_path(self.robot["robot"]["parameters_json"])

    @property
    def source_mesh_dir(self) -> Path:
        return self.workspace_path(self.robot["robot"]["source_package"]) / "meshes"

    # ----------------------------------------------------------------- joints
    @property
    def joint_order(self) -> list[str]:
        return list(self.robot["joints"]["order"])

    @property
    def default_angles(self) -> dict[str, float]:
        angles = self.robot["joints"]["default_angles_rad"]
        return {name: float(angles[name]) for name in self.joint_order}

    @property
    def joint_limits(self) -> dict[str, tuple[float, float] | None]:
        limits = self.robot["joints"]["limits_rad"]
        result: dict[str, tuple[float, float] | None] = {}
        for name in self.joint_order:
            value = limits.get(name)
            result[name] = None if value is None else (float(value[0]), float(value[1]))
        return result

    def joint_group(self, joint_name: str) -> str:
        return str(self.robot["joints"]["group"][joint_name])

    def damping(self, joint_name: str) -> float:
        return float(self.robot["joints"]["damping"][self.joint_group(joint_name)])

    def effort_limit(self, joint_name: str) -> float:
        entry = self.robot["joints"]["effort_nm"]
        if isinstance(entry, dict):
            return float(entry[self.joint_group(joint_name)])
        return float(entry)

    def frictionloss(self, joint_name: str) -> float:
        return float(self.robot["joints"]["frictionloss"][self.joint_group(joint_name)])

    def armature(self, joint_name: str) -> float:
        return float(self.robot["joints"]["armature"][self.joint_group(joint_name)])

    # ------------------------------------------------------------------- misc
    @property
    def base_rotation(self) -> np.ndarray:
        return np.array(self.robot["base"]["source_rotation"], dtype=float)

    @property
    def base_origin(self) -> np.ndarray:
        """Origin of the training base frame *expressed in source coordinates*."""
        return np.array(self.robot["base"]["source_origin_m"], dtype=float)

    @property
    def base_transform(self):
        """``T_train<-source``: ``p_train = R @ (p_source - p_origin)``.

        The recorded origin is the source-frame position of the new base
        origin (hip-axis midpoint), so the translation is ``-R @ origin``.
        Using ``+origin`` silently shifts every link by 2x that vector, which
        is exactly the class of error this migration is meant to remove.
        """
        from .math3d import Transform

        rotation = self.base_rotation
        return Transform(rotation, -(rotation @ self.base_origin))

    def mesh_correction(self, link_name: str):
        """Extra rotation applied to one link's mesh/inertia, or ``None``.

        Used for ``base_link`` because the recorded source->training rotation
        carries an extra 180 deg roll that belongs to the legs, not to the
        chassis (see the note in ``configs/robot.yaml``).
        """
        from .math3d import Transform

        table = self.robot.get("base", {}).get("mesh_correction") or {}
        entry = table.get(link_name)
        if not isinstance(entry, list):
            return None
        rotation = np.array(entry, dtype=float)
        return Transform(rotation, np.zeros(3))

    @property
    def wheel_radius(self) -> float:
        return float(self.robot["geometry"]["wheel_radius_m"])

    @property
    def gas_spring_springs(self) -> list[dict[str, Any]]:
        return list(self.gas_springs_raw["springs"])

    @property
    def gas_springs_raw(self) -> dict[str, Any]:
        return self.gas_spring["gas_springs"]

    @property
    def gas_spring_configured(self) -> bool:
        """True when every spring has both anchors and a force model.

        Stroke limits are tracked separately: a spring can be physically
        modelled without verified stroke, but then the gate stays unpassed.
        """
        springs = self.gas_spring_springs
        if not springs:
            return False
        for spring in springs:
            for key in ("anchor_a", "anchor_b"):
                anchor = spring.get(key) or {}
                if not anchor.get("body") or anchor.get("xyz_m") is None:
                    return False
            if spring.get("force_ref_n") is None:
                return False
        return True

    @property
    def gas_spring_stroke_verified(self) -> bool:
        if not self.gas_spring_configured:
            return False
        return all(
            spring.get("length_min_m") is not None and spring.get("length_max_m") is not None
            for spring in self.gas_spring_springs
        )

    @property
    def gas_spring_profile(self) -> str:
        return str(self.gas_springs_raw.get("profile", "unspecified"))

    @property
    def gas_spring_force_sign_convention(self) -> str:
        return str(self.gas_springs_raw.get("force_sign_convention", "push_apart"))

    @property
    def spring_count(self) -> int:
        return len(self.gas_spring_springs) if self.gas_spring_configured else 0


def load_configs() -> Config:
    directory = configs_dir()
    return Config(
        robot=load_yaml(directory / "robot.yaml"),
        spawn=load_yaml(directory / "spawn.yaml"),
        gas_spring=load_yaml(directory / "gas_spring.yaml"),
        policy_contract=load_yaml(directory / "policy_contract.yaml"),
    )
