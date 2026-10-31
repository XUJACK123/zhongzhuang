"""Configuration loading with explicit provenance checks."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .paths import assets_dir, configs_dir, workspace_root


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
    models: dict[str, Any] = field(default_factory=dict)
    joint_conventions: dict[str, Any] = field(default_factory=dict)

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
        """Default hinge values for the *selected* model.

        The serial model keeps ``robot.yaml: joints.default_angles_rad`` (the old
        operating pose).  The closed model must start at the reference pose in
        its own coordinates, which live in ``joint_conventions.yaml:
        reference_pose.closed_qpos``; feeding it the serial numbers would leave
        both loops open by more than 3 rad.
        """
        source = str((self.model.get("policy_pose") or {}).get("source", "robot"))
        if source == "joint_conventions.closed_qpos":
            return self._closed_default_angles()
        angles = self.robot["joints"]["default_angles_rad"]
        return {name: float(angles[name]) for name in self.joint_order}

    def _closed_default_angles(self) -> dict[str, float]:
        block = ((self.joint_conventions or {}).get("reference_pose") or {}).get(
            "closed_qpos"
        ) or {}
        right = block.get("right") or {}
        multiplier = float(block.get("left_multiplier", -1.0))
        result: dict[str, float] = {}
        for name in self.joint_order:
            if name in right:
                result[name] = float(right[name])
            elif name.startswith("l") and f"r{name[1:]}" in right:
                result[name] = multiplier * float(right[f"r{name[1:]}"])
            else:
                raise ValueError(
                    f"model {self.model_name!r} has no default value for joint {name!r}"
                )
        return result

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

    # -------------------------------------------------------- control period
    @property
    def policy_dt(self) -> float:
        """Simulated seconds per policy step."""
        return float(self.robot["control"]["policy_dt_s"])

    def decimation_for(self, model_timestep: float) -> int:
        """Physics steps per policy step for *the compiled model's* timestep.

        Derived from the model rather than from ``simulation.timestep_s``: the
        serial XML steps at 5 ms and the closed-loop XML at 2.5 ms, so using
        the config constant silently halved the closed model's control period
        to 5 ms (found by the first GPU parity run, which compared 2 physics
        substeps against the CPU env's 4).
        """
        timestep = float(model_timestep)
        steps = int(round(self.policy_dt / timestep))
        if steps < 1:
            raise ValueError(
                f"policy_dt_s {self.policy_dt} is shorter than the model timestep "
                f"{timestep}; the model cannot be stepped that finely"
            )
        realised = steps * timestep
        if abs(realised - self.policy_dt) > 1e-9:
            raise ValueError(
                f"model timestep {timestep} does not divide policy_dt_s "
                f"{self.policy_dt} (nearest: {steps} steps = {realised})"
            )
        return steps

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

    # ------------------------------------------------------------ model choice
    @property
    def model_name(self) -> str:
        """Selected model: ``FIFTY_MODEL`` if set, else ``models.yaml: meta.default``."""
        default = str((self.models.get("meta") or {}).get("default", "serial"))
        return os.environ.get("FIFTY_MODEL", default).strip() or default

    @property
    def model(self) -> dict[str, Any]:
        table = self.models.get("models") or {}
        if self.model_name not in table:
            raise ValueError(
                f"unknown model {self.model_name!r}; known: {', '.join(sorted(table)) or '<none>'}"
            )
        return dict(table[self.model_name])

    @property
    def model_status(self) -> str:
        return str(self.model.get("status", "unknown"))

    def model_xml(self) -> Path:
        """The MJCF this model selection resolves to (``assets/mjcf/<xml>``)."""
        name = str(self.model["xml"])
        path = Path(name)
        return path if path.is_absolute() else assets_dir() / "mjcf" / path

    def body_name(self, semantic: str) -> str:
        """Model-specific body name for a semantic key such as ``thigh_right``."""
        bodies = self.model.get("bodies") or {}
        if semantic not in bodies:
            raise ValueError(
                f"model {self.model_name!r} has no body mapping for {semantic!r}"
            )
        return str(bodies[semantic])

    @property
    def passive_joints(self) -> list[str]:
        return [str(name) for name in (self.model.get("passive_joints") or [])]

    @property
    def uses_keyframe_reset(self) -> bool:
        return str(self.model.get("reset", "default_angles")) == "keyframe"

    @property
    def keyframe_name(self) -> str:
        return str(self.model.get("keyframe", "reference"))

    def tendon_name(self, spring_name: str) -> str:
        """Tendon that carries ``spring_name`` in this model (suffix differs)."""
        return f"{spring_name}{self.model.get('tendon_suffix', '_length')}"

    @property
    def model_expected(self) -> dict[str, Any]:
        return dict(self.model.get("expected") or {})


def load_configs() -> Config:
    directory = configs_dir()
    models_path = directory / "models.yaml"
    conventions_path = directory / "joint_conventions.yaml"
    return Config(
        robot=load_yaml(directory / "robot.yaml"),
        spawn=load_yaml(directory / "spawn.yaml"),
        gas_spring=load_yaml(directory / "gas_spring.yaml"),
        policy_contract=load_yaml(directory / "policy_contract.yaml"),
        models=load_yaml(models_path) if models_path.is_file() else {},
        joint_conventions=(
            load_yaml(conventions_path) if conventions_path.is_file() else {}
        ),
    )
