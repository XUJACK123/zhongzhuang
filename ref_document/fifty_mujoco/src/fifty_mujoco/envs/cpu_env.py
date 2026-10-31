"""Single MuJoCo CPU environment: loading, name mapping, reset and stepping.

Phase P0-P2 uses this only for geometry / spawn audits.  Observations,
rewards, history and PD live in later phases (P4/P5) and are deliberately not
implemented here.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from ..config import Config
from ..control.gas_spring import load_springs, spring_force, spring_geometry
from ..assets.model_binding import passive_joint_values

#: Semantic body keys the env addresses, mapped to the *serial* names this code
#: was written against.  ``Config.body_name`` resolves each one to the selected
#: model's own name (``r0_link`` in the serial model, ``r_thigh`` in the closed
#: one), so everything downstream keeps its existing keys.
BODY_KEYS: tuple[tuple[str, str], ...] = (
    ("base_link", "base"),
    ("r0_link", "thigh_right"),
    ("r1_link", "shin_right"),
    ("right_wheel", "wheel_right"),
    ("l0_link", "thigh_left"),
    ("l1_link", "shin_left"),
    ("left_wheel", "wheel_left"),
)

#: Measured on MuJoCo 3.17: a positive tendon-motor command produces
#: ``qfrc_actuator = J_length^T * force``, i.e. it pushes the two anchors APART
#: and does positive work when the tendon lengthens.  This matches the Fudan
#: model (`d.ctrl = 300` for a supporting gas spring) and was verified both by
#: the generalized force and by a 1 s free-base run (positive command extends
#: the knee, negative collapses it).  So no negation is applied.
SPRING_CTRL_SIGN = 1.0


class FiftyCpuEnv:
    """Thin, explicit wrapper around ``mujoco.MjModel`` / ``mujoco.MjData``."""

    def __init__(self, xml_path: str | Path, config: Config):
        self.xml_path = Path(xml_path)
        if not self.xml_path.is_file():
            raise FileNotFoundError(f"MJCF not found: {self.xml_path}")
        self.model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        self.data = mujoco.MjData(self.model)
        self.config = config
        self._build_index()

    # ------------------------------------------------------------------ index
    def _build_index(self) -> None:
        self.joint_ids: dict[str, int] = {}
        self.qpos_adr: dict[str, int] = {}
        self.dof_adr: dict[str, int] = {}
        self.body_ids: dict[str, int] = {}
        self.wheel_geom_ids: dict[str, list[int]] = {}
        self.robot_geom_ids: list[int] = []

        for joint_name in self.config.joint_order:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id < 0:
                raise ValueError(f"joint {joint_name!r} missing from {self.xml_path.name}")
            self.joint_ids[joint_name] = int(joint_id)
            self.qpos_adr[joint_name] = int(self.model.jnt_qposadr[joint_id])
            self.dof_adr[joint_name] = int(self.model.jnt_dofadr[joint_id])

        for link_key, semantic in BODY_KEYS:
            model_name = self.config.body_name(semantic)
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, model_name)
            if body_id < 0:
                raise ValueError(f"body {model_name!r} ({semantic}) missing from {self.xml_path.name}")
            self.body_ids[link_key] = int(body_id)

        wheel_bodies = {
            self.body_ids["right_wheel"]: "right_wheel",
            self.body_ids["left_wheel"]: "left_wheel",
        }
        for geom_id in range(self.model.ngeom):
            body_id = int(self.model.geom_bodyid[geom_id])
            if body_id == 0:
                continue
            self.robot_geom_ids.append(geom_id)
            if body_id in wheel_bodies:
                self.wheel_geom_ids.setdefault(wheel_bodies[body_id], []).append(geom_id)

        # ---------------------------------------------------------- actuators
        self.actuator_ids: dict[str, int] = {}
        for actuator_id in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
            if name:
                self.actuator_ids[name] = actuator_id
        self.joint_motor_ids: dict[str, int] = {}
        for joint_name in self.config.joint_order:
            actuator_name = f"{joint_name}_motor"
            if actuator_name not in self.actuator_ids:
                raise ValueError(f"actuator {actuator_name!r} missing from {self.xml_path.name}")
            self.joint_motor_ids[joint_name] = self.actuator_ids[actuator_name]

        self.springs = load_springs(self.config) if self.config.gas_spring_configured else []
        self.spring_motor_ids: dict[str, int] = {}
        self.tendon_ids: dict[str, int] = {}
        for spring in self.springs:
            actuator_name = f"{spring.name}_force"
            tendon_name = self.config.tendon_name(spring.name)
            if actuator_name not in self.actuator_ids:
                raise ValueError(f"actuator {actuator_name!r} missing from {self.xml_path.name}")
            tendon_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_TENDON, tendon_name)
            if tendon_id < 0:
                raise ValueError(f"tendon {tendon_name!r} missing from {self.xml_path.name}")
            self.spring_motor_ids[spring.name] = self.actuator_ids[actuator_name]
            self.tendon_ids[spring.name] = int(tendon_id)

        # Passive hinges (closed chain) are not driven by the policy; their reset
        # values live in the model keyframe.  Empty for the serial model.
        self.passive_defaults: dict[str, float] = (
            passive_joint_values(
                self.xml_path, self.config.joint_order, self.config.keyframe_name
            )
            if self.config.uses_keyframe_reset
            else {}
        )
        self.passive_qpos_adr: dict[str, int] = {}
        for joint_name in self.passive_defaults:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id >= 0:
                self.passive_qpos_adr[joint_name] = int(self.model.jnt_qposadr[joint_id])

    # ------------------------------------------------------------------- state
    @property
    def root_qpos_adr(self) -> int:
        return 0

    def reset(
        self,
        root_position: np.ndarray,
        joint_angles: dict[str, float] | None = None,
        joint_velocities: dict[str, float] | None = None,
    ) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0:3] = np.asarray(root_position, dtype=float)
        quat = np.asarray(self.config.spawn["spawn"]["root_quat_wxyz"], dtype=float)
        self.data.qpos[3:7] = quat / np.linalg.norm(quat)
        angles = joint_angles if joint_angles is not None else self.config.default_angles
        velocities = joint_velocities or {}
        for joint_name, angle in angles.items():
            if joint_name not in self.qpos_adr:
                continue
            self.data.qpos[self.qpos_adr[joint_name]] = float(angle)
            self.data.qvel[self.dof_adr[joint_name]] = float(velocities.get(joint_name, 0.0))
        for joint_name, value in self.passive_defaults.items():
            address = self.passive_qpos_adr.get(joint_name)
            if address is not None:
                self.data.qpos[address] = float(value)
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def spawn_qpos(self, root_position: Any) -> np.ndarray:
        """Full ``qpos`` for a spawn pose, passive joints included.

        ``WarpVecEnv`` needs the whole vector (it copies it into every world), and
        a zeros vector would leave the closed chain open at every reset.  The root
        height comes from the caller because the spawn audit owns it.
        """
        values = np.zeros(self.model.nq)
        values[0:3] = np.asarray(root_position, dtype=float)
        quat = np.asarray(self.config.spawn["spawn"]["root_quat_wxyz"], dtype=float)
        values[3:7] = quat / np.linalg.norm(quat)
        for joint_name, angle in self.config.default_angles.items():
            if joint_name in self.qpos_adr:
                values[self.qpos_adr[joint_name]] = float(angle)
        for joint_name, value in self.passive_defaults.items():
            address = self.passive_qpos_adr.get(joint_name)
            if address is not None:
                values[address] = float(value)
        return values

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)

    def forward(self) -> None:
        mujoco.mj_forward(self.model, self.data)

    # ---------------------------------------------------------------- control
    def set_joint_motor_torques(self, torques: dict[str, float]) -> None:
        for joint_name, torque in torques.items():
            self.data.ctrl[self.joint_motor_ids[joint_name]] = float(torque)

    def zero_joint_motors(self) -> None:
        for actuator_id in self.joint_motor_ids.values():
            self.data.ctrl[actuator_id] = 0.0

    def spring_state(self) -> dict[str, dict[str, Any]]:
        """Length, rate, force and stroke status per spring at the current state."""
        state: dict[str, dict[str, Any]] = {}
        for spring in self.springs:
            geometry = spring_geometry(self, spring)
            force = spring_force(spring, geometry["length_m"], geometry["length_rate_m_s"])
            tendon_id = self.tendon_ids[spring.name]
            state[spring.name] = {
                "length_m": geometry["length_m"],
                "length_rate_m_s": geometry["length_rate_m_s"],
                "force_n": force,
                "tendon_length_m": float(self.data.ten_length[tendon_id]),
                "tendon_velocity_m_s": float(self.data.ten_velocity[tendon_id]),
                "length_jacobian": geometry["jacobian"].copy(),
                "generalized_force": geometry["jacobian"] * force,
            }
        return state

    def apply_passive_springs(self) -> dict[str, dict[str, Any]]:
        """Write the tendon commands.  Independent of joint-motor enable state."""
        state = self.spring_state()
        for name, entry in state.items():
            self.data.ctrl[self.spring_motor_ids[name]] = SPRING_CTRL_SIGN * entry["force_n"]
        return state

    # ------------------------------------------------------------------ report
    def state(self) -> dict[str, Any]:
        quat = np.asarray(self.data.qpos[3:7], dtype=float)
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, quat / np.linalg.norm(quat))
        return {
            "root_position_m": self.data.qpos[0:3].tolist(),
            "root_quat_wxyz": quat.tolist(),
            "root_quat_norm": float(np.linalg.norm(quat)),
            "root_rotation": rotation.reshape(3, 3).tolist(),
            "joint_positions_rad": {
                name: float(self.data.qpos[adr]) for name, adr in self.qpos_adr.items()
            },
            "joint_velocities_rad_s": {
                name: float(self.data.qvel[adr]) for name, adr in self.dof_adr.items()
            },
            "ncon": int(self.data.ncon),
            "time_s": float(self.data.time),
        }

    def model_summary(self) -> dict[str, Any]:
        return {
            "xml_path": str(self.xml_path.resolve()),
            "xml_sha256": _sha256(self.xml_path),
            "nq": int(self.model.nq),
            "nv": int(self.model.nv),
            "nu": int(self.model.nu),
            "nbody": int(self.model.nbody),
            "ngeom": int(self.model.ngeom),
            "ntendon": int(self.model.ntendon),
            "nmesh": int(self.model.nmesh),
            "timestep_s": float(self.model.opt.timestep),
            "total_mass_kg": float(sum(self.model.body_mass[1:])),
            "joints": [
                {
                    "name": name,
                    "qpos_adr": self.qpos_adr[name],
                    "dof_adr": self.dof_adr[name],
                    "type": int(self.model.jnt_type[self.joint_ids[name]]),
                    "limited": bool(self.model.jnt_limited[self.joint_ids[name]]),
                    "range": self.model.jnt_range[self.joint_ids[name]].tolist(),
                    "axis": self.model.jnt_axis[self.joint_ids[name]].tolist(),
                    "damping": float(self.model.dof_damping[self.dof_adr[name]]),
                    "frictionloss": float(self.model.dof_frictionloss[self.dof_adr[name]]),
                    "armature": float(self.model.dof_armature[self.dof_adr[name]]),
                }
                for name in self.config.joint_order
            ],
            "bodies": [
                {
                    "name": name,
                    "mass_kg": float(self.model.body_mass[body_id]),
                    "ipos_m": self.model.body_ipos[body_id].tolist(),
                    "inertia_kgm2": self.model.body_inertia[body_id].tolist(),
                }
                for name, body_id in self.body_ids.items()
            ],
            "actuators": [
                {"name": name, "id": actuator_id} for name, actuator_id in self.actuator_ids.items()
            ],
            "tendons": [
                {"name": mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_TENDON, tendon_id),
                 "id": tendon_id}
                for tendon_id in range(self.model.ntendon)
            ],
            "springs": [spring.as_dict() for spring in self.springs],
        }


def _sha256(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()
