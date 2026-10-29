"""Passive gas-spring model (doc sections 12.4 - 12.6).

Convention
----------
``length``          distance between the two anchors, metres.
``length_rate``     d(length)/dt, positive when the spring extends.
``force``           positive pushes the two anchors APART (gas spring / strut).
``generalized``     ``J_l^T * force`` with ``J_l = d(length)/d(q)``, so a positive
                    force does positive work when the spring extends.

MuJoCo tendon actuators behave like cables: a positive control force *shortens*
the tendon.  Therefore the runtime command is ``ctrl = -force``; the sign is
re-verified by ``spring-audit`` on every run rather than assumed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from ..config import Config


@dataclass(frozen=True)
class SpringParams:
    name: str
    side: str
    body_a: str
    xyz_a: np.ndarray
    body_b: str
    xyz_b: np.ndarray
    length_min_m: float | None
    length_max_m: float | None
    length_ref_m: float | None
    force_ref_n: float
    stiffness_n_per_m: float
    damping_n_s_per_m: float
    friction_n: float
    velocity_smoothing_m_s: float | None
    force_min_n: float
    force_max_n: float
    parameter_status: str
    stroke_status: str
    source: str | None

    @property
    def force_limit(self) -> float:
        return float(max(abs(self.force_min_n), abs(self.force_max_n)))

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "side": self.side,
            "anchor_a": {"body": self.body_a, "xyz_m": self.xyz_a.tolist()},
            "anchor_b": {"body": self.body_b, "xyz_m": self.xyz_b.tolist()},
            "length_min_m": self.length_min_m,
            "length_max_m": self.length_max_m,
            "force_ref_n": self.force_ref_n,
            "stiffness_n_per_m": self.stiffness_n_per_m,
            "damping_n_s_per_m": self.damping_n_s_per_m,
            "friction_n": self.friction_n,
            "parameter_status": self.parameter_status,
            "stroke_status": self.stroke_status,
            "source": self.source,
        }


def load_springs(config: Config) -> list[SpringParams]:
    springs: list[SpringParams] = []
    for entry in config.gas_spring_springs:
        anchor_a = entry["anchor_a"]
        anchor_b = entry["anchor_b"]
        springs.append(
            SpringParams(
                name=str(entry["name"]),
                side=str(entry.get("side", "")),
                body_a=str(anchor_a["body"]),
                xyz_a=np.asarray(anchor_a["xyz_m"], dtype=float),
                body_b=str(anchor_b["body"]),
                xyz_b=np.asarray(anchor_b["xyz_m"], dtype=float),
                length_min_m=entry.get("length_min_m"),
                length_max_m=entry.get("length_max_m"),
                length_ref_m=entry.get("length_ref_m"),
                force_ref_n=float(entry["force_ref_n"]),
                stiffness_n_per_m=float(entry.get("stiffness_n_per_m") or 0.0),
                damping_n_s_per_m=float(entry.get("damping_n_s_per_m") or 0.0),
                friction_n=float(entry.get("friction_n") or 0.0),
                velocity_smoothing_m_s=entry.get("velocity_smoothing_m_s"),
                force_min_n=float(entry.get("force_min_n") or 0.0),
                force_max_n=float(entry.get("force_max_n") or entry["force_ref_n"]),
                parameter_status=str(entry.get("parameter_status", "unknown")),
                stroke_status=str(entry.get("stroke_status", "unknown")),
                source=entry.get("source"),
            )
        )
    return springs


def spring_force(params: SpringParams, length: float, length_rate: float = 0.0) -> float:
    """F(l, dl/dt) in newtons, positive = push apart (doc equation 12.4)."""
    force = params.force_ref_n
    if params.stiffness_n_per_m and params.length_ref_m is not None:
        force += params.stiffness_n_per_m * (params.length_ref_m - length)
    if params.damping_n_s_per_m:
        force -= params.damping_n_s_per_m * length_rate
    if params.friction_n:
        smoothing = params.velocity_smoothing_m_s
        if not smoothing or smoothing <= 0.0:
            raise ValueError(
                f"{params.name}: friction is enabled but velocity_smoothing_m_s is not positive"
            )
        force -= params.friction_n * math.tanh(length_rate / smoothing)
    return float(min(max(force, params.force_min_n), params.force_max_n))


def check_stroke(params: SpringParams, length: float) -> dict[str, Any]:
    """Stroke check with an explicit 'unverified' state instead of a silent pass."""
    if params.length_min_m is None or params.length_max_m is None:
        return {
            "status": "unverified",
            "length_m": float(length),
            "passed": True,
            "note": "no verified stroke limits; the model does not clamp the length",
        }
    margin_low = length - params.length_min_m
    margin_high = params.length_max_m - length
    return {
        "status": "verified",
        "length_m": float(length),
        "margin_min_m": float(margin_low),
        "margin_max_m": float(margin_high),
        "passed": margin_low >= 0.0 and margin_high >= 0.0,
    }


def spring_geometry(env, params: SpringParams) -> dict[str, Any]:
    """Anchor positions, length, length rate and tendon Jacobian for one spring.

    The Jacobian is built from ``mj_jacSite`` translational blocks:
    ``J_l = e^T (J_b - J_a)``, which maps generalized velocity (nv) to dl/dt.
    """
    import mujoco

    site_a = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, f"{params.name}_a")
    site_b = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, f"{params.name}_b")
    if site_a < 0 or site_b < 0:
        raise ValueError(f"{params.name}: sites are missing from {env.xml_path.name}")

    pos_a = env.data.site_xpos[site_a].copy()
    pos_b = env.data.site_xpos[site_b].copy()
    delta = pos_b - pos_a
    length = float(np.linalg.norm(delta))
    if length <= 1e-9:
        raise ValueError(f"{params.name}: anchors coincide")
    direction = delta / length

    nv = env.model.nv
    jac_a = np.zeros((3, nv))
    jac_b = np.zeros((3, nv))
    jac_rot = np.zeros((3, nv))
    mujoco.mj_jacSite(env.model, env.data, jac_a, jac_rot, site_a)
    mujoco.mj_jacSite(env.model, env.data, jac_b, jac_rot, site_b)
    jac_length = direction @ (jac_b - jac_a)
    length_rate = float(jac_length @ env.data.qvel)

    return {
        "site_ids": (int(site_a), int(site_b)),
        "pos_a": pos_a,
        "pos_b": pos_b,
        "direction": direction,
        "length_m": length,
        "length_rate_m_s": length_rate,
        "jacobian": jac_length,
    }


def generalized_force(jac_length: np.ndarray, force: float) -> np.ndarray:
    """tau = J_l^T * F (positive force pushes apart)."""
    return jac_length * force
