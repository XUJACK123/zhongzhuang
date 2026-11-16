"""Project path resolution.

All paths are resolved relative to the project root so that the same code runs
on Linux and on the original Windows checkout.
"""

from __future__ import annotations

import os
from pathlib import Path

# <workspace>/ref_document/fifty_mujoco/src/fifty_mujoco/paths.py
PROJECT_ROOT: Path = Path(__file__).resolve().parents[2]

# <workspace>/zhongzhuang (the folder that contains ref_document/ and solidwork_graph/)
DEFAULT_WORKSPACE_ROOT: Path = PROJECT_ROOT.parents[1]


def workspace_root() -> Path:
    """Workspace root, overridable with the FIFTY_WORKSPACE environment variable."""
    env = os.environ.get("FIFTY_WORKSPACE")
    if env:
        return Path(env).expanduser().resolve()
    return DEFAULT_WORKSPACE_ROOT


def source_robot_dir() -> Path:
    """Original CAD export package (read only)."""
    return workspace_root() / "solidwork_graph" / "fifty_version_with_meshes"


def training_robot_dir() -> Path:
    """Current G1 training package (read only; used for auditing the defects)."""
    return workspace_root() / "solidwork_graph" / "fifty_version_training"


def configs_dir() -> Path:
    return PROJECT_ROOT / "configs"


def assets_dir() -> Path:
    return PROJECT_ROOT / "assets"


def reports_dir() -> Path:
    return PROJECT_ROOT / "reports"


def ensure_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path
