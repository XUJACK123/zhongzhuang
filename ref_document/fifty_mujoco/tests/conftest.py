"""Shared fixtures.

The asset build is expensive (mesh scanning + MuJoCo compilation), so it runs
once per session and the reports are shared by the model tests.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC = PROJECT_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from fifty_mujoco.config import load_configs  # noqa: E402


@pytest.fixture(scope="session")
def config():
    return load_configs()


@pytest.fixture(scope="session")
def built_assets(config):
    from fifty_mujoco.assets.build_corrected import build_corrected_assets
    from fifty_mujoco.assets.build_mjcf import write_mjcf_variants

    corrected = build_corrected_assets(config)
    paths = write_mjcf_variants(config, corrected)
    return corrected, paths


@pytest.fixture(scope="session")
def validation(config, built_assets):
    from fifty_mujoco.assets.validate_mjcf import validate

    corrected, paths = built_assets
    return validate(config, corrected, paths)


@pytest.fixture(scope="session")
def audit_report(config):
    from fifty_mujoco.assets.audit import audit_assets

    return audit_assets(config)
