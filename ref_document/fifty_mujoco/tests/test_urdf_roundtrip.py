"""Round-trip of the corrected URDF and of the physical numbers it carries."""

from __future__ import annotations

import numpy as np

from fifty_mujoco.assets.urdf_io import load_urdf


def test_corrected_urdf_reloads_with_same_physics(built_assets):
    corrected, _ = built_assets
    reloaded = load_urdf(corrected.report["corrected_urdf"])

    assert abs(reloaded.total_mass() - corrected.robot.total_mass()) <= 1e-12
    for name, link in corrected.robot.links.items():
        assert np.allclose(reloaded.links[name].inertial.mass, link.inertial.mass)
        assert np.allclose(reloaded.links[name].inertial.com, link.inertial.com, atol=1e-11)
        assert np.allclose(reloaded.links[name].inertial.inertia, link.inertial.inertia, atol=1e-11)
        assert np.allclose(reloaded.links[name].inertial.rpy, 0.0, atol=1e-12)


def test_mesh_references_round_trip(built_assets):
    corrected, _ = built_assets
    reloaded = load_urdf(corrected.report["corrected_urdf"])
    for name, link in reloaded.links.items():
        mesh = link.visuals[0].geometry.mesh_file
        assert mesh.startswith("meshes/"), f"{name}: {mesh}"
        assert mesh.endswith(".STL")


def test_joint_limits_round_trip(built_assets, config):
    corrected, _ = built_assets
    reloaded = load_urdf(corrected.report["corrected_urdf"])
    for name, limits in config.joint_limits.items():
        joint = reloaded.joints[name]
        if limits is None:
            assert joint.lower is None and joint.upper is None
            assert joint.type == "continuous"
        else:
            assert abs(joint.lower - limits[0]) <= 1e-11
            assert abs(joint.upper - limits[1]) <= 1e-11
            assert joint.type == "revolute"
