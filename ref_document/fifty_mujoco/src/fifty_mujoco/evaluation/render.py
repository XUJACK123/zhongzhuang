"""Offscreen snapshots of the model (doc section 10.6 fixed camera evidence).

Renders a fixed set of views so that the appearance, the collision primitives
and the gas-spring anchor sites can be checked without a GUI session.  Run
``inspect --viewer`` for interactive inspection.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import mujoco
import numpy as np

#: (name, azimuth, elevation, distance, lookat)
DEFAULT_VIEWS: tuple[tuple[str, float, float, float, tuple[float, float, float]], ...] = (
    ("side", 90.0, 0.0, 1.9, (0.0, 0.0, 0.40)),
    ("front", 180.0, 0.0, 1.9, (0.0, 0.0, 0.40)),
    ("rear", 0.0, 0.0, 1.9, (0.0, 0.0, 0.40)),
    ("perspective", 135.0, -12.0, 2.1, (0.0, 0.0, 0.40)),
    ("top", 90.0, -89.0, 1.6, (0.0, 0.0, 0.36)),
    ("right_leg", 90.0, -2.0, 0.75, (0.10, -0.24, 0.16)),
)


def render_views(
    env,
    out_dir: str | Path,
    prefix: str = "model",
    width: int = 1280,
    height: int = 800,
    views=DEFAULT_VIEWS,
    show_collision: bool = False,
) -> list[dict[str, Any]]:
    from PIL import Image

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    renderer = mujoco.Renderer(env.model, height=height, width=width)
    scene_option = mujoco.MjvOption()
    # MuJoCo 3.17 controls site/geom visibility by group, not by an mjVIS flag.
    for group in range(len(scene_option.sitegroup)):
        scene_option.sitegroup[group] = 1
    # Groups: 0 = world, 1 = floor, 2 = visual meshes, 3 = collision primitives.
    # Collision primitives surround the real mesh (the chassis collider is a box
    # that encloses the whole chassis), so they must stay hidden unless asked.
    for group in range(len(scene_option.geomgroup)):
        scene_option.geomgroup[group] = 1 if (group < 3 or show_collision) else 0

    written: list[dict[str, Any]] = []
    try:
        for name, azimuth, elevation, distance, lookat in views:
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.azimuth = azimuth
            camera.elevation = elevation
            camera.distance = distance
            camera.lookat[:] = np.asarray(lookat, dtype=float)

            renderer.update_scene(env.data, camera=camera, scene_option=scene_option)
            image = renderer.render()
            path = out_dir / f"{prefix}_{name}.png"
            Image.fromarray(image).save(path)
            written.append(
                {
                    "view": name,
                    "path": str(path),
                    "azimuth": azimuth,
                    "elevation": elevation,
                    "distance": distance,
                    "lookat": list(lookat),
                    "size": [width, height],
                }
            )
    finally:
        renderer.close()
    return written
