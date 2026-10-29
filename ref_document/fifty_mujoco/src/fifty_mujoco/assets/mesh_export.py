"""STL -> OBJ export for the visual MJCF.

Why this exists: MuJoCo's STL decoder rejects files with more than 200 000
faces, and the chassis mesh has 1 020 702.  The OBJ path has no such limit.
The STL files are never modified; the OBJ files are derived artefacts and the
STL hashes stay in the audit report.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from . import stl

_STL_DTYPE = np.dtype(
    [
        ("normal", "<f4", (3,)),
        ("vertices", "<f4", (3, 3)),
        ("attribute", "<u2"),
    ]
)


def _triangle_vertices(path: Path) -> np.ndarray:
    count, _ = stl._read_header(path)
    data = np.fromfile(path, dtype=_STL_DTYPE, count=count, offset=84)
    return data["vertices"].reshape(-1, 3).astype(np.float64)


def stl_to_obj(stl_path: str | Path, obj_path: str | Path) -> dict:
    """Write an OBJ with 1e-6 m resolution.  Faces follow all vertices."""
    stl_path = Path(stl_path)
    obj_path = Path(obj_path)
    vertices = _triangle_vertices(stl_path)
    indices = np.arange(1, len(vertices) + 1, dtype=np.int64).reshape(-1, 3)

    obj_path.parent.mkdir(parents=True, exist_ok=True)
    with obj_path.open("w", encoding="ascii") as handle:
        handle.write(f"# generated from {stl_path.name} (fifty_mujoco)\n")
        np.savetxt(handle, vertices, fmt="v %.6f %.6f %.6f")
        np.savetxt(handle, indices, fmt="f %d %d %d")

    return {
        "stl": stl_path.name,
        "obj": obj_path.name,
        "faces": int(len(indices)),
        "vertices": int(len(vertices)),
        "obj_bytes": int(obj_path.stat().st_size),
        "stl_sha256": stl.sha256(stl_path),
    }


def ensure_obj(stl_path: str | Path, obj_dir: str | Path, force: bool = False) -> dict:
    stl_path = Path(stl_path)
    obj_dir = Path(obj_dir)
    obj_path = obj_dir / (stl_path.stem + ".obj")
    if obj_path.is_file() and not force and obj_path.stat().st_mtime >= stl_path.stat().st_mtime:
        count, _ = stl._read_header(stl_path)
        return {
            "stl": stl_path.name,
            "obj": obj_path.name,
            "faces": int(count),
            "obj_bytes": int(obj_path.stat().st_size),
            "stl_sha256": stl.sha256(stl_path),
            "cached": True,
        }
    result = stl_to_obj(stl_path, obj_path)
    result["cached"] = False
    return result
