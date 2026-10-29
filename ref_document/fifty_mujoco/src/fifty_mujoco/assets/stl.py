"""Binary STL scanning: hashes, triangle counts and transformed point clouds.

The CAD meshes are 1.0-1.2 M triangles in total, so the reader is vectorised
and never materialises per-triangle Python objects.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np

from ..math3d import Transform

_STL_DTYPE = np.dtype(
    [
        ("normal", "<f4", (3,)),
        ("vertices", "<f4", (3, 3)),
        ("attribute", "<u2"),
    ]
)


def _read_header(path: Path) -> tuple[int, int]:
    size = path.stat().st_size
    with path.open("rb") as handle:
        header = handle.read(84)
    if len(header) < 84:
        raise ValueError(f"{path}: file too small for a binary STL")
    count = int(np.frombuffer(header[80:84], dtype="<u4")[0])
    expected = 84 + 50 * count
    if size != expected:
        raise ValueError(
            f"{path}: not a binary STL (size {size} != 84 + 50*{count} = {expected}); "
            "ASCII STL is not supported"
        )
    return count, size


def sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def stl_info(path: str | Path) -> dict:
    path = Path(path)
    count, size = _read_header(path)
    return {"file": path.name, "triangles": count, "bytes": size, "sha256": sha256(path)}


def stl_vertices(path: str | Path, transform: Transform | None = None) -> np.ndarray:
    """Unique-ish vertex cloud of a binary STL, optionally rigidly transformed."""
    path = Path(path)
    count, _ = _read_header(path)
    data = np.fromfile(path, dtype=_STL_DTYPE, count=count, offset=84)
    vertices = data["vertices"].reshape(-1, 3).astype(np.float64)
    if transform is not None:
        vertices = vertices @ transform.R.T + transform.t
    return vertices


def aabb(vertices: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return vertices.min(axis=0), vertices.max(axis=0)


def trimmed_aabb(vertices: np.ndarray, lower: float = 0.02, upper: float = 0.98) -> tuple[np.ndarray, np.ndarray]:
    """Per-axis percentile box; drops stray CAD spikes and screw threads."""
    low = np.percentile(vertices, lower * 100.0, axis=0)
    high = np.percentile(vertices, upper * 100.0, axis=0)
    return low, high


def top_point(vertices: np.ndarray) -> np.ndarray:
    """Highest point (+Z) of the cloud: used as the structural 'top marker'."""
    index = int(np.argmax(vertices[:, 2]))
    return vertices[index].copy()


def radius_about_axis(vertices: np.ndarray, axis_index: int = 2) -> float:
    """95th-percentile radius about a coordinate axis (robust to tessellation)."""
    others = [i for i in range(3) if i != axis_index]
    radial = np.linalg.norm(vertices[:, others], axis=1)
    return float(np.percentile(radial, 95.0))


def extent_along(vertices: np.ndarray, axis_index: int, lower: float = 0.02, upper: float = 0.98) -> float:
    low, high = trimmed_aabb(vertices, lower, upper)
    return float(high[axis_index] - low[axis_index])
