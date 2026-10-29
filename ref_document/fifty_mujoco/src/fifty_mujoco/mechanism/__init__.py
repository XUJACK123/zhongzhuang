"""Planar mechanism geometry: loader, solver and consistency checks.

The package is deliberately independent of MuJoCo, torch and numpy:

* the geometry in ``configs/mechanism.yaml`` comes from the STEP audit, which is
  standard library only, so a CAD workstation with a bare CPython can verify it;
* a solver that cannot run without the simulation stack would make the geometry
  unverifiable exactly when it matters - before anything is simulated.

Layout
------
``loader``        parse + validate ``configs/mechanism.yaml``
``planar``        distance-constraint solver for planar closed chains
``leg``           the two-link leg model (analytic FK/IK) and the CAD cross-check
``pose``          exact FK for the ``link_a_on_shin`` candidate assignment
``transmission``  exact FK for the ``link_a_on_thigh`` candidate assignment
``check``         assemble the verification report used by ``cli mechanism-check``

The two assignments share the same eight pivots and the same fourteen lengths.
``mechanism-check`` reports both, because the CAD mates - not the geometry -
decide which one is real.
"""

from __future__ import annotations

from .loader import (
    Body,
    Link,
    Mechanism,
    MechanismConfigError,
    Node,
    load_mechanism,
)
from .planar import (
    PlanarSystem,
    SolveResult,
    TopologyNotConfirmedError,
    circle_intersection,
)

__all__ = [
    "Body",
    "Link",
    "Mechanism",
    "MechanismConfigError",
    "Node",
    "PlanarSystem",
    "SolveResult",
    "TopologyNotConfirmedError",
    "circle_intersection",
    "load_mechanism",
]
