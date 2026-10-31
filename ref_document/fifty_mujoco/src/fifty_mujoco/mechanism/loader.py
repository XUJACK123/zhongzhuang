"""Load and validate ``configs/mechanism.yaml``.

The validation is intentionally strict about one thing: an evidence label must
be earned. ``measured`` and ``validated`` entries are rejected unless they carry
a ``method`` and a ``date``, because the usual way a CAD-derived number becomes
"true" is somebody editing the status string.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping

import yaml

from ..paths import configs_dir

#: Evidence vocabulary. ``robot.yaml`` documents the first, third and fourth;
#: the two CAD-only labels were added when the STEP audit started producing
#: geometry, and they must never be read as measurements.
#:
#: ``cad_confirmed`` (2026-10-30) is for a relation a person read in the *native*
#: assembly - a mate, not a number: no parser can recover it, and it is still not
#: a hardware measurement. It exists so that "the audit could not find it" and
#: "SolidWorks confirms it" stop sharing one label.
STATUS_LEVELS: tuple[str, ...] = (
    "provisional",
    "cad_extracted",
    "cad_confirmed",
    "computed_from_points",
    "candidate_inferred",
    "datasheet",
    "measured",
    "validated",
)

#: Statuses that claim the real robot was involved.
EVIDENCE_STATUSES: frozenset[str] = frozenset({"datasheet", "measured", "validated"})


class MechanismConfigError(ValueError):
    """A ``mechanism.yaml`` entry is missing its source or its evidence."""


@dataclass(frozen=True)
class Node:
    """One planar pivot, in the shared YZ datum (millimetres)."""

    name: str
    yz_mm: tuple[float, float]
    status: str
    source: str
    shared_by: tuple[str, ...] = ()

    @property
    def yz_m(self) -> tuple[float, float]:
        return (self.yz_mm[0] / 1000.0, self.yz_mm[1] / 1000.0)


@dataclass(frozen=True)
class Link:
    """A rigid distance between two nodes."""

    name: str
    a: str
    b: str
    length_mm: float
    status: str
    source: str
    reproduces: str | None = None
    part: str | None = None

    @property
    def length_m(self) -> float:
        return self.length_mm / 1000.0


@dataclass(frozen=True)
class Body:
    """A candidate rigid body: the nodes that move together."""

    name: str
    nodes: tuple[str, ...]
    status: str
    source: str | None = None
    evidence: tuple[str, ...] = ()
    confirm_in_cad: str | None = None


@dataclass(frozen=True)
class Mechanism:
    """Parsed ``mechanism.yaml`` with the derived accessors the tools need."""

    path: Path
    meta: Mapping[str, Any]
    nodes: Mapping[str, Node]
    links: Mapping[str, Link]
    derived: Mapping[str, Link]
    bodies: Mapping[str, Body]
    topology: Mapping[str, Any]
    unknowns: Mapping[str, Any] = field(default_factory=dict)

    # -- convenience ------------------------------------------------------
    @property
    def all_links(self) -> dict[str, Link]:
        return {**self.links, **self.derived}

    @property
    def inferred_topology_allowed(self) -> bool:
        chain = self.topology.get("chain") or {}
        return bool(chain.get("use_inferred_topology", False))

    def node_mm(self, name: str) -> tuple[float, float]:
        try:
            return self.nodes[name].yz_mm
        except KeyError as error:  # pragma: no cover - guard against bad callers
            raise MechanismConfigError(f"unknown node {name!r}") from error

    def node_m(self, name: str) -> tuple[float, float]:
        return self.nodes[name].yz_m

    def length_mm(self, name: str) -> float:
        links = self.all_links
        if name not in links:
            raise MechanismConfigError(f"unknown link {name!r}")
        return links[name].length_mm

    def length_m(self, name: str) -> float:
        return self.length_mm(name) / 1000.0

    def nodes_of(self, body: str) -> tuple[str, ...]:
        return self.bodies[body].nodes

    def assembly_pose_m(self) -> dict[str, tuple[float, float]]:
        """Every node at the CAD pose, in metres."""
        return {name: node.yz_m for name, node in self.nodes.items()}


def _non_empty(value: Any) -> bool:
    if isinstance(value, str):
        return bool(value.strip())
    if isinstance(value, (list, tuple)):
        return bool(value)
    return False


def _entry(
    block: Mapping[str, Any],
    key: str,
    required: Iterable[str],
    where: str,
    source_keys: Iterable[str] = ("source",),
) -> Mapping[str, Any]:
    value = block.get(key)
    if not isinstance(value, Mapping):
        raise MechanismConfigError(f"{where}.{key}: expected a mapping")
    missing = [name for name in required if name not in value]
    if missing:
        raise MechanismConfigError(
            f"{where}.{key}: missing {', '.join(missing)}"
            " (every geometric entry needs its source and its evidence level)"
        )
    status = str(value["status"])
    if status not in STATUS_LEVELS:
        raise MechanismConfigError(
            f"{where}.{key}.status: {status!r} is not one of {', '.join(STATUS_LEVELS)}"
        )
    if status in EVIDENCE_STATUSES:
        absent = [name for name in ("method", "date") if name not in value]
        if absent:
            raise MechanismConfigError(
                f"{where}.{key}: status {status!r} needs {' and '.join(absent)};"
                " a CAD-derived number cannot be promoted by editing the label"
            )
    keys = list(source_keys)
    if not any(_non_empty(value.get(name)) for name in keys):
        raise MechanismConfigError(
            f"{where}.{key}: {' or '.join(keys)} (non-empty) is required"
        )
    return value


def _pair(value: Any, where: str) -> tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise MechanismConfigError(f"{where}: expected a two-element [y, z] list")
    try:
        return (float(value[0]), float(value[1]))
    except (TypeError, ValueError) as error:
        raise MechanismConfigError(f"{where}: {value!r} is not numeric") from error


def _load_nodes(block: Mapping[str, Any]) -> dict[str, Node]:
    nodes: dict[str, Node] = {}
    for name, raw in block.items():
        entry = _entry(block, name, ("yz_mm", "status", "source"), "planar_nodes")
        shared = entry.get("shared_by") or []
        if not isinstance(shared, (list, tuple)):
            raise MechanismConfigError(f"planar_nodes.{name}.shared_by: expected a list")
        nodes[name] = Node(
            name=name,
            yz_mm=_pair(entry["yz_mm"], f"planar_nodes.{name}.yz_mm"),
            status=str(entry["status"]),
            source=str(entry["source"]),
            shared_by=tuple(str(item) for item in shared),
        )
    if not nodes:
        raise MechanismConfigError("planar_nodes: at least one node is required")
    return nodes


def _load_links(
    block: Mapping[str, Any], nodes: Mapping[str, Node], where: str
) -> dict[str, Link]:
    links: dict[str, Link] = {}
    for name, _ in block.items():
        entry = _entry(block, name, ("between", "length_mm", "status", "source"), where)
        between = entry["between"]
        if not isinstance(between, (list, tuple)) or len(between) != 2:
            raise MechanismConfigError(f"{where}.{name}.between: expected two node names")
        first, second = str(between[0]), str(between[1])
        for node in (first, second):
            if node not in nodes:
                raise MechanismConfigError(f"{where}.{name}.between: unknown node {node!r}")
        try:
            length = float(entry["length_mm"])
        except (TypeError, ValueError) as error:
            raise MechanismConfigError(f"{where}.{name}.length_mm is not numeric") from error
        if length <= 0.0:
            raise MechanismConfigError(f"{where}.{name}.length_mm must be positive")
        links[name] = Link(
            name=name,
            a=first,
            b=second,
            length_mm=length,
            status=str(entry["status"]),
            source=str(entry["source"]),
            reproduces=str(entry["reproduces"]) if entry.get("reproduces") else None,
            part=str(entry["part"]) if entry.get("part") else None,
        )
    return links


def _load_bodies(block: Mapping[str, Any], nodes: Mapping[str, Node]) -> dict[str, Body]:
    bodies: dict[str, Body] = {}
    for name, _ in block.items():
        entry = _entry(
            block, name, ("nodes", "status"), "bodies", source_keys=("source", "evidence")
        )
        members = entry["nodes"]
        if not isinstance(members, (list, tuple)) or not members:
            raise MechanismConfigError(f"bodies.{name}.nodes: expected a non-empty list")
        for node in members:
            if str(node) not in nodes:
                raise MechanismConfigError(f"bodies.{name}.nodes: unknown node {node!r}")
        bodies[name] = Body(
            name=name,
            nodes=tuple(str(node) for node in members),
            status=str(entry["status"]),
            source=str(entry["source"]) if entry.get("source") else None,
            evidence=tuple(str(item) for item in (entry.get("evidence") or [])),
            confirm_in_cad=(
                str(entry["confirm_in_cad"]) if entry.get("confirm_in_cad") else None
            ),
        )
    return bodies


def parse_mechanism(data: Mapping[str, Any], path: Path) -> Mechanism:
    """Validate an already-parsed mapping (kept separate so tests can inject one)."""
    for section in ("meta", "planar_nodes", "link_lengths", "bodies", "topology"):
        if section not in data:
            raise MechanismConfigError(f"{section}: required section is missing")
    nodes = _load_nodes(data["planar_nodes"])
    links = _load_links(data["link_lengths"], nodes, "link_lengths")
    derived = _load_links(data.get("derived_lengths") or {}, nodes, "derived_lengths")
    bodies = _load_bodies(data["bodies"], nodes)
    topology = data["topology"]
    if not isinstance(topology, Mapping):
        raise MechanismConfigError("topology: expected a mapping")
    chain = topology.get("chain") or {}
    if not isinstance(chain, Mapping):
        raise MechanismConfigError("topology.chain: expected a mapping")
    if "use_inferred_topology" in chain and not isinstance(chain["use_inferred_topology"], bool):
        raise MechanismConfigError("topology.chain.use_inferred_topology must be a boolean")
    return Mechanism(
        path=path,
        meta=data["meta"],
        nodes=nodes,
        links=links,
        derived=derived,
        bodies=bodies,
        topology=topology,
        unknowns=data.get("unknowns") or {},
    )


def load_mechanism(path: str | Path | None = None) -> Mechanism:
    """Load ``configs/mechanism.yaml`` (or ``path``) and validate every entry."""
    target = Path(path) if path is not None else configs_dir() / "mechanism.yaml"
    with target.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, Mapping):
        raise MechanismConfigError(f"{target}: expected a mapping at the top level")
    return parse_mechanism(data, target)
