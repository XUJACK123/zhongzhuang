#!/usr/bin/env python3
"""Build ``reports/index.json``: one row per report, with a content hash.

Why this exists (progress.md §18.10): there are dozens of reports and nothing
says which run produced which file. Two rules keep the index useful:

* every ``reports/*.json`` file is listed, whether or not it looks important -
  an index that silently skips files is worse than no index;
* each row carries the SHA-256 of the file it describes, so a report edited by
  hand after the fact is detectable.

Standard library only: this must run on the CAD workstation too.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPORTS_DIR = PROJECT_ROOT / "reports"
INDEX_NAME = "index.json"

#: Keys copied out of a report when it happens to have them. Nothing is required:
#: a report without a ``tool`` key is still indexed.
IDENTIFYING_KEYS: tuple[str, ...] = ("tool", "stage", "status", "generated_at", "evidence")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _identify(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError):
        return {"readable_json": False}
    if not isinstance(payload, dict):
        return {"readable_json": True, "top_level": type(payload).__name__}
    row: dict[str, Any] = {"readable_json": True}
    for key in IDENTIFYING_KEYS:
        if key in payload and isinstance(payload[key], (str, int, float, bool)):
            row[key] = payload[key]
    return row


def build_index(reports_dir: Path | None = None) -> dict[str, Any]:
    directory = Path(reports_dir) if reports_dir is not None else REPORTS_DIR
    if not directory.is_dir():
        raise FileNotFoundError(directory)
    entries: list[dict[str, Any]] = []
    for path in sorted(directory.glob("*.json")):
        if path.name == INDEX_NAME:
            continue
        stat = path.stat()
        entry: dict[str, Any] = {
            "name": path.name,
            "bytes": stat.st_size,
            "modified_utc": datetime.fromtimestamp(
                stat.st_mtime, tz=timezone.utc
            ).isoformat(timespec="seconds"),
            "sha256": _sha256(path),
        }
        entry.update(_identify(path))
        entries.append(entry)
    return {
        "tool": "report-index",
        "schema": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "directory": directory.name,
        "count": len(entries),
        "note": (
            "generated file; run `python tools/report_index.py` after adding or"
            " regenerating a report. tests/test_report_index.py fails when the set"
            " of names or the hashes disagree with the directory."
        ),
        "entries": entries,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build reports/index.json")
    parser.add_argument("--reports", default=None, help="directory (default: reports/)")
    parser.add_argument("--out", default=None, help="output path (default: <dir>/index.json)")
    parser.add_argument("--check", action="store_true", help="exit 1 if the index is stale")
    args = parser.parse_args(argv)

    directory = Path(args.reports) if args.reports else REPORTS_DIR
    index = build_index(directory)
    out = Path(args.out) if args.out else directory / INDEX_NAME
    if args.check:
        existing = json.loads(out.read_text(encoding="utf-8")) if out.is_file() else {}
        stale = _differences(existing, index)
        for line in stale:
            print(f"stale: {line}")
        print(f"{directory}: {len(stale)} difference(s)")
        return 0 if not stale else 1
    out.write_text(json.dumps(index, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"{index['count']} report(s) indexed -> {out}")
    return 0


def _differences(existing: dict[str, Any], fresh: dict[str, Any]) -> list[str]:
    """Names and hashes only: ``generated_at`` always differs, so it is excluded."""
    problems: list[str] = []
    old = {entry["name"]: entry for entry in existing.get("entries") or []}
    new = {entry["name"]: entry for entry in fresh.get("entries") or []}
    for name in sorted(set(new) - set(old)):
        problems.append(f"{name}: on disk but not in the index")
    for name in sorted(set(old) - set(new)):
        problems.append(f"{name}: in the index but not on disk")
    for name in sorted(set(old) & set(new)):
        if old[name].get("sha256") != new[name].get("sha256"):
            problems.append(f"{name}: sha256 changed")
    return problems


if __name__ == "__main__":
    raise SystemExit(main())
