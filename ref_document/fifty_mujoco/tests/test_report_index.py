"""``reports/index.json`` must describe the directory it lives in.

The index exists so a report can be traced back to the run that produced it
(progress.md §18.10). A stale index is worse than none: it silently points at a
file that no longer matches. The test compares names and hashes, never timestamps
- ``generated_at`` differs on every rebuild by design.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from report_index import INDEX_NAME, REPORTS_DIR, _differences, build_index  # noqa: E402

INDEX_PATH = REPORTS_DIR / INDEX_NAME


@pytest.fixture(scope="module")
def index() -> dict:
    assert INDEX_PATH.is_file(), (
        f"{INDEX_PATH} is missing; run `python tools/report_index.py`"
    )
    return json.loads(INDEX_PATH.read_text(encoding="utf-8"))


def test_the_index_lists_every_report_and_nothing_else(index):
    differences = _differences(index, build_index(REPORTS_DIR))
    assert differences == [], (
        "reports/index.json is stale; run `python tools/report_index.py`:\n  "
        + "\n  ".join(differences)
    )


def test_every_entry_carries_a_hash_and_a_size(index):
    assert index["count"] == len(index["entries"])
    for entry in index["entries"]:
        assert len(entry["sha256"]) == 64, entry["name"]
        assert entry["bytes"] > 0, entry["name"]


def test_the_index_does_not_index_itself(index):
    assert all(entry["name"] != INDEX_NAME for entry in index["entries"])


def test_unreadable_json_is_still_listed(tmp_path):
    (tmp_path / "broken.json").write_text("{not json", encoding="utf-8")
    (tmp_path / "fine.json").write_text('{"tool": "x", "status": "passed"}', encoding="utf-8")
    built = build_index(tmp_path)
    names = [entry["name"] for entry in built["entries"]]
    assert names == ["broken.json", "fine.json"]
    assert built["entries"][0]["readable_json"] is False
    assert built["entries"][1]["status"] == "passed"


def test_a_changed_file_is_reported_as_stale(tmp_path):
    path = tmp_path / "a.json"
    path.write_text('{"tool": "a"}', encoding="utf-8")
    first = build_index(tmp_path)
    path.write_text('{"tool": "b"}', encoding="utf-8")
    second = build_index(tmp_path)
    differences = _differences(first, second)
    assert any("a.json" in line and "sha256" in line for line in differences)


def test_an_added_file_is_reported_as_stale(tmp_path):
    (tmp_path / "a.json").write_text("{}", encoding="utf-8")
    first = build_index(tmp_path)
    (tmp_path / "b.json").write_text("{}", encoding="utf-8")
    differences = _differences(first, build_index(tmp_path))
    assert any("b.json" in line and "not in the index" in line for line in differences)
