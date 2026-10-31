"""CLI behaviour: every advertised command is wired up, nothing is left gated."""

from __future__ import annotations

import pytest

from fifty_mujoco.cli import GATED_COMMANDS, build_parser, main


@pytest.mark.parametrize(
    "command",
    [
        "audit-assets",
        "build-assets",
        "validate-mjcf",
        "inspect",
        "spawn-audit",
        "spring-audit",
        "joint-test",
        "spring-sweep",
        "contact-test",
        "contract-test",
        "prepare-teacher",
        "evaluate-teacher",
        "smoke",
        "benchmark",
        "train",
        "evaluate-stand",
        "evaluate-lqr",
        "step-audit",
        "mechanism-check",
        "joint-conventions",
        "stage-plan",
        "gpu-parity",
        "export",
        "play",
        "render",
        "doctor",
    ],
)
def test_every_phase_command_exists(command):
    parser = build_parser()
    actions = [a for a in parser._actions if getattr(a, "choices", None)]
    subcommands = set()
    for action in actions:
        subcommands.update(action.choices)
    assert command in subcommands


def test_no_command_is_left_gated():
    # P0-P10 are implemented; the gate map must stay empty so a half-finished
    # phase cannot silently ship again.
    assert GATED_COMMANDS == {}


def test_missing_required_argument_exits_non_zero():
    with pytest.raises(SystemExit) as excinfo:
        main(["play"])
    assert excinfo.value.code != 0


def test_unknown_command_is_rejected():
    with pytest.raises(SystemExit):
        main(["definitely-not-a-command"])


def test_unknown_backend_is_rejected():
    parser = build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(["inspect", "--backend", "gpu"])
