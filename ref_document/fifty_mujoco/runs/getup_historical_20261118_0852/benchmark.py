"""Run paired CPU comparisons using separate reset attitudes and physics seeds."""

import argparse
from collections import Counter
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import sys

RUN = Path(__file__).resolve().parent
PROJECT = RUN.parents[1]
FROZEN = RUN.parent / "getup_seated_20261118_035341" / "frozen"


def read(path):
    return json.loads(path.read_text())


def write(path, data):
    path.write_text(json.dumps(data, indent=2) + "\n")


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def collect(spec, destination):
    groups = {}
    for task in spec:
        report = read(destination / (task["label"] + ".json"))
        groups.setdefault(task["policy"], {}).setdefault(task["condition"], []).extend(report["rows"])
    result = {}
    for policy, conditions in groups.items():
        result[policy] = {}
        for condition, rows in conditions.items():
            actual = [row for row in rows if row["episode_parameters"]["motor_gain"] != [1.] * 6]
            reference = {row["seed"]: row["success"] for row in groups["baseline"][condition]}
            wins = sum(row["success"] and not reference[row["seed"]] for row in rows)
            losses = sum(not row["success"] and reference[row["seed"]] for row in rows)
            assert all(row["support_peak_n"] == 0 and row["support_peak_nm"] == 0 for row in rows)
            assert all(row["balance_ticks"] >= 1000 for row in rows if row["success"])
            result[policy][condition] = {
                "success": sum(row["success"] for row in rows),
                "trials": len(rows), "perturbed_success": sum(row["success"] for row in actual),
                "perturbed_trials": len(actual), "wins_vs_baseline": wins, "losses_vs_baseline": losses,
                "failure": dict(Counter(row["failure"] for row in rows if not row["success"])),
            }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--split", choices=["development", "heldout"], required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=10)
    parser.add_argument("--baseline-only", action="store_true")
    parser.add_argument("--hour", action="store_true", help="include the medium and wide perturbation groups")
    parser.add_argument("candidates", nargs="*", help="label=rise_path[,balance_path]")
    args = parser.parse_args()
    destination = args.out.resolve()
    default_balance = FROZEN / "balance.pt"
    checkpoint = {"baseline": (FROZEN / "primary.pt", default_balance)}
    for arg in args.candidates:
        label, path = arg.split("=", 1)
        if not label.isalnum() or label in checkpoint:
            parser.error(f"invalid or repeated label {label}")
        rise, _, balance = path.partition(",")
        checkpoint[label] = (Path(rise).resolve(), Path(balance).resolve() if balance else default_balance)
    bank_config = read(RUN / "evaluation_configuration.json")
    old_path = FROZEN / "configuration.json"
    bank = bank_config["seated_back"]["reset_bank"].copy()
    group_defs = [("original_nominal", old_path, False)]
    for tag in ("173944", "205441"):
        bank_config["seated_back"]["reset_bank"] = [entry for entry in bank if entry["source_snapshot"] == tag]
        path = RUN / f"evaluation_{tag}_configuration.json"
        write(path, bank_config)
        group_defs.append((f"historical_{tag}_nominal", path, False))
        group_defs.append((f"historical_{tag}_mild", path, True))
    if args.hour:
        for stage in ("medium", "wide"):
            configuration = read(RUN / f"hour_extension/{stage}_configuration.json")
            configuration["seated_back"]["reset_bank"] = bank
            path = RUN / f"evaluation_hour_{stage}.json"
            write(path, configuration)
            group_defs.append((f"historical_{stage}", path, True))
    if args.split == "heldout":
        group_defs.append(("original_broad", old_path, True))
        beginning, parts, size = (1016200 if args.hour else 1014200), 4, 8
    else:
        beginning, parts, size = (916200 if args.hour else 914200), 2, 5
    spec = []
    for label, (path, balance) in checkpoint.items():
        for idx, (condition, config, domain) in enumerate(group_defs):
            for chunk in range(parts):
                spec.append({
                    "label": f"{label}_{condition}_{chunk}", "policy": label, "condition": condition,
                    "checkpoint": str(path), "balance": str(balance), "config": str(config),
                    "expected_checkpoint_sha256": digest(path), "expected_balance_sha256": digest(balance),
                    "reset_range": [0, 0], "settle_seconds": 1, "domain": domain, "split": args.split,
                    "count": size, "seed_base": beginning + 100*idx + size*chunk,
                })
    if args.baseline_only:
        assert len(checkpoint) == 1
    spec_path = destination.with_suffix(".json")
    write(spec_path, spec)
    subprocess.run([sys.executable, str(PROJECT / "tools/realdata_batch_evaluate.py"),
                    "--spec", str(spec_path), "--out", str(destination), "--workers", str(args.workers)],
                   cwd=PROJECT, check=True)
    summary = collect(spec, destination)
    write(destination / "paired_summary.json", summary)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
