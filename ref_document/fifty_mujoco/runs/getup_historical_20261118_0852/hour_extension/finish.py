"""Select on development results, freeze candidates, and run paired holdout."""

from argparse import ArgumentParser
from hashlib import sha256
import json
from pathlib import Path
from shutil import copy2
import subprocess
import sys

HERE = Path(__file__).resolve().parent
RUN = HERE.parent
PROJECT = HERE.parents[2]
ORIGINAL = HERE.parents[1] / "getup_seated_20261118_035341/frozen"


def read(path):
    return json.loads(path.read_text())


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def benchmark(mode, name, candidates, workers):
    subprocess.run([sys.executable, str(RUN / "benchmark.py"), "--split", mode, "--hour",
                    "--out", str(HERE / name), "--workers", str(workers),
                    *(f"{label}={rise},{balance}" for label, (rise, balance) in candidates.items())],
                   cwd=PROJECT, check=True)
    return read(HERE / name / "paired_summary.json")


def development_score(groups):
    nominal = sum(row["success"] for condition, row in groups.items() if condition.endswith("_nominal"))
    mild = [row for condition, row in groups.items() if condition.endswith("_mild")]
    medium = groups["historical_medium"]
    wide = groups.get("historical_wide", {}).get("perturbed_success", 0)
    return (nominal >= 29, 2 * medium["perturbed_success"] + 2 * wide
            + sum(row["perturbed_success"] for row in mild),
            nominal, medium["success"] + sum(row["success"] for row in mild))


def main():
    parser = ArgumentParser()
    parser.add_argument("--workers", type=int, default=12)
    args = parser.parse_args()
    assert rows(HERE / "stages.jsonl")[-1]["event"] == "training_complete"
    base = ORIGINAL / "balance.pt"
    candidates = {
        "shortcpu": (RUN / "cpu_refit/iteration_003_rank0.pt", base),
        "cpucontrol": (RUN / "cpu_control/iteration_006_rank0.pt", base),
        "gpucontrol": (RUN / "gpu_control/iteration_014_rank0.pt", base),
    }
    for stage in ("warm", "medium", "wide"):
        folder = HERE / stage
        if (folder / "best_rank0.pt").exists():
            candidates[stage] = (folder / "best_rank0.pt", base)
            late = rows(folder / "rows.jsonl")
            pool = [row for row in late[len(late)//2:] if row["iteration"] % 3 == 0]
            if pool:
                selected = max(pool, key=lambda row: row["best_score"])["iteration"]
                path = folder / f"iteration_{selected:03d}_rank0.pt"
                if path.exists() and digest(path) != digest(folder / "best_rank0.pt"):
                    candidates[stage+"late"] = (path, base)
    balances = rows(HERE / "balance_cpu/rows.jsonl")
    if balances:
        by_gain = sorted(balances, key=lambda row: (
            2*(row["best_perturbed"]-row["baseline_perturbed"]) + row["best"]-row["baseline"],
            row["best_perturbed"], row["best"]), reverse=True)
        for i, row in enumerate(by_gain[:2], start=1):
            balance = HERE / f"balance_cpu/iteration_{row['iteration']:03d}_rank0.pt"
            candidates[f"cpujoint{i}"] = (candidates["shortcpu"][0], balance)
        if "medium" in candidates:
            candidates["mediumjoint"] = (candidates["medium"][0], candidates["cpujoint1"][1])

    for batch in ("development_midpoint", "development_medium_mid"):
        path = HERE / batch / "paired_summary.json"
        if not path.exists():
            continue
        results = read(path)
        best = max((name for name in results if name != "baseline"),
                   key=lambda name: development_score(results[name]))
        protocol = read(path.parent / "protocol.json")
        task = next(task for task in protocol if task["policy"] == best)
        pair = Path(task["checkpoint"]), Path(task["balance"])
        fingerprints = tuple(map(digest, pair))
        if all(tuple(map(digest, other)) != fingerprints for other in candidates.values()):
            candidates[best] = pair

    dev = benchmark("development", "development_final", candidates, args.workers)
    controls = ["cpucontrol", "gpucontrol"]
    tuned = [name for name in candidates if name not in controls and name != "shortcpu"]
    winner = max(tuned, key=lambda name: development_score(dev[name]))
    control = max(controls, key=lambda name: development_score(dev[name]))
    manifest = {"chosen_on": "CPU development only", "historical_hour": winner, "original_pose_control": control,
                "development_score": {name: list(development_score(groups)) for name, groups in dev.items()},
                "models": {}}
    finalists = {"hourcandidate": candidates[winner], "control": candidates[control], "shortcpu": candidates["shortcpu"]}
    frozen = HERE / "frozen"
    frozen.mkdir(exist_ok=False)
    fixed = {}
    for label, (rise, balance) in finalists.items():
        rise_target, balance_target = frozen / f"{label}_rise.pt", frozen / f"{label}_balance.pt"
        copy2(rise, rise_target)
        copy2(balance, balance_target)
        fixed[label] = rise_target, balance_target
        manifest["models"][label] = {"source_rise": str(rise), "source_balance": str(balance),
                                        "rise_sha256": digest(rise_target), "balance_sha256": digest(balance_target)}
    (frozen / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    result = benchmark("heldout", "heldout", fixed, args.workers)
    (HERE / "final_summary.json").write_text(json.dumps({"frozen": manifest, "comparison": result},
                                                       indent=2) + "\n")


if __name__ == "__main__":
    main()
