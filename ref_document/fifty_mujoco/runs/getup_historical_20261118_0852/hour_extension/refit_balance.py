"""Refine the handoff balance policy on CPU using the historical reset data."""

from argparse import ArgumentParser
from concurrent.futures import ProcessPoolExecutor
from hashlib import sha256
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from time import monotonic

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2] / "tools"))
from realdata_parallel_train import getup_evaluate
from realdata_seated_affine_cpu import adjusted
from realdata_train import evaluation_policy, save_checkpoint


def load(path):
    return torch.load(path, map_location="cpu", weights_only=True)


def evaluate(job):
    values, baseline, rise_path, config, seed, count = job
    torch.set_num_threads(1)
    with TemporaryDirectory(prefix="getup_balance_") as temp:
        candidate = Path(temp) / "balance.pt"
        torch.save({"model": adjusted(load(baseline), values)}, candidate)
        settings = {"task": "getup", "balance_checkpoint": str(candidate), "config": config,
                    "reset_range": [0., 0.], "settle_seconds": 1., "domain": True}
        result = getup_evaluate(evaluation_policy(load(rise_path)), settings, count=count, seed_base=seed)
    rows = result["rows"]
    perturbed = [row for row in rows if row["episode_parameters"]["motor_gain"] != [1.] * 6]
    perturbed_wins = sum(row["success"] for row in perturbed)
    score = 110 * result["passed"] + 140 * perturbed_wins + sum(row["balance_ticks"] for row in rows) / 100
    return {"score": score, "values": values, "passed": result["passed"], "perturbed": len(perturbed),
            "perturbed_wins": perturbed_wins, "rows": rows}


def main():
    parser = ArgumentParser()
    for name in ("rise", "balance", "config", "out"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=864000)
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--population", type=int, default=18)
    parser.add_argument("--replicates", type=int, default=8)
    parser.add_argument("--seconds", type=int, default=3100)
    parser.add_argument("--iterations", type=int, default=75)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(1)
    config = json.loads(args.config.read_text())
    baseline = load(args.balance)
    seed_low, seed_high = config["seed_ranges"]["train"]
    if not seed_low <= args.seed <= seed_high - args.replicates * args.iterations:
        parser.error("seed range outside training split")
    rng = np.random.default_rng(args.seed)
    identity = np.array([1., 1., 1., 0., 0., 0.])
    center = identity.copy()
    scale = np.array([.012, .014, .028, .006, .008, .015])
    minimum = scale * .2
    low = np.array([.9, .9, .82, -.05, -.06, -.08])
    high = np.array([1.1, 1.1, 1.18, .05, .06, .08])
    start = monotonic()
    elites = identity[None]
    with ProcessPoolExecutor(max_workers=args.workers) as pool, (args.out / "rows.jsonl").open("w") as log:
        for i in range(args.iterations):
            if monotonic() - start >= args.seconds:
                break
            population = (center + rng.normal(size=(args.population, 6)) * scale).clip(low, high)
            population[:len(elites)] = elites
            population[-1] = identity
            seed = args.seed + i * args.replicates
            jobs = [(p.tolist(), args.balance, args.rise, config, seed, args.replicates) for p in population]
            rows = list(pool.map(evaluate, jobs))
            best = sorted(rows, key=lambda row: row["score"], reverse=True)[:4]
            elites = np.array([row["values"] for row in best])
            center = .45 * center + .55 * elites.mean(axis=0)
            scale = np.maximum(minimum, .6 * scale + .4 * elites.std(axis=0))
            summary = {"iteration": i+1, "seed_base": seed, "trials": args.replicates,
                       "best": best[0]["passed"], "best_perturbed": best[0]["perturbed_wins"],
                       "perturbed": best[0]["perturbed"], "baseline": rows[-1]["passed"],
                       "baseline_perturbed": rows[-1]["perturbed_wins"], "wall_s": monotonic() - start}
            for rank, row in enumerate(best[:3]):
                path = args.out / f"iteration_{i+1:03d}_rank{rank}.pt"
                save_checkpoint(path, {"schema": "historical_getup_balance_refit_v1",
                                       "configuration": config, "model": adjusted(baseline, row["values"]),
                                       "rise_sha256": sha256(args.rise.read_bytes()).hexdigest(),
                                       "baseline_sha256": sha256(args.balance.read_bytes()).hexdigest(),
                                       "training": summary, "adjustment": row["values"]})
            (args.out / f"iteration_{i+1:03d}.json").write_text(json.dumps(best, indent=2) + "\n")
            print(json.dumps(summary), flush=True)
            log.write(json.dumps(summary) + "\n")
            log.flush()


if __name__ == "__main__":
    main()
