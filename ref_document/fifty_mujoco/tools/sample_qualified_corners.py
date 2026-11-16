#!/usr/bin/env python3
"""Development-only uncertainty endpoints; these are not heldout samples."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import itertools
import json
import multiprocessing
from pathlib import Path
import time
from sample_qualified_evaluate import CLASSES, initialize, run
from sample_qualified_export import sha


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--checkpoint", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--workers", type=int, default=4)
    p.add_argument("--duration", type=float, default=20.)
    p.add_argument("--deadline-epoch", type=float, required=True)
    args = p.parse_args()
    if args.out.exists() or args.out.with_suffix(".rows.jsonl").exists():
        p.error("fresh output required")
    jobs = []
    for i, (legs, friction, spring) in enumerate(itertools.product(itertools.product((.8, 1.), repeat=4), (.6, .9), (.9, 1.1))):
        params = {"motor_gain": [legs[0], legs[1], 1., legs[2], legs[3], 1.],
                  "effective_friction": friction, "spring_scale": spring}
        for j, name in enumerate(CLASSES):
            jobs.append({"name": name, "seed": 22000000+i*10+j, "duration": args.duration,
                         "domain": "development_corner", "fixed_parameters": params})
    rows, started = [], time.monotonic()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.with_suffix(".rows.jsonl").open("w") as stream, ProcessPoolExecutor(
        max_workers=args.workers, mp_context=multiprocessing.get_context("spawn"),
        initializer=initialize, initargs=(str(args.checkpoint),)) as pool:
        iterator, pending = iter(jobs), {}
        for _ in range(args.workers):
            job = next(iterator, None)
            if job is not None and time.time() < args.deadline_epoch-120:
                pending[pool.submit(run, job)] = job
        while pending:
            future = next(as_completed(pending))
            pending.pop(future)
            row = future.result()
            rows.append(row)
            stream.write(json.dumps(row)+"\n")
            stream.flush()
            if len(rows) % 10 == 0:
                print(json.dumps({"done": len(rows), "total": len(jobs), "passed": sum(r["thresholds_met"] for r in rows),
                                  "physical_failures": sum(not r["completed"] for r in rows), "wall_s": time.monotonic()-started}), flush=True)
            if time.time() < args.deadline_epoch-120:
                job = next(iterator, None)
                if job is not None:
                    pending[pool.submit(run, job)] = job
    report = {"schema": "qualified_development_corners_v1", "checkpoint": str(args.checkpoint),
              "checkpoint_sha256": sha(args.checkpoint), "expected": len(jobs), "complete": len(rows) == len(jobs),
              "passed": sum(r["thresholds_met"] for r in rows), "physical_failures": sum(not r["completed"] for r in rows),
              "wall_seconds": time.monotonic()-started, "rows": rows,
              "scope": "Development endpoints of the declared uncertainty box, one seed per class/condition; not a probability distribution or hardware acceptance."}
    args.out.write_text(json.dumps(report, indent=2)+"\n")


if __name__ == "__main__":
    main()
