"""Continue successive GPU stages through the recorded one-hour training window."""

from argparse import ArgumentParser
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
from time import sleep

LOCAL = Path(__file__).resolve().parent
PROJECT = LOCAL.parents[2]
PREVIOUS = LOCAL.parents[1] / "getup_seated_20261118_035341"


def read(path):
    return json.loads(path.read_text())


def log(event, **metadata):
    row = {"at": datetime.now().astimezone().isoformat(), "event": event, **metadata}
    with (LOCAL / "stages.jsonl").open("a") as stream:
        stream.write(json.dumps(row) + "\n")
    print(json.dumps(row), flush=True)


def active(pid):
    path = Path(f"/proc/{pid}/cmdline")
    try:
        command = path.read_bytes()
    except FileNotFoundError:
        return False
    return b"realdata_seated_sequence_train.py" in command and b"hour_extension/warm" in command


def train(stage, parent, seconds, seed, bonus):
    prior = read(LOCAL / parent / "best.json")
    path = LOCAL / f"{stage}_prior.json"
    path.write_text(json.dumps({"motion": prior["motion"], "rows": [{"thresholds": prior["thresholds"]}]},
                               indent=2) + "\n")
    log("stage_start", stage=stage, budget_s=seconds, seed=seed, parent_stage=parent)
    with (LOCAL / f"{stage}.log").open("w") as stream:
        subprocess.run(
            [sys.executable, str(PROJECT / "tools/realdata_seated_sequence_train.py"),
             "--tracking", str(PREVIOUS / "balance_provisional.pt"),
             "--balance", str(PREVIOUS / "frozen/balance.pt"),
             "--prior", str(path), "--config", str(LOCAL / f"{stage}_configuration.json"),
             "--out", str(LOCAL / stage), "--population", "28", "--replicates", "10",
             "--iterations", "160", "--seconds", str(seconds), "--seed", str(seed),
             "--perturbed-bonus", str(bonus)],
            cwd=PROJECT, stdout=stream, stderr=subprocess.STDOUT, check=True,
        )
    log("stage_end", stage=stage)


def main():
    parser = ArgumentParser()
    parser.add_argument("--warm-pid", required=True, type=int)
    args = parser.parse_args()
    schedule = read(LOCAL / "schedule.json")
    start = datetime.fromisoformat(schedule["start"]).timestamp()
    deadline = datetime.fromisoformat(schedule["target_end"]).timestamp()
    log("monitoring_warm", pid=args.warm_pid)
    while active(args.warm_pid):
        sleep(8)
    log("stage_end", stage="warm")
    medium_end = start + 42 * 60
    parent = "warm"
    now = datetime.now().timestamp()
    if medium_end > now + 60:
        train("medium", parent, round(medium_end - now) - 10, 865000, 215)
        parent = "medium"
    now = datetime.now().timestamp()
    if deadline > now + 40:
        train("wide", parent, round(deadline - now), 869000, 260)
    log("training_complete")


if __name__ == "__main__":
    main()
