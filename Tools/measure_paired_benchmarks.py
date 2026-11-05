#!/usr/bin/env python3
"""Alternate preserved before/after benchmark DLLs without concurrent builds."""
import argparse
import csv
import os
from pathlib import Path
import subprocess
import tempfile


def measure(args):
    if args.cpu is not None:
        if not hasattr(os, "sched_setaffinity"):
            raise ValueError("--cpu requires a platform with sched_setaffinity")
        os.sched_setaffinity(0, {args.cpu})
    environment = dict(os.environ, DOTNET_TieredCompilation="0")
    rows = {"before": [], "after": []}
    binaries = {"before": args.before.resolve(), "after": args.after.resolve()}
    for binary in binaries.values():
        if not binary.is_file():
            raise ValueError(f"Missing benchmark DLL: {binary}")
    order = 0
    with tempfile.TemporaryDirectory(prefix="maphy-paired-") as temporary:
        for repeat in range(1, args.repeat + 1):
            # Reverse which version starts each pair to reduce order bias.
            versions = ("before", "after") if repeat % 2 else ("after", "before")
            for version in versions:
                order += 1
                output = Path(temporary) / f"{version}-{repeat}.csv"
                command = [args.dotnet, str(binaries[version]), "--scenario", args.scenario,
                           "--count", str(args.count), "--warmup", str(args.warmup),
                           "--iterations", str(args.iterations), "--repeat", "1",
                           "--profile", "--out", str(output)]
                subprocess.run(command, env=environment, check=True)
                with output.open(newline="", encoding="utf-8-sig") as source:
                    measured = list(csv.DictReader(source))
                if not measured:
                    raise ValueError("No benchmark scenarios ran")
                for row in measured:
                    row["Repeat"] = str(repeat)
                    row["ExecutionOrder"] = str(order)
                rows[version].extend(measured)
                print(f"Pair {repeat}/{args.repeat}: {version} complete", flush=True)
    for version, measured in rows.items():
        output = Path(f"{args.out_prefix}-{version}.csv")
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", newline="", encoding="utf-8") as target:
            writer = csv.DictWriter(target, fieldnames=measured[0].keys())
            writer.writeheader()
            writer.writerows(measured)
        print(output)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--after", type=Path, required=True)
    parser.add_argument("--out-prefix", type=Path, required=True)
    parser.add_argument("--dotnet", default="dotnet")
    parser.add_argument("--scenario", default="CCD,StaticQuery,DynamicBroadphase,SleepingBodies")
    parser.add_argument("--count", type=int, default=1000)
    parser.add_argument("--warmup", type=int, default=8)
    parser.add_argument("--iterations", type=int, default=30)
    parser.add_argument("--repeat", type=int, default=6)
    parser.add_argument("--cpu", type=int, help="Pin this runner and its children to one allowed CPU (Linux)")
    options = parser.parse_args()
    if min(options.count, options.iterations, options.repeat) <= 0 or options.warmup < 0:
        parser.error("count, iterations and repeat must be positive; warmup must be nonnegative")
    measure(options)
