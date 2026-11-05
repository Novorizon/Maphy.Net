#!/usr/bin/env python3
"""Validate matched sparse CCD benchmark runs; report, but do not gate on, timing."""
import argparse
import csv
import statistics
from pathlib import Path

def load(path):
    with Path(path).open(newline="", encoding="utf-8-sig") as source:
        rows = list(csv.DictReader(source))
    keyed = {(r["Name"], r["Repeat"]): r for r in rows}
    if not keyed or len(keyed) != len(rows):
        raise ValueError("Empty or duplicate benchmark rows")
    return keyed

def compare(reference, indexed):
    before, after = load(reference), load(indexed)
    if before.keys() != after.keys():
        raise ValueError("Benchmark cases differ")
    for key, a in before.items():
        b = after[key]
        for field in ("Iterations", "WarmupIterations", "Hash", "Bodies", "Colliders"):
            if a[field] != b[field]: raise ValueError(f"{key}: different {field}")
        if a["CCDBroadphaseEnabled"].lower() != "false" or b["CCDBroadphaseEnabled"].lower() != "true":
            raise ValueError("Expected reference scan followed by indexed CCD")
        for row in (a, b):
            if any(int(row[field]) for field in ("SimulationAllocatedBytes", "HashAllocatedBytes")):
                raise ValueError(f"{key}: measured allocation")
            if row["Overflow"].lower() != "false" or row["CCDIndexOverflow"].lower() != "false":
                raise ValueError(f"{key}: capacity overflow")
        if int(b["CCDFallbackScans"]) or int(b["CCDShapeTests"]) > int(a["CCDShapeTests"]):
            raise ValueError(f"{key}: sparse CCD failed to prune without fallback")
    baseline = statistics.median(float(r["SimulationMicrosecondsPerIteration"]) for r in before.values())
    accelerated = statistics.median(float(r["SimulationMicrosecondsPerIteration"]) for r in after.values())
    print(f"Matched {len(before)} CCD measurements; reference {baseline / 1000:.3f} ms, indexed {accelerated / 1000:.3f} ms, ratio {baseline / accelerated:.2f}x")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("reference")
    parser.add_argument("indexed")
    args = parser.parse_args()
    compare(args.reference, args.indexed)
