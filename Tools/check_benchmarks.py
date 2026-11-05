#!/usr/bin/env python3
"""Allocation/capacity gate, deliberately independent of wall-clock speed."""
import argparse
import csv
from pathlib import Path

def check(path):
    with Path(path).open(newline="", encoding="utf-8-sig") as source:
        rows = list(csv.DictReader(source))
    if not rows:
        raise ValueError("No benchmark scenarios ran")
    seen = {}
    for row in rows:
        if int(row["SimulationAllocatedBytes"]) or int(row["HashAllocatedBytes"]):
            raise ValueError(f"Managed allocation in {row['Name']} repeat {row['Repeat']}")
        if row["Overflow"].lower() != "false":
            raise ValueError(f"Insufficient reserved capacity in {row['Name']}")
        if row.get("CCDIndexOverflow", "false").lower() != "false":
            raise ValueError(f"CCD index overflow in {row['Name']}")
        if "CCDProxiesCreated" in row and row["Name"].startswith("CCD/") and row["CCDBroadphaseEnabled"].lower() == "true":
            if int(row["CCDProxiesCreated"]) or int(row["CCDIndexRebuilds"]) or not int(row["CCDProxiesReused"]):
                raise ValueError(f"Warmed sparse CCD did not reuse its index in {row['Name']}")
        if row.get("ProfileEnabled", "false").lower() == "true":
            stages = ("SyncMs", "CCDPreparationMs", "IntegrationMs", "PairsMs", "ContactsMs",
                      "IslandsMs", "VelocitySolveMs", "PositionSolveMs", "FinalizeMs", "StatsMs")
            times = [float(row[field]) for field in stages]
            if min(times) < 0 or sum(times) > float(row["SimulationElapsedMs"]) + 0.02:
                raise ValueError(f"Invalid phase timing totals in {row['Name']}")
            if "SleepMs" in row and abs(float(row["FinalizeMs"]) - float(row["SleepMs"]) - float(row["SanitizeMs"])) > 0.01:
                raise ValueError(f"Invalid finalization subphase totals in {row['Name']}")
        name = row["Name"]
        if name in seen and seen[name] != row["Hash"]:
            raise ValueError(f"Repeated runs diverged in {name}")
        seen[name] = row["Hash"]
    print(f"Validated {len(rows)} measurements: no allocations/overflows and stable repeat hashes")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("csv")
    check(parser.parse_args().csv)
