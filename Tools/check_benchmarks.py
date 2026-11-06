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
        if row.get("AnyOverflow", "false").lower() != "false":
            raise ValueError(f"A measured frame overflowed in {row['Name']}")
        if "SimulationP50Ms" in row:
            percentiles = [float(row[k]) for k in ("SimulationP50Ms", "SimulationP95Ms", "SimulationP99Ms")]
            if percentiles != sorted(percentiles) or percentiles[0] < 0:
                raise ValueError(f"Invalid frame percentiles in {row['Name']}")
            if int(row["ContactsMin"]) > int(row["ContactsMax"]) or int(row["PairsMin"]) > int(row["PairsMax"]):
                raise ValueError(f"Invalid contact/pair ranges in {row['Name']}")
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
            if "CCDQueryMs" in row:
                if float(row["CCDShapeMs"]) > float(row["CCDQueryMs"]) + 0.003 or float(row["CCDQueryMs"]) > float(row["IntegrationMs"]) + 0.003:
                    raise ValueError(f"Invalid nested CCD query timing in {row['Name']}")
                if float(row["CCDProxyMs"]) > sum(float(row[k]) for k in ("SyncMs", "CCDPreparationMs", "IntegrationMs")) + 0.01:
                    raise ValueError(f"Invalid nested CCD proxy timing in {row['Name']}")
            if "SleepMs" in row and abs(float(row["FinalizeMs"]) - float(row["SleepMs"]) - float(row["SanitizeMs"])) > 0.01:
                raise ValueError(f"Invalid finalization subphase totals in {row['Name']}")
        name = row["Name"]
        signature = (row["Hash"], row.get("TraceHash"))
        if name in seen and seen[name] != signature:
            raise ValueError(f"Repeated runs diverged in {name}")
        seen[name] = signature
    print(f"Validated {len(rows)} measurements: no allocations/overflows and stable repeat hashes")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("csv")
    check(parser.parse_args().csv)
