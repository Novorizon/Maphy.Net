#!/usr/bin/env python3
"""Compare every corpus row, ignoring only platform newline encoding."""
import argparse
from pathlib import Path

def compare(expected, actual):
    left = Path(expected).read_text(encoding="utf-8-sig").splitlines()
    right = Path(actual).read_text(encoding="utf-8-sig").splitlines()
    if len(left) != len(right):
        raise ValueError(f"Row count differs: {len(left)} != {len(right)}")
    if not left or left[0] != "Maphy deterministic corpus v1":
        raise ValueError("Unsupported or empty corpus")
    if sum(row.startswith("frame,") for row in left) != 1000:
        raise ValueError("Expected exactly 1,000 replay frames")
    for row, (a, b) in enumerate(zip(left, right), 1):
        if a != b:
            raise ValueError(f"First mismatch at row {row}:\nexpected {a}\nactual   {b}")
    print(f"Matched {len(left)} rows, including 1,000 replay frames")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("expected")
    parser.add_argument("actual")
    args = parser.parse_args()
    compare(args.expected, args.actual)
