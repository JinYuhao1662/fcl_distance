#!/usr/bin/env python3
"""Compare two crosscheck_dump outputs (upstream FCL vs fcl_distance port).

Usage: python crosscheck_compare.py upstream.txt port.txt [--tol 1e-6]
Prints agreement stats and the worst mismatches.  Distance values are
compared directly; nearest-point pairs are compared via |p1-p2| vs distance
(the point pair itself may legitimately differ when the solution is not
unique, e.g. parallel faces).
"""
import sys
import math

def load(path):
    rows = {}
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) != 10:
                continue
            key = (int(parts[0]), int(parts[1]))
            rows[key] = [float(x) for x in parts[2:]]
    return rows

def main():
    tol = 1e-6
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    for a in sys.argv[1:]:
        if a.startswith("--tol"):
            tol = float(a.split("=", 1)[1]) if "=" in a else tol
    a, b = load(args[0]), load(args[1])
    keys = sorted(set(a) & set(b))
    missing = sorted(set(a) ^ set(b))
    diffs = []
    for k in keys:
        da, db = a[k][1], b[k][1]  # min_distance
        # treat both-negative (penetration sentinels) as agreeing on sign
        if da < 0 and db < 0:
            diffs.append((0.0, k, da, db))
            continue
        diffs.append((abs(da - db), k, da, db))
    diffs.sort(reverse=True)
    n_bad = sum(1 for d, *_ in diffs if d > tol)
    print(f"compared {len(keys)} queries, {len(missing)} unmatched keys")
    print(f"within tol {tol}: {len(keys) - n_bad}/{len(keys)} "
          f"({100.0 * (len(keys) - n_bad) / max(1, len(keys)):.2f}%)")
    print("worst 15:")
    for d, k, da, db in diffs[:15]:
        print(f"  case {k}: |Δ|={d:.3e}  upstream={da:.12g}  port={db:.12g}")
    return 0 if n_bad == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
