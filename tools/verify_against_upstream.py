#!/usr/bin/env python3
"""Prove the port differs from upstream FCL 0.7.0 only in the intended ways.

Diffs every shipped header against its upstream counterpart and classifies
each changed hunk.  Anything that does not fall into an expected category is
reported as UNEXPECTED — that list must stay empty.

    python tools/verify_against_upstream.py
"""
import difflib
import os
import re
import sys

UP = r"D:\dev\fcl\include\fcl"
PORT = r"D:\dev\fcl_distance\include\fcl"

# Files that have no upstream counterpart (generated or written for this port).
PORT_ONLY = {"config.h", "export.h", "fcl.h"}

MARKER = re.compile(r'^// fcl_distance: header-only extraction of FCL 0\.7\.0 ')
EXTERN_TEMPLATE = re.compile(r'^\s*extern template\b')
DROPPED_INCLUDE = re.compile(
    r'^\s*#\s*include\s+"fcl/('
    r'math/motion/|'
    r'narrowphase/detail/traversal/distance/[a-z_]*conservative_advancement'
    r')')
INLINED_MARK = re.compile(r'fcl_distance: definitions? moved here from')


def is_only_extern_templates(removed):
    """True if `removed` consists solely of complete `extern template ...;`
    statements (which may span several lines), blank lines and comments."""
    saw_one = False
    i = 0
    n = len(removed)
    while i < n:
        line = removed[i]
        s = line.strip()
        if not s or s.startswith("//"):
            i += 1
            continue
        if not EXTERN_TEMPLATE.match(line):
            return False
        # consume through the terminating ';'
        while i < n and not removed[i].rstrip().endswith(";"):
            i += 1
        if i >= n:            # statement was cut off by the hunk boundary
            return False
        saw_one = True
        i += 1
    return saw_one


def classify(added, removed, in_inlined_block):
    """Return (category, ok)."""
    a = [l for l in added if l.strip()]

    if not removed and a and all(MARKER.match(l) for l in a):
        return "provenance marker", True
    if not a and is_only_extern_templates(removed):
        return "extern template removed", True
    if not a and all(DROPPED_INCLUDE.match(l) or not l.strip() for l in removed):
        return "pruned-subsystem include removed", True
    if not removed and in_inlined_block:
        return "definition inlined from .cpp", True
    return "UNEXPECTED", False


def main():
    shipped = []
    for dp, dn, fn in os.walk(PORT):
        for f in fn:
            rel = os.path.relpath(os.path.join(dp, f), PORT).replace("\\", "/")
            if rel.endswith((".h", ".hpp")):
                shipped.append(rel)
    shipped.sort()

    counts = {}
    unexpected = []
    identical = 0
    no_upstream = []

    for rel in shipped:
        up = os.path.join(UP, rel.replace("/", os.sep))
        po = os.path.join(PORT, rel.replace("/", os.sep))
        if rel in PORT_ONLY or not os.path.isfile(up):
            no_upstream.append(rel)
            continue
        with open(up, encoding="utf-8", errors="replace") as f:
            ulines = f.read().splitlines()
        with open(po, encoding="utf-8", errors="replace") as f:
            plines = f.read().splitlines()
        if ulines == plines:
            identical += 1
            continue

        sm = difflib.SequenceMatcher(None, ulines, plines, autojunk=False)
        for tag, i1, i2, j1, j2 in sm.get_opcodes():
            if tag == "equal":
                continue
            removed = ulines[i1:i2]
            added = plines[j1:j2]
            in_inlined = any(INLINED_MARK.search(l) for l in added)
            cat, ok = classify(added, removed, in_inlined)
            counts[cat] = counts.get(cat, 0) + 1
            if not ok:
                unexpected.append((rel, i1 + 1, removed[:6], added[:6]))

    print("shipped headers        : %d" % len(shipped))
    print("byte-identical to fcl  : %d" % identical)
    print("no upstream counterpart: %d  %s" % (len(no_upstream), no_upstream))
    print("\nchange categories:")
    for k in sorted(counts):
        print("  %-34s %4d hunks" % (k, counts[k]))

    if unexpected:
        print("\n*** %d UNEXPECTED hunks ***" % len(unexpected))
        for rel, line, rem, add in unexpected[:25]:
            print("\n  %s:%d" % (rel, line))
            for l in rem:
                print("    - %s" % l)
            for l in add:
                print("    + %s" % l)
        return 1

    print("\nOK: every difference from upstream falls into an expected category.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
