"""Best/worst-case PIC24 cycle counts over an xc16-objdump listing of one ISR.

Back edges (poll loops) are counted once; marks report the cycle range from
entry to the completion of the first instruction matching each regex.
"""

import re
import sys
from functools import lru_cache

LINE = re.compile(r"^\s+([0-9a-f]+):\s+(?:[0-9a-f]{2} ){3}\s*(\S+)\s*(.*)$")


def cost(mn):
    if mn.startswith("mov.d"):
        return 2
    if mn == "retfie":
        return 3
    if mn in ("rcall", "call", "goto"):
        return 2
    return 1


def main(path, *marks):
    avoid = {int(m[1:], 16) for m in marks if m.startswith("!")}
    marks = [m for m in marks if not m.startswith("!")]
    with open(path, encoding="ascii") as listing:
        found = (LINE.match(line) for line in listing)
        ins = [(int(m.group(1), 16), m.group(2), m.group(3)) for m in found if m]
    idx = {a: i for i, (a, _, _) in enumerate(ins)}

    def succ(i):
        _, mn, op = ins[i]
        if mn == "retfie":
            return []
        if mn == "bra":
            parts = op.split(",")
            tgt = int(parts[-1].split()[0], 16)
            if len(parts) == 1:
                return [(idx[tgt], 2)]
            return [(i + 1, 1), (idx[tgt], 2)]
        return [(i + 1, 0)]

    back, seen, stack = set(), set(), set()

    def dfs(i):
        seen.add(i)
        stack.add(i)
        for j, _ in succ(i):
            if j in stack:
                back.add((i, j))
            elif j not in seen:
                dfs(j)
        stack.discard(i)

    sys.setrecursionlimit(10000)
    dfs(0)

    @lru_cache(None)
    def span(i, stop):
        if stop is not None and stop.search(ins[i][1] + " " + ins[i][2]):
            return cost(ins[i][1]), cost(ins[i][1])
        nxt = [
            (j, c) for j, c in succ(i) if (i, j) not in back and ins[j][0] not in avoid
        ]
        if not nxt:
            done = stop is None and ins[i][1] == "retfie"
            return (cost(ins[i][1]),) * 2 if done else (None, None)
        lo, hi = None, None
        for j, c in nxt:
            a, b = span(j, stop)
            if a is None:
                continue
            extra = c if c else cost(ins[i][1])
            lo = a + extra if lo is None else min(lo, a + extra)
            hi = b + extra if hi is None else max(hi, b + extra)
        return lo, hi

    for i, j in sorted(back):
        print(f"loop {ins[i][0]:04x} -> {ins[j][0]:04x} (counted once)")
    lo, hi = span(0, None)
    print(f"entry -> retfie: best {lo:d}, worst {hi:d} TCY")
    for mk in marks:
        lo, hi = span(0, re.compile(mk))
        print(f"entry -> {mk}: best {lo}, worst {hi} TCY")


if __name__ == "__main__":
    main(*sys.argv[1:])
