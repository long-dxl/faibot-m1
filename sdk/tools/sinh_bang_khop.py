#!/usr/bin/env python3
"""Generate the joint table of sdk/README.md from sdk/faibotic/spec.py (no hand-typed numbers).

    python3 sdk/tools/sinh_bang_khop.py            # print the table
    python3 sdk/tools/sinh_bang_khop.py --ghi      # rewrite the block between the joints-table markers in sdk/README.md
    python3 sdk/tools/sinh_bang_khop.py --check    # exit 1 if README.md differs from the generated table
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

SDK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SDK))

from faibotic.cli import joint_table  # noqa: E402

README = SDK / "README.md"
START, END = "<!-- joints-table:start -->", "<!-- joints-table:end -->"


def table() -> str:
    rows = ["| Joint | Servo ID | Model | Bus | Limits |", "|---|---|---|---|---|"]
    for r in sorted(joint_table(), key=lambda r: r["servo_id"]):
        lim = "continuous (wheel)" if r["min"] is None else f"{r['min']:g} .. {r['max']:g} {r['unit']}"
        rows.append(f"| {r['joint']} | {r['servo_id']} | {r['servo']} | {r['bus']} | {lim} |")
    return "\n".join(rows)


def render(text: str) -> str:
    i, j = text.index(START), text.index(END)
    return text[:i + len(START)] + "\n" + table() + "\n" + text[j:]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ghi", action="store_true", help="write the table into README.md")
    ap.add_argument("--check", action="store_true", help="fail if README.md is out of date")
    a = ap.parse_args()
    if not (a.ghi or a.check):
        print(table())
        return 0
    old = README.read_text(encoding="utf-8")
    new = render(old)
    if a.check:
        if old != new:
            print("sdk/README.md joint table is out of date: run sdk/tools/sinh_bang_khop.py --ghi", file=sys.stderr)
            return 1
        return 0
    README.write_text(new, encoding="utf-8")
    print("README.md joint table written")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
