"""Compress the hand-tuned course table from mm/int16 to cm/uint8+int8.

Usage:
    python compress_route.py emit     # snapshot + print the C block (before editing)
    python compress_route.py verify   # expand the edited header and diff it

The firmware keeps 71 points in Inc/course_route.h.  Storing millimetres as
int16 costs 6 B per point (426 B); centimetres cost 2 B per point (142 B) and
are expanded into the RAM copy at startup.  Rounding to the nearest centimetre
moves every point by at most 5 mm.

All output files are written as UTF-8 next to this script so the numbers can be
read without PowerShell redirection (which produces UTF-16).
"""

import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
HEADER = ROOT / "code_test_xetranhvatcan" / "Inc" / "course_route.h"
BEFORE = HERE / "route_mm_before.txt"
AFTER = HERE / "route_mm_after.txt"
BLOCK = HERE / "course_route_cm_block.txt"
REPORT = HERE / "compress_route.txt"

MM_PATTERN = re.compile(r"\{\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(\d+)\s*\}")
CM_PATTERN = re.compile(r"\{\s*(\d+)\s*,\s*(-?\d+)\s*\}")


def round_cm(mm):
    """Round to the nearest centimetre, halves away from zero."""
    return int(mm / 10 + 0.5) if mm >= 0 else -int(-mm / 10 + 0.5)


def read_mm_table(text):
    start = text.index("static const int16_t course_route[][3] = {")
    body = text[start:text.index("};", start)]
    return [(int(x), int(y), int(leg)) for x, y, leg in MM_PATTERN.findall(body)]


def read_cm_table(text):
    start = text.index("course_route_cm[COURSE_ROUTE_COUNT] = {")
    body = text[start:text.index("};", start)]
    return [(int(x), int(y)) for x, y in CM_PATTERN.findall(body)]


def emit():
    text = HEADER.read_text(encoding="utf-8")
    points = read_mm_table(text)
    count = len(points)
    legs = [index for index in range(1, count) if points[index][2] != points[index - 1][2]]
    cm_points = [(round_cm(x), round_cm(y)) for x, y, _ in points]
    worst = max(max(abs(cx * 10 - x), abs(cy * 10 - y))
                for (x, y, _), (cx, cy) in zip(points, cm_points))
    xs = [cx for cx, _ in cm_points]
    ys = [cy for _, cy in cm_points]

    lines = [f"points           : {count}",
             f"leg changes at   : {legs} (legs {[points[i][2] for i in [0] + legs]})",
             f"worst rounding   : {worst} mm",
             f"x_cm range       : {min(xs)} .. {max(xs)}  (uint8 0..255)",
             f"y_cm range       : {min(ys)} .. {max(ys)}  (int8 -128..127)",
             f"flash            : {count * 6} B -> {count * 2} B (saves {count * 4} B)",
             f"RAM expanded copy: +{count * 6} B",
             ""]

    BEFORE.write_text("\n".join(f"{x:5d} {y:5d} {leg}" for x, y, leg in points) + "\n",
                      encoding="utf-8")
    AFTER.write_text("\n".join(f"{cx * 10:5d} {cy * 10:5d} {leg}"
                               for (cx, cy), (_, _, leg) in zip(cm_points, points)) + "\n",
                     encoding="utf-8")

    block = ["static const struct { uint8_t x_cm; int8_t y_cm; }",
             "    course_route_cm[COURSE_ROUTE_COUNT] = {"]
    for row in range(0, count, 6):
        chunk = cm_points[row:row + 6]
        block.append("    " + " ".join(f"{{{cx:4d},{cy:4d}}}" for cx, cy in chunk) + ",")
    block.append("};")
    BLOCK.write_text("\n".join(block) + "\n", encoding="utf-8")

    REPORT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


def verify():
    text = HEADER.read_text(encoding="utf-8")
    before = [tuple(int(v) for v in line.split()) for line in
              BEFORE.read_text(encoding="utf-8").splitlines() if line.strip()]
    cm_points = read_cm_table(text)
    count = len(cm_points)
    problems = []
    if count != len(before):
        problems.append(f"point count {count} != {len(before)}")
    worst = 0
    for index, ((cx, cy), (x, y, _)) in enumerate(zip(cm_points, before)):
        dx, dy = abs(cx * 10 - x), abs(cy * 10 - y)
        worst = max(worst, dx, dy)
        if dx > 5 or dy > 5:
            problems.append(f"point {index}: ({x},{y}) -> ({cx * 10},{cy * 10})")
    legs = [index for index in range(1, len(before)) if before[index][2] != before[index - 1][2]]
    expected_legs = [i for i in (14, 69) if i < count]
    if legs != expected_legs:
        problems.append(f"leg changes {legs} != {expected_legs}")
    lines = [f"points      : {count}",
             f"worst delta : {worst} mm",
             f"leg changes : {legs}",
             "RESULT      : " + ("OK" if not problems else "FAIL")]
    lines.extend("  " + problem for problem in problems)
    REPORT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    return 0 if not problems else 1


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "emit"
    sys.exit(verify() if mode == "verify" else (emit() or 0))
