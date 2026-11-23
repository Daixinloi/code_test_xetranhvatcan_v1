"""Rewrite Inc/course_route.h: store the hand-tuned table in centimetres.

The table is replaced by a 2 B/point cm table plus a CourseRoute_Init() that
expands it to millimetres in RAM at startup. Marker based, so it does not depend
on the exact spacing of the generated rows. Run "verify" afterwards to check the
expanded values against the original millimetre table kept in route_mm_before.txt.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
HEADER = ROOT / "code_test_xetranhvatcan" / "Inc" / "course_route.h"
BEFORE = pathlib.Path(__file__).resolve().parent / "route_mm_before.txt"

START = "static const int16_t course_route[][3] = {"
END = "};"
PLANNED = "#define course_route course_route_planned"

# x_cm, y_cm, from the 71 mm points rounded to the nearest centimetre.
CM = [
    (42, 58), (42, 60), (42, 61), (42, 63), (43, 64), (43, 65),
    (43, 67), (44, 68), (44, 69), (47, 76), (48, 76), (48, 77),
    (48, 78), (49, 79), (49, 80), (49, 81), (49, 82), (50, 83),
    (50, 86), (50, 87), (50, 88), (51, 89), (51, 89), (52, 90),
    (53, 90), (53, 90), (54, 90), (68, 90), (69, 90), (69, 90),
    (70, 90), (71, 90), (72, 90), (73, 90), (73, 89), (74, 89),
    (77, 87), (78, 86), (78, 86), (79, 85), (80, 85), (81, 85),
    (82, 85), (83, 85), (83, 85), (90, 85), (91, 85), (93, 85),
    (94, 85), (95, 84), (97, 84), (98, 83), (99, 83), (100, 82),
    (108, 77), (109, 76), (110, 75), (111, 74), (112, 72), (113, 71),
    (115, 70), (116, 70), (118, 69), (119, 69), (124, 69), (126, 68),
    (126, 67), (127, 66), (127, 65), (128, 64), (128, -7),
]
LEG_NORTH, LEG_EAST = 14, 69  # first index of each new leg


def block():
    out = [
        "/* 2026-11-22 flash optimisation: 71 points at 2 B each (uint8 cm x,",
        " * int8 cm y) instead of 6 B each, so the table costs 142 B of flash",
        " * instead of 426 B. CourseRoute_Init() expands it to millimetres in the",
        " * 426 B RAM copy at startup. Rounding to the nearest centimetre moves a",
        " * point by at most 5 mm; the finish leg lands on X = 1280 mm, still 35 mm",
        " * clear of the U block and 75 mm clear of T15. X is unsigned because the",
        " * course stays inside 0..2550 mm; Y is signed because the nose point sits",
        " * at -70 mm. Legs follow the point index: 0..13 north, 14..68 east,",
        " * 69..70 south. */",
        "#define COURSE_ROUTE_COUNT 71u",
        "static int16_t course_route[COURSE_ROUTE_COUNT][3];",
        "static const struct { uint8_t x_cm; int8_t y_cm; }",
        "    course_route_cm[COURSE_ROUTE_COUNT] = {",
    ]
    for i in range(0, len(CM), 6):
        row = " ".join("{%4d,%4d}," % (x, y) for x, y in CM[i:i + 6])
        out.append("    " + row)
    out += [
        "};",
        "static inline void CourseRoute_Init(void)",
        "{",
        "    for (unsigned i = 0; i < COURSE_ROUTE_COUNT; ++i) {",
        "        course_route[i][0] = (int16_t)course_route_cm[i].x_cm * 10;",
        "        course_route[i][1] = (int16_t)course_route_cm[i].y_cm * 10;",
        "        course_route[i][2] = i < %du ? 1 : i < %du ? 2 : 3;" % (LEG_NORTH, LEG_EAST),
        "    }",
        "}",
    ]
    return out


def apply():
    text = HEADER.read_text(encoding="utf-8")
    nl = "\r\n" if "\r\n" in text else "\n"
    lines = text.split(nl)
    if len(CM) != 71:
        raise SystemExit("expected 71 points, got %d" % len(CM))
    try:
        start = lines.index(START)
    except ValueError:
        raise SystemExit("start marker not found")
    end = lines.index(END, start)
    if PLANNED not in lines:
        raise SystemExit("planned marker not found")
    planned = lines.index(PLANNED)
    new = lines[:start] + block() + lines[end + 1:]
    # The planned branch needs a no-op init so callers stay branch agnostic.
    # planned < start, so the index is still valid after the replacement.
    if "CourseRoute_Init" not in new[planned + 1]:
        new.insert(planned + 1, "static inline void CourseRoute_Init(void) {}")
    HEADER.write_text(nl.join(new), encoding="utf-8")
    print("replaced %d lines with %d lines" % (end - start + 1, len(block())))
    print("flash 426 B -> 142 B (saves 284 B); RAM +426 B")


def verify():
    """Expand the cm table and compare with the millimetre table snapshot."""
    text = HEADER.read_text(encoding="utf-8")
    rows = re.findall(r"\{\s*(-?\d+),\s*(-?\d+)\},", text)
    cm = [(int(a), int(b)) for a, b in rows if abs(int(a)) < 300]
    if len(cm) != 71:
        raise SystemExit("expected 71 cm points, got %d" % len(cm))
    expanded = [(x * 10, y * 10) for x, y in cm]
    before = []
    for line in BEFORE.read_text(encoding="utf-8").splitlines():
        for a, b, _leg in re.findall(r"(-?\d+)\s+(-?\d+)\s+(\d+)", line):
            before.append((int(a), int(b)))
    if len(before) != 71:
        raise SystemExit("expected 71 reference points, got %d" % len(before))
    worst = 0
    for i, ((ax, ay), (bx, by)) in enumerate(zip(expanded, before)):
        d = max(abs(ax - bx), abs(ay - by))
        worst = max(worst, d)
        if d > 5:
            print("point %d: %s vs %s (%d mm)" % (i, (ax, ay), (bx, by), d))
    print("points checked  : %d" % len(before))
    print("worst axis error: %d mm" % worst)
    print("finish point    : %s (was %s)" % (expanded[-1], before[-1]))
    return 1 if worst > 5 else 0


if __name__ == "__main__":
    sys.exit(verify() if "verify" in sys.argv else (apply() or 0))
