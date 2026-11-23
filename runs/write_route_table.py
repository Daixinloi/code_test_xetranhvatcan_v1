"""Rewrite the course_route table in Inc/course_route.h from Tools/plan_route.py.

Keeps the file layout (4 points per line) so the diff stays readable and the
firmware table always matches the planner table.

Leg tags follow the baseline rule already used by the flashed build:
  index < 14  -> 1 (north leg)
  index < 69  -> 2 (east leg)
  otherwise   -> 3 (south leg)
`plan_route.leg_tags()` derives the tag from each segment's direction and gets
one point in the T14 knee wrong, so the fixed split is used here instead.
"""
import sys
from pathlib import Path

PROJ = Path(r"c:\Users\daitr\Downloads\code_test_xetranhvatcan_v1\code_test_xetranhvatcan")
sys.path.insert(0, str(PROJ / "Tools"))
import plan_route  # noqa: E402

HEADER = PROJ / "Inc" / "course_route.h"
points = plan_route.CURRENT_ROUTE
tags = [1 if i < 14 else 2 if i < 69 else 3 for i in range(len(points))]

rows = []
for (x, y), tag in zip(points, tags):
    rows.append(f"    {{{int(x)}, {int(y)}, {tag}}},")

lines = []
for i in range(0, len(rows), 4):
    lines.append("".join(rows[i:i + 4]))
table = "\n".join(lines) + "\n"

text = HEADER.read_text(encoding="utf-8")
start = text.index("static const int16_t course_route[][3] = {")
end = text.index("};", start)
new_text = text[:start] + "static const int16_t course_route[][3] = {\n" + table + text[end:]
HEADER.write_text(new_text, encoding="utf-8")

print(f"points={len(points)}")
print(f"first={points[0]} last={points[-1]}")
print(f"legs={sorted(set(tags))}")
