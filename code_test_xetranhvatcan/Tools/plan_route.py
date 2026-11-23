#!/usr/bin/env python3
"""A* shortest-path + obstacle-avoidance planner for the fixed U course.

Uses exactly the obstacle table the firmware uses (Src/grid_map.c:
grid_static_obstacles[18]) so the planned polyline is guaranteed collision free
for the same footprint the MCU assumes.

Jobs:
  validate - measure length and minimum clearance of a route (the hand-tuned
             table in Inc/course_route.h).
  plan     - run A* over the inflated arena and emit the shortest polyline,
             then shorten it with a line-of-sight pass.

All distances are millimetres. The arena is 1800 x 1200 mm
(GRID_WIDTH 36 * GRID_CELL_MM 50, GRID_HEIGHT 24 * 50).

Run:
    python Tools/plan_route.py validate
    python Tools/plan_route.py plan
    python Tools/plan_route.py plan --emit-header
"""

import argparse
import heapq
import math

ARENA_W = 1800
ARENA_H = 1200

# Same order and coordinates as grid_static_obstacles[18] in Src/grid_map.c.
# Index 17 is the middle exclusion (the U-shaped block).
OBSTACLES = [
    (0, 100, 100, 200),        # T1
    (500, 200, 600, 300),      # T2
    (250, 300, 350, 400),      # T3
    (500, 400, 600, 500),      # T4
    (0, 500, 100, 600),        # T5
    (0, 700, 100, 800),        # T6
    (300, 850, 400, 950),      # T7
    (400, 1100, 500, 1200),    # T8
    (600, 700, 700, 800),      # T9
    (700, 600, 800, 700),      # T10
    (800, 950, 900, 1050),     # T11
    (900, 600, 1000, 700),     # T12
    (1100, 1100, 1200, 1200),  # T13
    (1200, 800, 1300, 900),    # T14
    (1400, 400, 1500, 500),    # T15
    (1400, 100, 1500, 200),    # T16
    (1700, 200, 1800, 300),    # T17
    (600, 0, 1200, 600),       # middle exclusion
]
MIDDLE_INDEX = 17

# Robot 90 x 90 mm. Firmware numbers:
#   turn_padding = ceil(hypot(90,90)/2) + clearance(0) + margin(10) = 74
#   straight pad = (90+1)/2 + 20 + 0 = 65   (used while driving straight)
#   middle_extra = 50                        (U_MIDDLE_CLEARANCE_MM)
# 64 mm is the physical half-diagonal of the 90 x 90 mm body, so a route at
# >= 64 mm from every box cannot touch anything in any orientation. The planner
# in Src/navigation.c adds U_MIDDLE_CLEARANCE_MM (50 mm) on top of that for the
# field mode; that extra margin is deliberately NOT used here, because the
# hand-tuned fixed course threads the gaps between T7 and T9.
INFLATE_MM = 64
MIDDLE_EXTRA_MM = 0

START_ENTRY = (420, 0)      # where the car crosses the start line
START_NOSE = (420, -45)     # cfg.start_y_mm = -45
FINISH_ENTRY = (1275, 0)    # where the car crosses the finish line
FINISH_NOSE = (1275, -65)   # last point of the hand-tuned route

# Hand-tuned route currently in Inc/course_route.h (the flashed build).
# Final leg sits at X = 1275 mm: 27 mm left of the flashed 1302 mm so the body
# gets 80 mm to T15/T16 instead of 53 mm. The finish corridor is only
# 1200..1400 mm wide (U block edge to T15 edge), so a 90 mm body leaves just
# 110 mm of legal centre travel: 1245..1355 mm. A 70 mm shift (X = 1232) would
# put the body 13 mm inside the U block, so 1275 is the practical limit.
CURRENT_ROUTE = [
    (420, 582), (420, 596), (422, 611), (423, 625), (426, 639), (430, 652),
    (434, 666), (439, 679), (444, 692), (474, 755), (478, 764), (481, 773),
    (484, 782), (487, 791), (489, 800), (492, 809), (493, 819), (495, 828),
    (498, 862), (500, 871), (503, 878), (507, 885), (512, 890), (518, 895),
    (525, 898), (533, 899), (542, 900), (677, 900), (685, 900), (694, 899),
    (702, 899), (710, 898), (718, 897), (726, 898), (734, 892), (742, 885),
    (768, 867), (776, 861), (784, 856), (792, 851), (800, 851), (808, 851),
    (816, 851), (825, 851), (833, 851), (901, 851), (914, 851), (927, 849),
    (940, 846), (952, 842), (965, 837), (976, 831), (988, 825), (999, 818),
    (1078, 772), (1091, 762), (1104, 750), (1112, 735), (1120, 720),
    (1132, 708), (1146, 700), (1160, 696), (1175, 694), (1190, 692),
    (1240, 690), (1258, 680), (1264, 670), (1269, 661), (1272, 652),
    (1275, 643), (1275, -65),
]


def _pad(rect, pad):
    x0, y0, x1, y1 = rect
    return (x0 - pad, y0 - pad, x1 + pad, y1 + pad)


def _inside(rect, x, y):
    return rect[0] <= x <= rect[2] and rect[1] <= y <= rect[3]


def _segment_hits_rect(rect, x0, y0, x1, y1):
    """Liang-Barsky clip: does the segment touch the axis-aligned rectangle?"""
    dx, dy = x1 - x0, y1 - y0
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, x0 - rect[0]), (dx, rect[2] - x0),
                 (-dy, y0 - rect[1]), (dy, rect[3] - y0)):
        if p == 0:
            if q < 0:
                return False
        else:
            t = q / p
            if p < 0:
                if t > t1:
                    return False
                if t > t0:
                    t0 = t
            else:
                if t < t0:
                    return False
                if t < t1:
                    t1 = t
    return t0 <= t1


def _inflated(pad=INFLATE_MM, middle_extra=MIDDLE_EXTRA_MM):
    out = []
    for i, rect in enumerate(OBSTACLES):
        extra = middle_extra if i == MIDDLE_INDEX else 0
        out.append(_pad(rect, pad + extra))
    return out


def _clearance(x, y, boxes):
    """Distance from a point to the nearest inflated obstacle edge."""
    best = float("inf")
    for r in boxes:
        if _inside(r, x, y):
            return -1.0
        cx = min(max(x, r[0]), r[2])
        cy = min(max(y, r[1]), r[3])
        best = min(best, math.hypot(x - cx, y - cy))
    return best


def _point_free(x, y, boxes):
    return all(not _inside(r, x, y) for r in boxes)


def _segment_free(x0, y0, x1, y1, boxes):
    return all(not _segment_hits_rect(r, x0, y0, x1, y1) for r in boxes)


# ------------------------------------------------------------------ A* search
class Grid:
    def __init__(self, step_mm=10, boxes=None):
        self.step = step_mm
        self.boxes = boxes if boxes is not None else _inflated()
        self.nx = ARENA_W // step_mm + 1
        self.ny = ARENA_H // step_mm + 1
        self.free = bytearray(self.nx * self.ny)
        for iy in range(self.ny):
            for ix in range(self.nx):
                if _point_free(ix * step_mm, iy * step_mm, self.boxes):
                    self.free[iy * self.nx + ix] = 1

    def idx(self, x, y):
        ix = min(max(int(round(x / self.step)), 0), self.nx - 1)
        iy = min(max(int(round(y / self.step)), 0), self.ny - 1)
        return iy * self.nx + ix

    def xy(self, n):
        return (n % self.nx) * self.step, (n // self.nx) * self.step

    def ok(self, n):
        return bool(self.free[n])

    def nearest_free(self, x, y):
        n = self.idx(x, y)
        if self.ok(n):
            return n
        cx, cy = self.xy(n)
        best, best_d = None, float("inf")
        for iy in range(max(0, n // self.nx - 8), min(self.ny, n // self.nx + 9)):
            for ix in range(max(0, n % self.nx - 8), min(self.nx, n % self.nx + 9)):
                m = iy * self.nx + ix
                if not self.ok(m):
                    continue
                d = math.hypot(ix * self.step - cx, iy * self.step - cy)
                if d < best_d:
                    best, best_d = m, d
        return best


STEPS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, math.sqrt(2)), (1, -1, math.sqrt(2)),
         (-1, 1, math.sqrt(2)), (1, 1, math.sqrt(2))]


def astar(grid, start_xy, goal_xy):
    start = grid.nearest_free(*start_xy)
    goal = grid.nearest_free(*goal_xy)
    if start is None or goal is None:
        raise SystemExit("start or goal is fully blocked")
    gx, gy = grid.xy(goal)
    cost = {start: 0.0}
    parent = {start: None}
    heap = [(math.hypot(gx - start_xy[0], gy - start_xy[1]), 0.0, start)]
    seen = set()
    while heap:
        _, g, node = heapq.heappop(heap)
        if node in seen:
            continue
        seen.add(node)
        if node == goal:
            break
        for dx, dy, w in STEPS:
            ix = node % grid.nx + dx
            iy = node // grid.nx + dy
            if ix < 0 or iy < 0 or ix >= grid.nx or iy >= grid.ny:
                continue
            nxt = iy * grid.nx + ix
            if not grid.ok(nxt):
                continue
            ng = g + w * grid.step
            if ng < cost.get(nxt, float("inf")):
                cost[nxt] = ng
                parent[nxt] = node
                nx, ny = grid.xy(nxt)
                heapq.heappush(heap, (ng + math.hypot(gx - nx, gy - ny), ng, nxt))
    if goal not in parent:
        raise SystemExit("no path found")
    path = []
    node = goal
    while node is not None:
        path.append(grid.xy(node))
        node = parent[node]
    path.reverse()
    return path


def shortcut(path, boxes):
    """Greedy line-of-sight shortening: jump to the farthest visible point."""
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _segment_free(path[i][0], path[i][1],
                                              path[j][0], path[j][1], boxes):
            j -= 1
        out.append(path[j])
        i = j
    return out


def resample(path, step_mm=15.0):
    """Densify so the MCU follower (45 mm advance window) sees a smooth curve."""
    out = [path[0]]
    for a, b in zip(path, path[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(round(d / step_mm)))
        for k in range(1, n + 1):
            t = k / n
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return out


def route_length(path):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def route_min_clearance(path, boxes):
    worst = float("inf")
    for a, b in zip(path, path[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(d / 5.0))
        for k in range(n + 1):
            t = k / n
            worst = min(worst, _clearance(a[0] + (b[0] - a[0]) * t,
                                          a[1] + (b[1] - a[1]) * t, boxes))
    return worst


def leg_of(a, b):
    """Route leg tag used by the firmware: 1=north, 2=east, 3=south."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    if abs(dx) >= abs(dy):
        return 2
    return 1 if dy > 0 else 3


def leg_tags(points):
    tags = [leg_of(a, b) for a, b in zip(points, points[1:])]
    return tags + [tags[-1] if tags else 1]


def report(path, boxes, label):
    print(f"{label}: {len(path)} points, length {route_length(path):.1f} mm, "
          f"min clearance {route_min_clearance(path, boxes):.1f} mm")
    blocked = [p for p in path if not _point_free(p[0], p[1], boxes)]
    print(f"  points inside an inflated obstacle: {len(blocked)}")
    bad = [(a, b) for a, b in zip(path, path[1:])
           if not _segment_free(a[0], a[1], b[0], b[1], boxes)]
    print(f"  segments crossing an inflated obstacle: {len(bad)}")
    for a, b in bad[:6]:
        print(f"    {a} -> {b}")


def emit_header(path, name="course_route"):
    tags = leg_tags(path)
    lines = ["/* Generated by Tools/plan_route.py - A* shortest path, avoid mode. */",
             "#ifndef COURSE_ROUTE_PLANNED_H", "#define COURSE_ROUTE_PLANNED_H",
             "static const int16_t course_route_planned[][3] = {"]
    for (x, y), tag in zip(path, tags):
        lines.append(f"    {{{int(round(x))}, {int(round(y))}, {tag}}},")
    lines += ["};", "#endif"]
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("command", choices=["validate", "plan"])
    ap.add_argument("--step", type=int, default=10)
    ap.add_argument("--inflate", type=int, default=INFLATE_MM)
    ap.add_argument("--middle-extra", type=int, default=MIDDLE_EXTRA_MM)
    ap.add_argument("--spacing", type=float, default=15.0)
    ap.add_argument("--emit-header", action="store_true")
    args = ap.parse_args()

    boxes = _inflated(args.inflate, args.middle_extra)
    print(f"inflation {args.inflate} mm, middle extra {args.middle_extra} mm "
          f"(middle block inflated by {args.inflate + args.middle_extra} mm)")

    # The table in course_route.h starts at the first waypoint, so the entry
    # segment from the start pose is prepended here for a fair comparison.
    current_full = [START_NOSE] + CURRENT_ROUTE

    if args.command == "validate":
        report(current_full, boxes, "current route (with start entry)")
        return

    grid = Grid(args.step, boxes)
    raw = astar(grid, START_ENTRY, FINISH_ENTRY)
    print(f"  raw A* path: {len(raw)} nodes, {route_length(raw):.1f} mm")
    path = shortcut(raw, boxes)
    print(f"  after line-of-sight shortcut: {len(path)} points, "
          f"{route_length(path):.1f} mm")
    path = resample(path, args.spacing)
    path = [START_NOSE] + path + [FINISH_NOSE]
    report(path, boxes, "planned route")
    print(f"  current route: {route_length(current_full):.1f} mm")
    print(f"  saving: {route_length(current_full) - route_length(path):+.1f} mm")
    if args.emit_header:
        print()
        print(emit_header(path))


if __name__ == "__main__":
    main()
