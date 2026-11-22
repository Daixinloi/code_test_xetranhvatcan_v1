"""Shortest course route on a heading-aware, 5 mm grid."""

import heapq
import math


STEP_MM = 5
MIN_TURN_RUN_MM = 25
EDGE_MM = 100
STRAIGHT_PAD_MM = 65
TURN_PAD_MM = 74
DIAGONAL_PAD_MM = 82
MIDDLE_MARGIN_MM = 50
CENTERED_MIDDLE_MARGIN_MM = 15
T14_PASS_Y_MM = 696
T9_SIDE_MARGIN_MM = 10
FINISH_Y_MM = 125
FINISH_EXIT_Y_MM = -55
MOVES = ((0, 1), (1, 1), (1, 0), (1, -1),
         (0, -1), (-1, -1), (-1, 0), (-1, 1))


def obstacle_margin(index, total, pad):
    if index == total - 1:
        return pad + MIDDLE_MARGIN_MM
    if index == 8 and pad == STRAIGHT_PAD_MM:
        return pad + T9_SIDE_MARGIN_MM
    return pad


def segment_clear(start, end, obstacles, pad=None, middle_margin=MIDDLE_MARGIN_MM):
    dx, dy = end[0] - start[0], end[1] - start[1]
    if pad is None:
        pad = DIAGONAL_PAD_MM if dx and dy else STRAIGHT_PAD_MM
    for index, rect in enumerate(obstacles):
        margin = (pad + middle_margin if index == len(obstacles) - 1
                  else obstacle_margin(index, len(obstacles), pad))
        enter, leave = 0.0, 1.0
        for position, delta, low, high in zip(start, (dx, dy), rect[:2], rect[2:]):
            low -= margin
            high += margin
            if delta:
                a, b = sorted(((low - position) / delta, (high - position) / delta))
                enter, leave = max(enter, a), min(leave, b)
            elif not low <= position <= high:
                enter = math.inf
            if enter > leave:
                break
        if enter <= leave:
            return False
    return True


def rounded_route(points, obstacles):
    """Return (x, y, rolling) waypoints with checked circular fillets."""
    points = list(points)
    shifted = (496, 770)
    if len(points) > 3 and points[2] == (490, 770) and all(
        segment_clear(shifted, points[i], obstacles) for i in (1, 3)
    ):
        # Shift the tangent away from the east face of T7.
        points[2] = shifted
    result = [(*points[0], 0)]
    for a, b, c in zip(points, points[1:], points[2:]):
        incoming, outgoing = math.dist(a, b), math.dist(b, c)
        ux, uy = (b[0] - a[0]) / incoming, (b[1] - a[1]) / incoming
        vx, vy = (c[0] - b[0]) / outgoing, (c[1] - b[1]) / outgoing
        angle = math.atan2(ux * vy - uy * vx, ux * vx + uy * vy)
        radius = 100
        cut = radius * math.tan(abs(angle) * 0.5)
        if abs(angle) < math.radians(12) or cut > 0.45 * min(incoming, outgoing):
            result.append((*b, 0))
            continue
        tx, ty = b[0] - ux * cut, b[1] - uy * cut
        sign = math.copysign(1, angle)
        cx, cy = tx - uy * radius * sign, ty + ux * radius * sign
        start_angle = math.atan2(ty - cy, tx - cx)
        steps = math.ceil(abs(angle) / math.radians(12))
        arc = [(round(cx + radius * math.cos(start_angle + angle * i / steps)),
                round(cy + radius * math.sin(start_angle + angle * i / steps)))
               for i in range(steps + 1)]
        approach_clear = segment_clear(result[-1][:2], arc[0], obstacles)
        departure_clear = segment_clear(arc[-1], c, obstacles)
        if approach_clear and departure_clear and all(
            segment_clear(p, q, obstacles, DIAGONAL_PAD_MM + 4) for p, q in zip(arc, arc[1:])
        ):
            result.extend((*p, 1) for p in arc)
        else:
            result.append((*b, 0))
    return result + [(*points[-1], 0)]


def tight_segment_clear(a, b, obstacles):
    angle = abs(math.remainder(math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])), 90))
    angle = math.radians(min(45, angle + 3))
    reach = 45 * (math.cos(angle) + math.sin(angle))
    return segment_clear(a, b, obstacles, math.ceil(reach + 5), CENTERED_MIDDLE_MARGIN_MM)


def inside_connector():
    x, y = 968, 880
    level, r1, r2 = T14_PASS_Y_MM, 90, 85
    angle, count = math.radians(70), 8
    run = (y - level - (r1 + r2) * (1 - math.cos(angle))) / math.sin(angle)
    points = [(x + r1 * math.sin(angle * i / count),
               y - r1 * (1 - math.cos(angle * i / count))) for i in range(count + 1)]
    px, py = points[-1]
    px += run * math.cos(angle)
    py -= run * math.sin(angle)
    points.extend((px + r2 * (math.sin(angle) - math.sin(angle * (1 - i / count))),
                   py - r2 * (math.cos(angle * (1 - i / count)) - math.cos(angle)))
                  for i in range(count + 1))
    return [(round(a), round(b)) for a, b in points]


def finish_connector():
    level, lane, radius = T14_PASS_Y_MM, 1300, 44
    return [(round(lane - radius + radius * math.sin(math.pi * i / 18)),
             round(level - radius * (1 - math.cos(math.pi * i / 18)))) for i in range(10)]


def course_with_curves(points, obstacles):
    rounded = rounded_route(points, obstacles)
    if (515, 880, 0) not in rounded:
        return rounded
    split = rounded.index((515, 880, 0))
    # Hold north a little longer before the first arc, matching the floor test.
    entry = [(x, y + 4 if marker == 1 and 640 <= y <= 702 else y, marker)
             for x, y, marker in rounded[:split]]
    ax, ay = rounded[split - 1][:2]
    angle = math.atan2(880 - ay, 503 - ax)
    radius = 58
    cut = radius * math.tan(angle / 2)
    cx = 503 - cut * math.cos(angle) + radius * math.sin(angle)
    cy = 880 - cut * math.sin(angle) - radius * math.cos(angle)
    count = math.ceil(math.degrees(angle) / 10)
    arc = [(round(cx + radius * math.cos(math.pi / 2 + angle * (1 - i / count))),
            round(cy + radius * math.sin(math.pi / 2 + angle * (1 - i / count))))
           for i in range(count + 1)]
    # While heading east, ease right into the lane below T11.
    inner = arc + [(780, 872), (912, 875)] + inside_connector() + finish_connector()
    goal = inner[-1][0]
    tail = [(goal, FINISH_Y_MM), (goal, FINISH_EXIT_Y_MM)]
    check = [(ax, ay)] + inner + tail
    if all(tight_segment_clear(a, b, obstacles) for a, b in zip(check, check[1:])):
        return entry + [(*p, 2) for p in inner] + [(*tail[0], 3), (*tail[1], 0)]
    return rounded


def simplify(points, obstacles):
    # Keep the fixed entry and the south-facing final segment.
    result = points[:1]
    index = 1
    while index < len(points) - 3:
        result.append(points[index])
        index = next(end for end in range(len(points) - 3, index, -1)
                     if segment_clear(points[index], points[end], obstacles))
    result += points[index:]
    compact = result[:1]
    for a, b, c in zip(result, result[1:], result[2:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        ex, ey = c[0] - b[0], c[1] - b[1]
        if dx * ey != dy * ex or dx * ex + dy * ey < 0:
            compact.append(b)
        elif b[1] == FINISH_Y_MM:
            compact.append(b)
    return compact + result[-1:]


def shortest_route(width, height, obstacles, start):
    step = STEP_MM
    cols = width // step + 1
    rows = height // step + 1

    def blocked(pad):
        mask = bytearray(cols * rows)
        for index, (x0, y0, x1, y1) in enumerate(obstacles):
            margin = obstacle_margin(index, len(obstacles), pad)
            left = max(0, math.ceil((x0 - margin) / step))
            right = min(cols - 1, (x1 + margin) // step)
            fill = b"\x01" * (right - left + 1)
            for row in range(max(0, math.ceil((y0 - margin) / step)),
                             min(rows - 1, (y1 + margin) // step) + 1):
                first = row * cols + left
                mask[first:first + len(fill)] = fill
        return mask

    straight, turn, diagonal = (blocked(pad) for pad in
                                (STRAIGHT_PAD_MM, TURN_PAD_MM, DIAGONAL_PAD_MM))
    limit = EDGE_MM // step
    finish_row = FINISH_Y_MM // step
    finish_columns = [
        col for col in range(1200 // step, cols - limit)
        if all(not turn[row * cols + col] for row in range(finish_row + 1))
    ]
    if not finish_columns:
        raise ValueError("Không có cửa đích đủ rộng")
    low_finish, high_finish = finish_columns[0], finish_columns[-1]

    def estimate(col, row):
        lateral = max(low_finish - col, col - high_finish, 0)
        return step * math.hypot(lateral, row - finish_row)

    initial_col = int(start[0]) // step
    initial_cell = limit * cols + initial_col
    initial = initial_cell * 8
    if start[0] % step or any(turn[row * cols + initial_col] for row in range(limit + 1)):
        raise ValueError("Làn qua vạch xuất phát bị chắn hoặc lệch lưới")
    queue = [(estimate(initial_col, limit), 0.0, initial)]
    best = {initial: 0.0}
    parent = {}
    finish_columns = set(finish_columns)
    while queue:
        _, distance, key = heapq.heappop(queue)
        if distance != best[key]:
            continue
        cell, heading = divmod(key, 8)
        row, col = divmod(cell, cols)
        if row == finish_row and heading == 4 and col in finish_columns:
            cells = [(col * step, row * step)]
            while key != initial:
                key = parent[key]
                row, col = divmod(key // 8, cols)
                cells.append((col * step, row * step))
            points = [tuple(start[:2])]
            ordered = cells[::-1]
            for a, b, c in zip(ordered, ordered[1:], ordered[2:]):
                if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]):
                    points.append(b)
            points.insert(1, ordered[0])
            points.extend((ordered[-1], (ordered[-1][0], FINISH_EXIT_Y_MM)))
            return simplify(points, obstacles)
        for direction, (dx, dy) in enumerate(MOVES):
            if heading != direction and turn[cell]:
                continue
            run = MIN_TURN_RUN_MM // step if heading != direction else 1
            nc, nr = col + run * dx, row + run * dy
            if not (limit <= nc < cols - limit and limit <= nr < rows - limit):
                continue
            destination = nr * cols + nc
            stride = dy * cols + dx
            if direction & 1 and any(
                diagonal[cell + stride * i + offset]
                for i in range(run) for offset in (0, stride, dx, dy * cols)
            ):
                continue
            if not direction & 1 and any(straight[cell + stride * i]
                                         for i in range(1, run + 1)):
                continue
            updated = distance + run * (step * math.sqrt(2) if direction & 1 else step)
            target = destination * 8 + direction
            if updated >= best.get(target, math.inf):
                continue
            best[target] = updated
            parent[target] = key
            heapq.heappush(queue, (updated + estimate(nc, nr), updated, target))
    raise ValueError("Không tìm được đường qua sân theo khoảng hở đã chọn")
