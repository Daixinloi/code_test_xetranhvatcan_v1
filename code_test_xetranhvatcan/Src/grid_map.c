#include "grid_map.h"
#include <math.h>
#include <string.h>

/* Millimetres. Traced from the supplied picture, NOT a surveyed map.
 * Origin bottom-left, +X right, +Y up. Rectangles are closed for collision. */
const GridRect grid_static_obstacles[18] = {
    {0,100,100,200}, {500,200,600,300}, {250,300,350,400},
    {500,400,600,500}, {0,500,100,600}, {0,700,100,800},
    {300,850,400,950}, {400,1100,500,1200}, {600,700,700,800},
    {700,600,800,700}, {800,950,900,1050}, {900,600,1000,700},
    {1100,1100,1200,1200}, {1200,800,1300,900},
    {1400,400,1500,500}, {1400,100,1500,200}, {1700,200,1800,300},
    {600,0,1200,600}
};

static bool Get(const uint8_t *bits, unsigned n) { return (bits[n / 8] >> (n % 8)) & 1u; }
static void Set(uint8_t *bits, unsigned n) { bits[n / 8] |= (uint8_t)(1u << (n % 8)); }

static bool Contains(GridRect r, int x, int y, int inflation)
{
    /* Square expansion conservatively contains the circular turn footprint. */
    return x >= r.x0 - inflation && x <= r.x1 + inflation &&
           y >= r.y0 - inflation && y <= r.y1 + inflation;
}

static void Inflate(uint8_t *blocked, GridRect r, int pad)
{
    for (unsigned n = 0; n < GRID_NODES; ++n) {
        int x = (n % GRID_WIDTH) * GRID_CELL_MM + GRID_CELL_MM / 2;
        int y = (n / GRID_WIDTH) * GRID_CELL_MM + GRID_CELL_MM / 2;
        if (Contains(r, x, y, pad)) Set(blocked, n);
    }
}

void Grid_Init(GridMap *m, uint16_t inflation_mm)
{
    memset(m, 0, sizeof(*m));
    m->inflation_mm = inflation_mm;
    m->turn_padding_mm = inflation_mm;
    m->result = GRID_NO_PATH;
    /* Boundary fence. Stay inside even at the open start/finish lines. */
    for (unsigned n = 0; n < GRID_NODES; ++n) {
        int x = (n % GRID_WIDTH) * GRID_CELL_MM + GRID_CELL_MM / 2;
        int y = (n / GRID_WIDTH) * GRID_CELL_MM + GRID_CELL_MM / 2;
        if (x <= inflation_mm || y <= inflation_mm ||
            x >= GRID_WIDTH * GRID_CELL_MM - inflation_mm ||
            y >= GRID_HEIGHT * GRID_CELL_MM - inflation_mm) Set(m->blocked, n);
    }
    for (unsigned i = 0; i < 18; ++i) Inflate(m->blocked, grid_static_obstacles[i], inflation_mm);
    memcpy(m->static_blocked, m->blocked, sizeof(m->static_blocked));
}

void Grid_SetMiddleClearance(GridMap *m, uint16_t extra_mm)
{
    m->middle_extra_mm = extra_mm;
    Inflate(m->blocked, grid_static_obstacles[17], m->inflation_mm + extra_mm);
    Inflate(m->static_blocked, grid_static_obstacles[17], m->inflation_mm + extra_mm);
}

uint16_t Grid_Cell(float x, float y)
{
    if (!isfinite(x) || !isfinite(y) || x < 0 || y < 0 ||
        x >= GRID_WIDTH * GRID_CELL_MM || y >= GRID_HEIGHT * GRID_CELL_MM) return GRID_NONE;
    return (uint16_t)((int)(y / GRID_CELL_MM) * GRID_WIDTH + (int)(x / GRID_CELL_MM));
}

void Grid_SetFrontSensors(GridMap *m, const int16_t forward[3], const int16_t left[3],
    const int16_t angles[3], uint16_t center_mm, uint16_t diagonal_mm)
{
    const int c[4] = {1,0,-1,0}, s[4] = {0,1,0,-1};
    for (unsigned i = 0; i < 3; ++i) {
        float angle = angles[i] * 0.01745329252f;
        uint16_t minimum = i == 1 ? center_mm : diagonal_mm;
        float ex = forward[i] + minimum * cosf(angle);
        float ey = left[i] + minimum * sinf(angle);
        for (unsigned h = 0; h < 4; ++h) {
            float *r = m->front_rays[h][i];
            r[0] = forward[i]*c[h] - left[i]*s[h];
            r[1] = forward[i]*s[h] + left[i]*c[h];
            r[2] = ex*c[h] - ey*s[h]; r[3] = ex*s[h] + ey*c[h];
        }
    }
    memset(m->front_checked, 0, sizeof(m->front_checked));
    memset(m->front_unsafe, 0, sizeof(m->front_unsafe));
    m->front_enabled = true;
}

bool Grid_IsBlocked(const GridMap *m, uint16_t n)
{
    return n >= GRID_NODES || Get(m->blocked, n);
}

static GridRect ObservedBox(const GridMap *m, unsigned cell)
{
    int x = cell % GRID_WIDTH * GRID_CELL_MM;
    int y = cell / GRID_WIDTH * GRID_CELL_MM;
    unsigned quarters = m->observed_quarters[cell];
    return (GridRect){
        (int16_t)(x + ((quarters & 0x5) ? 0 : GRID_CELL_MM / 2)),
        (int16_t)(y + ((quarters & 0x3) ? 0 : GRID_CELL_MM / 2)),
        (int16_t)(x + ((quarters & 0xa) ? GRID_CELL_MM : GRID_CELL_MM / 2)),
        (int16_t)(y + ((quarters & 0xc) ? GRID_CELL_MM : GRID_CELL_MM / 2))
    };
}

bool Grid_AddHit(GridMap *m, float x, float y)
{
    uint16_t n = Grid_Cell(x, y);
    if (n == GRID_NONE) return false;
    unsigned quarter = 1u << ((((int)y % GRID_CELL_MM >= GRID_CELL_MM / 2) ? 2 : 0) +
                              ((int)x % GRID_CELL_MM >= GRID_CELL_MM / 2));
    if (m->observed_quarters[n] & quarter) return false;
    for (unsigned i = 0; i < 18; ++i)
        if (Contains(grid_static_obstacles[i], (int)x, (int)y, 10)) return false;
    if (!m->observed_quarters[n]) m->observed_cells[m->observed_count++] = n;
    m->observed_quarters[n] |= (uint8_t)quarter;
    memset(m->front_checked, 0, sizeof(m->front_checked));
    memset(m->front_unsafe, 0, sizeof(m->front_unsafe));
    Inflate(m->blocked, ObservedBox(m, n), m->inflation_mm);
    return true;
}

void Grid_ClearHits(GridMap *m)
{
    memcpy(m->blocked, m->static_blocked, sizeof(m->blocked));
    memset(m->observed_quarters, 0, sizeof(m->observed_quarters));
    memset(m->front_checked, 0, sizeof(m->front_checked));
    memset(m->front_unsafe, 0, sizeof(m->front_unsafe));
    m->observed_count = m->path_length = m->expanded = 0;
    m->result = GRID_NO_PATH;
}

static bool VerticalIntersects(GridRect r, int pad, float x, float y0, float y1)
{
    return x >= r.x0 - pad && x <= r.x1 + pad &&
           y1 >= r.y0 - pad && y0 <= r.y1 + pad;
}

static bool VerticalCorridorClear(const GridMap *m, float x, float low, float high,
    int padding, uint16_t middle_extra)
{
    for (unsigned i = 0; i < 18; ++i) {
        if (i < GRID_MOVABLE_COUNT && (m->live_clear_mask & (1u << i))) continue;
        int pad = padding + (i == 17 ? middle_extra : 0);
        if (VerticalIntersects(grid_static_obstacles[i], pad, x, low, high)) return false;
    }
    for (unsigned k = 0; k < m->observed_count; ++k) {
        GridRect box = ObservedBox(m, m->observed_cells[k]);
        if (VerticalIntersects(box, padding, x, low, high)) return false;
    }
    return true;
}

bool Grid_FindStartEntry(const GridMap *m, float x, float y, float *entry_y)
{
    if (!isfinite(x) || !isfinite(y) || !entry_y || y > 0 || y < -400 ||
        x <= m->turn_padding_mm || x >= 600 - m->turn_padding_mm) return false;
    float target = GRID_CELL_MM / 2;
    while (target <= m->turn_padding_mm) target += GRID_CELL_MM;
    if (Grid_IsBlocked(m, Grid_Cell(x, target))) return false;
    /* Unlike the bottom fence, T1 and every observed obstacle still apply
     * during entry. Never move first and discover an impossible route later. */
    if (!VerticalCorridorClear(m, x, y, target, m->turn_padding_mm, m->middle_extra_mm)) return false;
    *entry_y = target;
    return true;
}

bool Grid_CanFinish(const GridMap *m, float x, float y, float exit_y)
{
    return Grid_CanFinishWithPadding(m, x, y, exit_y, m->turn_padding_mm);
}

bool Grid_CanFinishWithPadding(const GridMap *m, float x, float y, float exit_y, uint16_t pad)
{
    return Grid_CanFinishWithClearance(m, x, y, exit_y, pad, m->middle_extra_mm);
}

bool Grid_CanFinishWithClearance(const GridMap *m, float x, float y, float exit_y,
    uint16_t pad, uint16_t middle_extra)
{
    return isfinite(x) && isfinite(y) && isfinite(exit_y) && y >= exit_y &&
        x > 1200 + pad + middle_extra &&
        x < GRID_WIDTH * GRID_CELL_MM - pad &&
        VerticalCorridorClear(m, x, exit_y, y, pad, middle_extra);
}

static bool SegmentHits(GridRect r, int pad, float x, float y, float dx, float dy)
{
    float enter = 0, leave = 1;
    float start[2] = {x, y}, delta[2] = {dx, dy};
    float low[2] = {r.x0 - pad, r.y0 - pad}, high[2] = {r.x1 + pad, r.y1 + pad};
    for (unsigned axis = 0; axis < 2; ++axis) {
        if (fabsf(delta[axis]) < 0.0001f) {
            if (start[axis] < low[axis] || start[axis] > high[axis]) return false;
        } else {
            float a = (low[axis] - start[axis]) / delta[axis];
            float b = (high[axis] - start[axis]) / delta[axis];
            if (a > b) { float t = a; a = b; b = t; }
            enter = fmaxf(enter, a); leave = fminf(leave, b);
            if (enter > leave) return false;
        }
    }
    return true;
}

static bool BoxesOverlap(GridRect r, int pad, int x0, int y0, int x1, int y1)
{
    return r.x1+pad >= x0 && r.x0-pad <= x1 && r.y1+pad >= y0 && r.y0-pad <= y1;
}

bool Grid_CanTravel(const GridMap *m, float x0, float y0, float x1, float y1)
{
    return Grid_CanTravelWithTolerance(m, x0, y0, x1, y1, 0);
}

static bool SegmentClear(const GridMap *m, int pad, float x0, float y0, float x1, float y1,
    uint16_t middle_extra)
{
    int minx=(int)(x0<x1?x0:x1)-1, maxx=(int)(x0>x1?x0:x1)+1;
    int miny=(int)(y0<y1?y0:y1)-1, maxy=(int)(y0>y1?y0:y1)+1;
    for (unsigned i = 0; i < 18; ++i) {
        if (i < GRID_MOVABLE_COUNT && (m->live_clear_mask & (1u << i))) continue;
        int obstacle_pad = pad + (i == 17 ? middle_extra : 0);
        if (BoxesOverlap(grid_static_obstacles[i],obstacle_pad,minx,miny,maxx,maxy) &&
            SegmentHits(grid_static_obstacles[i], obstacle_pad, x0, y0, x1-x0, y1-y0)) return false;
    }
    for (unsigned k = 0; k < m->observed_count; ++k) {
        GridRect r = ObservedBox(m, m->observed_cells[k]);
        if (BoxesOverlap(r,pad,minx,miny,maxx,maxy) &&
            SegmentHits(r, pad, x0, y0, x1-x0, y1-y0)) return false;
    }
    return true;
}

bool Grid_CanEnter(const GridMap *m, float x0, float y0, float x1, float y1)
{
    int pad = m->turn_padding_mm;
    return isfinite(x0) && isfinite(y0) && isfinite(x1) && isfinite(y1) &&
        fminf(x0,x1) > pad && fmaxf(x0,x1) < grid_static_obstacles[17].x0 - pad &&
        fminf(y0,y1) >= -400.0f && fmaxf(y0,y1) < GRID_HEIGHT * GRID_CELL_MM - pad &&
        SegmentClear(m, pad, x0, y0, x1, y1, m->middle_extra_mm);
}

bool Grid_CanTravelWithTolerance(const GridMap *m, float x0, float y0,
    float x1, float y1, int16_t tolerance_mm)
{
    return Grid_CanTravelWithClearance(m, x0, y0, x1, y1,
        m->inflation_mm - tolerance_mm, m->middle_extra_mm);
}

bool Grid_CanTravelWithClearance(const GridMap *m, float x0, float y0,
    float x1, float y1, int pad, uint16_t middle_extra)
{
    if (pad < 0) return false;
    return isfinite(x0) && isfinite(y0) && isfinite(x1) && isfinite(y1) &&
        fminf(x0,x1) > pad && fminf(y0,y1) > pad &&
        fmaxf(x0,x1) < GRID_WIDTH * GRID_CELL_MM - pad &&
        fmaxf(y0,y1) < GRID_HEIGHT * GRID_CELL_MM - pad &&
        SegmentClear(m, pad, x0, y0, x1, y1, middle_extra);
}

bool Grid_CanRotate(const GridMap *m, float x, float y)
{
    return Grid_CanTravelWithTolerance(m, x, y, x, y, m->inflation_mm - m->turn_padding_mm);
}

static bool FrontClear(const GridMap *m, float x, float y, unsigned heading)
{
    for (unsigned ray = 0; ray < 3; ++ray) {
        const float *r = m->front_rays[heading][ray];
        float sx = x+r[0], sy = y+r[1], dx = r[2]-r[0], dy = r[3]-r[1];
        float ex=x+r[2], ey=y+r[3];
        int minx=(int)(sx<ex?sx:ex)-1,maxx=(int)(sx>ex?sx:ex)+1;
        int miny=(int)(sy<ey?sy:ey)-1,maxy=(int)(sy>ey?sy:ey)+1;
        for (unsigned i = 0; i < 18; ++i)
            if (BoxesOverlap(grid_static_obstacles[i],0,minx,miny,maxx,maxy) &&
                SegmentHits(grid_static_obstacles[i], 0, sx, sy, dx, dy)) return false;
        for (unsigned k = 0; k < m->observed_count; ++k) {
            GridRect box = ObservedBox(m, m->observed_cells[k]);
            if (BoxesOverlap(box,0,minx,miny,maxx,maxy) && SegmentHits(box,0,sx,sy,dx,dy)) return false;
        }
    }
    return true;
}

static bool FrontAtCell(GridMap *m, unsigned cell, unsigned heading)
{
    unsigned bit = heading*GRID_NODES + cell;
    if (!Get(m->front_checked, bit)) {
        float x = cell % GRID_WIDTH * GRID_CELL_MM + GRID_CELL_MM/2;
        float y = cell / GRID_WIDTH * GRID_CELL_MM + GRID_CELL_MM/2;
        if (!FrontClear(m,x,y,heading)) Set(m->front_unsafe, bit);
        Set(m->front_checked, bit);
    }
    return !Get(m->front_unsafe, bit);
}

bool Grid_CanStep(GridMap *m, uint16_t from, uint16_t to)
{
    if (Grid_IsBlocked(m, from) || Grid_IsBlocked(m, to)) return false;
    int dx = (int)(to % GRID_WIDTH) - (int)(from % GRID_WIDTH);
    int dy = (int)(to / GRID_WIDTH) - (int)(from / GRID_WIDTH);
    if ((dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy) != 1) return false;
    if (!m->front_enabled) return true;
    unsigned heading = dx > 0 ? 0 : dy > 0 ? 1 : dx < 0 ? 2 : 3;
    if (!FrontAtCell(m,from,heading) || !FrontAtCell(m,to,heading)) return false;
    float mx = ((from % GRID_WIDTH + to % GRID_WIDTH) * GRID_CELL_MM + GRID_CELL_MM) * 0.5f;
    float my = ((from / GRID_WIDTH + to / GRID_WIDTH) * GRID_CELL_MM + GRID_CELL_MM) * 0.5f;
    return FrontClear(m,mx,my,heading);
}

static unsigned Heuristic(unsigned a, unsigned b)
{
    int dx = (int)(a % GRID_WIDTH) - (int)(b % GRID_WIDTH);
    int dy = (int)(a / GRID_WIDTH) - (int)(b / GRID_WIDTH);
    return (unsigned)((dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy));
}

GridResult Grid_Begin(GridMap *m, uint16_t start, uint16_t goal)
{
    return Grid_BeginAbove(m, start, goal, 0);
}

GridResult Grid_BeginAbove(GridMap *m, uint16_t start, uint16_t goal, uint8_t min_row)
{
    m->path_length = m->expanded = 0;
    m->min_row = min_row;
    if (Grid_IsBlocked(m, start) || Grid_IsBlocked(m, goal) ||
        start / GRID_WIDTH < min_row || goal / GRID_WIDTH < min_row)
        return m->result = GRID_BAD_POSE;
    memset(m->node_state, 0, sizeof(m->node_state));
    for (unsigned i = 0; i < GRID_NODES; ++i) m->cost[i] = m->parent[i] = GRID_NONE;
    m->start = start; m->goal = goal;
    m->cost[start] = 0; m->node_state[start] = 1;
    return m->result = GRID_SEARCHING;
}

GridResult Grid_Advance(GridMap *m, unsigned budget)
{
    if (m->front_enabled && budget > 1) budget = 1;
    while (budget-- && m->result == GRID_SEARCHING) {
        unsigned best = GRID_NONE, best_f = UINT16_MAX, best_h = UINT16_MAX;
        for (unsigned n = 0; n < GRID_NODES; ++n) {
            if (m->node_state[n] != 1) continue;
            unsigned h = Heuristic(n, m->goal), f = m->cost[n] + h;
            if (f < best_f || (f == best_f && h < best_h)) {
                best = n; best_f = f; best_h = h;
            }
        }
        if (best == GRID_NONE) return m->result = GRID_NO_PATH;
        if (best == m->goal) {
            unsigned n = best;
            do {
                if (m->path_length >= GRID_NODES) return m->result = GRID_NO_PATH;
                m->path[m->path_length++] = (uint16_t)n;
                n = m->parent[n];
            } while (n != GRID_NONE);
            for (unsigned i = 0; i < m->path_length / 2; ++i) {
                uint16_t t = m->path[i];
                m->path[i] = m->path[m->path_length - 1 - i];
                m->path[m->path_length - 1 - i] = t;
            }
            return m->result = GRID_FOUND;
        }
        m->node_state[best] = 2; ++m->expanded;
        int neighbours[4] = {
            best % GRID_WIDTH ? (int)best - 1 : -1,
            best % GRID_WIDTH < GRID_WIDTH - 1 ? (int)best + 1 : -1,
            best >= GRID_WIDTH ? (int)best - GRID_WIDTH : -1,
            best < GRID_NODES - GRID_WIDTH ? (int)best + GRID_WIDTH : -1
        };
        for (unsigned i = 0; i < 4; ++i) {
            int next = neighbours[i];
            if (next < 0 || Grid_IsBlocked(m, (uint16_t)next) || m->node_state[next] == 2) continue;
            if ((unsigned)next / GRID_WIDTH < m->min_row) continue;
            if (!Grid_CanStep(m, (uint16_t)best, (uint16_t)next)) continue;
            if (m->turn_padding_mm != m->inflation_mm && best != m->start &&
                (int)best - m->parent[best] != next - (int)best &&
                !Grid_CanRotate(m, best % GRID_WIDTH * GRID_CELL_MM + GRID_CELL_MM / 2,
                    best / GRID_WIDTH * GRID_CELL_MM + GRID_CELL_MM / 2)) continue;
            if (m->cost[best] + 1 < m->cost[next]) {
                m->cost[next] = m->cost[best] + 1;
                m->parent[next] = (uint16_t)best;
                m->node_state[next] = 1;
            }
        }
    }
    return m->result;
}
