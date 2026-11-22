#include "grid_map.h"
#include "navigation_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #c); exit(1); } } while (0)
static GridMap map;

/* Independent BFS oracle: A* must produce a shortest cardinal path. */
static unsigned BFS(unsigned start, unsigned goal)
{
    uint16_t queue[GRID_NODES], dist[GRID_NODES];
    memset(dist, 0xff, sizeof(dist));
    unsigned head = 0, tail = 0;
    queue[tail++] = start; dist[start] = 0;
    while (head < tail) {
        unsigned u = queue[head++];
        if (u == goal) return dist[u];
        for (unsigned v = 0; v < GRID_NODES; ++v) {
            int dx = abs((int)(u % GRID_WIDTH) - (int)(v % GRID_WIDTH));
            int dy = abs((int)(u / GRID_WIDTH) - (int)(v / GRID_WIDTH));
            if (dx + dy != 1 || dist[v] != UINT16_MAX || !Grid_CanStep(&map, u, v)) continue;
            queue[tail++] = v; dist[v] = dist[u] + 1;
        }
    }
    return UINT16_MAX;
}

static void Search(uint16_t start, uint16_t goal)
{
    CHECK(Grid_Begin(&map, start, goal) == GRID_SEARCHING);
    unsigned calls = 0;
    while (map.result == GRID_SEARCHING) {
        unsigned before = map.expanded;
        Grid_Advance(&map, 4);
        CHECK(map.expanded - before <= 4);
        CHECK(++calls <= GRID_NODES);
    }
    unsigned oracle = BFS(start, goal);
    if (oracle == UINT16_MAX) { CHECK(map.result == GRID_NO_PATH); return; }
    CHECK(map.result == GRID_FOUND);
    CHECK(map.path_length == oracle + 1);
    CHECK(map.path[0] == start && map.path[map.path_length - 1] == goal);
    for (unsigned i = 0; i < map.path_length; ++i) {
        CHECK(!Grid_IsBlocked(&map, map.path[i]));
        if (!i) continue;
        unsigned a = map.path[i - 1], b = map.path[i];
        CHECK(abs((int)(a % GRID_WIDTH) - (int)(b % GRID_WIDTH)) +
              abs((int)(a / GRID_WIDTH) - (int)(b / GRID_WIDTH)) == 1);
    }
}

int main(int argc, char **argv)
{
    (void)argv;
    CHECK(Grid_Cell(-1, 20) == GRID_NONE && Grid_Cell(1800, 20) == GRID_NONE);
    CHECK(Grid_Cell(NAN, 20) == GRID_NONE && Grid_Cell(20, INFINITY) == GRID_NONE);
    Grid_Init(&map, 0);
    uint16_t start = Grid_Cell(275, 75), goal = Grid_Cell(1575, 75);
    for (unsigned i = 0; i < 18; ++i) {
        GridRect r = grid_static_obstacles[i];
        CHECK(Grid_IsBlocked(&map, Grid_Cell((r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2)));
    }
    Search(start, goal);
    CHECK(map.result == GRID_FOUND);
    unsigned n = map.path[map.path_length / 2];
    float hx = (n % GRID_WIDTH) * GRID_CELL_MM + 25;
    float hy = (n / GRID_WIDTH) * GRID_CELL_MM + 25;
    CHECK(Grid_AddHit(&map, hx, hy));
    CHECK(map.observed_count==1 && map.observed_cells[0]==n);
    CHECK(!Grid_AddHit(&map, hx, hy));
    CHECK(map.observed_count==1);
    CHECK(!Grid_AddHit(&map, -20, 10));
    CHECK(!Grid_AddHit(&map, 750, 300));
    Search(start, goal);
    CHECK(map.result == GRID_FOUND);
    for (unsigned i = 0; i < map.path_length; ++i) CHECK(map.path[i] != n);
    Grid_Init(&map, 50);
    CHECK(Grid_IsBlocked(&map, Grid_Cell(25, 1000))); /* fence inflation */
    CHECK(Grid_IsBlocked(&map, Grid_Cell(575, 575))); /* exclusion inflation */
    Search(start, goal);
    for (unsigned y = 0; y < GRID_HEIGHT; ++y)
        Grid_AddHit(&map, 1075, y * GRID_CELL_MM + 25);
    Search(start, goal);
    CHECK(map.result == GRID_NO_PATH);
    CHECK(Grid_Begin(&map, GRID_NONE, goal) == GRID_BAD_POSE);
    Grid_Init(&map, 0);
    Search(start, start);
    CHECK(map.path_length == 1);
    float entry_y;
    Grid_Init(&map, 64); /* 9x9 cm turning radius, no additional test margin */
    CHECK(Grid_FindStartEntry(&map, 200, -45, &entry_y));
    CHECK(entry_y == 75);
    CHECK(!Grid_FindStartEntry(&map, 50, -45, &entry_y)); /* T1 / edge */
    CHECK(!Grid_FindStartEntry(&map, 700, -45, &entry_y)); /* not start gate */
    CHECK(!Grid_FindStartEntry(&map, NAN, -45, &entry_y));
    CHECK(Grid_AddHit(&map, 200, 25));
    CHECK(!Grid_FindStartEntry(&map, 200, -45, &entry_y)); /* new entry obstacle */
    Grid_Init(&map, 139); /* radius64 + clearance50 + uncertainty25 */
    CHECK(!Grid_FindStartEntry(&map, 200, -45, &entry_y));
    Grid_Init(&map, 74); /* actual test body radius64 + nominal margin10 */
    CHECK(Grid_FindStartEntry(&map, 200, -45, &entry_y));
    CHECK(Grid_CanTravel(&map, 200, 75, 225, 75)); /* entry alignment */
    CHECK(!Grid_CanTravel(&map, 225, 75, 25, 75)); /* swept segment hits T1 */
    CHECK(!Grid_CanTravel(&map, 200, -45, 225, 75)); /* entry uses its own gate check */
    CHECK(!Grid_CanTravel(&map, NAN, 75, 225, 75));
    Search(Grid_Cell(200,75), Grid_Cell(1575,75));
    CHECK(map.result == GRID_FOUND);
    Grid_SetFrontSensors(&map, navigation_config.sensor_forward_mm,
        navigation_config.sensor_left_mm, navigation_config.sensor_angle_deg, 50);
    CHECK(!Grid_IsBlocked(&map, Grid_Cell(225,225)));
    CHECK(!Grid_CanStep(&map, Grid_Cell(225,175), Grid_Cell(225,225)));
    CHECK(Grid_CanStep(&map, Grid_Cell(225,225), Grid_Cell(225,175)));
    CHECK(Grid_CanStep(&map, Grid_Cell(425,175), Grid_Cell(425,225)));
    Search(Grid_Cell(200,75), Grid_Cell(1575,75));
    CHECK(map.result == GRID_FOUND);
    CHECK(Grid_AddHit(&map, 425, 320));
    CHECK(!Grid_IsBlocked(&map, Grid_Cell(425,225)));
    CHECK(!Grid_CanStep(&map, Grid_Cell(425,175), Grid_Cell(425,225))); /* cache invalidated */
    Search(Grid_Cell(200,75), Grid_Cell(1575,75)); /* path or no path agrees with BFS */
    if (argc > 1) {
        Grid_Init(&map, 50);
        Search(start, goal);
        printf("{\"cell_mm\":50,\"width\":36,\"height\":24,\"inflation_mm\":50,\"obstacles\":[");
        for (unsigned i = 0; i < 18; ++i) {
            GridRect r = grid_static_obstacles[i];
            printf("%s[%d,%d,%d,%d]", i ? "," : "", r.x0, r.y0, r.x1, r.y1);
        }
        printf("],\"path\":[");
        for (unsigned i = 0; i < map.path_length; ++i)
            printf("%s%u", i ? "," : "", map.path[i]);
        puts("]}");
    } else puts("PASS: 17 obstacles, inflated footprint, A* matches BFS, dynamic replan, no route, bounded work");
    return 0;
}
