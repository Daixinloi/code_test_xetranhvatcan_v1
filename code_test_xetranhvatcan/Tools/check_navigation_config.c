/* Host geometry preflight, independent of sensors and motor hardware. */
#include "navigation_config.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
static GridMap map;
int main(int argc, char **argv)
{
    bool json = argc > 1 && strcmp(argv[1], "--json") == 0;
    const NavigationConfig *c = &navigation_config;
    unsigned body = (unsigned)ceilf(hypotf(c->length_mm, c->width_mm) / 2);
    unsigned pad = body + c->clearance_mm + c->position_margin_mm;
    Grid_Init(&map, (uint16_t)pad);
    Grid_SetFrontSensors(&map, c->sensor_forward_mm, c->sensor_left_mm, c->sensor_angle_deg, 50, 50);
    if (!json) {
    printf("Robot %u x %u mm; start=(%d,%d) mm, heading=%d deg\n",
        c->length_mm, c->width_mm, c->start_x_mm, c->start_y_mm, c->start_heading_deg);
    printf("Inflation: body=%u + clearance=%u + margin=%u -> %u mm\n",
        body, c->clearance_mm, c->position_margin_mm, pad);
    }
    float entry_y;
    bool entry = Grid_FindStartEntry(&map, c->start_x_mm, c->start_y_mm, &entry_y);
    bool goal = !Grid_IsBlocked(&map, Grid_Cell(c->goal_x_mm, c->goal_y_mm));
    if (!json) printf("North entry: %s; goal: %s; autonomous configuration: %s\n",
        entry ? "CLEAR" : "BLOCKED", goal ? "CLEAR" : "BLOCKED",
        c->enabled ? "enabled for supervised test" : "disabled");
    if (entry && goal) {
        Grid_Begin(&map, Grid_Cell(c->start_x_mm, entry_y), Grid_Cell(c->goal_x_mm, c->goal_y_mm));
        while (map.result == GRID_SEARCHING) Grid_Advance(&map, 4);
        if (!json) printf("Route: %s, %u cells\n", map.result == GRID_FOUND ? "FOUND" : "NO PATH", map.path_length);
    }
    if (json) {
        printf("{\"inflation_mm\":%u,\"start\":[%d,%d],\"body\":[%u,%u],\"entry_y\":%.0f,\"path\":[",
            pad,c->start_x_mm,c->start_y_mm,c->length_mm,c->width_mm,entry ? entry_y : 0);
        for (unsigned i=0;i<map.path_length;++i) printf("%s%u",i ? "," : "",map.path[i]);
        printf("],\"sensors\":[");
        for (unsigned i=0;i<3;++i)
            printf("%s[%d,%d,%d]",i ? "," : "",c->sensor_forward_mm[i],c->sensor_left_mm[i],c->sensor_angle_deg[i]);
        puts("]}");
    }
    return 0;
}
