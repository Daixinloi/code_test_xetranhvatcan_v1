#ifndef GRID_MAP_H
#define GRID_MAP_H
#include <stdbool.h>
#include <stdint.h>

#define GRID_CELL_MM 50
#define GRID_WIDTH 36
#define GRID_HEIGHT 24
#define GRID_NODES (GRID_WIDTH * GRID_HEIGHT)
#define GRID_BYTES ((GRID_NODES + 7) / 8)
#define GRID_NONE UINT16_MAX
#define GRID_MOVABLE_COUNT 17u
#define GRID_T3_INDEX 2u
#define GRID_T9_INDEX 8u

typedef struct { int16_t x0, y0, x1, y1; } GridRect;
extern const GridRect grid_static_obstacles[18]; /* T1..T17, middle exclusion */
typedef enum { GRID_SEARCHING, GRID_FOUND, GRID_NO_PATH, GRID_BAD_POSE } GridResult;
typedef struct {
    uint8_t blocked[GRID_BYTES];
    uint16_t observed_cells[GRID_NODES], observed_count;
    uint8_t front_checked[4 * GRID_BYTES], front_unsafe[4 * GRID_BYTES];
    float front_rays[4][3][4]; /* origin/end offsets for E,N,W,S */
    bool front_enabled;
    uint32_t live_clear_mask; /* Movable boxes checked by live ToF in field mode. */
    uint8_t node_state[GRID_NODES]; /* 0=unseen, 1=open, 2=closed */
    uint16_t cost[GRID_NODES], parent[GRID_NODES], path[GRID_NODES];
    uint16_t start, goal, path_length, expanded, inflation_mm, turn_padding_mm, middle_extra_mm;
    uint8_t min_row;
    GridResult result;
    uint8_t static_blocked[GRID_BYTES];
    uint8_t observed_quarters[GRID_NODES];
} GridMap;

void Grid_Init(GridMap *map, uint16_t inflation_mm);
void Grid_SetMiddleClearance(GridMap *map, uint16_t extra_mm);
void Grid_SetFrontSensors(GridMap *map, const int16_t forward_mm[3],
    const int16_t left_mm[3], const int16_t angle_deg[3], uint16_t center_mm,
    uint16_t diagonal_mm);
uint16_t Grid_Cell(float x_mm, float y_mm);
bool Grid_IsBlocked(const GridMap *map, uint16_t cell);
/* One range hit adds occupied evidence, never erases previous evidence. */
bool Grid_AddHit(GridMap *map, float x_mm, float y_mm);
void Grid_ClearHits(GridMap *map);
/* Preflight a straight northbound entry through the open start line.
 * Checks the entire swept corridor, including the section outside the grid. */
bool Grid_FindStartEntry(const GridMap *map, float x_mm, float y_mm, float *entry_y_mm);
bool Grid_CanEnter(const GridMap *map, float x0_mm, float y0_mm, float x1_mm, float y1_mm);
bool Grid_CanFinish(const GridMap *map, float x_mm, float y_mm, float exit_y_mm);
bool Grid_CanFinishWithPadding(const GridMap *map, float x_mm, float y_mm,
    float exit_y_mm, uint16_t pad_mm);
bool Grid_CanFinishWithClearance(const GridMap *map, float x_mm, float y_mm,
    float exit_y_mm, uint16_t pad_mm, uint16_t middle_extra_mm);
bool Grid_CanTravel(const GridMap *map, float x0_mm, float y0_mm, float x1_mm, float y1_mm);
bool Grid_CanRotate(const GridMap *map, float x_mm, float y_mm);
/* Positive tolerance spends tracking margin. Negative tolerance adds margin. */
bool Grid_CanTravelWithTolerance(const GridMap *map, float x0_mm, float y0_mm,
    float x1_mm, float y1_mm, int16_t tolerance_mm);
bool Grid_CanTravelWithClearance(const GridMap *map, float x0_mm, float y0_mm,
    float x1_mm, float y1_mm, int pad_mm, uint16_t middle_extra_mm);
bool Grid_CanStep(GridMap *map, uint16_t from, uint16_t to);
GridResult Grid_Begin(GridMap *map, uint16_t start, uint16_t goal);
GridResult Grid_BeginAbove(GridMap *map, uint16_t start, uint16_t goal, uint8_t min_row);
/* Small bounded expansion budget keeps 100 Hz MPU sampling responsive. */
GridResult Grid_Advance(GridMap *map, unsigned budget);
#endif
