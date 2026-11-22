#ifndef NAVIGATION_H
#define NAVIGATION_H
#include "grid_map.h"

typedef struct {
    bool enabled; /* Allows supervised operation; does not certify map accuracy. */
    uint16_t length_mm, width_mm, clearance_mm, position_margin_mm;
    int16_t start_x_mm, start_y_mm, goal_x_mm, goal_y_mm;
    int16_t start_heading_deg; /* Map east=0, north=90, west=180, south=270 */
    int16_t sensor_forward_mm[3], sensor_left_mm[3], sensor_angle_deg[3];
} NavigationConfig;
typedef struct {
    uint16_t mm; bool valid; uint32_t tick;
    bool no_echo; /* Fresh device sample without usable range; never an ACK error. */
} NavigationRange;
typedef enum {
    NAV_UNCONFIGURED, NAV_STOPPED, NAV_READY, NAV_PLANNING, NAV_TURNING,
    NAV_MOVING, NAV_WAIT_SENSOR, NAV_ARRIVED, NAV_NO_PATH, NAV_POSE_LOST,
    NAV_MPU_FAULT, NAV_MOTION_FAULT, NAV_TOF_FAULT,
    NAV_ENTERING, NAV_ENTRY_BLOCKED
} NavigationState;
typedef struct {
    uint32_t state, replans, observed_hits, expanded;
    int32_t x_mm, y_mm, heading_mdeg;
    uint32_t path_length, path_index;
    uint32_t invalid_front_mask; /* bit0=TL bit1=G bit2=TR */
    uint32_t planner_last_ms, planner_max_ms;
} NavigationDiagnostics;
extern volatile NavigationDiagnostics nav_diag;

void Navigation_Init(const NavigationConfig *config);
/* Explicit operator confirmation: robot is at the surveyed start, facing
 * start_heading. Does not move motors, erase obstacles, or recalibrate MPU. */
bool Navigation_ConfirmStart(void);
bool Navigation_Start(void);
bool Navigation_StartEmptyFloorTest(void);
bool Navigation_IsEmptyFloorTest(void);
void Navigation_Cancel(void);
void Navigation_InvalidatePose(void);
void Navigation_Update(const NavigationRange front[3], int16_t speed);
bool Navigation_IsActive(void);
const char *Navigation_GetLabel(void);
const GridMap *Navigation_GetMap(void);
/* World projection also exposed for calibration checks without moving motors. */
bool Navigation_ProjectRange(const NavigationConfig *config, unsigned sensor,
    float centre_x_mm, float centre_y_mm, float heading_deg, uint16_t range_mm,
    float *hit_x_mm, float *hit_y_mm);
#endif
