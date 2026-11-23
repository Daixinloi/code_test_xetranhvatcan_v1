#ifndef NAVIGATION_H
#define NAVIGATION_H
#include "grid_map.h"

typedef struct {
    bool enabled; /* Allows supervised operation; does not certify map accuracy. */
    uint16_t length_mm, width_mm, clearance_mm, position_margin_mm;
    int16_t start_x_mm, start_y_mm, goal_x_mm, goal_y_mm;
    int16_t start_heading_deg; /* Map east=0, north=90, west=180, south=270 */
    int16_t sensor_forward_mm[3], sensor_left_mm[3], sensor_angle_deg[3];
    bool route_u;
} NavigationConfig;
typedef struct {
    uint16_t mm; bool valid; uint32_t tick;
    bool no_echo; /* Fresh device sample without usable range; never an ACK error. */
} NavigationRange;
typedef enum {
    NAV_UNCONFIGURED, NAV_STOPPED, NAV_READY, NAV_PLANNING, NAV_TURNING,
    NAV_MOVING, NAV_WAIT_SENSOR, NAV_ARRIVED, NAV_NO_PATH, NAV_POSE_LOST,
    NAV_MPU_FAULT, NAV_MOTION_FAULT, NAV_TOF_FAULT,
    NAV_ENTERING, NAV_ENTRY_BLOCKED, NAV_WAIT_CLEARANCE,
    NAV_EXITING, NAV_EXIT_BLOCKED, NAV_RETREATING, NAV_SEARCH_BLOCKED, NAV_BORDER
} NavigationState;
typedef struct {
    uint32_t state, replans, observed_hits, expanded;
    int32_t x_mm, y_mm, heading_mdeg;
    uint32_t path_length, path_index;
    uint32_t invalid_front_mask; /* bit0=TL bit1=G bit2=TR */
    uint32_t planner_last_ms, planner_max_ms;
    uint32_t route_leg; /* 0=north, 1=east, 2=south */
    uint32_t motion_result; /* MotionStatus captured before NAV_MOTION_FAULT stops it. */
    uint32_t near_front_mask; /* Most recent close returns: bit0=TL, bit1=G, bit2=TR. */
    uint16_t near_front_mm[3];
    uint32_t last_fault_state, last_fault_tick; /* Retained across v confirmations. */
    uint32_t retreats;
} NavigationDiagnostics;
extern volatile NavigationDiagnostics nav_diag;

void Navigation_Init(const NavigationConfig *config);
/* Confirm a new run at the surveyed start, facing start_heading.
 * Clears observations from the previous run without moving or recalibrating. */
bool Navigation_ConfirmStart(void);
bool Navigation_Start(void);
bool Navigation_StartEmptyFloorTest(void);
bool Navigation_IsEmptyFloorTest(void);
bool Navigation_IsRetreating(void);
void Navigation_Cancel(void);
void Navigation_InvalidatePose(void);
void Navigation_Update(const NavigationRange front[3], int16_t speed);
bool Navigation_IsActive(void);
bool Navigation_UsesCenteredField(void);
const char *Navigation_GetLabel(void);
const GridMap *Navigation_GetMap(void);
/* World projection also exposed for calibration checks without moving motors. */
bool Navigation_ProjectRange(const NavigationConfig *config, unsigned sensor,
    float centre_x_mm, float centre_y_mm, float heading_deg, uint16_t range_mm,
    float *hit_x_mm, float *hit_y_mm);
#endif
