#include "navigation.h"
#include "front_clearance.h"
#include "motion.h"
#include "motor.h"
#include "course_route.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define DEG_RAD 0.01745329252f
#define NAV_SENSOR_AGE_MS 200u
#define NAV_SENSOR_WAIT_MS 2000u
#define NAV_CLEAR_SETTLE_MS 150u
#define NAV_FIELD_CLEAR_SETTLE_MS 20u
#define NAV_FIELD_TURN_SETTLE_MS 30u
#define NAV_OBSERVE_MM 300u
#define NAV_FIELD_OBSERVE_MM 180u
#define NAV_HIT_MATCH_MM 25.0f
#define NAV_STRAIGHT_RUN_MM 150.0f
#define U_MIDDLE_CLEARANCE_MM 50u
#define U_CENTERED_MIDDLE_CLEARANCE_MM 15u
#define U_CENTERED_NORTH_MAX_Y_MM 1010.0f
#define U_CENTERED_EAST_MAX_Y_MM 1035.0f
#define U_NORTH_GATE_Y_MM 775
#define U_FIELD_GATE_Y_MM 730
#define U_FIRST_EAST_POINT 19u /* End of the first curved section in course_route.h. */
#define U_FIELD_SOUTH_X_MM 1327
#define U_FIELD_SOUTH_AFTER_T14_X_MM 1380
#define U_FIELD_EDGE_MM 100.0f
#define U_SOUTH_LANE_MAX_X_MM 1630.0f
#define U_SOUTH_INWARD_X_MM 1550.0f
#define U_PROBE_RETRY_MS 250u
#define U_PROBE_RETRY_DISTANCE_MM 110.0f
#define U_T9_APPROACH_X_MM 485.0f
#define U_T9_PASS_Y_MM 895.0f
#define U_T9_LANE_Y_MM 880.0f
#define U_REFERENCE_CRUISE_MM_S 210u
#define U_AISLE_CLEAR_SAMPLE_MS 30u
#define U_CLOSE_MM 25u
#define U_CLOSE_RETREAT_MM 20.0f
#define U_CLOSE_RETREAT_MIN_MM 10.0f
#define U_LEFT_LANE_MM 600
#define U_RIGHT_LANE_MM 1200
#define U_TRACKING_TOLERANCE_MM 3u
#define U_STRAIGHT_SIDE_MM 20u
#define U_T1_EDGE_TOLERANCE_MM 20.0f
#define U_ENTRY_CHOICES 25u
#define U_RETREAT_BRAKE_MS 100u
#define U_RETREAT_MAX_MM 90.0f
#define U_RETREAT_RESERVE_MM 10.0f
#define U_RETREAT_MIN_MM 20.0f
#define U_RETREAT_YAW_LIMIT_DEG 5.0f
#define U_RETREAT_YAW_HARD_LIMIT_DEG 8.0f
#define U_RETREAT_YAW_RECOVER_DEG 12.0f
#define U_RETREAT_YAW_SETTLE_MS 100u
#define U_BYPASS_DISTANCE_MM 250.0f
#define U_BYPASS_DIAGONAL_MM 30u
#define U_LOCAL_STEP_MM 50.0f
#define U_NARROW_SHIFT_MM 40.0f
#define U_SOUTH_SHIFT_MM 60.0f
#define U_EXTRA_SHIFT_MM 40.0f
#define U_MIN_SIDE_PROBE_MM 10.0f
#define U_T14_ROLL_LOOKAHEAD_MM 15.0f
#define U_T14_TURN_START_X_MM 1235.0f
#define U_REFERENCE_TRACK_STEP_DEG 5.0f
#define U_REFERENCE_START_STEER_DEG 8.0f
#define U_MOVING_HEADING_LIMIT_DEG 15.0f
#define U_MOVING_HEADING_HARD_LIMIT_DEG 24.0f
#define U_MOVING_HEADING_RECOVER_MS 140u
#define U_T9_SHIFT_LIMIT_X_MM 835.0f
#define U_EAST_WIDE_END_MM 1100.0f
#define U_T15_TOP_MM 500.0f
#define U_T15_APPROACH_Y_MM 600.0f
#define U_SOUTH_JOIN_START_Y_MM 1005.0f
#define U_SOUTH_JOIN_NEAR_Y_MM 865.0f
#define U_SOUTH_JOIN_MARGIN_MM 10.0f
#define U_JOIN_CORNER_MARGIN_MM 8
#define U_T15_APPROACH_LEFT_MM 1350.0f
#define U_T15_APPROACH_RIGHT_MM 1550.0f
#define U_DISTANCE_ROUNDING_MM 0.001f
#define NAV_TRACE_CAPACITY 192u
#define NAV_TRACE_INTERVAL_MS 125u

typedef enum { U_NORTH, U_EAST, U_SOUTH } RouteLeg;
typedef enum {
    LOCAL_IDLE, LOCAL_SHIFT, LOCAL_CHECK, LOCAL_FORWARD, LOCAL_BACKTRACK, LOCAL_JOIN,
    LOCAL_REFERENCE, LOCAL_ARC, LOCAL_BEND
} LocalStep;
static const int16_t east_gates[][2] = {
    {1375, 775}, {1425, 875}, {1375, 975}, {1475, 975}, {1525, 775}
};
static const int16_t finish_offsets[] = {0, 50, -50, 100, -100, 150, -150, 200, -200, -250};

static GridMap map;
static uint16_t turn_padding;
static NavigationConfig cfg;
static bool configured, active, pose_valid;
static NavigationState state;
static float x, y, yaw_offset, next_heading, next_distance;
static uint16_t path_index;
static int16_t last_encoder;
static float last_yaw;
static int8_t odometry_direction;
static uint32_t sensor_seen[3], waiting_since;
static uint32_t moving_heading_lag_at;
static struct { float x, y; bool valid; } candidate[3];
static uint32_t clear_since;
static bool clearance_paused;
static NavigationState resume_state;
static bool resume_after_turn;
static bool entry_pending;
static float entry_x, entry_y, entry_lateral_y;
static unsigned entry_choice;
static bool entry_lateral, entry_replan, entry_prefer_left;
static RouteLeg route_leg;
static unsigned goal_choice;
static bool finishing;
static bool empty_floor_test;
static bool fixed_line_course;
static unsigned fixed_line_point;
static bool narrow_detour;
static int16_t detour_step;
static bool tried_other_side;
static bool returning_to_sweep_start;
static float detour_progress;
static float sweep_start;
static LocalStep local_step;
static float local_x, local_y;
static float join_x, join_y;
static bool join_active;
static uint8_t join_attempts;
static uint8_t reference_point;
static bool reference_active;
static bool fast_first_curve;
static bool reference_detour_active;
static struct {
    float x, y, start, shift, lead, delay, pass, end, merge;
    uint32_t started, updated;
    uint8_t rejoin;
} arc;
static struct {
    float goal;
    uint32_t started;
    LocalStep resume;
} bend;
static uint8_t reference_detour_point;
static bool t11_aligned;
static uint32_t t11_face_tick;
static float t11_face_x;
static struct { float x, y; uint32_t tick; } turn_hit[3];
static bool side_blocked;
static bool shift_extended;
static bool backtracked;
static bool prefer_lateral;
static uint32_t aisle_seen_mask, aisle_tick[GRID_MOVABLE_COUNT];
static uint8_t aisle_sensor[GRID_MOVABLE_COUNT];
static float tail_mm, tail_heading, retreat_hit_x, retreat_hit_y;
static float bypass_mm;
static uint32_t close_seen, close_ticks[3], braked_at;
static uint32_t retreat_yaw_ok_at;
static uint16_t last_retreat_hit;
static uint32_t last_probe_retry;
static float probe_retry_x, probe_retry_y;
volatile NavigationDiagnostics nav_diag;

/* Recent samples survive loss of the SWD connection, until a new run starts. */
static volatile struct {
    uint16_t elapsed_10ms;
    int16_t x_mm, y_mm, heading_cdeg, target_cdeg;
    uint16_t front_mm[3];
    uint8_t state;
} nav_trace[NAV_TRACE_CAPACITY];
static volatile uint32_t nav_trace_count;
static uint32_t nav_trace_start, nav_trace_last;

static void RecordTrace(const NavigationRange front[3], uint32_t now, bool force)
{
    unsigned interval = state == NAV_SEARCH_BLOCKED || state == NAV_WAIT_CLEARANCE ||
        state == NAV_WAIT_SENSOR ? 750u : NAV_TRACE_INTERVAL_MS;
    if (!force && nav_trace_count && now - nav_trace_last < interval) return;
    unsigned index = nav_trace_count % NAV_TRACE_CAPACITY;
    nav_trace[index].elapsed_10ms = (uint16_t)((now - nav_trace_start) / 10u);
    nav_trace[index].x_mm = (int16_t)nav_diag.x_mm;
    nav_trace[index].y_mm = (int16_t)nav_diag.y_mm;
    int32_t heading = nav_diag.heading_mdeg / 10 % 36000;
    if (heading > 18000) heading -= 36000;
    if (heading < -18000) heading += 36000;
    nav_trace[index].heading_cdeg = (int16_t)heading;
    nav_trace[index].target_cdeg = (int16_t)(next_heading * 100.0f);
    for (unsigned i = 0; i < 3; ++i)
        nav_trace[index].front_mm[i] = front && front[i].valid &&
            now - front[i].tick <= NAV_SENSOR_AGE_MS ? front[i].mm : UINT16_MAX;
    nav_trace[index].state = (uint8_t)state;
    nav_trace_last = now;
    ++nav_trace_count;
}

static float MapYaw(void) { return Motion_GetAngle() + yaw_offset; }
static bool CenteredField(void)
{
    return cfg.route_u && cfg.start_x_mm == COURSE_ROUTE_START_X_MM && cfg.start_y_mm == -45;
}
static bool EarlyCourseBend(void)
{
    return CenteredField() && x < 590.0f && y < 825.0f;
}

static bool NorthCorner(void)
{
    return CenteredField() && route_leg == U_NORTH && y >= U_FIELD_GATE_Y_MM;
}
static bool FieldRoute(void)
{
    return cfg.route_u && (!empty_floor_test || CenteredField());
}

static uint16_t CourseMiddleClearance(void)
{
    return CenteredField() && route_leg == U_SOUTH && x >= 1250.0f ?
        U_CENTERED_MIDDLE_CLEARANCE_MM : map.middle_extra_mm;
}

static bool CourseTravelWithTolerance(float x0, float y0, float x1, float y1, int16_t tolerance)
{
    return Grid_CanTravelWithClearance(&map, x0, y0, x1, y1,
        map.inflation_mm - tolerance, CourseMiddleClearance());
}

static bool CourseTravel(float x0, float y0, float x1, float y1)
{
    return CourseTravelWithTolerance(x0, y0, x1, y1, 0);
}

static bool CourseRotate(float tx, float ty)
{
    return CourseTravelWithTolerance(tx, ty, tx, ty, map.inflation_mm - map.turn_padding_mm);
}

static int16_t LegForwardStep(void)
{
    return route_leg == U_NORTH ? GRID_WIDTH : route_leg == U_EAST ? 1 : -GRID_WIDTH;
}

static float LegHeading(void)
{
    return route_leg == U_NORTH ? 90.0f : route_leg == U_EAST ? 0.0f : -90.0f;
}

static bool InsideFieldMargin(float tx, float ty)
{
    return tx >= U_FIELD_EDGE_MM && tx <= GRID_WIDTH * GRID_CELL_MM - U_FIELD_EDGE_MM &&
        ty >= U_FIELD_EDGE_MM && ty <= GRID_HEIGHT * GRID_CELL_MM - U_FIELD_EDGE_MM;
}

static bool InLeg(float tx, float ty)
{
    if (!InsideFieldMargin(tx, ty)) return false;
    float pad = map.turn_padding_mm;
    if (route_leg == U_NORTH) return tx < U_LEFT_LANE_MM - pad &&
        (!CenteredField() || ty <= U_CENTERED_NORTH_MAX_Y_MM);
    if (route_leg == U_EAST) return ty > 600.0f + pad + map.middle_extra_mm &&
        (!CenteredField() || ty <= U_CENTERED_EAST_MAX_Y_MM);
    return tx > U_RIGHT_LANE_MM + pad && tx <= U_SOUTH_LANE_MAX_X_MM;
}

static float ForwardProgress(void)
{
    if (route_leg == U_NORTH) return y;
    return route_leg == U_EAST ? x : -y;
}
static bool TightDiagonals(void)
{
    return bypass_mm > 0 || narrow_detour ||
        (FieldRoute() && (entry_pending || route_leg == U_EAST));
}

static void State(NavigationState value)
{
    state = value;
    nav_diag.state = value;
    nav_diag.x_mm = (int32_t)x; nav_diag.y_mm = (int32_t)y;
    nav_diag.heading_mdeg = (int32_t)(MapYaw() * 1000.0f);
    nav_diag.path_index = path_index; nav_diag.path_length = map.path_length;
    nav_diag.expanded = map.expanded;
    nav_diag.route_leg = route_leg;
}

static void Fail(NavigationState reason)
{
    Motion_Cancel(); active = false;
    Motion_SetCruise(0);
    Motion_SetCourseSteering(false);
    if (reason != NAV_ARRIVED) {
        nav_diag.last_fault_state = reason;
        nav_diag.last_fault_tick = HAL_GetTick();
    }
    State(reason);
}

static void Pause(NavigationState pause_state)
{
    if (state != NAV_WAIT_SENSOR && state != NAV_WAIT_CLEARANCE) {
        resume_after_turn = state == NAV_TURNING && FieldRoute();
        resume_state = state == NAV_PLANNING || state == NAV_SEARCH_BLOCKED ? state : NAV_READY;
        Motion_Cancel();
        memset(candidate, 0, sizeof(candidate));
    }
    State(pause_state);
}

static void Resume(void)
{
    clearance_paused = false;
    waiting_since = resume_after_turn ? HAL_GetTick() : 0;
    resume_after_turn = false;
    /* Keep a partially computed route, or check the next segment again
     * from the pose captured just before braking. */
    State(resume_state);
}

static void Odometry(void)
{
    if (!odometry_direction) return;
    int16_t current = Encoder_GetCount2();
    int delta = (int16_t)((uint16_t)current - (uint16_t)last_encoder);
    last_encoder = current;
    float yaw = MapYaw();
    /* Include the encoder ticks after PWM cutoff, until the next command.
     * Pivots clear the translation direction before their motor output. */
    float mm = abs(delta) * (CenteredField() ? MOTION_FIELD_MM_PER_COUNT : MOTION_WHEEL_MM_PER_COUNT);
    float midpoint = (last_yaw + yaw) * 0.5f * DEG_RAD;
    if (state == NAV_MOVING || state == NAV_ENTERING)
        tail_mm = fminf(U_RETREAT_MAX_MM + U_RETREAT_RESERVE_MM, tail_mm + mm);
    if (state == NAV_MOVING)
        bypass_mm = fmaxf(0.0f, bypass_mm - mm);
    if (odometry_direction < 0) {
        tail_mm = fmaxf(0.0f, tail_mm - mm);
        mm = -mm;
    }
    x += mm * cosf(midpoint); y += mm * sinf(midpoint);
    if (detour_step && ForwardProgress() - detour_progress >= (backtracked ? 35.0f : 80.0f)) {
        detour_step = 0;
        returning_to_sweep_start = false;
        tried_other_side = false;
        side_blocked = false;
        backtracked = false;
        prefer_lateral = false;
    }
    last_yaw = yaw;
    nav_diag.x_mm = (int32_t)x; nav_diag.y_mm = (int32_t)y;
    nav_diag.heading_mdeg = (int32_t)(yaw*1000.0f);
}

static void StartOdometry(int8_t direction)
{
    last_encoder = 0;
    last_yaw = MapYaw();
    odometry_direction = direction;
    moving_heading_lag_at = 0;
}

static bool StartHeadingTurn(void)
{
    odometry_direction = 0;
    return Motion_StartTurnHeading(next_heading - yaw_offset);
}

static void SetMapFrontClearance(uint16_t diagonal_mm)
{
    Grid_SetFrontSensors(&map, cfg.sensor_forward_mm, cfg.sensor_left_mm, cfg.sensor_angle_deg,
        CLEARANCE_CENTER_MIN_MM, diagonal_mm);
}

static void InitMap(uint16_t padding)
{
    Grid_Init(&map, padding);
    map.turn_padding_mm = turn_padding;
    if (cfg.route_u) Grid_SetMiddleClearance(&map, U_MIDDLE_CLEARANCE_MM);
    SetMapFrontClearance(CLEARANCE_DIAGONAL_MIN_MM);
}

void Navigation_Init(const NavigationConfig *config)
{
    memset((void *)&nav_diag, 0, sizeof(nav_diag));
    cfg = *config;
    active = pose_valid = false;
    empty_floor_test = false;
    reference_detour_active = false;
    entry_pending = false;
    route_leg = U_NORTH;
    bool outside_start = cfg.start_y_mm <= 0 && cfg.start_y_mm >= -(int)cfg.length_mm / 2 &&
        cfg.start_x_mm > 0 && cfg.start_x_mm < 600 && cfg.start_heading_deg == 90;
    configured = cfg.enabled && cfg.length_mm > 0 && cfg.width_mm > 0 &&
        cfg.length_mm < 800 && cfg.width_mm < 800 &&
        cfg.clearance_mm <= 300 && cfg.position_margin_mm <= 300 &&
        (outside_start || Grid_Cell(cfg.start_x_mm, cfg.start_y_mm) != GRID_NONE) &&
        Grid_Cell(cfg.goal_x_mm, cfg.goal_y_mm) != GRID_NONE;
    float radius = ceilf(hypotf(cfg.length_mm, cfg.width_mm) * 0.5f);
    turn_padding = (uint16_t)(radius + cfg.clearance_mm + cfg.position_margin_mm);
    InitMap(turn_padding);
    x = cfg.start_x_mm; y = cfg.start_y_mm; yaw_offset = 0;
    memset(sensor_seen, 0, sizeof(sensor_seen));
    path_index = 0;
    State(configured ? NAV_STOPPED : NAV_UNCONFIGURED);
}

bool Navigation_ConfirmStart(void)
{
    if (!configured) { State(NAV_UNCONFIGURED); return false; }
    if (active) return false;
    if (!Motion_IsMPUReady()) { pose_valid = false; State(NAV_MPU_FAULT); return false; }
    Grid_ClearHits(&map);
    nav_diag.observed_hits = 0;
    path_index = 0;
    x = cfg.start_x_mm; y = cfg.start_y_mm;
    yaw_offset = cfg.start_heading_deg - Motion_GetAngle();
    if (y > 0 && Grid_IsBlocked(&map, Grid_Cell(x, y))) {
        pose_valid = false; State(NAV_NO_PATH); return false;
    }
    pose_valid = true;
    State(NAV_READY);
    return true;
}

static float ExitTarget(void)
{
    return -(0.5f * cfg.length_mm + cfg.position_margin_mm);
}

static unsigned GoalChoices(void)
{
    if (!cfg.route_u || route_leg == U_NORTH) return 1;
    if (route_leg == U_EAST) return sizeof(east_gates) / sizeof(east_gates[0]);
    return sizeof(finish_offsets) / sizeof(finish_offsets[0]);
}

static uint16_t RouteGoal(void)
{
    if (!cfg.route_u) return Grid_Cell(cfg.goal_x_mm, cfg.goal_y_mm);
    if (route_leg == U_NORTH) return Grid_Cell(225, U_NORTH_GATE_Y_MM);
    if (route_leg == U_EAST) return Grid_Cell(east_gates[goal_choice][0], east_gates[goal_choice][1]);
    float goal_x = cfg.goal_x_mm + finish_offsets[goal_choice];
    if (!Grid_CanFinish(&map, goal_x, cfg.goal_y_mm, ExitTarget())) return GRID_NONE;
    return Grid_Cell(goal_x, cfg.goal_y_mm);
}

static bool FindEntry(void)
{
    entry_y = GRID_CELL_MM / 2;
    while (entry_y <= map.turn_padding_mm) entry_y += GRID_CELL_MM;
    if (reference_active) entry_y = 125.0f;
    unsigned count = cfg.route_u && !empty_floor_test ? 2 * U_ENTRY_CHOICES : 1;
    int column = (int)(x / GRID_CELL_MM);
    for (; entry_choice < count; ++entry_choice) {
        unsigned lane = entry_choice % U_ENTRY_CHOICES;
        int step = (int)(lane / 2);
        int offset = (lane & 1u) ? -step : step;
        if (entry_prefer_left) offset = -offset;
        entry_x = lane == 0 ? x : (column + offset) * GRID_CELL_MM + GRID_CELL_MM / 2;
        if (Grid_IsBlocked(&map, Grid_Cell(entry_x, entry_y))) continue;
        entry_lateral = entry_choice >= U_ENTRY_CHOICES;
        if (!entry_lateral && Grid_CanEnter(&map, x, y, entry_x, entry_y)) return true;
        if (entry_lateral && lane && Grid_CanEnter(&map, x, y, entry_x, y) &&
            Grid_CanEnter(&map, entry_x, y, entry_x, entry_y)) {
            entry_lateral_y = y;
            return true;
        }
    }
    return false;
}

static bool Plan(void)
{
    Motion_Cancel();
    local_step = LOCAL_IDLE;
    clearance_paused = false;
    resume_after_turn = false;
    entry_replan = false;
    if (finishing) { State(NAV_READY); return true; }
    path_index = 0;
    if (entry_pending && !FindEntry()) {
        pose_valid = false;
        Fail(NAV_ENTRY_BLOCKED); return false;
    }
    /* Field control uses only the course heading and checked local moves. */
    if (FieldRoute()) { State(NAV_READY); return true; }
    ++nav_diag.replans;
    uint16_t start = entry_pending ? Grid_Cell(entry_x, entry_y) : Grid_Cell(x, y);
    uint8_t min_row = cfg.route_u && route_leg == U_EAST ? U_NORTH_GATE_Y_MM / GRID_CELL_MM : 0;
    for (; goal_choice < GoalChoices(); ++goal_choice) {
        if (Grid_BeginAbove(&map, start, RouteGoal(), min_row) != GRID_BAD_POSE) {
            State(NAV_PLANNING);
            return true;
        }
    }
    pose_valid = false;
    Fail(cfg.route_u && route_leg == U_SOUTH ? NAV_EXIT_BLOCKED : NAV_NO_PATH);
    return false;
}

static bool Start(bool test)
{
    if (!configured) { State(NAV_UNCONFIGURED); return false; }
    if (!pose_valid) { State(NAV_POSE_LOST); return false; }
    if (!Motion_IsMPUReady()) { State(NAV_MPU_FAULT); return false; }
    if (active) {
        if (empty_floor_test == test) return true;
        Navigation_Cancel();
        State(NAV_POSE_LOST);
        return false; /* Mode switches require an explicit new start pose. */
    }
    empty_floor_test = test;
    fixed_line_course = COURSE_FIXED_LINE_TEST && cfg.route_u &&
        cfg.start_x_mm == COURSE_ROUTE_START_X_MM;
    fixed_line_point = 0;
    nav_trace_count = 0;
    nav_trace_start = HAL_GetTick();
    odometry_direction = 0;
    last_probe_retry = 0;
    narrow_detour = false;
    detour_step = 0;
    returning_to_sweep_start = false;
    tried_other_side = false;
    local_step = LOCAL_IDLE;
    side_blocked = false;
    backtracked = false;
    prefer_lateral = false;
    join_active = false;
    join_attempts = 0;
    reference_point = 0;
    reference_detour_active = false;
    reference_active = CenteredField();
    fast_first_curve = false;
    t11_aligned = false;
    t11_face_tick = 0;
    Motion_SetCourseSteering(reference_active);
    Motion_SetCruise(reference_active ? U_REFERENCE_CRUISE_MM_S : 0u);
    memset(turn_hit, 0, sizeof(turn_hit));
    uint16_t pad = turn_padding;
    if (cfg.route_u && (!test || CenteredField())) {
        uint16_t side = cfg.length_mm > cfg.width_mm ? cfg.length_mm : cfg.width_mm;
        uint16_t straight_pad = (side + 1u) / 2u + U_STRAIGHT_SIDE_MM + cfg.clearance_mm;
        if (straight_pad < pad) pad = straight_pad;
    }
    if (pad != map.inflation_mm) InitMap(pad);
    else SetMapFrontClearance(CLEARANCE_DIAGONAL_MIN_MM);
    map.live_clear_mask = 0;
    aisle_seen_mask = 0;
    if (test) {
        /* Use the virtual arena and add confirmed hits after a retreat. */
        Grid_ClearHits(&map);
        nav_diag.observed_hits=0;
    }
    memset(sensor_seen, 0, sizeof(sensor_seen));
    memset(candidate, 0, sizeof(candidate));
    tail_mm = 0.0f;
    bypass_mm = 0.0f;
    close_seen = 0;
    last_retreat_hit = GRID_NONE;
    entry_pending = y <= 0;
    entry_prefer_left = false;
    entry_choice = 0;
    entry_lateral = false;
    nav_diag.motion_result = MOTION_IDLE;
    nav_diag.retreats = 0;
    nav_diag.near_front_mask = 0;
    memset((void *)nav_diag.near_front_mm, 0, sizeof(nav_diag.near_front_mm));
    route_leg = U_NORTH;
    goal_choice = 0;
    finishing = false;
    waiting_since = 0;
    active = true;
    return Plan();
}
bool Navigation_Start(void) { return Start(false); }
bool Navigation_StartEmptyFloorTest(void) { return Start(true); }
bool Navigation_IsEmptyFloorTest(void) { return active && empty_floor_test; }
bool Navigation_IsRetreating(void) { return active && state == NAV_RETREATING; }

void Navigation_Cancel(void)
{
    Motion_SetCruise(0);
    Motion_SetCourseSteering(false);
    if (active) {
        Odometry();
        Motion_Cancel();
        State(NAV_STOPPED);
        RecordTrace(NULL, HAL_GetTick(), true);
    }
    active = pose_valid = false;
    empty_floor_test = false;
    reference_detour_active = false;
}

void Navigation_InvalidatePose(void)
{
    if (!active) return;
    Odometry(); pose_valid = false;
    Fail(NAV_POSE_LOST);
    RecordTrace(NULL, HAL_GetTick(), true);
}

bool Navigation_ProjectRange(const NavigationConfig *config, unsigned sensor,
    float centre_x, float centre_y, float heading, uint16_t range, float *hit_x, float *hit_y)
{
    if (!config || sensor >= 3 || !hit_x || !hit_y || range >= 8000 ||
        !isfinite(centre_x) || !isfinite(centre_y) || !isfinite(heading)) return false;
    float yaw = heading * DEG_RAD, c = cosf(yaw), s = sinf(yaw);
    float ray = yaw + config->sensor_angle_deg[sensor] * DEG_RAD;
    *hit_x = centre_x + config->sensor_forward_mm[sensor] * c -
        config->sensor_left_mm[sensor] * s + range * cosf(ray);
    *hit_y = centre_y + config->sensor_forward_mm[sensor] * s +
        config->sensor_left_mm[sensor] * c + range * sinf(ray);
    return true;
}

static bool KnownEntryFace(unsigned sensor, float hit_x, float hit_y)
{
    const GridRect t1 = grid_static_obstacles[0];
    return FieldRoute() && entry_pending && sensor == 0 &&
        hit_x >= t1.x1 && hit_x <= t1.x1 + U_T1_EDGE_TOLERANCE_MM &&
        hit_y >= t1.y0 && hit_y <= t1.y1;
}

static bool Observe(const NavigationRange front[3])
{
    bool changed = false;
    uint16_t limit = CenteredField() && local_step == LOCAL_REFERENCE ?
        (EarlyCourseBend() ? 290u : 245u) :
        FieldRoute() ? NAV_FIELD_OBSERVE_MM : NAV_OBSERVE_MM;
    for (unsigned i = 0; i < 3; ++i) {
        if (CenteredField() && i != 1) continue;
        uint32_t previous_tick = sensor_seen[i];
        if (previous_tick == front[i].tick) continue;
        sensor_seen[i] = front[i].tick;
        if (!front[i].valid || front[i].mm > limit ||
            (TightDiagonals() && i != 1 &&
                front[i].mm < CLEARANCE_DIAGONAL_MIN_MM)) {
            candidate[i].valid = false;
            continue;
        }
        if (FieldRoute()) {
            /* Adjacent rays still protect the vehicle live, but they should
             * not paint a wide occupied strip across a parallel aisle. */
            float lateral = cfg.sensor_left_mm[i] +
                front[i].mm * sinf(cfg.sensor_angle_deg[i] * DEG_RAD);
            if (fabsf(lateral) > cfg.width_mm * 0.5f + cfg.position_margin_mm) {
                candidate[i].valid = false;
                continue;
            }
        }
        float hit_x, hit_y;
        if (!Navigation_ProjectRange(&cfg, i, x, y, MapYaw(), front[i].mm, &hit_x, &hit_y)) continue;
        if (KnownEntryFace(i, hit_x, hit_y)) {
            candidate[i].valid = false;
            continue;
        }
        float dx = hit_x - candidate[i].x, dy = hit_y - candidate[i].y;
        bool confirmed = candidate[i].valid && front[i].tick - previous_tick <= NAV_SENSOR_AGE_MS &&
            dx*dx + dy*dy <= NAV_HIT_MATCH_MM * NAV_HIT_MATCH_MM;
        candidate[i].x = hit_x;
        candidate[i].y = hit_y;
        candidate[i].valid = true;
        if ((!cfg.route_u || confirmed) && Grid_AddHit(&map, hit_x, hit_y)) {
            changed = true; ++nav_diag.observed_hits;
        }
    }
    return changed;
}

static float RetreatDistance(void)
{
    float distance = fminf(U_RETREAT_MAX_MM, tail_mm - U_RETREAT_RESERVE_MM);
    if (distance < (FieldRoute() ? U_CLOSE_RETREAT_MIN_MM : U_RETREAT_MIN_MM) ||
        fabsf(remainderf(tail_heading - MapYaw(), 360.0f)) > MOTION_ANGLE_TOLERANCE_DEG)
        return 0.0f;
    return distance;
}

static bool BackUp(float distance)
{
    if (CenteredField()) return false;
    float heading = tail_heading;
    float heading_rad = heading * DEG_RAD;
    float back_x = x - cosf(heading_rad) * distance;
    float back_y = y - sinf(heading_rad) * distance;
    /* Stay on the swept section just driven, including at the start opening. */
    bool clear = entry_pending ? Grid_CanEnter(&map, x, y, back_x, back_y) :
        CourseTravel(x, y, back_x, back_y);
    if (!clear || (FieldRoute() && !entry_pending && !InsideFieldMargin(back_x, back_y))) return false;
    if (!Motion_StartDistanceHeading(-1, distance, heading - yaw_offset)) {
        pose_valid = false;
        Fail(NAV_MOTION_FAULT);
        return true;
    }
    next_heading = heading;
    StartOdometry(-1);
    retreat_yaw_ok_at = HAL_GetTick();
    ++nav_diag.retreats;
    State(NAV_RETREATING);
    return true;
}

static bool TurnRoom(const NavigationRange front[3], uint32_t near, float retreat);

static bool TryTurnClearanceRetreat(const NavigationRange front[3], uint32_t near)
{
    if (backtracked || !FieldRoute() || entry_pending || finishing ||
        fabsf(remainderf(tail_heading - MapYaw(), 360.0f)) > U_RETREAT_YAW_LIMIT_DEG)
        return false;
    float max_distance = fminf(U_RETREAT_MAX_MM, tail_mm - U_RETREAT_RESERVE_MM);
    for (float distance = U_RETREAT_MIN_MM; distance <= max_distance; distance += 10.0f) {
        float heading = tail_heading * DEG_RAD;
        float tx = x - distance * cosf(heading);
        float ty = y - distance * sinf(heading);
        if (!InsideFieldMargin(tx, ty) || !CourseRotate(tx, ty) ||
            !TurnRoom(front, near, distance))
            continue;
        if (BackUp(distance)) {
            backtracked = true;
            prefer_lateral = true;
            tried_other_side = false;
            side_blocked = false;
            return true;
        }
    }
    return false;
}

static bool TryLocalStep(const NavigationRange front[3], uint32_t near, bool allow_forward);
static bool TryTurnBack(const NavigationRange front[3], uint32_t near);
static bool NearOutsideTurn(const NavigationRange front[3], uint32_t near);

static uint16_t CloseRange(unsigned sensor)
{
    /* Allow for coasting when the right corner passes T9. */
    return sensor == 2 && CenteredField() && route_leg == U_EAST &&
        x >= 520.0f && x < 630.0f && y >= 835.0f && y < 925.0f
        ? 50u : U_CLOSE_MM;
}

static bool TryShortBackUp(const NavigationRange front[3], uint32_t confirmed)
{
    float distance = fminf(U_CLOSE_RETREAT_MM, tail_mm - U_RETREAT_RESERVE_MM);
    if (!FieldRoute() || distance < U_CLOSE_RETREAT_MIN_MM ||
        fabsf(remainderf(tail_heading - MapYaw(), 360.0f)) > U_RETREAT_YAW_LIMIT_DEG)
        return false;
    for (unsigned i = 0; i < 3; ++i) {
        if (!(confirmed & (1u << i)) || front[i].mm >= CloseRange(i) ||
            !Navigation_ProjectRange(&cfg, i, x, y, MapYaw(), front[i].mm, &retreat_hit_x, &retreat_hit_y))
            continue;
        uint16_t hit = Grid_Cell(retreat_hit_x, retreat_hit_y);
        if (hit == GRID_NONE || hit == last_retreat_hit) continue;
        if (!BackUp(distance) &&
            (distance < 2.0f * U_CLOSE_RETREAT_MIN_MM || !BackUp(distance * 0.5f)))
            continue;
        last_retreat_hit = hit;
        if (!entry_pending && !finishing && (!CenteredField() || (confirmed & 2u))) {
            prefer_lateral = true;
            /* Do not send the vehicle straight back into the same approach. */
            side_blocked = local_step == LOCAL_SHIFT;
            if (!returning_to_sweep_start) tried_other_side = false;
        }
        return true;
    }
    return false;
}

static void TryRetreat(const NavigationRange front[3], uint32_t near, uint32_t now)
{
    if (!FieldRoute() && now - braked_at < U_RETREAT_BRAKE_MS) return;

    close_seen &= near;
    uint32_t confirmed = 0;
    for (unsigned i = 0; i < 3; ++i) {
        uint32_t bit = 1u << i;
        if (!(near & bit)) continue;
        if ((close_seen & bit) && close_ticks[i] != front[i].tick) confirmed |= bit;
        close_seen |= bit;
        close_ticks[i] = front[i].tick;
    }
    if (CenteredField()) {
        if (confirmed) {
            if (local_step == LOCAL_SHIFT) side_blocked = true;
            local_step = LOCAL_IDLE;
            TryLocalStep(front, near, false);
        }
        return;
    }
    if (TryShortBackUp(front, confirmed)) return;
    float distance = RetreatDistance();
    if (confirmed && FieldRoute() && !TurnRoom(front, near, 0.0f) &&
        TryTurnClearanceRetreat(front, near)) return;
    for (unsigned i = 0; i < 3; ++i) {
        if (!(confirmed & (1u << i)) || !Navigation_ProjectRange(&cfg, i, x, y, MapYaw(),
                front[i].mm, &retreat_hit_x, &retreat_hit_y)) continue;

        uint16_t hit = Grid_Cell(retreat_hit_x, retreat_hit_y);
        if (local_step == LOCAL_SHIFT) side_blocked = true;
        if (local_step != LOCAL_CHECK) local_step = LOCAL_IDLE;
        if (TryLocalStep(front, near, false) || TryTurnBack(front, near)) return;
        if (hit != GRID_NONE && hit != last_retreat_hit && distance > 0.0f && BackUp(distance)) {
            last_retreat_hit = hit;
            return;
        }
    }
}

static void WaitForClearance(const NavigationRange front[3], uint32_t near, uint32_t now)
{
    bool obstacle = CenteredField() ? (near & 2u) != 0 : near != 0;
    if ((local_step == LOCAL_ARC || local_step == LOCAL_BEND) && near) {
        Grid_ClearHits(&map);
        reference_active = false;
        local_step = LOCAL_CHECK;
    }
    if (obstacle && local_step == LOCAL_REFERENCE) {
        reference_active = false;
        local_step = LOCAL_IDLE;
    }
    if (obstacle && local_step == LOCAL_JOIN) {
        join_active = false;
        local_step = LOCAL_CHECK;
    }
    if (state != NAV_WAIT_CLEARANCE) {
        clearance_paused = true;
        close_seen = 0;
        braked_at = now;
        Pause(NAV_WAIT_CLEARANCE);
    }
    if (near) {
        clear_since = now;
        if (cfg.route_u) TryRetreat(front, near, now);
    } else if (now - clear_since >= (FieldRoute() ? NAV_FIELD_CLEAR_SETTLE_MS : NAV_CLEAR_SETTLE_MS)) {
        Resume();
    } else {
        close_seen = 0;
    }
}

static bool FieldExitClear(void);

static void FinishRun(void)
{
    tail_mm = 0.0f;
    float distance = y - ExitTarget();
    if (distance < 1.0f) {
        pose_valid = false;
        Fail(NAV_ARRIVED);
        return;
    }
    if (!FieldExitClear()) {
        if (y > map.turn_padding_mm && ++goal_choice < GoalChoices()) {
            finishing = false;
            Plan();
        } else {
            pose_valid = false;
            Fail(NAV_EXIT_BLOCKED);
        }
        return;
    }
    next_heading = -90.0f;
    if (fabsf(remainderf(next_heading - MapYaw(), 360.0f)) > MOTION_ANGLE_TOLERANCE_DEG) {
        if (!CourseRotate(x, y)) { pose_valid = false; Fail(NAV_EXIT_BLOCKED); return; }
        if (!StartHeadingTurn()) { Fail(NAV_MPU_FAULT); return; }
        State(NAV_TURNING);
        return;
    }
    if (!Motion_StartDistanceHeading(1, fminf(MOTION_STEP_MM, distance), next_heading - yaw_offset)) {
        Fail(NAV_MOTION_FAULT);
        return;
    }
    StartOdometry(1);
    State(NAV_EXITING);
}

static bool EntryClear(void)
{
    return Grid_CanEnter(&map, x, y, entry_x, entry_lateral ? entry_lateral_y : entry_y) &&
        (!entry_lateral || Grid_CanEnter(&map, entry_x, entry_lateral_y, entry_x, entry_y));
}

static void SetForwardHeading(void)
{
    if (tail_mm > 0.0f && fabsf(remainderf(next_heading - tail_heading, 360.0f)) >
            MOTION_ANGLE_TOLERANCE_DEG)
        tail_mm = 0.0f;
    tail_heading = next_heading;
}

static void RunEntry(void)
{
    float target_y = entry_lateral ? entry_lateral_y : entry_y;
    float dx = entry_x - x, dy = target_y - y;
    float distance = hypotf(dx, dy);
    if (distance < 2.0f && (entry_lateral || y > map.turn_padding_mm)) {
        if (entry_lateral) entry_lateral = false;
        else {
            entry_pending = false;
            if (FieldRoute()) Grid_ClearHits(&map);
            if (entry_replan) Plan();
        }
        return;
    }
    if (distance < 1.0f || !EntryClear()) {
        pose_valid = false;
        Fail(NAV_ENTRY_BLOCKED);
        return;
    }
    next_heading = atan2f(dy, dx) / DEG_RAD;
    if (fabsf(remainderf(next_heading - MapYaw(), 360.0f)) > MOTION_ANGLE_TOLERANCE_DEG) {
        tail_mm = 0.0f;
        if (!StartHeadingTurn()) { Fail(NAV_MPU_FAULT); return; }
        State(NAV_TURNING);
        return;
    }
    SetForwardHeading();
    float limit = reference_active && !entry_lateral ? 150.0f : MOTION_STEP_MM;
    if (!Motion_StartDistanceHeading(1, fminf(limit, distance), next_heading - yaw_offset)) {
        Fail(NAV_MOTION_FAULT);
        return;
    }
    StartOdometry(1);
    State(NAV_ENTERING);
}

static bool TrackEastTurnHits(void)
{
    return FieldRoute() && route_leg == U_EAST && x >= 430.0f &&
        x < (CenteredField() ? 1170.0f : 825.0f) && y < 960.0f;
}

static bool TurnRoom(const NavigationRange front[3], uint32_t near, float retreat)
{
    /* Live readings are relative to the body, so only the static map
     * needs the full allowance for odometry error. */
    float pad = map.turn_padding_mm - (FieldRoute() && retreat == 0.0f ? 5.0f : 0.0f);
    if (TrackEastTurnHits()) {
        float angle = MapYaw() * DEG_RAD;
        float turn_x = x - retreat * cosf(angle);
        float turn_y = y - retreat * sinf(angle);
        /* A front ray can lose a nearby face as the body begins to turn. */
        float margin = CenteredField() ? (x < 940.0f ? 26.0f : 28.0f) : 16.0f;
        float hit_pad = pad + margin + (x < 540.0f ? 4.0f : 0.0f);
        for (unsigned i = 0; i < 3; i += 2) {
            float dx = turn_x - turn_hit[i].x, dy = turn_y - turn_hit[i].y;
            if (turn_hit[i].tick && dx * dx + dy * dy <= hit_pad * hit_pad) return false;
        }
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (FieldRoute() ? front[i].mm >= FrontClearance_MinMm(i) : !(near & (1u << i))) continue;
        float angle = cfg.sensor_angle_deg[i] * DEG_RAD;
        float dx = cfg.sensor_forward_mm[i] + front[i].mm * cosf(angle) + retreat;
        float dy = cfg.sensor_left_mm[i] + front[i].mm * sinf(angle);
        if (dx * dx + dy * dy <= pad * pad)
            return false;
    }
    return true;
}

static bool NearOutsideTurn(const NavigationRange front[3], uint32_t near)
{
    return TurnRoom(front, near, 0.0f);
}

static bool LocalTargetAtDistance(int16_t step, uint32_t near, const NavigationRange front[3],
    float distance)
{
    int dx = step == 1 ? 1 : step == -1 ? -1 : 0;
    int dy = step == GRID_WIDTH ? 1 : step == -GRID_WIDTH ? -1 : 0;
    float heading = dx ? (dx > 0 ? 0.0f : 180.0f) : (dy > 0 ? 90.0f : -90.0f);
    bool turn = fabsf(remainderf(heading - MapYaw(), 360.0f)) > MOTION_ANGLE_TOLERANCE_DEG;
    float tx = x + dx * distance;
    float ty = y + dy * distance;
    bool sideways = step != LegForwardStep();
    if (!InLeg(tx, ty) || !CourseTravel(x, y, tx, ty)) return false;
    /* Leave space to turn north before T9's west face. */
    if (route_leg == U_EAST && !(map.live_clear_mask & (1u << GRID_T9_INDEX)) &&
        !sideways && y < U_T9_LANE_Y_MM - 2.0f &&
        x < U_LEFT_LANE_MM && tx > U_T9_APPROACH_X_MM) return false;
    if (cfg.start_x_mm == COURSE_ROUTE_START_X_MM && route_leg == U_EAST && !sideways &&
        x < 1300.0f && tx > 1110.0f && y >= 880.0f && y < 980.0f) return false;
    /* Make the lateral move before the body reaches T15's turn envelope. */
    if (route_leg == U_SOUTH && !sideways && y > U_T15_TOP_MM && ty < U_T15_APPROACH_Y_MM &&
        tx > U_T15_APPROACH_LEFT_MM && tx < U_T15_APPROACH_RIGHT_MM) return false;
    if (sideways && !CourseRotate(tx, ty)) return false;
    if (!sideways && !CourseRotate(tx, ty)) {
        if (route_leg == U_SOUTH) {
            if (!Grid_CanFinish(&map, tx, ty, ExitTarget())) return false;
        } else if (!CourseTravel(tx, ty, tx + dx * 20.0f, ty + dy * 20.0f)) {
            return false;
        }
    }
    if (turn) {
        if (!CourseRotate(x, y) || !NearOutsideTurn(front, near)) return false;
    } else if (near || front[1].mm < distance + CLEARANCE_CENTER_MIN_MM) {
        return false;
    }
    local_x = tx;
    local_y = ty;
    local_step = sideways ? LOCAL_SHIFT : LOCAL_FORWARD;
    shift_extended = false;
    clearance_paused = false;
    State(NAV_READY);
    return true;
}

static float SideRoom(const NavigationRange front[3], unsigned i)
{
    if (!CenteredField() && front[0].mm < CLEARANCE_DIAGONAL_MIN_MM &&
        front[2].mm < CLEARANCE_DIAGONAL_MIN_MM) return 100.0f;
    float range = front[i].mm;
    float angle = cfg.sensor_angle_deg[i] * DEG_RAD;
    float lateral = cfg.sensor_left_mm[i] + range * sinf(angle);
    if (fabsf(lateral) >= cfg.width_mm * 0.5f + 4.0f) return 100.0f;
    /* A corner can lie in the swept body even when the centre ray is open. */
    return cfg.sensor_forward_mm[i] + range * cosf(angle) - map.turn_padding_mm -
        (route_leg != U_SOUTH ? 8.0f : 0.0f);
}

static bool DiagonalClear(float tx, float ty)
{
    return CourseTravelWithTolerance(x, y, tx, ty,
        map.inflation_mm - map.turn_padding_mm - U_JOIN_CORNER_MARGIN_MM);
}

static float HeadingFootprint(float heading)
{
    float angle = fminf(45.0f, fabsf(remainderf(heading, 90.0f)) + 3.0f) * DEG_RAD;
    return 0.5f * fmaxf(cfg.length_mm, cfg.width_mm) * (cosf(angle) + sinf(angle));
}

static bool CourseFinishClear(float tx, float ty)
{
    if (CenteredField() && tx >= 1250.0f)
        return Grid_CanFinishWithClearance(&map, tx, ty, ExitTarget(),
            (uint16_t)ceilf(HeadingFootprint(-90.0f) + 5.0f),
            U_CENTERED_MIDDLE_CLEARANCE_MM);
    return Grid_CanFinish(&map, tx, ty, ExitTarget());
}

static bool FieldExitClear(void)
{
    if (CenteredField() && x < 1350.0f) {
        float heading = MapYaw();
        float span = HeadingFootprint(heading);
        return fabsf(remainderf(heading + 90.0f, 360.0f)) <= MOTION_ANGLE_TOLERANCE_DEG &&
            Grid_CanFinishWithClearance(&map, x, y, ExitTarget(), (uint16_t)ceilf(span + 5.0f),
                U_CENTERED_MIDDLE_CLEARANCE_MM);
    }
    return Grid_CanFinish(&map, x, y, ExitTarget());
}

static bool ReferenceClear(float tx, float ty, float bearing, int16_t marker)
{
    if (marker < 2) {
        return fabsf(tx - x) < 2.0f || fabsf(ty - y) < 2.0f ?
            CourseTravel(x, y, tx, ty) : DiagonalClear(tx, ty);
    }
    float span = fmaxf(HeadingFootprint(bearing), HeadingFootprint(MapYaw()));
    return Grid_CanTravelWithClearance(&map, x, y, tx, ty, (int)ceilf(span + 5.0f),
        x >= 940.0f ? U_CENTERED_MIDDLE_CLEARANCE_MM : map.middle_extra_mm);
}

static void AlignT11Face(const NavigationRange front[3])
{
    if (t11_aligned || !reference_active || route_leg != U_EAST || state != NAV_MOVING ||
        x < 620.0f || x >= 720.0f || y < 860.0f || y >= 895.0f ||
        front[1].mm <= 150 || front[2].mm <= 150 ||
        front[0].tick == t11_face_tick || fabsf(MapYaw()) >= 6.0f)
        return;

    float hx, hy;
    if (!Navigation_ProjectRange(&cfg, 0, x, y, MapYaw(), front[0].mm, &hx, &hy) ||
        hx < 765.0f || hx > 810.0f || hy < 905.0f || hy >= 923.0f) {
        t11_face_tick = 0;
        return;
    }
    /* Two rays must agree on the west face. Its lower corner gives a
     * conservative lower bound on Y when travel was longer than the encoder. */
    if (t11_face_tick && front[0].tick - t11_face_tick <= NAV_SENSOR_AGE_MS &&
        fabsf(hx - t11_face_x) <= 8.0f) {
        float adjustment = 950.0f - hy;
        y += adjustment;
        nav_diag.y_mm = (int32_t)y;
        for (unsigned i = 0; i < 3; i += 2) turn_hit[i].y += adjustment;
        t11_aligned = true;
    }
    t11_face_tick = front[0].tick;
    t11_face_x = hx;
}

static void ReferenceCruise(int16_t marker, float remaining, float heading, uint16_t front_mm)
{
    bool near_t11 = route_leg == U_EAST && x >= 690.0f && x < 990.0f && y < 925.0f;
    unsigned straight = near_t11 ? 145u : U_REFERENCE_CRUISE_MM_S;
    unsigned speed = marker == 3 ? 150u : marker && remaining < 95.0f ?
        (marker == 2 ? 85u : 105u) :
        (fabsf(remainderf(heading, 90.0f)) < 4.0f ? straight : 175u);
    unsigned room = front_mm > CLEARANCE_CENTER_MIN_MM ?
        (unsigned)front_mm - CLEARANCE_CENTER_MIN_MM : 0u;
    /* Lower the speed target before the centre ToF reaches the stop threshold. */
    unsigned approach_speed = 85u + 2u * room;
    Motion_SetCruise(speed < approach_speed ? speed : approach_speed);
}

static float ForwardRoom(const NavigationRange front[3])
{
    return fminf(front[1].mm - (float)CLEARANCE_CENTER_MIN_MM,
        fminf(SideRoom(front, 0), SideRoom(front, 2)));
}

static bool PreferRight(const NavigationRange front[3])
{
    return front[2].mm >= front[0].mm || (front[2].mm >= 150 && front[0].mm >= 150);
}

static float ArcEase(float t)
{
    t = fmaxf(0.0f, fminf(1.0f, t));
    return t * t * (3.0f - 2.0f * t);
}

static void ArcPoint(float forward, float *px, float *py)
{
    float side = forward <= arc.pass ?
        arc.shift * ArcEase((forward - arc.delay) / (arc.lead - arc.delay)) :
        arc.shift + (arc.merge - arc.shift) * ArcEase((forward - arc.pass) / (arc.end - arc.pass));
    *px = arc.x + (route_leg == U_EAST ? forward : route_leg == U_NORTH ? -side : side);
    *py = arc.y + (route_leg == U_EAST ? side : route_leg == U_NORTH ? forward : -forward);
}

static bool ArcClear(float x0, float y0, float x1, float y1)
{
    uint16_t middle = x0 >= 940.0f ? U_CENTERED_MIDDLE_CLEARANCE_MM : CourseMiddleClearance();
    return InsideFieldMargin(x1, y1) && Grid_CanTravelWithClearance(&map, x0, y0, x1, y1,
        map.inflation_mm + 4, middle);
}

static bool CommandRollingHeading(const NavigationRange front[3], float goal)
{
    float yaw = MapYaw();
    float error = remainderf(goal - yaw, 360.0f);
    float room = ForwardRoom(front);
    if (room < 18.0f) return false;
    float heading = yaw + fmaxf(-12.0f, fminf(12.0f, error));
    float rad = heading * DEG_RAD;
    float forward_x = cosf(rad), forward_y = sinf(rad);
    float distance = fminf(45.0f, room);
    if (!ArcClear(x, y, x + 8.0f*cosf(yaw*DEG_RAD), y + 8.0f*sinf(yaw*DEG_RAD)) ||
        !ArcClear(x, y, x + 18.0f*forward_x, y + 18.0f*forward_y) ||
        !Motion_RetargetForward(distance, heading - yaw_offset))
        return false;
    next_heading = heading;
    local_x = x + distance * forward_x;
    local_y = y + distance * forward_y;
    return true;
}

static bool StartRollingBend(const NavigationRange front[3], uint32_t near, uint32_t now)
{
    float yaw = MapYaw();
    float error = remainderf(next_heading - yaw, 360.0f);
    float room = ForwardRoom(front);
    if (near || fabsf(error) > 100.0f || room < 35.0f) return false;
    /* Check a conservative forward arc before committing to a bend. Live
     * ranges still cover the real steering and wheel response during the turn. */
    float turn = error * DEG_RAD / 4.0f;
    float step = fabsf(turn) * 100.0f;
    float px = x, py = y;
    for (unsigned i = 0; i < 4; ++i) {
        float rad = yaw * DEG_RAD + turn * (i + 0.5f);
        float tx = px + step*cosf(rad), ty = py + step*sinf(rad);
        if (!ArcClear(px, py, tx, ty)) return false;
        px = tx;
        py = ty;
    }
    bend.goal = next_heading;
    bend.resume = local_step;
    bend.started = now;
    Motion_SetCruise(85u);
    if (!Motion_StartDistanceHeading(1, fminf(45.0f, room), yaw - yaw_offset)) return false;
    StartOdometry(1);
    local_step = LOCAL_BEND;
    next_heading = yaw;
    State(NAV_MOVING);
    return true;
}

static bool AdvanceRollingBend(const NavigationRange front[3], uint32_t now)
{
    if (now - bend.started > 2600u || !CommandRollingHeading(front, bend.goal)) return false;
    if (fabsf(remainderf(bend.goal - MapYaw(), 360.0f)) > MOTION_ANGLE_TOLERANCE_DEG)
        return true;
    local_step = bend.resume == LOCAL_REFERENCE || bend.resume == LOCAL_SHIFT ?
        bend.resume : LOCAL_FORWARD;
    return true;
}

static bool ArcSectionClear(float begin, float end)
{
    float x0, y0;
    ArcPoint(begin, &x0, &y0);
    for (unsigned i = 1; i <= 5; ++i) {
        float tx, ty;
        ArcPoint(begin + (end - begin) * i / 5.0f, &tx, &ty);
        if (!ArcClear(x0, y0, tx, ty)) return false;
        x0 = tx;
        y0 = ty;
    }
    return true;
}

static bool FindArcMerge(unsigned range, uint32_t start)
{
    unsigned count = sizeof(course_route) / sizeof(course_route[0]);
    for (unsigned i = reference_point; i + 1 < count && HAL_GetTick() - start < 20u; ++i) {
        float dx = course_route[i][0] - arc.x, dy = course_route[i][1] - arc.y;
        arc.end = route_leg == U_EAST ? dx : route_leg == U_NORTH ? dy : -dy;
        arc.merge = route_leg == U_EAST ? dy : route_leg == U_NORTH ? -dx : dx;
        /* A connector farther from the hit can start earlier. Allow the
         * entire box and rear body clearance before steering back towards it. */
        arc.pass = range + (arc.shift * (arc.merge - arc.shift) >= 0 ? 125.0f : 205.0f);
        float run = arc.end - arc.pass;
        if (run < 85.0f || run > 210.0f || fabsf(arc.merge - arc.shift) > run) continue;
        if (ArcSectionClear(arc.lead, arc.pass) && ArcSectionClear(arc.pass, arc.end)) {
            arc.rejoin = (uint8_t)(i + 1u);
            return true;
        }
    }
    return false;
}

static bool BeginArc(const NavigationRange front[3], uint32_t now)
{
    if (!CenteredField() || Motion_GetStatus() != MOTION_RUNNING ||
        fabsf(remainderf(MapYaw() - LegHeading(), 360.0f)) > 8.0f ||
        front[1].mm < 120 || !candidate[1].valid) return false;

    arc.x = x;
    arc.y = y;
    arc.start = ForwardProgress();
    arc.lead = front[1].mm - 30.0f;
    float first_side = PreferRight(front) ? -1.0f : 1.0f;
    for (unsigned choice = 0; choice < 12 && HAL_GetTick() - now < 20u; ++choice) {
        float width = 85.0f + 15.0f * ((choice + 1u) % 3u);
        arc.shift = (choice < 6 ? first_side : -first_side) * width;
        if (front[arc.shift < 0 ? 2 : 0].mm < CLEARANCE_CENTER_RELEASE_MM) continue;
        arc.delay = choice / 3u & 1u ? arc.lead * 0.3f : 0.0f;
        arc.pass = arc.lead;
        if (!ArcSectionClear(0.0f, arc.lead) || !FindArcMerge(front[1].mm, now) ||
            !Motion_RetargetForward(45.0f, LegHeading() - yaw_offset)) continue;
        next_heading = LegHeading();
        Motion_SetCruise(100u);
        arc.started = arc.updated = now;
        local_step = LOCAL_ARC;
        return true;
    }
    return false;
}

static void AdvanceArc(const NavigationRange front[3], uint32_t now)
{
    if (now - arc.updated < 30u) return;
    arc.updated = now;
    float progress = ForwardProgress() - arc.start;
    float end = arc.end;
    float tx, ty;
    /* Look ahead along the curve instead of aiming at successive stop points. */
    ArcPoint(progress + 45.0f, &tx, &ty);
    float yaw = MapYaw();
    float error = remainderf(atan2f(ty - y, tx - x) / DEG_RAD - yaw, 360.0f);
    float distance = fminf(45.0f, ForwardRoom(front));
    float check = fminf(22.0f, distance);
    float heading = yaw;
    bool steered = false;
    if (now - arc.started <= 9000u && distance >= 12.0f) {
        float ahead = remainderf(LegHeading() - yaw, 360.0f);
        for (unsigned i = 0; i < 3; ++i) {
            float aim = error + (ahead - error) * (0.5f * i);
            heading = yaw + fmaxf(-12.0f, fminf(12.0f, aim));
            float rad = heading * DEG_RAD;
            if (ArcClear(x, y, x + check*cosf(rad), y + check*sinf(rad)) &&
                Motion_RetargetForward(distance, heading - yaw_offset)) {
                steered = true;
                break;
            }
        }
    }
    if (!steered) {
        Motion_Cancel();
        Grid_ClearHits(&map);
        reference_active = false;
        local_step = LOCAL_CHECK;
        State(NAV_READY);
        return;
    }
    next_heading = heading;
    if (progress < end || fabsf(remainderf(heading - LegHeading(), 360.0f)) > 5.0f ||
        fabsf(remainderf(yaw - LegHeading(), 360.0f)) > 8.0f) return;
    Grid_ClearHits(&map);
    reference_point = arc.rejoin;
    local_x = x + distance*cosf(heading*DEG_RAD);
    local_y = y + distance*sinf(heading*DEG_RAD);
    local_step = LOCAL_REFERENCE;
}

static bool CanSteerReference(const NavigationRange front[3], uint32_t near,
    float heading_error, float remaining, int16_t marker)
{
    if (near || !marker || marker == 3 || fabsf(heading_error) > U_REFERENCE_START_STEER_DEG ||
        remaining < 12.0f || ForwardRoom(front) < 22.0f) return false;
    float yaw = MapYaw();
    /* The first centimetre can still follow the old yaw as PWM catches up. */
    return ReferenceClear(x + 12.0f * cosf(yaw * DEG_RAD), y + 12.0f * sinf(yaw * DEG_RAD),
        yaw, marker);
}

static bool BeginReferenceDetour(unsigned first_point)
{
    if (!CenteredField() || reference_detour_active) return false;
    uint16_t start = Grid_Cell(x, y);
    if (start == GRID_NONE || Grid_IsBlocked(&map, start)) return false;

    uint8_t min_row = NorthCorner() ? start / GRID_WIDTH :
        route_leg == U_EAST ? U_NORTH_GATE_Y_MM / GRID_CELL_MM : 0;
    unsigned count = sizeof(course_route) / sizeof(course_route[0]);
    for (unsigned point = first_point; point < count; ++point) {
        uint16_t goal = Grid_Cell(course_route[point][0], course_route[point][1]);
        if ((NorthCorner() && course_route[point][1] < y) ||
            goal == start || goal == GRID_NONE || Grid_IsBlocked(&map, goal) ||
            goal / GRID_WIDTH < min_row)
            continue;
        GridResult result = Grid_BeginAbove(&map, start, goal, min_row);
        if (result == GRID_BAD_POSE || result == GRID_NO_PATH) continue;

        reference_detour_active = true;
        reference_detour_point = (uint8_t)point;
        path_index = 0;
        local_step = LOCAL_IDLE;
        Motion_Cancel();
        Motion_SetCruise(0);
        ++nav_diag.replans;
        State(NAV_PLANNING);
        return true;
    }
    return false;
}

static void ExtendShift(const NavigationRange front[3])
{
    if (!CenteredField() || shift_extended || returning_to_sweep_start) return;
    /* Preserve room to face east again between the T9 and T11 footprints. */
    if (route_leg == U_EAST && x < U_T9_SHIFT_LIMIT_X_MM) return;
    float remaining = hypotf(local_x - x, local_y - y);
    if (remaining > 15.0f) return;
    float angle = next_heading * DEG_RAD;
    float tx = local_x + U_EXTRA_SHIFT_MM * cosf(angle);
    float ty = local_y + U_EXTRA_SHIFT_MM * sinf(angle);
    float distance = remaining + U_EXTRA_SHIFT_MM;
    if (ForwardRoom(front) < distance + 8.0f || !InLeg(tx, ty) ||
        !CourseTravel(x, y, tx, ty) || !CourseRotate(tx, ty) ||
        !TurnRoom(front, 0, -distance) ||
        !Motion_ExtendForward(distance, next_heading - yaw_offset)) return;
    shift_extended = true;
    local_x = tx;
    local_y = ty;
}

static bool PassedReferencePoint(float dx, float dy, float distance, int16_t marker)
{
    if (distance <= 2.0f) return true;
    if (NorthCorner() && dy < -8.0f) return true;
    if (!reference_point || marker < 1 || marker > 2 || distance > 22.0f) return false;
    const int16_t *point = course_route[reference_point];
    const int16_t *previous = course_route[reference_point - 1];
    /* Keep forward progress across a waypoint even after passing its centre. */
    return dx * (point[0] - previous[0]) + dy * (point[1] - previous[1]) <= 0.0f;
}

static bool FollowFastFirstCurve(const NavigationRange front[3])
{
    if (!CenteredField() || x >= 700.0f || y < 565.0f ||
        reference_point > U_FIRST_EAST_POINT) return false;
    if (!fast_first_curve && Motion_GetMeasuredForwardSpeed() < 235.0f) return false;
    fast_first_curve = true;

    float bearing;
    bool joining_east = x >= 550.0f && y >= 865.0f;
    if (joining_east) {
        /* Guide the car back to the east lane before handing control over. */
        if (x >= 630.0f && fabsf(y - 878.0f) < 10.0f &&
            fabsf(remainderf(MapYaw(), 360.0f)) < 8.0f)
            return false;
        bearing = fmaxf(-33.0f, fminf(20.0f, (878.0f - y) * 0.8f));
    } else {
        /* Follow a point on the curve, with less lookahead near T9. */
        float lookahead = y < 746.0f ? 60.0f : 16.0f;
        unsigned point = reference_point;
        for (;;) {
            float px = point ? course_route[point - 1][0] : COURSE_ROUTE_START_X_MM;
            float py = point ? course_route[point - 1][1] : 0.0f;
            float dx = course_route[point][0] - px, dy = course_route[point][1] - py;
            float length = hypotf(dx, dy);
            float from = 0.0f;
            if (point == reference_point) {
                from = fmaxf(0.0f, ((x - px) * dx + (y - py) * dy) / length);
                if (from >= length && point < U_FIRST_EAST_POINT) {
                    ++reference_point;
                    ++point;
                    continue;
                }
            }
            if (point == U_FIRST_EAST_POINT || lookahead <= length - from) {
                float fraction = fminf(1.0f, (from + lookahead) / length);
                bearing = atan2f(py + fraction * dy - y, px + fraction * dx - x) / DEG_RAD;
                break;
            }
            lookahead -= length - from;
            ++point;
        }
    }
    if (!CommandRollingHeading(front, bearing)) return false;
    if (joining_east) reference_point = U_FIRST_EAST_POINT;
    Motion_SetCruise(100u);
    return true;
}

static void ExtendReference(const NavigationRange front[3])
{
    if (state == NAV_EXITING) {
        if (FieldExitClear())
            Motion_ExtendForward(fminf(100.0f, fminf(y - ExitTarget(), ForwardRoom(front))),
                next_heading - yaw_offset);
        return;
    }
    if (!reference_active || (state == NAV_ENTERING ? entry_lateral || y < 90.0f :
        local_step != LOCAL_REFERENCE))
        return;
    if (state == NAV_MOVING && FollowFastFirstCurve(front)) return;

    float committed = hypotf(local_x - x, local_y - y);
    float dx = course_route[reference_point][0] - x;
    float dy = course_route[reference_point][1] - y;
    float remaining = hypotf(dx, dy);
    int16_t marker = course_route[reference_point][2];
    float bearing = atan2f(dy, dx) / DEG_RAD;
    if (!ReferenceClear(course_route[reference_point][0], course_route[reference_point][1],
            bearing, marker) && BeginReferenceDetour(reference_point))
        return;
    if (state == NAV_MOVING && committed > 35.0f) {
        if (!reference_point || front[1].mm < 115u)
            ReferenceCruise(marker, remaining, next_heading, front[1].mm);
        if (remaining < committed || ForwardRoom(front) < committed) return;
        float change = fabsf(remainderf(bearing - next_heading, 360.0f));
        if (change < 0.75f || change > U_REFERENCE_TRACK_STEP_DEG) return;
        float fraction = committed / remaining;
        float tx = x + dx * fraction, ty = y + dy * fraction;
        if (ReferenceClear(tx, ty, bearing, marker) && Motion_SteerForward(bearing - yaw_offset)) {
            next_heading = bearing;
            local_x = tx;
            local_y = ty;
        }
        return;
    }
    ReferenceCruise(marker, remaining, next_heading, front[1].mm);
    float lookahead = x > U_T14_TURN_START_X_MM ? U_T14_ROLL_LOOKAHEAD_MM : 9.0f;
    bool roll = marker && marker < 3 &&
        (remaining <= lookahead || PassedReferencePoint(dx, dy, remaining, marker));
    if (roll) {
        dx = course_route[reference_point + 1][0] - x;
        dy = course_route[reference_point + 1][1] - y;
        remaining = hypotf(dx, dy);
    }
    bearing = atan2f(dy, dx) / DEG_RAD;
    if (fabsf(remainderf(bearing - next_heading, 360.0f)) >
        (roll ? MOTION_FORWARD_ROLL_MAX_DEG : MOTION_ANGLE_TOLERANCE_DEG) ||
        fabsf(remainderf(bearing - MapYaw(), 360.0f)) > MOTION_FORWARD_ROLL_MAX_DEG)
        return;

    bool exit = route_leg == U_SOUTH && reference_point + 1u == (sizeof(course_route) / sizeof(course_route[0])) &&
        y <= cfg.goal_y_mm + 35.0f && FieldExitClear();
    float room = fminf(100.0f, ForwardRoom(front));
    float distance = fminf(room, exit ? y - ExitTarget() : remaining);
    if (distance <= 0.0f) return;
    float tx = exit ? x : x + dx * distance / remaining;
    float ty = exit ? y - distance : y + dy * distance / remaining;
    if (!exit && !ReferenceClear(tx, ty, bearing,
            roll ? course_route[reference_point + 1][2] : marker)) return;
    if (!Motion_ExtendForward(distance, (exit ? -90.0f : bearing) - yaw_offset)) return;
    if (roll) ++reference_point;
    next_heading = exit ? -90.0f : bearing;
    local_x = tx;
    local_y = ty;
    if (state == NAV_ENTERING) {
        entry_pending = entry_replan = false;
        Grid_ClearHits(&map);
        local_step = LOCAL_REFERENCE;
        State(NAV_MOVING);
    } else if (exit) {
        finishing = true;
        State(NAV_EXITING);
    }
}

static bool TryReference(const NavigationRange front[3], uint32_t near)
{
    if (!reference_active) return false;
    while (reference_point < (sizeof(course_route) / sizeof(course_route[0]))) {
        float tx = course_route[reference_point][0], ty = course_route[reference_point][1];
        float dx = tx - x, dy = ty - y;
        float remaining = hypotf(dx, dy);
        int16_t marker = course_route[reference_point][2];
        if (PassedReferencePoint(dx, dy, remaining, marker)) { ++reference_point; continue; }
        float bearing = atan2f(dy, dx) / DEG_RAD;
        float heading_error = remainderf(bearing - MapYaw(), 360.0f);
        if (near) break;
        if (!ReferenceClear(tx, ty, bearing, marker)) {
            if (BeginReferenceDetour(reference_point)) return true;
            break;
        }
        if (fabsf(heading_error) > MOTION_ANGLE_TOLERANCE_DEG &&
            !CanSteerReference(front, near, heading_error, remaining, marker) &&
            (!CourseRotate(x, y) || !NearOutsideTurn(front, near)))
            break;
        ReferenceCruise(marker, remaining, bearing, front[1].mm);
        float fraction = fminf(1.0f, 100.0f / remaining);
        local_x = x + dx * fraction;
        local_y = y + dy * fraction;
        local_step = LOCAL_REFERENCE;
        return true;
    }
    reference_active = false;
    Motion_SetCruise(0);
    return false;
}

static bool FindCornerJoin(void)
{
    if (route_leg == U_NORTH && x < 330.0f && y >= 650.0f && y < U_FIELD_GATE_Y_MM - 15) {
        join_y = U_FIELD_GATE_Y_MM;
        join_x = x + (join_y - y);
    } else if (route_leg == U_EAST && x >= 470.0f && x < 510.0f &&
        y >= U_FIELD_GATE_Y_MM - 2 && y < U_T9_PASS_Y_MM - 40 &&
        !(map.live_clear_mask & (1u << GRID_T9_INDEX))) {
        join_x = 533.0f;
        join_y = U_T9_PASS_Y_MM + 20.0f;
    } else if (route_leg == U_SOUTH && y <= U_SOUTH_JOIN_START_Y_MM &&
        y > U_T15_APPROACH_Y_MM && !CourseFinishClear(x, y)) {
        /* Coming off T14, align south with an extra turn radius in front
         * of T15 so the front sensors can check the new lane sooner. */
        float room_ahead = y > U_SOUTH_JOIN_NEAR_Y_MM ?
            map.turn_padding_mm : U_SOUTH_JOIN_MARGIN_MM;
        join_y = U_T15_TOP_MM + map.turn_padding_mm + room_ahead;
        float min_x = CenteredField() ? U_RIGHT_LANE_MM + map.turn_padding_mm + 1.0f :
            U_FIELD_SOUTH_X_MM;
        float best_shift = GRID_WIDTH * GRID_CELL_MM;
        for (float tx = min_x; tx <= U_SOUTH_LANE_MAX_X_MM; tx += 5.0f) {
            float shift = fabsf(tx - x);
            if (shift >= best_shift ||
                !CourseFinishClear(tx - U_SOUTH_JOIN_MARGIN_MM, join_y) ||
                !CourseFinishClear(tx + U_SOUTH_JOIN_MARGIN_MM, join_y) ||
                !DiagonalClear(tx, join_y)) continue;
            best_shift = shift;
            join_x = tx;
        }
        return best_shift < GRID_WIDTH * GRID_CELL_MM;
    } else {
        return false;
    }
    return InLeg(join_x, join_y) && DiagonalClear(join_x, join_y);
}

static bool TryCornerJoin(const NavigationRange front[3], uint32_t near)
{
    if (!join_active) {
        uint8_t bit = 1u << route_leg;
        if (route_leg == U_SOUTH && y > U_SOUTH_JOIN_NEAR_Y_MM &&
            (front[1].mm < 180u ||
             fabsf(remainderf(LegHeading() - MapYaw(), 360.0f)) > MOTION_ANGLE_TOLERANCE_DEG))
            return false;
        if ((join_attempts & bit) || near || !NearOutsideTurn(front, near) ||
            !CourseRotate(x, y) || !FindCornerJoin()) return false;
        join_attempts |= bit;
        join_active = true;
    }
    float dx = join_x - x, dy = join_y - y;
    float remaining = hypotf(dx, dy);
    if (near || remaining <= 3.0f || !DiagonalClear(join_x, join_y)) {
        join_active = false;
        return false;
    }
    float fraction = fminf(1.0f, 100.0f / remaining);
    local_x = x + dx * fraction;
    local_y = y + dy * fraction;
    local_step = LOCAL_JOIN;
    return true;
}

static bool LocalTarget(int16_t step, uint32_t near, const NavigationRange front[3])
{
    if (step == LegForwardStep()) {
        float east_end = U_FIELD_SOUTH_X_MM;
        if (route_leg == U_EAST && !CourseRotate(east_end, y))
            east_end = U_FIELD_SOUTH_AFTER_T14_X_MM;
        float left = route_leg == U_NORTH ? U_FIELD_GATE_Y_MM - y :
            route_leg == U_EAST ? east_end - x : y - cfg.goal_y_mm;
        float distance = fminf(left, fminf(100.0f, front[1].mm - (float)CLEARANCE_CENTER_MIN_MM));
        distance = fminf(distance, fminf(SideRoom(front, 0), SideRoom(front, 2)));
        while (distance > 2.0f) {
            if (LocalTargetAtDistance(step, near, front, distance)) return true;
            distance *= 0.5f;
        }
        return false;
    }
    float shift = (route_leg == U_NORTH || (route_leg == U_EAST && x < U_EAST_WIDE_END_MM)) ?
        U_LOCAL_STEP_MM : U_NARROW_SHIFT_MM;
    if (route_leg == U_SOUTH) shift = U_SOUTH_SHIFT_MM;
    return LocalTargetAtDistance(step, near, front, shift) ||
        LocalTargetAtDistance(step, near, front, shift * 0.5f) ||
        LocalTargetAtDistance(step, near, front, shift * 0.25f);
}

static bool ReturnToSweepStart(const NavigationRange front[3], uint32_t near)
{
    if (!detour_step) returning_to_sweep_start = false;
    if (!returning_to_sweep_start) return false;
    float offset = sweep_start - (route_leg == U_EAST ? y : x);
    float remaining = detour_step > 0 ? offset : -offset;
    if (remaining <= 3.0f) {
        returning_to_sweep_start = false;
        side_blocked = false;
        return false;
    }
    return !side_blocked &&
        LocalTargetAtDistance(detour_step, near, front, fminf(remaining, U_SOUTH_SHIFT_MM));
}

static bool TryLocalStep(const NavigationRange front[3], uint32_t near, bool allow_forward)
{
    if (!FieldRoute() || entry_pending || finishing || local_step) return false;
    if (ReturnToSweepStart(front, near)) return true;
    if (returning_to_sweep_start) return false;
    bool forward_clear = allow_forward && !near;
    if (forward_clear && !prefer_lateral && LocalTarget(LegForwardStep(), near, front)) return true;

    if (!detour_step && !PreferRight(front) && cfg.start_x_mm == COURSE_ROUTE_START_X_MM && route_leg == U_EAST &&
        x >= 1080.0f && x < 1126.0f && y >= 880.0f && y < 980.0f &&
        LocalTargetAtDistance(GRID_WIDTH, near, front, fminf(40.0f, 980.0f - y))) return true;

    /* Aim the first free sideways probe at the gap between T9 and T11,
     * where a full-width probe can step beyond the usable lane. */
    float to_lane = U_T9_LANE_Y_MM - y;
    if (!detour_step && (!CenteredField() || !PreferRight(front)) && route_leg == U_EAST &&
        x >= U_T9_APPROACH_X_MM - 50.0f && x < 520.0f &&
        to_lane >= 12.0f && to_lane <= U_LOCAL_STEP_MM &&
        !(map.live_clear_mask & (1u << GRID_T9_INDEX)) &&
        LocalTargetAtDistance(GRID_WIDTH, near, front, to_lane))
        return true;

    int16_t left = route_leg == U_NORTH ? -1 : route_leg == U_EAST ? GRID_WIDTH : 1;
    if (detour_step != left && detour_step != -left) {
        if (CenteredField()) {
            bool same_side = cosf((MapYaw() - LegHeading()) * DEG_RAD) >= 0.0f;
            detour_step = PreferRight(front) == same_side ? -left : left;
        } else {
            bool same_side = cosf((MapYaw() - LegHeading()) * DEG_RAD) >= 0.0f;
            detour_step = ((front[0].mm >= front[2].mm) == same_side) ? left : -left;
        }
        if (!CenteredField() && route_leg == U_SOUTH && x >= U_SOUTH_INWARD_X_MM) detour_step = -1;
        detour_progress = ForwardProgress();
        sweep_start = route_leg == U_EAST ? y : x;
        side_blocked = false;
        tried_other_side = false;
    }
    bool side_clear = !side_blocked && LocalTarget(detour_step, near, front);
    if (!side_clear && !tried_other_side) {
        tried_other_side = true;
        detour_step = -detour_step;
        side_blocked = false;
        if (CenteredField()) {
            returning_to_sweep_start = true;
            if (ReturnToSweepStart(front, near)) return true;
            if (returning_to_sweep_start) return false;
        }
        side_clear = LocalTarget(detour_step, near, front);
    }
    if (side_clear) return true;
    prefer_lateral = false;
    return forward_clear && LocalTarget(LegForwardStep(), near, front);
}

static bool TryTurnBack(const NavigationRange front[3], uint32_t near)
{
    if (CenteredField() || !FieldRoute() || entry_pending || finishing || backtracked || !tried_other_side ||
        !CourseRotate(x, y) || !NearOutsideTurn(front, near)) return false;
    float distance = fminf(U_RETREAT_MAX_MM, tail_mm - U_RETREAT_RESERVE_MM);
    if (distance < U_RETREAT_MIN_MM) return false;
    float heading = tail_heading * DEG_RAD;
    float tx = x - distance * cosf(heading);
    float ty = y - distance * sinf(heading);
    if (!InLeg(tx, ty) || !CourseTravel(x, y, tx, ty) ||
        !CourseRotate(tx, ty)) return false;
    backtracked = true;
    prefer_lateral = true;
    local_x = tx;
    local_y = ty;
    local_step = LOCAL_BACKTRACK;
    clearance_paused = false;
    State(NAV_READY);
    return true;
}

static void SearchBlocked(void);

static void CheckOpenAisle(const NavigationRange front[3], bool aligned)
{
    uint32_t seen = 0;
    if (aligned && front[1].mm >= CLEARANCE_CENTER_RELEASE_MM &&
        front[0].mm >= U_CLOSE_MM && front[2].mm >= U_CLOSE_MM) {
        bool east = route_leg == U_EAST;
        bool south = route_leg == U_SOUTH;
        float sign = south ? -1.0f : 1.0f;
        float yaw = MapYaw();
        float progress = sign * (east ? x : y);
        float cross = east ? y : x;
        float body_pad = map.inflation_mm;
        for (unsigned i = 0; i < 3; ++i) {
            if (CenteredField() && i != 1) continue;
            float ox, oy;
            if (!Navigation_ProjectRange(&cfg, i, x, y, yaw, 0, &ox, &oy)) continue;
            float angle = (yaw + cfg.sensor_angle_deg[i]) * DEG_RAD;
            float along = sign * (east ? cosf(angle) : sinf(angle));
            float side = east ? sinf(angle) : cosf(angle);
            float origin = sign * (east ? ox : oy);
            float origin_side = east ? oy : ox;
            /* A ray must pass through the traced box, not just beside it.
             * The permanently forbidden centre and boundaries remain in the map. */
            for (unsigned box = 0; box < GRID_MOVABLE_COUNT; ++box) {
                uint32_t bit = 1u << box;
                if ((map.live_clear_mask | seen) & bit) continue;
                const GridRect r = grid_static_obstacles[box];
                float near_face = east ? r.x0 : south ? -r.y1 : r.y0;
                float far_face = east ? r.x1 : south ? -r.y0 : r.y1;
                float low = east ? r.y0 : r.x0;
                float high = east ? r.y1 : r.x1;
                if (near_face - progress < 25.0f || near_face - progress > 350.0f ||
                    cross < low - body_pad || cross > high + body_pad) continue;
                float depth = (far_face - origin) / along;
                float far_side = origin_side + depth * side;
                if (depth <= 0 || far_side < low + 10 || far_side > high - 10 ||
                    front[i].mm < depth + 35.0f) continue;
                seen |= bit;
                if (!(aisle_seen_mask & bit) || aisle_sensor[box] != i) {
                    aisle_sensor[box] = i;
                    aisle_tick[box] = front[i].tick;
                    continue;
                }
                if (front[i].tick - aisle_tick[box] >= U_AISLE_CLEAR_SAMPLE_MS) {
                    map.live_clear_mask |= bit;
                    prefer_lateral = false;
                    detour_step = 0;
                }
            }
        }
    }
    aisle_seen_mask = seen;
}

static void CheckLegForward(const NavigationRange front[3], uint32_t near, uint32_t now)
{
    if (ReturnToSweepStart(front, near)) return;
    if (returning_to_sweep_start) {
        if (near) WaitForClearance(front, near, now);
        else SearchBlocked();
        return;
    }
    if (reference_active && !reference_detour_active && local_step != LOCAL_ARC) {
        if (TryReference(front, near)) return;
    }
    Motion_SetCruise(0);
    if (join_active && TryCornerJoin(front, near)) return;
    if ((route_leg == U_NORTH && y >= U_FIELD_GATE_Y_MM - 2 &&
            CourseRotate(x, y)) ||
        (route_leg == U_EAST && x >= U_FIELD_SOUTH_X_MM - 2 && InLeg(x, y) &&
            CourseRotate(x, y))) {
        route_leg = (RouteLeg)(route_leg + 1);
        detour_step = 0;
        tried_other_side = false;
        side_blocked = false;
        backtracked = false;
        prefer_lateral = false;
    }
    bool turn_clearance_pending =
        (route_leg == U_NORTH && y >= U_FIELD_GATE_Y_MM - 2 &&
            !CourseRotate(x, y)) ||
        (route_leg == U_EAST && x >= U_FIELD_SOUTH_AFTER_T14_X_MM - 2 &&
            !CourseRotate(x, y));
    if (turn_clearance_pending && TryTurnClearanceRetreat(front, near)) return;
    if (TryCornerJoin(front, near)) return;
    float heading = LegHeading();
    bool aligned = fabsf(remainderf(heading - MapYaw(), 360.0f)) <= MOTION_ANGLE_TOLERANCE_DEG;
    CheckOpenAisle(front, aligned);
    if (route_leg == U_SOUTH && y <= cfg.goal_y_mm + 2 &&
        FieldExitClear() && !near && (aligned || NearOutsideTurn(front, near))) {
        finishing = true;
        FinishRun();
        return;
    }
    if (!aligned) {
        if (!NearOutsideTurn(front, near) || !CourseRotate(x, y)) {
            if (TryTurnClearanceRetreat(front, near)) return;
            local_step = LOCAL_IDLE;
            if (TryLocalStep(front, near, false)) return;
            if (near) { WaitForClearance(front, near, now); return; }
            SearchBlocked();
            return;
        }
        local_step = LOCAL_CHECK;
        next_heading = heading;
        if (EarlyCourseBend()) {
            if (!StartRollingBend(front, near, now)) SearchBlocked();
            return;
        }
        if (!StartHeadingTurn()) { Fail(NAV_MPU_FAULT); return; }
        State(NAV_TURNING);
        return;
    }
    /* The normal post-turn wait has already checked samples from this heading. */
    local_step = LOCAL_IDLE;
    if (TryLocalStep(front, near, !turn_clearance_pending) || TryTurnBack(front, near)) return;
    float distance = RetreatDistance();
    if (!backtracked && distance > 0.0f) {
        retreat_hit_x = -1.0f;
        if (!turn_clearance_pending && BackUp(distance)) {
            backtracked = true;
            prefer_lateral = true;
            tried_other_side = false;
            side_blocked = false;
            return;
        }
    }
    if (near) WaitForClearance(front, near, now);
    else SearchBlocked();
}

static void SearchBlocked(void)
{
    if (!FieldRoute() || entry_pending || finishing) { Fail(NAV_NO_PATH); return; }
    Motion_Cancel();
    local_step = LOCAL_IDLE;
    close_seen = 0;
    waiting_since = HAL_GetTick();
    State(NAV_SEARCH_BLOCKED);
}

static void FixedLineMove(void)
{
    if (!Motion_IsMPUReady()) { pose_valid = false; Fail(NAV_MPU_FAULT); return; }
    MotionStatus motion = Motion_GetStatus();
    if (state == NAV_TURNING) {
        if (motion == MOTION_DONE) State(NAV_READY);
        else if (motion != MOTION_RUNNING) {
            nav_diag.motion_result = motion;
            pose_valid = false;
            Fail(NAV_MOTION_FAULT);
        }
        return;
    }
    if (state == NAV_MOVING && motion != MOTION_RUNNING && motion != MOTION_DONE) {
        nav_diag.motion_result = motion;
        pose_valid = false;
        Fail(NAV_MOTION_FAULT);
        return;
    }
    if (state != NAV_READY && state != NAV_MOVING) return;

    unsigned count = sizeof(course_route) / sizeof(course_route[0]);
    if (fixed_line_point >= count) {
        pose_valid = false;
        Fail(y <= -(float)cfg.length_mm * 0.5f ? NAV_ARRIVED : NAV_POSE_LOST);
        return;
    }
    while (fixed_line_point + 1u < count &&
           hypotf(course_route[fixed_line_point][0] - x,
                  course_route[fixed_line_point][1] - y) < 45.0f)
        ++fixed_line_point;
    const int16_t *point = course_route[fixed_line_point];
    route_leg = point[2] == 3 ? U_SOUTH : point[2] == 2 ? U_EAST : U_NORTH;
    nav_diag.route_leg = route_leg;
    float tx = point[0], ty = point[1];
    float dx = tx - x, dy = ty - y;
    float remaining = hypotf(dx, dy);
    if (fixed_line_point + 1u == count && remaining <= 8.0f) {
        Motion_Cancel();
        ++fixed_line_point;
        State(NAV_READY);
        return;
    }
    if (state == NAV_MOVING && motion == MOTION_DONE) {
        State(NAV_READY);
        return;
    }
    float bearing = atan2f(dy, dx) / DEG_RAD;
    if (state == NAV_MOVING) {
        Motion_RetargetForward(fminf(100.0f, remaining), bearing - yaw_offset);
        return;
    }
    if (fabsf(remainderf(bearing - MapYaw(), 360.0f)) >
        MOTION_ANGLE_TOLERANCE_DEG) {
        next_heading = bearing;
        if (!StartHeadingTurn()) { Fail(NAV_MPU_FAULT); return; }
        State(NAV_TURNING);
        return;
    }
    float distance = fminf(100.0f, remaining);
    if (!Motion_StartDistanceHeading(1, distance, bearing - yaw_offset)) {
        Fail(NAV_MOTION_FAULT);
        return;
    }
    StartOdometry(1);
    State(NAV_MOVING);
}

static void UpdateNavigation(const NavigationRange front[3], int16_t speed)
{
    if (empty_floor_test && speed > 400) speed = 400;
    Odometry();
    uint32_t now = HAL_GetTick();
    if (fixed_line_course) {
        float pad = HeadingFootprint(MapYaw());
        if (x < pad || x > GRID_WIDTH * GRID_CELL_MM - pad ||
            y < -180.0f + pad || y > GRID_HEIGHT * GRID_CELL_MM - pad ||
            (x > U_LEFT_LANE_MM - pad && x < U_RIGHT_LANE_MM + pad && y < 600.0f + pad)) {
            pose_valid = false;
            Fail(NAV_BORDER);
            return;
        }
        FixedLineMove();
        return;
    }
    if (reference_active && !reference_detour_active && local_step != LOCAL_ARC) {
        if (x >= U_FIELD_SOUTH_AFTER_T14_X_MM || (x >= 1290.0f && y < 695.0f))
            route_leg = U_SOUTH;
        else if (route_leg == U_NORTH && x >= U_T9_APPROACH_X_MM && y >= U_FIELD_GATE_Y_MM)
            route_leg = U_EAST;
        nav_diag.route_leg = route_leg;
    }
    if (FieldRoute()) {
        float pad = map.turn_padding_mm;
        bool bottom_gate = (entry_pending && x < U_LEFT_LANE_MM - pad) ||
            (finishing && route_leg == U_SOUTH && x > U_RIGHT_LANE_MM + pad);
        float center_pad = CenteredField() && x >= 940.0f ?
            HeadingFootprint(MapYaw()) + U_CENTERED_MIDDLE_CLEARANCE_MM : pad + map.middle_extra_mm;
        if (x < pad || x > GRID_WIDTH * GRID_CELL_MM - pad ||
            y > GRID_HEIGHT * GRID_CELL_MM - pad ||
            (!bottom_gate && y < pad) ||
            (y < 600.0f + center_pad &&
             x > U_LEFT_LANE_MM - center_pad && x < U_RIGHT_LANE_MM + center_pad)) {
            pose_valid = false;
            Fail(NAV_BORDER);
            return;
        }
    }
    if (!Motion_IsMPUReady()) { pose_valid = false; Fail(NAV_MPU_FAULT); return; }
    MotionStatus motion = Motion_GetStatus();
    if ((state == NAV_MOVING || state == NAV_TURNING || state == NAV_ENTERING ||
         state == NAV_EXITING || state == NAV_RETREATING) &&
        motion != MOTION_RUNNING && motion != MOTION_DONE) {
        nav_diag.motion_result = motion;
        pose_valid = false; Fail(NAV_MOTION_FAULT); return;
    }
    uint32_t bad = 0;
    uint32_t near = 0;
    bool narrow_passage = FieldRoute() && !entry_pending && state != NAV_TURNING &&
        front[1].mm >= CLEARANCE_CENTER_RELEASE_MM;
    for (unsigned i = 0; i < 3; ++i) {
        bool range_ok = front[i].valid && front[i].mm < 8000;
        bool fresh = now - front[i].tick <= NAV_SENSOR_AGE_MS;
        uint16_t limit = FrontClearance_MinMm(i);
        if (i != 1) {
            if (CenteredField() || narrow_passage) limit = CloseRange(i);
            else if (TightDiagonals()) limit = U_BYPASS_DIAGONAL_MM;
        }
        if (fresh && range_ok && (front[i].mm < limit || (FieldRoute() &&
            state == NAV_MOVING && i != 1 && SideRoom(front, i) <= 2.0f))) {
            near |= 1u << i;
            nav_diag.near_front_mm[i] = front[i].mm;
        }
        if ((empty_floor_test || cfg.route_u) && front[i].no_echo &&
            (front[i].mm == 8190 || front[i].mm == 8191)) range_ok = true;
        if (!range_ok || !fresh)
            bad |= 1u << i;
    }
    nav_diag.invalid_front_mask = bad;
    if (near) nav_diag.near_front_mask = near;
    if (!bad && !near) AlignT11Face(front);
    if (state == NAV_MOVING && TrackEastTurnHits()) {
        float yaw = MapYaw();
        for (unsigned i = 0; i < 3; i += 2) {
            if ((bad & (1u << i)) || front[i].mm > 90u || turn_hit[i].tick == front[i].tick)
                continue;
            if (Navigation_ProjectRange(&cfg, i, x, y, yaw, front[i].mm,
                    &turn_hit[i].x, &turn_hit[i].y))
                turn_hit[i].tick = front[i].tick;
        }
    }
    if (state == NAV_RETREATING) {
        float yaw_error = fabsf(remainderf(next_heading - MapYaw(), 360.0f));
        if (yaw_error <= U_RETREAT_YAW_LIMIT_DEG) retreat_yaw_ok_at = now;
        bool drift = yaw_error > U_RETREAT_YAW_HARD_LIMIT_DEG ||
            now - retreat_yaw_ok_at >= U_RETREAT_YAW_SETTLE_MS;
        if (!bad && drift && FieldRoute() && !entry_pending && !finishing &&
            yaw_error <= U_RETREAT_YAW_RECOVER_DEG && InsideFieldMargin(x, y)) {
            tail_mm = 0.0f;
            backtracked = true;
            Plan();
            waiting_since = now;
        } else if (bad || drift) {
            pose_valid = false;
            Fail(bad ? NAV_TOF_FAULT : NAV_POSE_LOST);
        } else if (motion == MOTION_DONE) {
            if ((!FieldRoute() || entry_pending) && Grid_AddHit(&map, retreat_hit_x, retreat_hit_y))
                ++nav_diag.observed_hits;
            bypass_mm = U_BYPASS_DISTANCE_MM;
            entry_choice = 0;
            Plan();
        }
        return;
    }
    if (bad) {
        close_seen = 0;
        if (clearance_paused) clear_since = now;
        if (state != NAV_WAIT_SENSOR) {
            waiting_since = now;
            Pause(NAV_WAIT_SENSOR);
        } else if (now - waiting_since >= NAV_SENSOR_WAIT_MS) Fail(NAV_TOF_FAULT);
        return;
    }
    if (state == NAV_SEARCH_BLOCKED) {
        if (now - last_probe_retry >= U_PROBE_RETRY_MS && (!last_probe_retry ||
            fabsf(x - probe_retry_x) + fabsf(y - probe_retry_y) >= U_PROBE_RETRY_DISTANCE_MM)) {
            last_probe_retry = now;
            probe_retry_x = x;
            probe_retry_y = y;
            tried_other_side = false;
            side_blocked = false;
        }
        if (near) TryRetreat(front, near, now);
        else if (now - waiting_since >= NAV_FIELD_CLEAR_SETTLE_MS) {
            State(NAV_READY);
            waiting_since = 0;
            CheckLegForward(front, near, now);
        }
        return;
    }
    if (state == NAV_TURNING && motion == MOTION_DONE) {
        if (FieldRoute()) memset(candidate, 0, sizeof(candidate));
        waiting_since = now;
        State(NAV_READY);
        return;
    }
    if (state == NAV_READY && waiting_since) {
        if (now - waiting_since < (FieldRoute() ? NAV_FIELD_TURN_SETTLE_MS : 100u)) return;
        for (unsigned i = 0; i < 3; ++i) {
            int32_t sample_age = (int32_t)(front[i].tick - waiting_since);
            if (sample_age < 0 || ((FieldRoute() || local_step) && sample_age == 0)) {
                if (now - waiting_since >= NAV_SENSOR_WAIT_MS) Fail(NAV_TOF_FAULT);
                return;
            }
        }
        waiting_since = 0;
    }
    /* After backing up, the old obstacle can still be inside the forward
     * stop range while outside the swept body of a turn. */
    bool can_turn_away = bypass_mm > 0 && near == 2u && !entry_pending && !finishing &&
        (state == NAV_PLANNING || state == NAV_READY || state == NAV_TURNING) &&
        cfg.sensor_forward_mm[1] + front[1].mm > map.turn_padding_mm;
    if (FieldRoute() && (state == NAV_TURNING || (state == NAV_READY && local_step)) &&
        !entry_pending && near)
        can_turn_away = NearOutsideTurn(front, near);
    if (state == NAV_READY && local_step == LOCAL_CHECK) can_turn_away = true;
    if ((empty_floor_test || cfg.route_u) && (near || clearance_paused) && !can_turn_away) {
        WaitForClearance(front, near, now);
        return;
    }
    if (near && !can_turn_away) { pose_valid = false; Fail(NAV_POSE_LOST); return; }
    if (state == NAV_WAIT_SENSOR) { Resume(); return; }
    if (state == NAV_TURNING) return;
    if (FieldRoute() && state == NAV_MOVING && local_step && local_step != LOCAL_ARC &&
        local_step != LOCAL_BEND && !near &&
        !map.observed_count && Observe(front)) {
        bool blocked = local_step == LOCAL_REFERENCE && course_route[reference_point][2] >= 2 ?
            !ReferenceClear(local_x, local_y, next_heading, 2) :
            !CourseTravel(x, y, local_x, local_y) || !CourseRotate(local_x, local_y);
        if (local_step == LOCAL_REFERENCE && !ReferenceClear(
                course_route[reference_point][0], course_route[reference_point][1],
                next_heading, 2) && BeginArc(front, now))
            return;
        if (blocked && local_step == LOCAL_REFERENCE && BeginReferenceDetour(reference_point)) {
            memset(candidate, 0, sizeof(candidate));
            return;
        }
        /* Prefer fresh sensor data whenever the local fallback starts. */
        Grid_ClearHits(&map);
        if (blocked) {
            Motion_Cancel();
            memset(candidate, 0, sizeof(candidate));
            if (local_step == LOCAL_JOIN) join_active = false;
            if (local_step == LOCAL_REFERENCE) reference_active = false;
            if (local_step == LOCAL_SHIFT) side_blocked = true;
            if (local_step == LOCAL_FORWARD) {
                prefer_lateral = true;
                detour_step = 0;
            }
            local_step = LOCAL_CHECK;
            State(NAV_READY);
            return;
        }
    }
    if (!near && !empty_floor_test && (!FieldRoute() || entry_pending) &&
        Observe(front) && !local_step) {
        if (state != NAV_ENTERING || !EntryClear()) {
            entry_prefer_left = CenteredField() ? x >= U_LEFT_LANE_MM / 2 : front[0].mm > front[2].mm;
            entry_choice = 0;
            Plan();
            return;
        }
        entry_replan = true;
    }
    if (state == NAV_PLANNING) {
        uint32_t planning_started = HAL_GetTick();
        GridResult result = Grid_Advance(&map, 4);
        nav_diag.planner_last_ms = HAL_GetTick() - planning_started;
        if (nav_diag.planner_last_ms > nav_diag.planner_max_ms)
            nav_diag.planner_max_ms = nav_diag.planner_last_ms;
        nav_diag.expanded = map.expanded;
        if (result == GRID_SEARCHING) return;
        if (result != GRID_FOUND) {
            if (reference_detour_active) {
                unsigned next_point = reference_detour_point + 1u;
                reference_detour_active = false;
                if (BeginReferenceDetour(next_point)) return;
                reference_active = false;
                State(NAV_READY);
                return;
            }
            if (cfg.route_u && ++goal_choice < GoalChoices()) Plan();
            else if (cfg.route_u && !entry_pending && !narrow_detour) {
                /* Retry with the tighter side-ray limit and the same footprint. */
                narrow_detour = true;
                SetMapFrontClearance(U_BYPASS_DIAGONAL_MM);
                goal_choice = 0;
                Plan();
            }
            else Fail(NAV_NO_PATH);
            return;
        }
        /* Align to the first cell once after entry (X=200, grid X=225).
         * After a mid-step replan, go directly toward the NEW next cell:
         * returning to the old centre can require an unsafe U-turn. */
        path_index = entry_pending ? 0 : 1;
        State(NAV_READY);
    }
    if (state == NAV_MOVING || state == NAV_ENTERING || state == NAV_EXITING) {
        if (motion == MOTION_RUNNING) {
            float error = fabsf(remainderf(next_heading - MapYaw(), 360.0f));
            bool reference_tracking = state == NAV_MOVING && local_step == LOCAL_REFERENCE;
            bool detour_tracking = state == NAV_MOVING &&
                (local_step == LOCAL_SHIFT || local_step == LOCAL_JOIN);
            float limit = reference_tracking ?
                MOTION_FORWARD_ROLL_MAX_DEG + 1.0f : U_MOVING_HEADING_LIMIT_DEG;
            if (error > limit) {
                if (!moving_heading_lag_at) moving_heading_lag_at = now;
                if ((reference_tracking || detour_tracking) &&
                    error <= U_MOVING_HEADING_HARD_LIMIT_DEG &&
                    now - moving_heading_lag_at < U_MOVING_HEADING_RECOVER_MS) {
                    Motion_SetCruise(85u);
                    return;
                }
                pose_valid = false;
                Fail(NAV_POSE_LOST);
                return;
            }
            moving_heading_lag_at = 0;
            if (!near && FieldRoute()) {
                if (local_step == LOCAL_ARC) AdvanceArc(front, now);
                else if (local_step == LOCAL_BEND) {
                    if (!AdvanceRollingBend(front, now)) SearchBlocked();
                }
                else if (local_step == LOCAL_SHIFT) ExtendShift(front);
                else ExtendReference(front);
            }
            return;
        }
        Motion_Cancel();
        if (state == NAV_MOVING && local_step) {
            if (local_step == LOCAL_SHIFT || local_step == LOCAL_ARC) {
                if (local_step == LOCAL_ARC) {
                    Grid_ClearHits(&map);
                    reference_active = false;
                }
                last_retreat_hit = GRID_NONE;
                prefer_lateral = false;
                local_step = LOCAL_CHECK;
                State(NAV_READY);
            } else {
                if (local_step == LOCAL_BACKTRACK) {
                    side_blocked = false;
                    tried_other_side = false;
                    detour_progress = ForwardProgress();
                }
                goal_choice = 0;
                Plan();
            }
            return;
        }
        if (state == NAV_MOVING) ++path_index;
        State(NAV_READY);
    }
    if (state != NAV_READY) return;
    if (entry_pending) {
        RunEntry();
        return;
    }
    if (finishing) { FinishRun(); return; }
    if (FieldRoute() && !reference_detour_active &&
        (local_step == LOCAL_IDLE || local_step == LOCAL_CHECK)) {
        CheckLegForward(front, near, now);
        if (state != NAV_READY || local_step == LOCAL_IDLE || local_step == LOCAL_CHECK) return;
    }
    if (!local_step && path_index >= map.path_length) {
        if (reference_detour_active) {
            reference_point = reference_detour_point;
            reference_detour_active = false;
            reference_active = true;
            path_index = 0;
            State(NAV_READY);
            return;
        }
        if (!cfg.route_u) { Fail(NAV_ARRIVED); return; }
        if (route_leg != U_SOUTH) {
            route_leg = (RouteLeg)(route_leg + 1);
            goal_choice = 0;
            detour_step = 0;
            tried_other_side = false;
            side_blocked = false;
            backtracked = false;
            prefer_lateral = false;
            Plan();
        } else {
            finishing = true;
            if (!near) FinishRun();
        }
        return;
    }
    if (!local_step && path_index > 0) {
        /* Keep the 5 cm planning grid, but execute collinear edges as one
         * bounded run. Avoid a new stop/kick at every cell on slippery floors.
         * ToF and odometry still run every navigation update; new hits cancel
         * this run immediately and replan from the measured partial position. */
        int dx = (int)(map.path[path_index] % GRID_WIDTH) -
                 (int)(map.path[path_index-1] % GRID_WIDTH);
        int dy = (int)(map.path[path_index] / GRID_WIDTH) -
                 (int)(map.path[path_index-1] / GRID_WIDTH);
        while (path_index+1 < map.path_length) {
            unsigned a = map.path[path_index], b = map.path[path_index+1];
            if ((int)(b%GRID_WIDTH)-(int)(a%GRID_WIDTH) != dx ||
                (int)(b/GRID_WIDTH)-(int)(a/GRID_WIDTH) != dy) break;
            float bx = b%GRID_WIDTH*GRID_CELL_MM+GRID_CELL_MM/2;
            float by = b/GRID_WIDTH*GRID_CELL_MM+GRID_CELL_MM/2;
            float limit = empty_floor_test ? 100.0f : NAV_STRAIGHT_RUN_MM;
            if (hypotf(bx-x,by-y) > limit || !CourseTravel(x,y,bx,by)) break;
            ++path_index;
        }
    }
    float tx = local_x, ty = local_y;
    if (!local_step) {
        unsigned cell = map.path[path_index];
        tx = (cell % GRID_WIDTH) * GRID_CELL_MM + GRID_CELL_MM / 2;
        ty = (cell / GRID_WIDTH) * GRID_CELL_MM + GRID_CELL_MM / 2;
    }
    if (NorthCorner() && ty < y - 8.0f) {
        /* A detour built before reaching the corner can still point south. */
        reference_detour_active = false;
        local_step = LOCAL_IDLE;
        if (reference_active && BeginReferenceDetour(reference_point)) return;
        CheckLegForward(front, near, now);
        return;
    }
    next_distance = hypotf(tx - x, ty - y);
    next_heading = atan2f(ty - y, tx - x) / DEG_RAD;
    float error = remainderf(next_heading - MapYaw(), 360.0f);
    if (next_distance < 1.0f) {
        if (local_step) { goal_choice = 0; Plan(); }
        else ++path_index;
        return;
    }
    /* Check the actual continuous segment, including entry-centre alignment
     * and a partial cell after replanning, not just the two grid centres. */
    bool narrow = map.inflation_mm < map.turn_padding_mm;
    int16_t tolerance = cfg.route_u && cfg.position_margin_mm >= U_TRACKING_TOLERANCE_MM ?
        U_TRACKING_TOLERANCE_MM : 0;
    if (narrow)
        tolerance = fabsf(remainderf(next_heading, 90.0f)) <= MOTION_ANGLE_TOLERANCE_DEG ?
            0 : map.inflation_mm - map.turn_padding_mm;
    bool clear = local_step == LOCAL_REFERENCE && course_route[reference_point][2] >= 2 ?
        ReferenceClear(tx, ty, next_heading, 2) :
        CourseTravelWithTolerance(x, y, tx, ty, tolerance);
    if (!clear) {
        if (local_step == LOCAL_JOIN) join_active = false;
        if (local_step == LOCAL_REFERENCE) reference_active = false;
        local_step = LOCAL_IDLE;
        if (!TryLocalStep(front, near, true) && !TryTurnBack(front, near))
            SearchBlocked();
        return;
    }
    if (fabsf(error) > MOTION_ANGLE_TOLERANCE_DEG &&
        !(local_step == LOCAL_REFERENCE &&
          CanSteerReference(front, near, error, next_distance, course_route[reference_point][2]))) {
        if (EarlyCourseBend()) {
            if (!StartRollingBend(front, near, now)) SearchBlocked();
            return;
        }
        if (((narrow || near) && !CourseRotate(x, y)) ||
            ((narrow_passage || CenteredField()) && !NearOutsideTurn(front, near))) {
            float distance = narrow_detour ? RetreatDistance() : 0.0f;
            if (local_step == LOCAL_JOIN) join_active = false;
            if (local_step == LOCAL_REFERENCE) reference_active = false;
            if (distance > 0.0f) {
                /* The planner already has the obstruction. Back away to
                 * make space for the turn and plan again from there. */
                retreat_hit_x = -1.0f;
                if (BackUp(distance)) return;
            }
            local_step = LOCAL_IDLE;
            if (!TryLocalStep(front, near, true) && !TryTurnBack(front, near))
                SearchBlocked();
            return;
        }
        if (!FieldRoute()) tail_mm = 0.0f;
        if (!StartHeadingTurn()) { Fail(NAV_MPU_FAULT); return; }
        State(NAV_TURNING);
        return;
    }
    if (near) { WaitForClearance(front, near, now); return; }
    if (local_step == LOCAL_JOIN || local_step == LOCAL_REFERENCE) {
        float room = ForwardRoom(front);
        if (room < next_distance) {
            if (room < 40.0f) {
                join_active = false;
                if (local_step == LOCAL_REFERENCE) reference_active = false;
                local_step = LOCAL_CHECK;
                return;
            }
            float fraction = room / next_distance;
            local_x = x + (tx - x) * fraction;
            local_y = y + (ty - y) * fraction;
            next_distance = room;
        }
    }
    /* Reconstructing the step from two map coordinates can round it up. */
    if (local_step == LOCAL_FORWARD &&
        next_distance - (front[1].mm - (float)CLEARANCE_CENTER_MIN_MM) > U_DISTANCE_ROUNDING_MM) {
        local_step = LOCAL_CHECK;
        return;
    }
    if (local_step == LOCAL_BACKTRACK && front[1].mm < next_distance + CLEARANCE_CENTER_MIN_MM) {
        SearchBlocked();
        return;
    }
    if (local_step == LOCAL_SHIFT && front[1].mm < next_distance + CLEARANCE_CENTER_MIN_MM) {
        float room = ForwardRoom(front);
        float fraction = room / next_distance;
        float side_x = x + (tx - x) * fraction, side_y = y + (ty - y) * fraction;
        /* Once facing sideways, use the range actually seen in that direction.
         * Take a shorter safe probe before trying the opposite side. */
        if (!CenteredField() || room < U_MIN_SIDE_PROBE_MM || !CourseRotate(side_x, side_y)) {
            side_blocked = true;
            local_step = LOCAL_CHECK;
            return;
        }
        next_distance = room;
        local_x = side_x;
        local_y = side_y;
    }
    if (!Motion_StartDistanceHeading(1, next_distance, next_heading - yaw_offset)) {
        Fail(NAV_MOTION_FAULT); return;
    }
    SetForwardHeading();
    StartOdometry(1);
    State(NAV_MOVING);
    int16_t limit = FieldRoute() ? 360 : 500;
    Motion_Update(speed > limit ? limit : speed);
}

void Navigation_Update(const NavigationRange front[3], int16_t speed)
{
    if (!active) return;
    NavigationState previous = state;
    UpdateNavigation(front, speed);
    /* Keep the samples used by state transitions even between trace ticks. */
    RecordTrace(front, HAL_GetTick(), previous != state);
}

bool Navigation_IsActive(void) { return active; }
const GridMap *Navigation_GetMap(void) { return &map; }
const char *Navigation_GetLabel(void)
{
    if (empty_floor_test) {
        switch (state) {
          case NAV_PLANNING: return "TEST-TIM-DUONG";
          case NAV_READY: return "TEST-SAN-SANG";
          case NAV_TURNING: return "TEST-XOAY";
          case NAV_MOVING: return "TEST-TIEN";
          case NAV_ENTERING: return "TEST-VAO-SAN";
          case NAV_ARRIVED: return "TEST-DEN-DICH";
          case NAV_WAIT_CLEARANCE: return "TEST-CHO-TOF-GAN";
          case NAV_EXITING: return "TEST-QUA-VACH";
          default: break;
        }
    }
    static const char *const waiting[] = {"MAP_CHO_TOF", "MAP_CHO_TOF_TL", "MAP_CHO_TOF_G",
        "MAP_CHO_TOF_TL_G", "MAP_CHO_TOF_TR", "MAP_CHO_TOF_TL_TR", "MAP_CHO_TOF_G_TR", "MAP_CHO_TOF_TL_G_TR"};
    static const char *const faults[] = {"MAP_LOI_TOF", "MAP_LOI_TOF_TL", "MAP_LOI_TOF_G",
        "MAP_LOI_TOF_TL_G", "MAP_LOI_TOF_TR", "MAP_LOI_TOF_TL_TR", "MAP_LOI_TOF_G_TR", "MAP_LOI_TOF_TL_G_TR"};
    if (state == NAV_WAIT_SENSOR) return waiting[nav_diag.invalid_front_mask&7];
    if (state == NAV_TOF_FAULT) return faults[nav_diag.invalid_front_mask&7];
    if (FieldRoute() && state == NAV_TURNING) return "U-XOAY";
    if (FieldRoute() && state == NAV_ENTERING) return "U-VAO-SAN";
    if (cfg.route_u && state == NAV_MOVING) {
        static const char *const legs[] = {"U-LEN", "U-SANG-PHAI", "U-XUONG"};
        return legs[route_leg];
    }
    static const char *const names[] = {"MAP_CAN_CAU_HINH", "MAP_DUNG", "MAP_SAN_SANG",
        "A*-TIM-DUONG", "A*-XOAY", "A*-TIEN", "MAP_CHO_TOF", "DA_DEN_DICH",
        "MAP_KHONG_CO_DUONG", "MAP_CAN_DAT_LAI_VI_TRI", "MAP_LOI_MPU", "MAP_LOI_CHUYEN_DONG", "MAP_LOI_TOF",
        "A*-VAO-SAN", "MAP_LOI_VAO_BI_CHAN", "U_CHO_TOF_GAN",
        "U-QUA-VACH", "U_LOI_LOI_RA", "U-LUI-NE", "U-DO-LOI", "U-CHAM-VIEN"};
    return names[state];
}
