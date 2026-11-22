#include "navigation.h"
#include "motion.h"
#include "motor.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define DEG_RAD 0.01745329252f
#define NAV_SENSOR_AGE_MS 200u
#define NAV_SENSOR_WAIT_MS 2000u
#define NAV_OBSERVE_MM 300u
#define NAV_STRAIGHT_RUN_MM 150.0f

static GridMap map;
static NavigationConfig cfg;
static bool configured, active, pose_valid;
static NavigationState state;
static float x, y, yaw_offset, next_heading, next_distance;
static uint16_t path_index;
static int16_t last_encoder;
static float last_yaw;
static uint32_t sensor_seen[3], waiting_since;
static bool entry_pending;
static float entry_y;
static bool empty_floor_test;
volatile NavigationDiagnostics nav_diag;

static float MapYaw(void) { return Motion_GetAngle() + yaw_offset; }
static void State(NavigationState value)
{
    state = value;
    nav_diag.state = value;
    nav_diag.x_mm = (int32_t)x; nav_diag.y_mm = (int32_t)y;
    nav_diag.heading_mdeg = (int32_t)(MapYaw() * 1000.0f);
    nav_diag.path_index = path_index; nav_diag.path_length = map.path_length;
    nav_diag.expanded = map.expanded;
}

static void Fail(NavigationState reason)
{
    Motion_Cancel(); active = false;
    State(reason);
}

static void Odometry(void)
{
    if (state != NAV_MOVING && state != NAV_ENTERING) return;
    int16_t current = Encoder_GetCount2();
    int delta = (int16_t)((uint16_t)current - (uint16_t)last_encoder);
    last_encoder = current;
    float yaw = MapYaw();
    /* Forward-only map driving. One calibrated encoder gives an estimate,
     * NOT proof of translation on a slippery floor. Never snap to grid centres. */
    float mm = abs(delta) * (3.14159265f * MOTION_WHEEL_DIAMETER_MM /
                             MOTION_COUNTS_PER_WHEEL_REV);
    float midpoint = (last_yaw + yaw) * 0.5f * DEG_RAD;
    x += mm * cosf(midpoint); y += mm * sinf(midpoint);
    last_yaw = yaw;
    nav_diag.x_mm = (int32_t)x; nav_diag.y_mm = (int32_t)y;
    nav_diag.heading_mdeg = (int32_t)(yaw*1000.0f);
}

void Navigation_Init(const NavigationConfig *config)
{
    memset((void *)&nav_diag, 0, sizeof(nav_diag));
    cfg = *config;
    active = pose_valid = false;
    empty_floor_test = false;
    entry_pending = false;
    bool outside_start = cfg.start_y_mm <= 0 && cfg.start_y_mm >= -(int)cfg.length_mm / 2 &&
        cfg.start_x_mm > 0 && cfg.start_x_mm < 600 && cfg.start_heading_deg == 90;
    configured = cfg.enabled && cfg.length_mm > 0 && cfg.width_mm > 0 &&
        cfg.length_mm < 800 && cfg.width_mm < 800 &&
        cfg.clearance_mm <= 300 && cfg.position_margin_mm <= 300 &&
        (outside_start || Grid_Cell(cfg.start_x_mm, cfg.start_y_mm) != GRID_NONE) &&
        Grid_Cell(cfg.goal_x_mm, cfg.goal_y_mm) != GRID_NONE;
    float radius = ceilf(hypotf(cfg.length_mm, cfg.width_mm) * 0.5f);
    Grid_Init(&map, (uint16_t)(radius + cfg.clearance_mm + cfg.position_margin_mm));
    Grid_SetFrontSensors(&map, cfg.sensor_forward_mm, cfg.sensor_left_mm, cfg.sensor_angle_deg, 50);
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
    x = cfg.start_x_mm; y = cfg.start_y_mm;
    yaw_offset = cfg.start_heading_deg - Motion_GetAngle();
    if (y > 0 && Grid_IsBlocked(&map, Grid_Cell(x, y))) {
        pose_valid = false; State(NAV_NO_PATH); return false;
    }
    pose_valid = true;
    State(NAV_READY);
    return true;
}

static bool Plan(void)
{
    Motion_Cancel();
    ++nav_diag.replans;
    path_index = 0;
    if (entry_pending && !Grid_FindStartEntry(&map, x, fminf(y, 0), &entry_y)) {
        Fail(NAV_ENTRY_BLOCKED); return false;
    }
    GridResult result = Grid_Begin(&map, Grid_Cell(x, entry_pending ? entry_y : y),
                                  Grid_Cell(cfg.goal_x_mm, cfg.goal_y_mm));
    if (result == GRID_BAD_POSE) { Fail(NAV_NO_PATH); return false; }
    State(NAV_PLANNING);
    return true;
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
    if (test) {
        /* Room returns do not belong to the virtual arena. Start the explicit
         * empty-floor test from the static map; live near-range/IO stops
         * remain enabled below. Never carry room obstacles into this mode. */
        uint16_t pad=map.inflation_mm;
        Grid_Init(&map,pad);
        Grid_SetFrontSensors(&map,cfg.sensor_forward_mm,cfg.sensor_left_mm,cfg.sensor_angle_deg,50);
        nav_diag.observed_hits=0;
        memset(sensor_seen,0,sizeof(sensor_seen));
    }
    entry_pending = y <= 0;
    active = true;
    return Plan();
}
bool Navigation_Start(void) { return Start(false); }
bool Navigation_StartEmptyFloorTest(void) { return Start(true); }
bool Navigation_IsEmptyFloorTest(void) { return active && empty_floor_test; }

void Navigation_Cancel(void)
{
    if (active) { Odometry(); Motion_Cancel(); State(NAV_STOPPED); }
    active = pose_valid = false;
    empty_floor_test = false;
}

void Navigation_InvalidatePose(void)
{
    if (!active) return;
    Odometry(); pose_valid = false;
    Fail(NAV_POSE_LOST);
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

static bool Observe(const NavigationRange front[3])
{
    bool changed = false;
    for (unsigned i = 0; i < 3; ++i) {
        if (!front[i].valid) continue; /* no range is not a fabricated far hit */
        if (sensor_seen[i] == front[i].tick) continue;
        sensor_seen[i] = front[i].tick;
        if (front[i].mm > NAV_OBSERVE_MM) continue;
        float hit_x, hit_y;
        if (!Navigation_ProjectRange(&cfg, i, x, y, MapYaw(), front[i].mm, &hit_x, &hit_y)) continue;
        if (Grid_AddHit(&map, hit_x, hit_y)) {
            changed = true; ++nav_diag.observed_hits;
        }
    }
    return changed;
}

void Navigation_Update(const NavigationRange front[3], int16_t speed)
{
    if (!active) return;
    if (empty_floor_test && speed > 400) speed = 400;
    Odometry();
    if (!Motion_IsMPUReady()) { pose_valid = false; Fail(NAV_MPU_FAULT); return; }
    MotionStatus motion = Motion_GetStatus();
    if ((state == NAV_MOVING || state == NAV_TURNING || state == NAV_ENTERING) &&
        motion != MOTION_RUNNING && motion != MOTION_DONE) {
        pose_valid = false; Fail(NAV_MOTION_FAULT); return;
    }
    uint32_t now = HAL_GetTick();
    uint32_t bad = 0;
    for (unsigned i = 0; i < 3; ++i) {
        bool range_ok = front[i].valid && front[i].mm < 8000;
        if (empty_floor_test && front[i].no_echo &&
            (front[i].mm == 8190 || front[i].mm == 8191)) range_ok = true;
        if (!range_ok || now - front[i].tick > NAV_SENSOR_AGE_MS)
            bad |= 1u << i;
    }
    nav_diag.invalid_front_mask = bad;
    if (bad) {
        if (state != NAV_WAIT_SENSOR) {
            Motion_Cancel(); waiting_since = now; State(NAV_WAIT_SENSOR);
        } else if (now - waiting_since >= NAV_SENSOR_WAIT_MS) Fail(NAV_TOF_FAULT);
        return;
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (front[i].valid && front[i].mm < 50) {
            /* Main clearance guard may retreat, invalidating grid pose. */
            Navigation_InvalidatePose(); return;
        }
    }
    if (state == NAV_WAIT_SENSOR) { Plan(); return; }
    if (state == NAV_TURNING) {
        if (motion == MOTION_RUNNING) return;
        /* Rotation changes the front field of view. Wait for new ToF samples
         * at the final heading before interpreting hits or driving forward. */
        waiting_since = now;
        State(NAV_READY);
        return;
    }
    if (state == NAV_READY && now - waiting_since < 100) return;
    if (state == NAV_READY && waiting_since) {
        for (unsigned i = 0; i < 3; ++i) {
            if ((int32_t)(front[i].tick - waiting_since) < 0) {
                if (now - waiting_since >= NAV_SENSOR_WAIT_MS) Fail(NAV_TOF_FAULT);
                return;
            }
        }
        waiting_since = 0;
    }
    if (!empty_floor_test && Observe(front)) { Plan(); return; }
    if (state == NAV_PLANNING) {
        uint32_t planning_started = HAL_GetTick();
        GridResult result = Grid_Advance(&map, 4);
        nav_diag.planner_last_ms = HAL_GetTick() - planning_started;
        if (nav_diag.planner_last_ms > nav_diag.planner_max_ms)
            nav_diag.planner_max_ms = nav_diag.planner_last_ms;
        nav_diag.expanded = map.expanded;
        if (result == GRID_SEARCHING) return;
        if (result != GRID_FOUND) { Fail(NAV_NO_PATH); return; }
        /* Align to the first cell once after entry (X=200, grid X=225).
         * After a mid-step replan, go directly toward the NEW next cell:
         * returning to the old centre can require an unsafe U-turn. */
        path_index = entry_pending ? 0 : 1;
        State(NAV_READY);
    }
    if (state == NAV_MOVING || state == NAV_ENTERING) {
        if (motion == MOTION_RUNNING) {
            float error = remainderf(next_heading - MapYaw(), 360.0f);
            if (fabsf(error) > 15.0f) { pose_valid = false; Fail(NAV_POSE_LOST); }
            return;
        }
        Motion_Cancel();
        if (state == NAV_ENTERING) {
            if (y >= entry_y - 1.0f) entry_pending = false;
        } else ++path_index;
        State(NAV_READY);
    }
    if (state != NAV_READY) return;
    if (entry_pending) {
        /* No rotation before entry. Keep the surveyed north heading and use
         * measured distance from negative Y; never teleport the pose to Y=0. */
        next_heading = 90;
        float error = remainderf(next_heading - MapYaw(), 360.0f);
        if (fabsf(error) > 15.0f) { pose_valid = false; Fail(NAV_POSE_LOST); return; }
        float distance = fminf(MOTION_STEP_MM, entry_y - y);
        if (distance < 1.0f) { entry_pending = false; return; }
        if (!Motion_StartDistanceHeading(1, distance, next_heading - yaw_offset)) {
            Fail(NAV_MOTION_FAULT); return;
        }
        last_encoder = 0; last_yaw = MapYaw();
        State(NAV_ENTERING);
        return;
    }
    if (path_index >= map.path_length) { Fail(NAV_ARRIVED); return; }
    if (path_index > 0) {
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
            if (hypotf(bx-x,by-y) > limit || !Grid_CanTravel(&map,x,y,bx,by)) break;
            ++path_index;
        }
    }
    unsigned cell = map.path[path_index];
    float tx = (cell % GRID_WIDTH) * GRID_CELL_MM + GRID_CELL_MM / 2;
    float ty = (cell / GRID_WIDTH) * GRID_CELL_MM + GRID_CELL_MM / 2;
    next_distance = hypotf(tx - x, ty - y);
    next_heading = atan2f(ty - y, tx - x) / DEG_RAD;
    float error = remainderf(next_heading - MapYaw(), 360.0f);
    if (next_distance < 1.0f) { ++path_index; return; }
    /* Check the actual continuous segment, including entry-centre alignment
     * and a partial cell after replanning, not just the two grid centres. */
    if (!Grid_CanTravel(&map, x, y, tx, ty)) { Fail(NAV_NO_PATH); return; }
    if (fabsf(error) > MOTION_ANGLE_TOLERANCE_DEG) {
        if (!Motion_StartTurnHeading(next_heading - yaw_offset)) { Fail(NAV_MPU_FAULT); return; }
        State(NAV_TURNING);
        return;
    }
    if (!Motion_StartDistanceHeading(1, next_distance, next_heading - yaw_offset)) {
        Fail(NAV_MOTION_FAULT); return;
    }
    last_encoder = 0; last_yaw = MapYaw();
    State(NAV_MOVING);
    Motion_Update(speed > 500 ? 500 : speed); /* Cap map drive speed on slippery floors. */
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
          default: break;
        }
    }
    static const char *const waiting[] = {"MAP_CHO_TOF", "MAP_CHO_TOF_TL", "MAP_CHO_TOF_G",
        "MAP_CHO_TOF_TL_G", "MAP_CHO_TOF_TR", "MAP_CHO_TOF_TL_TR", "MAP_CHO_TOF_G_TR", "MAP_CHO_TOF_TL_G_TR"};
    static const char *const faults[] = {"MAP_LOI_TOF", "MAP_LOI_TOF_TL", "MAP_LOI_TOF_G",
        "MAP_LOI_TOF_TL_G", "MAP_LOI_TOF_TR", "MAP_LOI_TOF_TL_TR", "MAP_LOI_TOF_G_TR", "MAP_LOI_TOF_TL_G_TR"};
    if (state == NAV_WAIT_SENSOR) return waiting[nav_diag.invalid_front_mask&7];
    if (state == NAV_TOF_FAULT) return faults[nav_diag.invalid_front_mask&7];
    static const char *const names[] = {"MAP_CAN_CAU_HINH", "MAP_DUNG", "MAP_SAN_SANG",
        "A*-TIM-DUONG", "A*-XOAY", "A*-TIEN", "MAP_CHO_TOF", "DA_DEN_DICH",
        "MAP_KHONG_CO_DUONG", "MAP_CAN_DAT_LAI_VI_TRI", "MAP_LOI_MPU", "MAP_LOI_CHUYEN_DONG", "MAP_LOI_TOF",
        "A*-VAO-SAN", "MAP_LOI_VAO_BI_CHAN"};
    return names[state];
}
