#include "navigation.h"
#include "motion.h"
#include "motor.h"
#ifndef NAV_NEW_COURSE_TEST
#define NAV_TEST_START_X_MM 200
#endif
#include "navigation_config.h"
#include "front_clearance.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %d: %s (nav=%lu x=%ld y=%ld leg=%lu heading=%ld)\n", \
    __LINE__, #c, (unsigned long)nav_diag.state, (long)nav_diag.x_mm, (long)nav_diag.y_mm, \
    (unsigned long)nav_diag.route_leg, (long)nav_diag.heading_mdeg); exit(1); } } while (0)
static uint32_t tick;
static bool ready = true;
static float yaw, target_yaw;
static int16_t encoder;
static int count_target;
static MotionStatus motion;
static bool turning;
static float drive_start_x, drive_start_y, drive_goal_yaw;
static float longest_step;
static unsigned distance_commands;
#ifdef NAV_NEW_COURSE_TEST
static unsigned simulated_cruise;
static float simulated_speed, simulated_fraction;
static bool simulated_steering_lag;
static bool simulated_fast_first_curve;
#endif
static int drive_direction;
static int16_t max_requested_speed;
static bool calibrated_course;
static bool extra_obstacle;
static uint32_t removed_boxes;
static GridRect extra_rect = {600,825,700,925};
uint32_t HAL_GetTick(void) { return tick; }
bool Motion_IsMPUReady(void) { return ready; }
float Motion_GetAngle(void) { return yaw; }
void Motion_Cancel(void) { motion = MOTION_IDLE; }
MotionStatus Motion_GetStatus(void) { return motion; }
void Motion_SetCourseSteering(bool enabled) { calibrated_course = enabled; }
static float EncoderMmPerCount(void)
{
    return calibrated_course ? MOTION_FIELD_MM_PER_COUNT : MOTION_WHEEL_MM_PER_COUNT;
}
void Motion_SetCruise(uint16_t speed)
{
#ifdef NAV_NEW_COURSE_TEST
    simulated_cruise = speed;
#else
    (void)speed;
#endif
}
int16_t Encoder_GetCount2(void) { return encoder; }
bool Motion_StartTurnHeading(float h)
{
    target_yaw = yaw + remainderf(h - yaw, 360);
    turning = true; motion = MOTION_RUNNING; return ready;
}
bool Motion_StartDistanceHeading(int dir, float mm, float heading)
{
    CHECK((dir == 1 || dir == -1) && mm >= 1 && mm <= 150);
    if (dir == -1) CHECK(mm >= 10 && mm <= 90);
    drive_direction = dir;
    drive_goal_yaw = heading;
    longest_step=fmaxf(longest_step,mm);distance_commands++;
    float heading_tolerance = calibrated_course && dir > 0 ? 8.0f : MOTION_ANGLE_TOLERANCE_DEG;
    CHECK(fabsf(remainderf(heading - yaw, 360)) <= heading_tolerance);
    encoder = 0;
#ifdef NAV_NEW_COURSE_TEST
    simulated_speed = simulated_fraction = 0;
#endif
    drive_start_x = nav_diag.x_mm; drive_start_y = nav_diag.y_mm;
    count_target = (int)ceilf(mm / EncoderMmPerCount());
    turning = false; motion = MOTION_RUNNING; return ready;
}
bool Motion_ExtendForward(float mm, float heading)
{
    float mm_per_count = EncoderMmPerCount();
    float remaining = (count_target - encoder) * mm_per_count;
    if (motion != MOTION_RUNNING || turning || drive_direction != 1 ||
        mm > 150 || remaining > 35 || mm < remaining + 6 ||
        fabsf(remainderf(heading - yaw, 360)) >
            (calibrated_course ? MOTION_FORWARD_ROLL_MAX_DEG : 17.0f)) return false;
    drive_goal_yaw = heading;
    count_target = encoder + (int)ceilf(mm / mm_per_count);
    return true;
}
bool Motion_SteerForward(float heading)
{
    if (motion != MOTION_RUNNING || turning || drive_direction != 1 ||
        fabsf(remainderf(heading - yaw, 360.0f)) > 14.0f) return false;
    drive_goal_yaw = heading;
    return true;
}
bool Motion_RetargetForward(float distance, float heading)
{
    if (!Motion_SteerForward(heading)) return false;
    count_target = encoder + (int)ceilf(distance / EncoderMmPerCount());
    return true;
}
void Motion_Update(int16_t speed) { if (speed>max_requested_speed) max_requested_speed=speed; }
float Motion_GetMeasuredForwardSpeed(void)
{
#ifdef NAV_NEW_COURSE_TEST
    return simulated_speed;
#else
    return 0.0f;
#endif
}
static void Advance(void)
{
    tick += 10;
    if (motion != MOTION_RUNNING) return;
    if (turning) {
        float error = target_yaw - yaw;
        if (fabsf(error) <= 1.5f) { yaw = target_yaw; motion = MOTION_DONE; }
        else yaw += error > 0 ? 1.5f : -1.5f;
    } else {
        int advance = 8;
#ifdef NAV_NEW_COURSE_TEST
        if (simulated_cruise && drive_direction > 0) {
            float mm_per_count = EncoderMmPerCount();
            float goal = fminf(simulated_cruise, sqrtf(900.0f * (count_target - encoder) * mm_per_count));
            bool fast = simulated_fast_first_curve && nav_diag.y_mm >= 500 &&
                nav_diag.y_mm < 875 && nav_diag.x_mm < 620;
            if (fast) goal = fmaxf(goal, 300.0f);
            simulated_speed += fmaxf(-4.5f, fminf(3.0f, goal - simulated_speed));
            simulated_fraction += fmaxf(4.0f, simulated_speed) * 0.01f / mm_per_count;
            advance = (int)simulated_fraction;
            simulated_fraction -= advance;
            float error = remainderf(drive_goal_yaw - yaw, 360.0f);
            float limit = fast ? 0.55f : simulated_steering_lag ? 1.2f : 1.5f;
            float gain = simulated_steering_lag ? 0.14f : 0.32f;
            yaw += fmaxf(-limit, fminf(limit, error * gain));
        } else if (calibrated_course) {
            simulated_fraction += 8.0f * MOTION_WHEEL_MM_PER_COUNT / EncoderMmPerCount();
            advance = (int)simulated_fraction;
            simulated_fraction -= advance;
        }
#endif
        encoder += advance;
        if (encoder >= count_target) { encoder = count_target; motion = MOTION_DONE; }
    }
}
static void Fresh(NavigationRange f[3])
{
    for (unsigned i = 0; i < 3; ++i) f[i] = (NavigationRange){.mm=1000,.valid=true,.tick=tick};
}
static float HitDistance(float x, float y, float dx, float dy, GridRect r)
{
    float lo = 0, hi = 1500;
    float p[2] = {x,y}, d[2] = {dx,dy}, a[2] = {r.x0,r.y0}, b[2] = {r.x1,r.y1};
    for (unsigned k = 0; k < 2; ++k) {
        if (fabsf(d[k]) < 0.0001f) { if (p[k] < a[k] || p[k] > b[k]) return 1500; }
        else {
            float near = (a[k]-p[k])/d[k], far = (b[k]-p[k])/d[k];
            if (near > far) { float t = near; near = far; far = t; }
            lo = fmaxf(lo, near); hi = fminf(hi, far);
            if (lo > hi) return 1500;
        }
    }
    return lo;
}
static void StaticMapRanges(NavigationRange f[3])
{
    Fresh(f);
    float angle = (yaw + navigation_config.start_heading_deg) * 0.01745329252f;
    float x = nav_diag.x_mm, y = nav_diag.y_mm;
    if (nav_diag.state == NAV_MOVING || nav_diag.state == NAV_ENTERING ||
        nav_diag.state == NAV_EXITING || nav_diag.state == NAV_RETREATING) {
        float mm = drive_direction * encoder * EncoderMmPerCount();
        x = drive_start_x + mm*cosf(angle); y = drive_start_y + mm*sinf(angle);
    }
    for (unsigned i = 0; i < 3; ++i) {
        float sx = x + 45*cosf(angle) - navigation_config.sensor_left_mm[i]*sinf(angle);
        float sy = y + 45*sinf(angle) + navigation_config.sensor_left_mm[i]*cosf(angle);
        float ray = angle + navigation_config.sensor_angle_deg[i]*0.01745329252f;
        float range = 1500;
        for (unsigned n = 0; n < 18; ++n)
            if (!(removed_boxes & (1u << n)))
                range = fminf(range, HitDistance(sx,sy,cosf(ray),sinf(ray),grid_static_obstacles[n]));
        if (extra_obstacle)
            range = fminf(range, HitDistance(sx,sy,cosf(ray),sinf(ray),extra_rect));
        f[i].mm = (uint16_t)range;
    }
}
static NavigationConfig Config(void)
{
    return (NavigationConfig){.enabled = true, .length_mm = 80, .width_mm = 80,
        .start_x_mm = 275, .start_y_mm = 1025, .goal_x_mm = 1575, .goal_y_mm = 1025,
        .start_heading_deg = 0, .sensor_angle_deg = {45,0,-45}};
}
static void Reset(void)
{
    NavigationConfig cfg = Config();
    ready = true; yaw = 0; motion = MOTION_IDLE; encoder = 0;
    Navigation_Init(&cfg);
    CHECK(!Navigation_Start()); /* Must explicitly confirm physical start pose. */
    CHECK(Navigation_ConfirmStart());
    CHECK(Navigation_Start());
}
static int TraceCourse(void)
{
    Navigation_Init(&navigation_config);
    CHECK(Navigation_ConfirmStart() && Navigation_StartEmptyFloorTest());
    NavigationRange missing[3] = {0};
    ready = false;
    Navigation_Update(missing, 300);
    CHECK(nav_diag.state == NAV_MPU_FAULT && !Navigation_IsActive());
    ready = true;
    Navigation_Init(&navigation_config);
    CHECK(Navigation_ConfirmStart() && Navigation_StartEmptyFloorTest());
    for (unsigned i = 0; i < 200 && motion != MOTION_RUNNING; ++i) {
        tick += 10;
        Navigation_Update(missing, 300);
    }
    CHECK(motion == MOTION_RUNNING);
    motion = MOTION_ENCODER_ERROR;
    tick += 10;
    Navigation_Update(missing, 300);
    CHECK(nav_diag.state == NAV_MOTION_FAULT && !Navigation_IsActive());
    Navigation_Init(&navigation_config);
    CHECK(Navigation_ConfirmStart() && Navigation_StartEmptyFloorTest());
    printf("{\"width\":%d,\"height\":%d,\"cell\":%d,\"body\":[%u,%u],\"stepMs\":10,\"obstacles\":[",
        GRID_WIDTH * GRID_CELL_MM, GRID_HEIGHT * GRID_CELL_MM, GRID_CELL_MM,
        navigation_config.length_mm, navigation_config.width_mm);
    for (unsigned i = 0; i < 18; ++i) {
        const GridRect r = grid_static_obstacles[i];
        printf("%s[%d,%d,%d,%d]", i ? "," : "", r.x0, r.y0, r.x1, r.y1);
    }
    printf("],\"frames\":[[%ld,%ld,%d,0]", (long)nav_diag.x_mm, (long)nav_diag.y_mm,
        navigation_config.start_heading_deg);
    int32_t east_max_y = 0;
    for (unsigned i = 0; i < 50000 && Navigation_IsActive(); ++i) {
        NavigationRange front[3];
        Advance();
        for (unsigned sensor = 0; sensor < 3; ++sensor)
            front[sensor] = (NavigationRange){.mm=8191, .tick=tick, .no_echo=true};
        Navigation_Update(front, 400);
        if (nav_diag.route_leg == 1 && nav_diag.y_mm > east_max_y)
            east_max_y = nav_diag.y_mm;
        printf(",[%ld,%ld,%.1f,%lu]", (long)nav_diag.x_mm, (long)nav_diag.y_mm,
            yaw + navigation_config.start_heading_deg, (unsigned long)nav_diag.route_leg);
    }
    CHECK(nav_diag.state == NAV_ARRIVED);
    CHECK(nav_diag.x_mm >= 1275 && nav_diag.x_mm <= 1325);
    CHECK(nav_diag.y_mm <= -54 && east_max_y <= 905);
    puts("]}");
    return 0;
}

int main(int argc, char **argv)
{
    if (argc == 2 && !strcmp(argv[1], "--trace")) return TraceCourse();
    NavigationRange front[3];
    NavigationConfig cfg = Config(); cfg.enabled = false;
    Navigation_Init(&cfg);
    CHECK(!Navigation_ConfirmStart() && !Navigation_Start());
    Reset();
    for (unsigned i = 0; i < 30000 && Navigation_IsActive(); ++i) {
        Advance(); Fresh(front); Navigation_Update(front, 420);
    }
    CHECK(nav_diag.state == NAV_ARRIVED && !Navigation_IsActive());
    CHECK(Grid_Cell(nav_diag.x_mm, nav_diag.y_mm) == Grid_Cell(1575,1025));

    Reset();
    bool injected = false;
    for (unsigned i = 0; i < 30000 && Navigation_IsActive(); ++i) {
        Advance(); Fresh(front);
        if (!injected && nav_diag.state == NAV_MOVING && encoder > 16) {
            front[1].mm = 120; injected = true;
        }
        Navigation_Update(front, 420);
    }
    CHECK(injected && nav_diag.observed_hits > 0 && nav_diag.replans > 1);
    CHECK(nav_diag.state == NAV_ARRIVED);

    Reset(); Advance(); Fresh(front); front[0].valid = false;
    Navigation_Update(front, 420);
    CHECK(nav_diag.state == NAV_WAIT_SENSOR && motion == MOTION_IDLE);
    CHECK(nav_diag.invalid_front_mask==1);
    CHECK(!strcmp(Navigation_GetLabel(),"MAP_CHO_TOF_TL"));
    tick += 2001; Fresh(front); front[0].valid = false;
    Navigation_Update(front, 420);
    CHECK(nav_diag.state == NAV_TOF_FAULT && !Navigation_IsActive());
    CHECK(!strcmp(Navigation_GetLabel(),"MAP_LOI_TOF_TL"));
    CHECK(Navigation_ConfirmStart() && nav_diag.last_fault_state == NAV_TOF_FAULT);
    Reset();Advance();Fresh(front);front[0].valid=front[2].valid=false;
    Navigation_Update(front,420);
    CHECK(nav_diag.invalid_front_mask==5);
    CHECK(!strcmp(Navigation_GetLabel(),"MAP_CHO_TOF_TL_TR"));

    Reset(); Advance(); Fresh(front); front[1].tick = tick - 201;
    Navigation_Update(front, 420);
    CHECK(nav_diag.state == NAV_WAIT_SENSOR);
    Advance(); Fresh(front); Navigation_Update(front, 420);
    CHECK(nav_diag.state == NAV_PLANNING);
    Navigation_InvalidatePose();
    CHECK(nav_diag.state == NAV_POSE_LOST && !Navigation_Start());
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    ready = false;
    Advance(); Fresh(front); Navigation_Update(front, 420);
    CHECK(nav_diag.state == NAV_MPU_FAULT && !Navigation_IsActive());

    Reset();
    for (unsigned i = 0; i < 10000 && nav_diag.state != NAV_MOVING; ++i) {
        Advance(); Fresh(front); Navigation_Update(front, 420);
    }
    CHECK(nav_diag.state == NAV_MOVING);
    motion = MOTION_ENCODER_ERROR;
    Advance(); Fresh(front); Navigation_Update(front, 420);
    CHECK(nav_diag.state == NAV_MOTION_FAULT && !Navigation_IsActive());
    CHECK(!Navigation_Start()); /* untrusted pose needs confirmation */
    Reset(); Navigation_Cancel();
    CHECK(!Navigation_IsActive() && !Navigation_Start());
    /* Nose at Y=0 means centre=-45 mm. Confirm without unsigned wrapping,
     * preflight the FULL route before moving, then enter north in 5 cm steps. */
    cfg = Config(); cfg.length_mm = cfg.width_mm = 90;
    cfg.start_x_mm = 200; cfg.start_y_mm = -45; cfg.start_heading_deg = 90;
    cfg.goal_x_mm = 1575; cfg.goal_y_mm = 75;
    ready = true; yaw = 0; motion = MOTION_IDLE;
    Navigation_Init(&cfg);
    CHECK(Navigation_ConfirmStart());
    CHECK(nav_diag.x_mm == 200 && nav_diag.y_mm == -45);
    CHECK(Navigation_Start());
    bool entered = false;
    for (unsigned i = 0; i < 30000 && Navigation_IsActive(); ++i) {
        Advance(); Fresh(front); Navigation_Update(front, 420);
        if (nav_diag.state == NAV_ENTERING) {
            CHECK(!turning && fabsf(remainderf(yaw,360)) < 0.01f);
            entered = true;
        }
    }
    CHECK(entered && nav_diag.state == NAV_ARRIVED);
    CHECK(Grid_Cell(nav_diag.x_mm, nav_diag.y_mm) == Grid_Cell(1575,75));
    /* Production conservative margins make this entry impossible. Refuse
     * before starting a motor; never silently shrink safety to find a path. */
    cfg.clearance_mm = 50; cfg.position_margin_mm = 25;
    Navigation_Init(&cfg);
    CHECK(Navigation_ConfirmStart());
    motion = MOTION_IDLE;
    CHECK(!Navigation_Start() && motion == MOTION_IDLE);
    CHECK(nav_diag.state == NAV_ENTRY_BLOCKED && nav_diag.y_mm == -45);
    cfg.start_y_mm = -46;
    Navigation_Init(&cfg);
    CHECK(!Navigation_ConfirmStart() && nav_diag.state == NAV_UNCONFIGURED);
    /* Production 25-degree geometry and the actual start outside the grid. */
    float hx, hy;
    CHECK(Navigation_ProjectRange(&navigation_config, 0, 200, -45, 90, 100, &hx, &hy));
    CHECK(fabsf(hx - (175 - 100*sinf(25*0.01745329252f))) < 0.01f);
    CHECK(fabsf(hy - 100*cosf(25*0.01745329252f)) < 0.01f);
    CHECK(Navigation_ProjectRange(&navigation_config, 2, 200, -45, 90, 100, &hx, &hy));
    CHECK(hx > 225 && fabsf(hy - 90.63078f) < 0.01f);
    CHECK(Navigation_ProjectRange(&navigation_config, 1, 200, -45, 90, 100, &hx, &hy));
    CHECK(fabsf(hx - 200) < 0.01f && fabsf(hy - 100) < 0.01f);
    CHECK(!Navigation_ProjectRange(&navigation_config, 3, 200, -45, 90, 100, &hx, &hy));
    CHECK(!Navigation_ProjectRange(&navigation_config, 0, 200, -45, 90, 8191, &hx, &hy));
    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    longest_step=0;distance_commands=0;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    CHECK(Navigation_GetMap()->inflation_mm == 65);
    CHECK(Navigation_GetMap()->turn_padding_mm == 74);
    /* Reproduce the three front returns observed before the failed G entry. */
    for (unsigned i = 0; i < 3; ++i) {
        Advance(); Fresh(front);
        front[0].mm = i == 0 ? 145 : 100;
        front[1] = (NavigationRange){.mm=8191, .tick=tick, .no_echo=true};
        front[2].mm = 211;
        Navigation_Update(front, 420);
        CHECK(Navigation_IsActive() && nav_diag.observed_hits == 0);
    }
    for (unsigned i = 0; i < 40000 && Navigation_IsActive(); ++i) {
        Advance(); StaticMapRanges(front); Navigation_Update(front, 420);
        CHECK(nav_diag.state != NAV_PLANNING && nav_diag.expanded == 0 && nav_diag.replans == 0);
        if (nav_diag.state == NAV_MOVING) {
            CHECK(nav_diag.x_mm >= 99 && nav_diag.x_mm <= 1701 && nav_diag.y_mm <= 1101);
            if (nav_diag.route_leg != 0) CHECK(nav_diag.y_mm >= 99);
            if (nav_diag.route_leg == 1 && nav_diag.x_mm > 506 && nav_diag.x_mm < 600 &&
                fabsf(remainderf(yaw + 90.0f, 360.0f)) < 3.0f)
                CHECK(nav_diag.y_mm >= 895);
        }
    }
    CHECK(nav_diag.state == NAV_ARRIVED && longest_step >= 90 && longest_step <= 101);
    CHECK(nav_diag.y_mm <= -54 && nav_diag.x_mm > 1324 && nav_diag.x_mm <= 1701);
    CHECK(!(Navigation_GetMap()->live_clear_mask & (1u << GRID_T9_INDEX)));
    CHECK(!(Navigation_GetMap()->live_clear_mask & (1u << GRID_T3_INDEX)));

    /* A new object on the north leg is mapped from repeated ToF samples
     * before it reaches the near-stop threshold, then safely bypassed. */
    extra_obstacle = true;
    extra_rect = (GridRect){150,800,250,900};
    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    bool mapped_before_near = false;
    for (unsigned i = 0; i < 40000 && Navigation_IsActive(); ++i) {
        Advance(); StaticMapRanges(front); Navigation_Update(front, 420);
        if (nav_diag.observed_hits && front[1].mm > CLEARANCE_CENTER_MIN_MM)
            mapped_before_near = true;
    }
    CHECK(mapped_before_near && nav_diag.retreats == 0 && nav_diag.state == NAV_ARRIVED);
    extra_obstacle = false;

    /* T3 has been physically removed and the entrance has an old hit. */
    removed_boxes = 1u << GRID_T3_INDEX;
    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    CHECK(Grid_AddHit((GridMap *)Navigation_GetMap(), 205, 255));
    bool passed_t3_lane = false;
    for (unsigned i = 0; i < 40000 && Navigation_IsActive(); ++i) {
        Advance(); StaticMapRanges(front); Navigation_Update(front, 420);
        passed_t3_lane |= nav_diag.route_leg == 0 && nav_diag.state == NAV_MOVING &&
            nav_diag.y_mm >= 300 && nav_diag.y_mm <= 410 &&
            abs(nav_diag.x_mm - navigation_config.start_x_mm) <= 5;
    }
    CHECK(passed_t3_lane && (Navigation_GetMap()->live_clear_mask & (1u << GRID_T3_INDEX)));
    CHECK(nav_diag.state == NAV_ARRIVED);
    removed_boxes = 0;

    /* A confirmed 24 mm return takes the short reverse before another probe. */
    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    bool close_early = false, backed_off = false, shifted_after_braking = false;
    int32_t brake_y = 0;
    for (unsigned i = 0; i < 40000 && Navigation_IsActive(); ++i) {
        Advance(); StaticMapRanges(front);
        if (!close_early && nav_diag.route_leg == 0 && nav_diag.state == NAV_MOVING &&
            nav_diag.y_mm >= 145 && nav_diag.y_mm <= 160) {
            close_early = true;
            brake_y = nav_diag.y_mm;
        }
        if (close_early && !nav_diag.retreats) front[1].mm = 24;
        Navigation_Update(front, 420);
        if (nav_diag.state == NAV_RETREATING) {
            CHECK(drive_direction == -1 && nav_diag.y_mm >= brake_y - 21);
            backed_off |= nav_diag.y_mm <= brake_y - 10;
        }
        if (backed_off && !shifted_after_braking && nav_diag.state == NAV_MOVING) {
            CHECK(fabsf(fabsf(remainderf(drive_goal_yaw, 360.0f)) - 90.0f) < 3.0f);
            shifted_after_braking = true;
        }
    }
    CHECK(close_early && backed_off && shifted_after_braking && nav_diag.state == NAV_ARRIVED);

    /* A close return after turning cannot authorize reversing off the
     * travelled track. Resume the course once the new heading clears. */
    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    bool injected_misaligned_close = false;
    unsigned misaligned_close_samples = 0;
    for (unsigned i = 0; i < 50000 && Navigation_IsActive(); ++i) {
        Advance(); Fresh(front);
        if (!injected_misaligned_close && nav_diag.route_leg == 2 &&
            nav_diag.state == NAV_READY && nav_diag.y_mm > 300 && nav_diag.y_mm < 380) {
            yaw = -155.0f;
            injected_misaligned_close = true;
        }
        if (injected_misaligned_close && misaligned_close_samples < 12) {
            front[1].mm = 10;
            ++misaligned_close_samples;
        }
        Navigation_Update(front, 420);
        CHECK(nav_diag.state != NAV_RETREATING);
    }
    CHECK(injected_misaligned_close && misaligned_close_samples == 12);
    CHECK(nav_diag.state == NAV_ARRIVED);

    /* Remove T9 from the physical rays only, leaving the traced map intact. */
    removed_boxes = 1u << GRID_T9_INDEX;
    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    bool passed_t9_lane = false;
    for (unsigned i = 0; i < 40000 && Navigation_IsActive(); ++i) {
        Advance(); StaticMapRanges(front); Navigation_Update(front, 420);
        passed_t9_lane |= nav_diag.route_leg == 1 && nav_diag.state == NAV_MOVING &&
            nav_diag.x_mm >= 560 && nav_diag.x_mm <= 645 && nav_diag.y_mm < 750 &&
            fabsf(remainderf(yaw + 90.0f, 360.0f)) < 3.0f;
    }
    CHECK(passed_t9_lane && (Navigation_GetMap()->live_clear_mask & (1u << GRID_T9_INDEX)) && nav_diag.state == NAV_ARRIVED);
    removed_boxes = 0;

    /* The same live check applies away from T3 and T9. */
    const uint32_t t14_bit = 1u << 13;
    removed_boxes = t14_bit;
    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    for (unsigned i = 0; i < 40000 && Navigation_IsActive(); ++i) {
        Advance(); StaticMapRanges(front); Navigation_Update(front, 420);
    }
    CHECK((Navigation_GetMap()->live_clear_mask & t14_bit) && nav_diag.state == NAV_ARRIVED);
    removed_boxes = 0;

    /* A close return just as an eastbound run starts leaves no proven
     * distance to reverse. The left side allows a stationary turn. */
    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    bool early_obstacle = false, early_turn = false, early_pause = false;
    for (unsigned i = 0; i < 50000 && Navigation_IsActive(); ++i) {
        Advance();
        Fresh(front);
        early_obstacle |= nav_diag.route_leg == 1 && nav_diag.state == NAV_MOVING &&
            fabsf(remainderf(yaw + 90.0f, 360.0f)) < 2.0f;
        if (early_obstacle && !early_turn) {
            early_turn = fabsf(remainderf(yaw + 90.0f, 360.0f)) > 10.0f;
            if (!early_turn) {
                front[1].mm = 40;
                front[2].mm = 80;
            }
        }
        Navigation_Update(front, 420);
        early_pause |= nav_diag.state == NAV_WAIT_CLEARANCE;
    }
    CHECK(early_obstacle && early_pause && early_turn);
    CHECK(nav_diag.retreats == 0 && nav_diag.state == NAV_ARRIVED);

    /* A straight passage accepts side clearances above 25 mm with a clear
     * centre beam. The 24 mm reading still stops translation. */
    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    for (unsigned i = 0; i < 50000 && Navigation_IsActive(); ++i) {
        Advance(); Fresh(front); Navigation_Update(front, 420);
        if (nav_diag.state == NAV_MOVING && nav_diag.route_leg == 1) break;
    }
    CHECK(nav_diag.state == NAV_MOVING);
    int32_t before_passage_x = nav_diag.x_mm;
    for (unsigned i = 0; i < 5; ++i) {
        Advance(); Fresh(front);
        front[0].mm = 26;
        front[2].mm = 28;
        Navigation_Update(front, 420);
        CHECK(nav_diag.state == NAV_MOVING && motion == MOTION_RUNNING);
    }
    CHECK(nav_diag.x_mm > before_passage_x);
    Advance(); Fresh(front);
    front[0].mm = 24;
    front[2].mm = 26;
    Navigation_Update(front, 420);
    CHECK(nav_diag.state == NAV_WAIT_CLEARANCE && motion == MOTION_IDLE);
    CHECK(nav_diag.near_front_mask == 1u);
    Navigation_Cancel();

    /* A blocked map leaves the field run armed and retries from the stopped
     * pose. Reopen the test aisle and confirm it resumes without a command. */
    cfg = navigation_config;
    cfg.start_x_mm = 250;
    cfg.start_y_mm = 110;
    Navigation_Init(&cfg);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    GridMap *blocked_map = (GridMap *)Navigation_GetMap();
    for (int32_t obstacle_x = 150; obstacle_x <= 500; obstacle_x += 50)
        Grid_AddHit(blocked_map, obstacle_x, 210);
    unsigned blocked_checks = 0;
    for (unsigned i = 0; i < 50000 && Navigation_IsActive(); ++i) {
        Advance(); StaticMapRanges(front); Navigation_Update(front, 420);
        if (nav_diag.state == NAV_SEARCH_BLOCKED && blocked_checks < 20) {
            CHECK(motion == MOTION_IDLE && Navigation_IsActive());
            if (++blocked_checks == 20) Grid_ClearHits(blocked_map);
        }
    }
    CHECK(blocked_checks == 20 && nav_diag.state == NAV_ARRIVED);

    /* Keep east and both perpendicular probes blocked until the vehicle
     * turns back and follows the already travelled aisle west. */
    cfg = navigation_config;
    cfg.start_x_mm = 475;
    cfg.start_y_mm = 995;
    Navigation_Init(&cfg);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    int32_t blocked_x = 0;
    bool checked_north = false, checked_south = false, headed_back = false;
    for (unsigned i = 0; i < 50000 && Navigation_IsActive(); ++i) {
        Advance(); Fresh(front);
        float heading = remainderf(yaw + 90.0f, 360.0f);
        if (!blocked_x && nav_diag.route_leg == 1 && nav_diag.state == NAV_MOVING &&
            nav_diag.x_mm >= 510 && nav_diag.x_mm < 525 && fabsf(heading) < 2.0f)
            blocked_x = nav_diag.x_mm;
        if (blocked_x && !headed_back) {
            if (fabsf(heading) < 3.0f) front[1].mm = 40;
            if (fabsf(fabsf(heading) - 90.0f) < 3.0f) {
                front[1].mm = 55;
                checked_north |= heading > 0 && nav_diag.state == NAV_READY;
                checked_south |= heading < 0 && nav_diag.state == NAV_READY;
            }
            headed_back = fabsf(heading) > 177.0f && nav_diag.state == NAV_MOVING &&
                nav_diag.x_mm < blocked_x - 20;
        }
        Navigation_Update(front, 420);
    }
    CHECK(blocked_x && checked_north && checked_south && headed_back);
    CHECK(nav_diag.state == NAV_ARRIVED);

    /* North probes follow the more open ToF side regardless of the
     * lane midpoint. A blocked east search can cross its origin. */
    for (unsigned test = 0; test < 2; ++test) {
        cfg = navigation_config;
        cfg.start_x_mm = test == 0 ? 250 : 400;
        cfg.start_y_mm = 110;
        Navigation_Init(&cfg);
        ready = true; yaw = 0; motion = MOTION_IDLE;
        CHECK(Navigation_ConfirmStart() && Navigation_Start());
        int32_t farthest_east = cfg.start_x_mm;
        bool shifted = false, found_opening = false;
        for (unsigned i = 0; i < 50000 && Navigation_IsActive(); ++i) {
            Advance();
            Fresh(front);
            if (!found_opening && fabsf(remainderf(yaw, 360.0f)) < 3.0f) {
                front[1].mm = 40;
                front[test == 0 ? 0 : 2].mm = 90;
            }
            Navigation_Update(front, 420);
            if (found_opening) continue;
            int32_t offset = nav_diag.x_mm - cfg.start_x_mm;
            CHECK(nav_diag.x_mm >= 74 && nav_diag.x_mm <= 526);
            if (!shifted && abs(offset) >= 5) {
                CHECK(test == 0 ? offset > 0 : offset < 0);
                shifted = true;
            }
            if (nav_diag.x_mm > farthest_east) farthest_east = nav_diag.x_mm;
            found_opening = test == 0 ? farthest_east >= 410 && offset <= -30 : offset <= -30;
        }
        CHECK(shifted && found_opening && nav_diag.state == NAV_ARRIVED);
    }

    /* A confirmed return in the middle of the corridor must divert entry. */
    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    for (unsigned i = 0; i < 3 && Navigation_IsActive(); ++i) {
        Advance(); Fresh(front);
        front[1].mm = 95;
        Navigation_Update(front, 420);
    }
    CHECK(nav_diag.observed_hits == 1 && Navigation_IsActive());
    bool detoured_entry = false;
    unsigned close_diagonal_turn = 0;
    for (unsigned i = 0; i < 40000 && Navigation_IsActive(); ++i) {
        Advance(); Fresh(front);
        if (nav_diag.state == NAV_TURNING && nav_diag.y_mm < 74) {
            front[0].mm = 33; /* Reproduce the close left return from the real entry turn. */
            ++close_diagonal_turn;
        }
        Navigation_Update(front, 420);
        if (nav_diag.state == NAV_ENTERING && abs(nav_diag.x_mm - navigation_config.start_x_mm) >= 20)
            detoured_entry = true;
    }
    CHECK(close_diagonal_turn > 0 && detoured_entry && nav_diag.state == NAV_ARRIVED);

    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    Advance(); Fresh(front);
    front[0].mm = 29;
    Navigation_Update(front, 420);
    CHECK(nav_diag.state == NAV_WAIT_CLEARANCE && motion == MOTION_IDLE);
    Navigation_Cancel();
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    for (unsigned i = 0; i < 3 && Navigation_IsActive(); ++i) {
        Advance(); Fresh(front);
        front[0].mm = 51;
        Navigation_Update(front, 420);
    }
    CHECK(nav_diag.state == NAV_ENTRY_BLOCKED && motion == MOTION_IDLE);
    CHECK(Navigation_GetMap()->observed_count == 1);
    CHECK(Navigation_ConfirmStart() && nav_diag.observed_hits == 0);
    CHECK(Navigation_GetMap()->observed_count == 0);
    CHECK(Grid_IsBlocked(Navigation_GetMap(), Grid_Cell(75, 150))); /* static T1 */
    CHECK(Grid_IsBlocked(Navigation_GetMap(), Grid_Cell(475, 425))); /* middle margin */
    CHECK(Navigation_Start());
    for (unsigned i = 0; i < 40000 && Navigation_IsActive(); ++i) {
        Advance(); Fresh(front); Navigation_Update(front, 420);
    }
    CHECK(nav_diag.state == NAV_ARRIVED);
    /* An obstacle unknown to the map crosses the northbound lane. */
    extra_rect = (GridRect){170, 600, 230, 660};
    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE; extra_obstacle = true;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    for (unsigned i = 0; i < 50000 && Navigation_IsActive(); ++i) {
        Advance(); StaticMapRanges(front);
        Navigation_Update(front, 420);
    }
    CHECK(nav_diag.expanded == 0);
    CHECK(nav_diag.state == NAV_ARRIVED);

    extra_rect = (GridRect){1350,750,1450,850};
    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    unsigned legs = 0;
    bool crossed = false;
    for (unsigned i = 0; i < 50000 && Navigation_IsActive(); ++i) {
        Advance(); StaticMapRanges(front); Navigation_Update(front, 420);
        legs |= 1u << nav_diag.route_leg;
        if (nav_diag.state == NAV_MOVING) {
            if (nav_diag.route_leg == 0) CHECK(nav_diag.x_mm >= 65 && nav_diag.x_mm <= 535);
            if (nav_diag.route_leg == 1) CHECK(nav_diag.y_mm >= 715 && nav_diag.y_mm <= 1135);
            if (nav_diag.route_leg == 2) CHECK(nav_diag.x_mm >= 1265 && nav_diag.x_mm <= 1735);
        }
        if (nav_diag.state == NAV_EXITING) {
            CHECK(nav_diag.x_mm > 1324 && nav_diag.x_mm < 1726);
            crossed = true;
        }
    }
    CHECK(legs == 7 && crossed && nav_diag.expanded == 0);
    CHECK(nav_diag.state == NAV_ARRIVED && nav_diag.y_mm <= -54);
    extra_obstacle = false;

    /* On the field, probe sideways and look east after each shift. J still
     * retreats and records the obstruction before finding another route. */
    for (unsigned test = 0; test < 4; ++test) {
        Navigation_Init(&navigation_config);
        ready = true; yaw = 0; motion = MOTION_IDLE;
        CHECK(Navigation_ConfirmStart());
        CHECK(test >= 2 ? Navigation_StartEmptyFloorTest() : Navigation_Start());
        bool close_started = false;
        bool reversed = false;
        bool turned_away = false;
        bool moved_to_side = false;
        bool resumed = false;
        bool probed_after_one_shift = false;
        int32_t encounter_x = 0;
        int32_t encounter_y = 0;
        int32_t required_lateral = test == 0 ? 60 : 30;
        unsigned diagonal_samples = 0;
        unsigned stale_open_samples = 0;
        unsigned retreat_yaw_samples = 0;
        for (unsigned i = 0; i < 50000 && Navigation_IsActive(); ++i) {
            Advance(); Fresh(front);
            bool stale_open = false;
            int32_t encounter_min_x = test == 0 ? 500 : test == 1 ? 230 : 875;
            bool encounter_zone = nav_diag.x_mm >= encounter_min_x &&
                nav_diag.x_mm < encounter_min_x + 25;
            if (!close_started && nav_diag.route_leg == 1 && nav_diag.state == NAV_MOVING &&
                encounter_zone &&
                fabsf(remainderf(yaw + 90.0f, 360.0f)) < 2.0f) {
                close_started = true;
                encounter_x = nav_diag.x_mm;
                encounter_y = nav_diag.y_mm;
            }
            if (close_started && test < 2 && !resumed) {
                bool east = fabsf(remainderf(yaw + 90.0f, 360.0f)) < 3.0f;
                int32_t lateral = nav_diag.y_mm - encounter_y;
                probed_after_one_shift |= east && lateral >= 30 && nav_diag.state == NAV_READY;
                if (east && lateral < required_lateral) {
                    front[1].mm = 40;
                    if (test == 0) front[2].mm = 80;
                    if (test == 1) front[0].mm = 80; /* Wider right crosses the lane edge. */
                }
                if (!resumed && nav_diag.state == NAV_MOVING && lateral >= 10 && east)
                    CHECK(lateral >= required_lateral);
                if (east && lateral >= required_lateral && nav_diag.state == NAV_MOVING)
                    resumed = true;
                if (east && lateral >= required_lateral) {
                    front[0].mm = 26;
                    front[2].mm = 28;
                    stale_open = nav_diag.state == NAV_READY && stale_open_samples < 5;
                    if (stale_open) {
                        ++stale_open_samples;
                        for (unsigned sensor = 0; sensor < 3; ++sensor) front[sensor].tick = tick - 60;
                    }
                }
            }
            if (test >= 2 && close_started && (nav_diag.retreats == 0 || nav_diag.state == NAV_RETREATING))
                front[1].mm = 40;
            if (nav_diag.retreats && nav_diag.state == NAV_PLANNING && diagonal_samples < 5) {
                front[0].mm = front[2].mm = 40;
                ++diagonal_samples;
            }
            /* The front ray still sees the obstruction when the planner
             * starts. It clears only once the vehicle turns aside. */
            if (nav_diag.retreats && nav_diag.state != NAV_RETREATING && !turned_away) {
                turned_away = fabsf(remainderf(yaw + 90.0f, 360.0f)) > 10.0f;
                if (!turned_away) front[1].mm = 45;
            }
            /* Reproduce the board's brief 5-degree excursion and verify
             * that the same error still stops a sustained deviation. */
            bool yaw_excursion = test >= 2 && nav_diag.state == NAV_RETREATING &&
                (test == 3 || retreat_yaw_samples < 4);
            float measured_yaw = yaw;
            if (yaw_excursion) { yaw = drive_goal_yaw + 5.01f; ++retreat_yaw_samples; }
            Navigation_Update(front, 420);
            if (yaw_excursion) yaw = measured_yaw;
            if (stale_open) CHECK(nav_diag.state != NAV_MOVING);
            if (nav_diag.state == NAV_RETREATING) {
                CHECK(Navigation_IsRetreating() && drive_direction == -1);
                reversed |= nav_diag.x_mm <= encounter_x - 35;
            }
            if (test == 2 && nav_diag.retreats && nav_diag.state == NAV_MOVING) {
                CHECK(turned_away);
                moved_to_side |= abs(nav_diag.y_mm - encounter_y) >= 25;
            }
        }
        CHECK(close_started);
        if (test == 3) {
            CHECK(retreat_yaw_samples >= 10 && nav_diag.state == NAV_POSE_LOST && motion == MOTION_IDLE);
            continue;
        }
        if (test == 2) {
            CHECK(retreat_yaw_samples == 4);
            CHECK(reversed && turned_away && moved_to_side && diagonal_samples == 5);
            CHECK(nav_diag.retreats == 1 && nav_diag.observed_hits > 0);
        } else {
            CHECK(probed_after_one_shift && resumed && stale_open_samples == 5 && nav_diag.retreats == 0);
        }
        CHECK(nav_diag.state == NAV_ARRIVED);
    }

    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    for (unsigned i = 0; i < 50; ++i) {
        Advance(); Fresh(front);
        front[1].mm = 40;
        Navigation_Update(front, 420);
    }
    CHECK(nav_diag.state == NAV_WAIT_CLEARANCE && nav_diag.retreats == 0);
    CHECK(nav_diag.y_mm == navigation_config.start_y_mm && motion == MOTION_IDLE);
    Navigation_Cancel();

    /* Fresh, repeatable room returns can block a narrow virtual passage
     * when added to the grid. They must not become obstacles in J. */
    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_StartEmptyFloorTest());
    for (unsigned i = 0; i < 8; ++i) {
        Advance(); Fresh(front);
        front[1].mm = 180;
        Navigation_Update(front, 420);
        CHECK(nav_diag.observed_hits == 0 && Navigation_GetMap()->observed_count == 0);
    }
    for (unsigned i = 0; i < 50000 && Navigation_IsActive(); ++i) {
        Advance(); StaticMapRanges(front); Navigation_Update(front, 420);
    }
    CHECK(nav_diag.state == NAV_ARRIVED);
    /* Legacy G stays strict. The U course and J can use fresh no-echo slots
     * without treating them as mapped hits or accepting an offline device. */
    NavigationConfig strict = navigation_config;
    strict.route_u = false;
    Navigation_Init(&strict); ready=true; yaw=0; motion=MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    Advance();Fresh(front);
    for (unsigned i=0;i<3;++i) front[i]=(NavigationRange){.mm=8191,.tick=tick,.no_echo=true};
    Navigation_Update(front,420);
    CHECK(nav_diag.state==NAV_WAIT_SENSOR && nav_diag.invalid_front_mask==7 && motion==MOTION_IDLE);
    Navigation_Cancel();
    Navigation_Init(&navigation_config);
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    Advance();
    for (unsigned i=0;i<3;++i) front[i]=(NavigationRange){.mm=8191,.tick=tick,.no_echo=true};
    Navigation_Update(front,420);
    CHECK(nav_diag.invalid_front_mask==0 && Navigation_IsActive());
    Navigation_Cancel();
    CHECK(Navigation_ConfirmStart() && Navigation_StartEmptyFloorTest());
    max_requested_speed=0;longest_step=0;
    for (unsigned n=0;n<40000 && Navigation_IsActive();++n) {
        Advance();
        for (unsigned i=0;i<3;++i) front[i]=(NavigationRange){.mm=8191,.tick=tick,.no_echo=true};
        Navigation_Update(front,950);
    }
    CHECK(nav_diag.state==NAV_ARRIVED && !strcmp(Navigation_GetLabel(),"TEST-DEN-DICH"));
    CHECK(max_requested_speed<=400 && longest_step<=100.0f);
    /* Room returns stay off the virtual course throughout the run. */
    CHECK(Navigation_ConfirmStart() && Navigation_StartEmptyFloorTest());
    for (unsigned n=0;n<40000 && Navigation_IsActive();++n) {
        Advance();Fresh(front);
        front[0].mm=192;front[1].mm=180;front[2].mm=210;
        Navigation_Update(front,420);
    }
    CHECK(nav_diag.state==NAV_ARRIVED && nav_diag.observed_hits==0);
    CHECK(Navigation_ConfirmStart() && Navigation_StartEmptyFloorTest());
    Advance();
    for (unsigned i=0;i<3;++i) front[i]=(NavigationRange){.mm=8191,.tick=tick-201,.no_echo=true};
    Navigation_Update(front,420);
    CHECK(nav_diag.state==NAV_WAIT_SENSOR && motion==MOTION_IDLE);
    Advance();
    for (unsigned i=0;i<3;++i) front[i]=(NavigationRange){.mm=8191,.tick=tick,.no_echo=true};
    front[0].no_echo=false; /* An offline/read-failed channel is never "far". */
    Navigation_Update(front,420);
    CHECK(nav_diag.state==NAV_WAIT_SENSOR && nav_diag.invalid_front_mask==1);
    Navigation_Cancel();
    CHECK(Navigation_ConfirmStart() && Navigation_StartEmptyFloorTest());

    /* Intermittent close returns must not starve the incremental planner. */
    uint32_t replans_before_noise = nav_diag.replans;
    for (unsigned pulse = 0; pulse < 8; ++pulse) {
        Advance(); Fresh(front); Navigation_Update(front, 420);
        uint32_t expanded_before_pause = nav_diag.expanded;
        CHECK(nav_diag.state == NAV_PLANNING);
        Advance(); Fresh(front); front[2].mm = 27;
        Navigation_Update(front, 420);
        CHECK(nav_diag.state == NAV_WAIT_CLEARANCE && motion == MOTION_IDLE);
        for (unsigned sample = 0; sample < 15; ++sample) {
            Advance(); Fresh(front);
            Navigation_Update(front, 420);
        }
        CHECK(nav_diag.state == NAV_PLANNING && nav_diag.expanded == expanded_before_pause);
        CHECK(nav_diag.replans == replans_before_noise);
    }

    /* A transient close return followed by fresh no-echo samples must pause
     * the car, then let it resume the interrupted entry. */
    for (unsigned i = 0; i < 1000 && motion != MOTION_RUNNING; ++i) {
        Advance(); Fresh(front);
        Navigation_Update(front, 420);
    }
    CHECK(nav_diag.state == NAV_ENTERING && motion == MOTION_RUNNING);
    Advance(); Fresh(front); front[1].mm = 130;
    Navigation_Update(front, 420);
    Advance();
    Navigation_Update(front, 420); /* Polling the same sample does not confirm it. */
    CHECK(nav_diag.observed_hits == 0 && motion == MOTION_RUNNING);
    Advance();Fresh(front);front[1].mm=40;
    Navigation_Update(front,420);
    CHECK(nav_diag.state == NAV_WAIT_CLEARANCE && motion == MOTION_IDLE);
    CHECK(!strcmp(Navigation_GetLabel(), "TEST-CHO-TOF-GAN"));
    for (unsigned i = 0; i < 14; ++i) {
        Advance(); Fresh(front);
        front[1] = (NavigationRange){.mm=8191,.tick=tick,.no_echo=true};
        Navigation_Update(front, 420);
        CHECK(nav_diag.state == NAV_WAIT_CLEARANCE && motion == MOTION_IDLE);
    }
    Advance(); Fresh(front);
    front[1] = (NavigationRange){.mm=8190,.tick=tick,.no_echo=true};
    Navigation_Update(front, 420);
    CHECK(nav_diag.state == NAV_READY && motion == MOTION_IDLE);
    CHECK(nav_diag.replans == replans_before_noise);
    for (unsigned i = 0; i < 40000 && Navigation_IsActive(); ++i) {
        Advance(); Fresh(front);
        Navigation_Update(front, 420);
    }
    CHECK(nav_diag.state == NAV_ARRIVED);

    CHECK(Navigation_ConfirmStart() && Navigation_StartEmptyFloorTest());
    for (unsigned i = 0; i <= 200; ++i) {
        Advance(); Fresh(front);
        front[1].mm = 40;
        Navigation_Update(front, 420);
        CHECK(motion == MOTION_IDLE);
    }
    CHECK(Navigation_IsActive() && nav_diag.state == NAV_WAIT_CLEARANCE);
    CHECK(nav_diag.near_front_mask == 2 && nav_diag.near_front_mm[1] == 40);
    for (unsigned i = 0; i < 15; ++i) {
        Advance(); Fresh(front);
        Navigation_Update(front, 420);
    }
    CHECK(nav_diag.state == NAV_PLANNING && motion == MOTION_IDLE);
    Navigation_Cancel();
    CHECK(!Navigation_IsActive() && motion == MOTION_IDLE);
    CHECK(Navigation_ConfirmStart() && Navigation_StartEmptyFloorTest());
    CHECK(Navigation_IsEmptyFloorTest());
    CHECK(!Navigation_Start()); /* G cannot silently remain in relaxed J mode. */
    CHECK(!Navigation_IsActive() && nav_diag.state==NAV_POSE_LOST);

    /* The gate allows the robot to continue across the start stripe. */
    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    for (unsigned i = 0; i < 1000 && nav_diag.y_mm < 120; ++i) {
        Advance(); Fresh(front);
        Navigation_Update(front, 420);
        CHECK(Navigation_IsActive());
    }
    CHECK(nav_diag.y_mm >= 120 && motion == MOTION_RUNNING);
    Navigation_Cancel();

    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    bool checked_reverse_drift = false;
    for (unsigned i = 0; i < 40000 && Navigation_IsActive(); ++i) {
        Advance(); StaticMapRanges(front);
        if (!checked_reverse_drift && nav_diag.state == NAV_MOVING &&
            nav_diag.route_leg == 0 && nav_diag.y_mm >= 145 && nav_diag.y_mm <= 160)
            front[1].mm = 24;
        if (!checked_reverse_drift && nav_diag.state == NAV_WAIT_CLEARANCE)
            front[1].mm = 24;
        if (!checked_reverse_drift && nav_diag.state == NAV_RETREATING) {
            yaw += 9.0f;
            Navigation_Update(front, 420);
            CHECK(Navigation_IsActive() && motion == MOTION_IDLE && nav_diag.state == NAV_READY);
            checked_reverse_drift = true;
        } else {
            Navigation_Update(front, 420);
        }
    }
    CHECK(checked_reverse_drift && nav_diag.state == NAV_ARRIVED);
    puts("PASS: map disabled until configured, pose confirmation, arrival, mid-step replan, stale ToF, MPU/encoder faults, stop");
    return 0;
}
