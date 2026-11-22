#include "navigation.h"
#include "motion.h"
#include "motor.h"
#include "navigation_config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %d: %s (nav=%lu)\n", __LINE__, #c, (unsigned long)nav_diag.state); exit(1); } } while (0)
static uint32_t tick;
static bool ready = true;
static float yaw, target_yaw;
static int16_t encoder;
static int count_target;
static MotionStatus motion;
static bool turning;
static float drive_start_x, drive_start_y;
static float longest_step;
static unsigned distance_commands;
static int16_t max_requested_speed;
static bool extra_obstacle;
static const GridRect extra_rect = {600,825,700,925};
uint32_t HAL_GetTick(void) { return tick; }
bool Motion_IsMPUReady(void) { return ready; }
float Motion_GetAngle(void) { return yaw; }
void Motion_Cancel(void) { motion = MOTION_IDLE; }
MotionStatus Motion_GetStatus(void) { return motion; }
int16_t Encoder_GetCount2(void) { return encoder; }
bool Motion_StartTurnHeading(float h)
{
    target_yaw = yaw + remainderf(h - yaw, 360);
    turning = true; motion = MOTION_RUNNING; return ready;
}
bool Motion_StartDistanceHeading(int dir, float mm, float heading)
{
    CHECK(dir == 1 && mm >= 1 && mm <= 150);
    longest_step=fmaxf(longest_step,mm);distance_commands++;
    CHECK(fabsf(remainderf(heading - yaw, 360)) <= MOTION_ANGLE_TOLERANCE_DEG);
    encoder = 0;
    drive_start_x = nav_diag.x_mm; drive_start_y = nav_diag.y_mm;
    count_target = (int)ceilf(mm * MOTION_COUNTS_PER_WHEEL_REV /
                            (3.14159265f * MOTION_WHEEL_DIAMETER_MM));
    turning = false; motion = MOTION_RUNNING; return ready;
}
void Motion_Update(int16_t speed) { if (speed>max_requested_speed) max_requested_speed=speed; }
static void Advance(void)
{
    tick += 10;
    if (motion != MOTION_RUNNING) return;
    if (turning) {
        float error = target_yaw - yaw;
        if (fabsf(error) <= 1.5f) { yaw = target_yaw; motion = MOTION_DONE; }
        else yaw += error > 0 ? 1.5f : -1.5f;
    } else {
        encoder += 8;
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
    if (nav_diag.state == NAV_MOVING || nav_diag.state == NAV_ENTERING) {
        float mm = encoder * (3.14159265f*MOTION_WHEEL_DIAMETER_MM/MOTION_COUNTS_PER_WHEEL_REV);
        x = drive_start_x + mm*cosf(angle); y = drive_start_y + mm*sinf(angle);
    }
    for (unsigned i = 0; i < 3; ++i) {
        float sx = x + 45*cosf(angle) - navigation_config.sensor_left_mm[i]*sinf(angle);
        float sy = y + 45*sinf(angle) + navigation_config.sensor_left_mm[i]*cosf(angle);
        float ray = angle + navigation_config.sensor_angle_deg[i]*0.01745329252f;
        float range = 1500;
        for (unsigned n = 0; n < 18; ++n)
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
int main(void)
{
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
    bool aligned_entry = false;
    for (unsigned i = 0; i < 40000 && Navigation_IsActive(); ++i) {
        Advance(); StaticMapRanges(front); Navigation_Update(front, 420);
        if (nav_diag.state == NAV_MOVING && nav_diag.path_index == 0)
            aligned_entry = true; /* explicit 25 mm entry-to-cell alignment */
    }
    CHECK(aligned_entry && nav_diag.state == NAV_ARRIVED);
    CHECK(longest_step>100 && longest_step<=150);
    CHECK(distance_commands<40); /* nominal 60-cell route needs fewer restarts */
    CHECK(Grid_Cell(nav_diag.x_mm, nav_diag.y_mm) ==
          Grid_Cell(navigation_config.goal_x_mm, navigation_config.goal_y_mm));
    /* Unknown 10x10 cm obstacle placed on the nominal route: ranges see it,
     * planner initially does not. Keep the actual 25-degree rays throughout. */
    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE; extra_obstacle = true;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    for (unsigned i = 0; i < 50000 && Navigation_IsActive(); ++i) {
        Advance(); StaticMapRanges(front);
        if (front[0].mm < 50 || front[1].mm < 50 || front[2].mm < 50)
            fprintf(stderr, "unknown test near: state=%lu xy=(%ld,%ld) yaw=%.2f counts=%d front=%u,%u,%u replans=%lu hits=%lu\n",
                (unsigned long)nav_diag.state, (long)nav_diag.x_mm, (long)nav_diag.y_mm,
                yaw+90,encoder,front[0].mm,front[1].mm,front[2].mm,
                (unsigned long)nav_diag.replans,(unsigned long)nav_diag.observed_hits);
        Navigation_Update(front, 420);
    }
    CHECK(nav_diag.observed_hits > 0 && nav_diag.replans > 1);
    CHECK(nav_diag.state == NAV_ARRIVED);
    extra_obstacle = false;
    /* Fresh no-range samples are accepted ONLY by the explicit empty-floor
     * test. Normal G, missing heartbeats and I2C/offline flags still stop. */
    Navigation_Init(&navigation_config); ready=true; yaw=0; motion=MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    Advance();Fresh(front);
    for (unsigned i=0;i<3;++i) front[i]=(NavigationRange){.mm=8191,.tick=tick,.no_echo=true};
    Navigation_Update(front,420);
    CHECK(nav_diag.state==NAV_WAIT_SENSOR && nav_diag.invalid_front_mask==7 && motion==MOTION_IDLE);
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
    /* Room returns must not add obstacles to the virtual J course. */
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
    Advance();Fresh(front);front[1].mm=40;
    Navigation_Update(front,420);
    CHECK(nav_diag.state==NAV_POSE_LOST && !Navigation_IsActive());
    CHECK(Navigation_ConfirmStart() && Navigation_StartEmptyFloorTest());
    CHECK(Navigation_IsEmptyFloorTest());
    CHECK(!Navigation_Start()); /* G cannot silently remain in relaxed J mode. */
    CHECK(!Navigation_IsActive() && nav_diag.state==NAV_POSE_LOST);
    puts("PASS: map disabled until configured, pose confirmation, arrival, mid-step replan, stale ToF, MPU/encoder faults, stop");
    return 0;
}
