#define main existing_navigation_checks
#define NAV_NEW_COURSE_TEST
#include "test_navigation.c"
#undef main
#include "../Src/navigation.c"

static bool t9_removed, t3_removed, expect_continuous;
static float north_travel_factor = 1.0f;
static float start_heading_error;
static const float radians = 0.01745329252f;
static const GridRect walls[] = {
    {-10,0,0,1200}, {1800,0,1810,1200}, {0,1200,1800,1210}
};

static bool OverlapBody(float px, float py, float angle, GridRect box)
{
    float c = cosf(angle), s = sinf(angle), half = 45.0f;
    float dx = (box.x0 + box.x1)*0.5f-px, dy = (box.y0 + box.y1)*0.5f-py;
    float w = (box.x1 - box.x0)*0.5f, h = (box.y1 - box.y0)*0.5f;
    float reach = half*(fabsf(c)+fabsf(s));
    return fabsf(dx) < w+reach && fabsf(dy) < h+reach &&
        fabsf(dx*c + dy*s) < half+w*fabsf(c)+h*fabsf(s) &&
        fabsf(-dx*s + dy*c) < half+w*fabsf(s)+h*fabsf(c);
}

static char const *Contact(float px, float py, float angle)
{
    static char const *names[] = {"T1","T2","T3","T4","T5","T6","T7","T8","T9",
        "T10","T11","T12","T13","T14","T15","T16","T17","middle"};
    for (unsigned i = 0; i < 18; ++i)
        if ((!t9_removed || i != GRID_T9_INDEX) && (!t3_removed || i != GRID_T3_INDEX) && OverlapBody(px, py, angle, grid_static_obstacles[i])) return names[i];
    for (unsigned i = 0; i < 3; ++i)
        if (OverlapBody(px, py, angle, walls[i])) return "wall";
    float reach = 45*(fabsf(cosf(angle))+fabsf(sinf(angle)));
    if (py - reach < -0.01f && px - reach < 1200 && px + reach > 600) return "bottom";
    if (extra_obstacle && course_route[0][1] != 730 &&
        OverlapBody(px,py,angle,extra_rect)) return "extra";
    return NULL;
}

static bool reflect_walls = true;
static uint16_t Measure(unsigned i, float px, float py, float angle)
{
    float c = cosf(angle), s = sinf(angle);
    float sx = px + navigation_config.sensor_forward_mm[i]*c - navigation_config.sensor_left_mm[i]*s;
    float sy = py + navigation_config.sensor_forward_mm[i]*s + navigation_config.sensor_left_mm[i]*c;
    float ray = angle + navigation_config.sensor_angle_deg[i]*radians;
    float dx = cosf(ray), dy = sinf(ray), range = 1500;
    for (unsigned n = 0; n < 18; ++n)
        if ((!t9_removed || n != GRID_T9_INDEX) && (!t3_removed || n != GRID_T3_INDEX)) range = fminf(range, HitDistance(sx,sy,dx,dy,grid_static_obstacles[n]));
    if (reflect_walls) for (unsigned n = 0; n < 3; ++n)
        range = fminf(range, HitDistance(sx,sy,dx,dy,walls[n]));
    if (extra_obstacle) range = fminf(range, HitDistance(sx,sy,dx,dy,extra_rect));
    return (uint16_t)range;
}

static unsigned rng;
static float Uniform(void)
{
    rng = rng * 1664525u + 1013904223u;
    return ((rng >> 8) / 16777216.0f) * 2.0f - 1.0f;
}

static void Run(const char *name, float scale, int slip_leg, float skid, float random_scale,
    unsigned seed, unsigned poll, FILE *trace)
{
    rng = seed;
    tick = 0; yaw = 0; motion = MOTION_IDLE; encoder = 0; distance_commands = 0; ready = true;
    Navigation_Init(&navigation_config);
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    float px = navigation_config.start_x_mm, py = navigation_config.start_y_mm;
    float factor = scale, skid_left = skid, max_error = 0;
    float first_bend_y = INFINITY;
    float min_middle_gap = INFINITY;
    unsigned last_command = 0, last_move = 0;
    bool applied = false;
    bool rolling_case = !strncmp(name, "arc_before", 10);
    unsigned arc_samples = 0;
    char const *outcome = "timeout";
    NavigationRange front[3];
    for (unsigned sensor = 0; sensor < 3; ++sensor)
        front[sensor] = (NavigationRange){.valid=true,
            .mm=Measure(sensor,px,py,(90.0f + start_heading_error)*radians)};
    for (unsigned i = 0; i < 18000 && Navigation_IsActive(); ++i) {
        int before = encoder;
        bool driving = motion == MOTION_RUNNING && !turning;
        float old_yaw = yaw;
        if (distance_commands != last_command) {
            factor = scale + random_scale*Uniform();
            last_command = distance_commands;
        }
        Advance();
        float physical_heading = yaw + navigation_config.start_heading_deg + start_heading_error;
        float angle = physical_heading*radians;
        if (driving) {
            CHECK(drive_direction > 0);
            float odom = (encoder - before) * EncoderMmPerCount();
            float actual = odom*factor;
            unsigned leg = nav_diag.route_leg;
            if (leg == 0) actual *= north_travel_factor;
            bool at_slip = leg == (unsigned)slip_leg &&
                ((leg == 0 && py > 185 && py < 430) || (leg == 1 && px > 550 && px < 950) ||
                 (leg == 2 && py < 580 && py > 230)) &&
                fabsf(remainderf(yaw + 90 - (leg == 0 ? 90 : leg == 1 ? 0 : -90),360)) < 3;
            if (at_slip && skid_left > 0 && drive_direction > 0) {
                float lost = fminf(actual,skid_left);
                actual -= lost;
                skid_left -= lost;
                applied |= lost > 0;
            }
            px += drive_direction*actual*cosf(angle);
            py += drive_direction*actual*sinf(angle);
        }
        if (driving || old_yaw != yaw) last_move = tick;
        if (!isfinite(first_bend_y) && yaw < -4.0f) first_bend_y = py;
        if (px >= 990.0f) {
            float reach = 45.0f * (fabsf(cosf(angle)) + fabsf(sinf(angle)));
            float across = fmaxf(0.0f, px - reach - 1200.0f);
            float above = fmaxf(0.0f, py - reach - 600.0f);
            min_middle_gap = fminf(min_middle_gap, hypotf(across, above));
        }
        if (!poll || i % poll == 0) {
            for (unsigned sensor = 0; sensor < 3; ++sensor)
                front[sensor] = (NavigationRange){.valid=true,.mm=Measure(sensor,px,py,angle),.tick=tick};
        }
        bool was_rolling = local_step == LOCAL_ARC;
        unsigned commands_before = distance_commands;
        Navigation_Update(front,420);
        if (rolling_case && (was_rolling || local_step == LOCAL_ARC)) {
            CHECK(driving && motion == MOTION_RUNNING && state == NAV_MOVING);
            CHECK(distance_commands == commands_before);
            if (!was_rolling) CHECK(front[1].mm >= 120);
            ++arc_samples;
        }
        if (expect_continuous) CHECK(state != NAV_TURNING && distance_commands <= 1);
        max_error = fmaxf(max_error,hypotf(px-nav_diag.x_mm,py-nav_diag.y_mm));
        if (trace && i % 5 == 0)
            fprintf(trace,"%.2f,%.2f,%.2f,%lu,%lu,%ld,%ld,%u,%u,%u,%u\n",
                px,py,physical_heading,
                (unsigned long)nav_diag.route_leg,(unsigned long)nav_diag.state,
                (long)nav_diag.x_mm,(long)nav_diag.y_mm,front[0].mm,front[1].mm,front[2].mm,
                (unsigned)(local_step == LOCAL_ARC ? 3 :
                    local_step == LOCAL_REFERENCE ? 2 : local_step == LOCAL_JOIN));
        const char *contact = Contact(px,py,angle);
        if (!strcmp(name, "t9_short") || !strcmp(name, "systematic_4pct") ||
            !strcmp(name, "systematic_5pct")) CHECK(!contact);
        if (contact) { if (getenv("SLIP_DIAG")) fprintf(stderr,"CONTACT %s %s st=%u heading=%.1f tof=%u/%u/%u local=%u\n",name,contact,state,angle/radians,front[0].mm,front[1].mm,front[2].mm,local_step); outcome = contact; break; }
        if (nav_diag.state == NAV_ARRIVED) {
            float reach = 45*(fabsf(cosf(angle))+fabsf(sinf(angle)));
            outcome = py + reach <= 0 && px - reach >= 1200 && px + reach <= 1800 ? "finish" : "short_finish";
            break;
        }
        if (!Navigation_IsActive()) { outcome = state == NAV_BORDER ? "border_stop" : "firmware_fault"; break; }
        if (tick-last_move > 12000) { outcome = "stuck"; if (getenv("SLIP_DIAG")) fprintf(stderr, "%s st=%u local=%u next=%.8f center=%u left=%u right=%u goal=%.5f,%.5f x=%.5f,%.5f moving=%u sides=%d,%d blocked=%d retreat=%u yaw=%.3f tail=%.2f/%.2f backtracked=%d\n",name,state,local_step,next_distance,front[1].mm,front[0].mm,front[2].mm,local_x,local_y,x,y,motion,side_blocked,tried_other_side,prefer_lateral,nav_diag.retreats,yaw+90,tail_mm,tail_heading,backtracked); break; }
    }
    if (!strcmp(name, "first_bend"))
        CHECK(first_bend_y >= 630.0f && first_bend_y <= 690.0f && !strcmp(outcome, "finish"));
    if (!strcmp(name, "fast_first_curve"))
        CHECK(fast_first_curve && !strcmp(outcome, "finish"));
    if (!strcmp(name, "north_short")) CHECK(!strcmp(outcome, "finish"));
    CHECK(nav_diag.retreats == 0);
    if (rolling_case) CHECK(arc_samples >= 30 && !strcmp(outcome, "finish"));
    if (!strcmp(name, "edge") || !strcmp(name, "scale_positive_5pct"))
        CHECK(t11_aligned && !strcmp(outcome, "finish"));
    if (!strcmp(name, "t11")) CHECK(!t11_aligned);
    printf("%s,%s,%lu,%.1f,%.1f,%ld,%ld,%.1f,%.2f,%d,%.1f,%.1f\n",name,outcome,
        (unsigned long)nav_diag.state,px,py,(long)nav_diag.x_mm,(long)nav_diag.y_mm,max_error,tick/1000.0f,
        applied,skid-skid_left,isfinite(min_middle_gap) ? min_middle_gap : -1.0f);
}

static void CheckCenterTof(void)
{
    Navigation_Init(&navigation_config);
    yaw = 0.0f;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    entry_pending = false;
    x = 420.0f;
    y = 220.0f;
    local_step = LOCAL_REFERENCE;
    StartOdometry(1);
    encoder = 0;
    motion = MOTION_RUNNING;
    State(NAV_MOVING);
    ReferenceCruise(0, 400.0f, 90.0f, 300);
    CHECK(simulated_cruise == U_REFERENCE_CRUISE_MM_S);
    ReferenceCruise(0, 400.0f, 90.0f, 80);
    CHECK(simulated_cruise < U_REFERENCE_CRUISE_MM_S && simulated_cruise > 0);

    NavigationRange front[3];
    Fresh(front);
    front[0].mm = 240;
    front[2].mm = 155;
    CHECK(PreferRight(front));
    front[2].mm = 90;
    CHECK(!PreferRight(front));
    front[0].mm = 70;
    CHECK(PreferRight(front));
    for (unsigned sample = 0; sample < 2; ++sample) {
        tick += 40;
        Fresh(front);
        front[0].mm = front[2].mm = 60;
        CHECK(!Observe(front));
    }
    CHECK(nav_diag.observed_hits == 0);
    front[0].mm = 24;
    Navigation_Update(front, 420);
    CHECK(state == NAV_WAIT_CLEARANCE && nav_diag.near_front_mask == 1u && reference_active);
    unsigned trace = (nav_trace_count - 1) % NAV_TRACE_CAPACITY;
    CHECK(nav_trace[trace].state == NAV_WAIT_CLEARANCE && nav_trace[trace].front_mm[0] == 24);
    tick += 40;
    Fresh(front);
    front[1].mm = 40;
    Navigation_Update(front, 420);
    CHECK(nav_diag.near_front_mask == 2u && !reference_active);
    ready = false;
    ++tick;
    Navigation_Update(front, 420);
    trace = (nav_trace_count - 1) % NAV_TRACE_CAPACITY;
    CHECK(nav_trace[trace].state == NAV_MPU_FAULT && nav_trace[trace].front_mm[1] == 40);
    ready = true;
}

static void CheckMapBoundaries(void)
{
    const int16_t positions[][2] = {{40, 850}, {700, 610}};
    for (unsigned i = 0; i < sizeof positions / sizeof positions[0]; ++i) {
        Navigation_Init(&navigation_config);
        yaw = 0.0f;
        CHECK(Navigation_ConfirmStart() && Navigation_Start());
        entry_pending = false;
        x = positions[i][0];
        y = positions[i][1];
        State(NAV_MOVING);
        NavigationRange front[3];
        Fresh(front);
        Navigation_Update(front, 420);
        CHECK(!Navigation_IsActive() && nav_diag.state == NAV_BORDER);
        CHECK(nav_trace_count == 1 && nav_trace[0].state == NAV_BORDER);
        CHECK(nav_trace[0].front_mm[1] == front[1].mm);
    }
}

static void CheckRollingBend(void)
{
    Navigation_Init(&navigation_config);
    yaw = 0.0f;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    entry_pending = false;
    reference_active = false;
    x = 260;
    y = 600;
    local_step = LOCAL_FORWARD;
    State(NAV_READY);
    NavigationRange front[3];
    Fresh(front);
    front[0].mm = front[1].mm = front[2].mm = 500;
    next_heading = -90.0f;
    CHECK(!StartRollingBend(front, 0, tick));
    next_heading = 55.0f;
    CHECK(StartRollingBend(front, 0, tick));
    unsigned commands = distance_commands;
    for (unsigned i = 0; i < 150 && local_step == LOCAL_BEND; ++i) {
        Advance();
        for (unsigned s = 0; s < 3; ++s) front[s].tick = tick;
        Navigation_Update(front, 420);
        CHECK(state == NAV_MOVING && motion == MOTION_RUNNING && !turning);
        CHECK(drive_direction == 1 && distance_commands == commands);
    }
    CHECK(local_step == LOCAL_FORWARD && fabsf(MapYaw() - 55.0f) <= 2.0f);
    Navigation_Cancel();
}

static void CheckLateralProbe(void)
{
    Navigation_Init(&navigation_config);
    yaw = 0.0f;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    entry_pending = false;
    x = 300.0f;
    y = 115.0f;
    NavigationRange front[3];
    Fresh(front);
    front[0].mm = 900;
    front[1].mm = 45;
    front[2].mm = 200;
    CHECK(TryLocalStep(front, 2u, false) && detour_step == 1 && local_x > x);
    local_step = LOCAL_IDLE;
    front[0].mm = 200;
    front[2].mm = 900;
    CHECK(TryLocalStep(front, 2u, false) && detour_step == 1 && local_x > x);
    local_step = LOCAL_IDLE;
    x = 372.0f;
    side_blocked = true;
    CHECK(TryLocalStep(front, 2u, false) && returning_to_sweep_start && local_x == 312.0f);
    x = 312.0f;
    local_step = LOCAL_IDLE;
    CHECK(TryLocalStep(front, 2u, false) && returning_to_sweep_start && local_x == 300.0f);
    x = 300.0f;
    local_step = LOCAL_IDLE;
    CHECK(TryLocalStep(front, 2u, false) && !returning_to_sweep_start && local_x < x);
    local_step = LOCAL_IDLE;
    x = 110.0f;
    y = 330.0f;
    detour_step = -1;
    tried_other_side = false;
    CHECK(TryLocalStep(front, 2u, false) && detour_step == 1 && local_x > x);

    x = 300.0f;
    y = 115.0f;
    local_x = 370.0f;
    local_y = y;
    local_step = LOCAL_SHIFT;
    yaw = -90.0f;
    Fresh(front);
    front[1].mm = 70;
    Navigation_Update(front, 420);
    CHECK(state == NAV_MOVING && fabsf(local_x - 320.0f) < 0.01f && !side_blocked);
}

static void CheckDetourHeadingRecovery(void)
{
    Navigation_Init(&navigation_config);
    ready = true; yaw = 0; motion = MOTION_IDLE;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    yaw_offset = 0.0f;
    route_leg = U_SOUTH;
    x = 1394.0f; y = 567.0f;
    reference_active = false;
    local_step = LOCAL_SHIFT;
    local_x = 1335.0f; local_y = 577.0f;
    next_heading = 180.0f;
    yaw = 180.0f;
    CHECK(Motion_StartDistanceHeading(1, 60.0f, next_heading));
    odometry_direction = 0;
    State(NAV_MOVING);

    NavigationRange front[3];
    yaw = 164.6f;
    tick += 10;
    Fresh(front);
    Navigation_Update(front, 420);
    CHECK(state == NAV_MOVING && Navigation_IsActive() &&
        motion == MOTION_RUNNING && simulated_cruise == 85u);

    yaw = 180.0f;
    tick += 10;
    Fresh(front);
    Navigation_Update(front, 420);
    CHECK(state == NAV_MOVING && Navigation_IsActive());

    yaw = 155.0f;
    tick += 10;
    Fresh(front);
    Navigation_Update(front, 420);
    CHECK(state == NAV_POSE_LOST && !Navigation_IsActive());
}

static void CheckReferenceObstacleDetour(void)
{
    tick = 0;
    yaw = 0;
    motion = MOTION_IDLE;
    encoder = 0;
    ready = true;
    Navigation_Init(&navigation_config);
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    yaw = -90.0f;
    yaw_offset = 90.0f;
    x = 600.0f;
    y = 974.0f;
    entry_pending = false;
    route_leg = U_EAST;
    reference_point = 19;
    reference_active = true;
    local_step = LOCAL_IDLE;
    State(NAV_READY);

    NavigationRange front[3];
    StaticMapRanges(front);
    CHECK(TryReference(front, 0));
    CHECK(state == NAV_PLANNING && reference_detour_active);

    for (unsigned n = 0; n < 30000 && reference_detour_active; ++n) {
        Advance();
        StaticMapRanges(front);
        Navigation_Update(front, 420);
        CHECK(!Contact(nav_diag.x_mm, nav_diag.y_mm, MapYaw() * radians));
    }
    CHECK(!reference_detour_active && reference_active);
}

static void CheckNorthCornerRejoin(void)
{
    yaw = 0.0f;
    Navigation_Init(&navigation_config);
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    entry_pending = false;
    route_leg = U_NORTH;
    x = 462.0f;
    y = 804.0f;
    reference_point = 3;
    State(NAV_READY);
    CHECK(PassedReferencePoint(433.0f - x, 692.0f - y, 115.0f, 1));
    CHECK(BeginReferenceDetour(reference_point));
    CHECK(course_route[reference_detour_point][1] >= y);
    CHECK(map.min_row == Grid_Cell(x, y) / GRID_WIDTH);
    for (unsigned i = 0; i < GRID_NODES && map.result == GRID_SEARCHING; ++i)
        Grid_Advance(&map, 1);
    CHECK(map.result == GRID_FOUND);
    for (unsigned i = 0; i < map.path_length; ++i)
        CHECK(map.path[i] / GRID_WIDTH >= map.min_row);
    Navigation_Cancel();
}

static void Batch(void)
{
    CheckCenterTof();
    CheckRollingBend();
    CheckNorthCornerRejoin();
    CheckMapBoundaries();
    CheckLateralProbe();
    /* A brief 20-degree reference lag can recover; a sustained one stops. */
    Navigation_Init(&navigation_config);
    yaw = 0.0f;
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    entry_pending = false;
    x = local_x = 420.0f;
    y = 220.0f;
    local_y = 320.0f;
    local_step = LOCAL_REFERENCE;
    next_heading = 70.0f;
    StartOdometry(1);
    encoder = 0;
    motion = MOTION_RUNNING;
    State(NAV_MOVING);
    NavigationRange front[3];
    tick += 40;
    Fresh(front);
    Navigation_Update(front, 420);
    CHECK(state == NAV_MOVING && motion == MOTION_RUNNING && simulated_cruise == 85u);
    yaw = -2.0f;
    tick += 80;
    Fresh(front);
    Navigation_Update(front, 420);
    CHECK(state == NAV_MOVING && !moving_heading_lag_at);
    yaw = 0.0f;
    tick += 40;
    Fresh(front);
    Navigation_Update(front, 420);
    tick += U_MOVING_HEADING_RECOVER_MS;
    Fresh(front);
    Navigation_Update(front, 420);
    CHECK(state == NAV_POSE_LOST && motion == MOTION_IDLE);
    Navigation_Init(&navigation_config);
    CHECK(Navigation_ConfirmStart() && Navigation_Start());
    yaw = 0.0f;
    encoder = 0;
    StartOdometry(1);
    float start_y = y;
    encoder = -100;
    State(NAV_MOVING);
    Odometry();
    State(NAV_READY);
    encoder -= 40;
    Odometry();
    CHECK(fabsf(y - start_y - 140.0f * MOTION_FIELD_MM_PER_COUNT) < 0.001f);
    CHECK(StartHeadingTurn());
    encoder -= 100;
    Odometry();
    CHECK(fabsf(y - start_y - 140.0f * MOTION_FIELD_MM_PER_COUNT) < 0.001f);

    puts("case,outcome,state,x,y,est_x,est_y,max_error,seconds,slip_applied,slip_mm,min_middle_gap_mm");
    FILE *p = fopen("build/slip_nominal.csv","w");
    if (p) fputs("tick,x,y,est_x,est_y,leg\n",p);
    Run("nominal",1,-1,0,0,0,0,p);
    if (p) fclose(p);
    Run("nominal_tof40ms",1,-1,0,0,0,4,NULL);
    for (int leg=0;leg<3;++leg) {
        for (unsigned slip=5;slip<=20;slip+=5) {
            char name[40];
            snprintf(name,sizeof(name),"leg%d_single_%umm",leg,slip);
            Run(name,1,leg,(float)slip,0,0,0,NULL);
        }
    }
    for (unsigned n=1;n<=5;++n) {
        char name[40];
        snprintf(name,sizeof(name),"systematic_%upct",n);
        Run(name,1.0f-n/100.0f,-1,0,0,0,0,NULL);
    }
    for (unsigned seed=1;seed<=30;++seed) {
        char name[40];
        snprintf(name,sizeof(name),"random_3pct_%u",seed);
        Run(name,1,-1,0,0.03f,seed,0,NULL);
    }
    extra_obstacle = true;
    GridRect cases[] = {{130,410,230,510}, {160,600,260,700}, {930,750,1030,850},
        {990,800,1090,900}, {1310,500,1410,600}, {1540,280,1640,380}, {1100,820,1200,920}, {1110,740,1210,840}, {930,1020,1030,1120}, {1060,800,1160,900}};
    for (unsigned box=0;box<sizeof(cases)/sizeof(cases[0]);++box) {
        char name[40];
        extra_rect = cases[box];
        snprintf(name,sizeof(name),"extra%u",box);
        Run(name,1,-1,0,0,0,4,NULL);
    }
    extra_obstacle = false;
}

int main(int argc, char **argv)
{
    reflect_walls = argc <= 1 || strcmp(argv[1], "lines");
    t9_removed = argc > 2 && !strcmp(argv[2], "removed");
    t3_removed = argc > 2 && !strcmp(argv[2], "removed3");
    if (argc == 3 && !strcmp(argv[1], "trace")) {
        reflect_walls = false;
        expect_continuous = course_route[0][1] != 730 &&
            (!strcmp(argv[2], "nominal") || !strcmp(argv[2], "slip") ||
             !strcmp(argv[2], "curve_lag") || !strcmp(argv[2], "t11") ||
             !strcmp(argv[2], "t14") || !strcmp(argv[2], "t14_lag"));
        if (!strcmp(argv[2], "nominal")) Run("nominal",1,-1,0,0,0,0,stdout);
        else if (!strcmp(argv[2], "fast_first_curve")) {
            simulated_fast_first_curve = true;
            expect_continuous = course_route[0][1] != 730;
            Run("fast_first_curve",1,-1,0,0,0,6,stdout);
        }
        else if (!strcmp(argv[2], "slip")) Run("slip",1,2,5,0,0,0,stdout);
        else if (!strcmp(argv[2], "first_bend")) {
            expect_continuous = true;
            /* Independent trial value inferred from the reported 65/90 cm bend. */
            Run("first_bend",0.130f / MOTION_FIELD_MM_PER_COUNT,-1,0,0,0,4,stdout);
        } else if (!strcmp(argv[2], "first_bend_old")) {
            /* Replay the old scale error with the same assumed floor distance. */
            Run("first_bend_old",0.130f / (100.0f / 1036.0f),-1,0,0,0,4,stdout);
        }
        else if (!strcmp(argv[2], "north_short")) {
            north_travel_factor = 0.97f;
            Run("north_short",1.016f,-1,0,0,0,4,stdout);
        } else if (!strcmp(argv[2], "t9_short")) {
            north_travel_factor = 0.96f;
            Run("t9_short",1.0f,-1,0,0,0,4,stdout);
        } else if (!strcmp(argv[2], "start_yaw_error")) {
            start_heading_error = -3.0f;
            Run("start_yaw_error",1.0f,-1,0,0,0,4,stdout);
        } else if (!strncmp(argv[2], "arc_before", 10)) {
            extra_obstacle = true;
            extra_rect = (GridRect){960,860,995,890};
            simulated_steering_lag = !strcmp(argv[2], "arc_before_lag");
            Run(argv[2],1.0f,-1,0,0,0,4,stdout);
        } else if (!strcmp(argv[2], "curve_lag")) {
            simulated_steering_lag = true;
            Run("curve_lag",1,-1,0,0,0,4,stdout);
        }
        else if (!strcmp(argv[2], "edge")) Run("edge",1.05f,-1,0,0,0,4,stdout);
        else if (!strcmp(argv[2], "extra")) {
            extra_obstacle = true;
            extra_rect = (GridRect){990,800,1090,900};
            Run("extra",1,-1,0,0,0,4,stdout);
        } else if (!strcmp(argv[2], "south")) {
            extra_obstacle = true;
            extra_rect = (GridRect){1540,280,1640,380};
            Run("south",1,-1,0,0,0,4,stdout);
        } else if (!strcmp(argv[2], "join")) {
            extra_obstacle = true;
            extra_rect = (GridRect){1320,525,1360,565};
            Run("join",1,-1,0,0,0,4,stdout);
        } else if (!strcmp(argv[2], "t9")) {
            extra_obstacle = true;
            extra_rect = (GridRect){530,815,570,855};
            Run("t9",1,-1,0,0,0,4,stdout);
        } else if (!strcmp(argv[2], "t11")) {
            extra_obstacle = true;
            extra_rect = (GridRect){800,925,900,970};
            Run("t11",1,-1,0,0,0,4,stdout);
        } else if (!strcmp(argv[2], "t14") || !strcmp(argv[2], "t14_lag")) {
            simulated_steering_lag = !strcmp(argv[2], "t14_lag");
            extra_obstacle = true;
            extra_rect = (GridRect){1200,750,1300,815};
            Run(argv[2],1,-1,0,0,0,4,stdout);
        } else if (!strcmp(argv[2], "heading_recovery")) {
            CheckDetourHeadingRecovery();
            puts("PASS: lateral detour recovers brief heading lag and rejects severe drift");
        } else if (!strcmp(argv[2], "reference_detour")) {
            CheckReferenceObstacleDetour();
            puts("PASS: blocked reference route replans around static obstacles without contact");
        } else return 2;
        return 0;
    }
    Batch();
    Run("scale_770_over_818",770.0f/818.0f,-1,0,0,0,0,NULL);
    Run("scale_positive_2pct",1.02f,-1,0,0,0,0,NULL);
    Run("scale_positive_5pct",1.05f,-1,0,0,0,0,NULL);
    Run("scale_positive_10pct",1.10f,-1,0,0,0,0,NULL);
    return 0;
}

