#include "front_clearance.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); } } while (0)

static ClearanceOutput Update(FrontClearance *g, uint32_t t, ClearanceReading f[3])
{
    return FrontClearance_Update(g, t, f, 0, 0);
}

int main(void)
{
    for (unsigned sensor = 0; sensor < 3; ++sensor) {
        FrontClearance g = {0};
        ClearanceReading f[3] = {{200, true}, {200, true}, {200, true}};
        uint16_t minimum = FrontClearance_MinMm(sensor);
        uint16_t release = FrontClearance_ReleaseMm(sensor);
        f[sensor].mm = minimum;
        CHECK(!Update(&g, 0, f).override); /* strict lower bound for each position */
        f[sensor].mm = minimum - 1;
        ClearanceOutput out = Update(&g, 10, f);
        CHECK(out.override && out.state == CLEARANCE_BRAKE && !out.reverse_pwm);
        out = Update(&g, 51, f);
        CHECK(out.state == CLEARANCE_REVERSE && out.reverse_pwm > 0);
        f[sensor].mm = minimum + 10;
        CHECK(Update(&g, 80, f).reverse_pwm > 0); /* do not stop at 50 again */
        f[sensor].mm = release;
        out = Update(&g, 100, f);
        CHECK(out.state == CLEARANCE_SETTLE && !out.reverse_pwm);
        CHECK(Update(&g, 219, f).override);
        out = Update(&g, 220, f);
        CHECK(out.state == CLEARANCE_DONE && !out.override);
        CHECK(!Update(&g, 221, f).override);
    }

    FrontClearance g = {0};
    ClearanceReading f[3] = {{20, true}, {25, true}, {0, false}};
    CHECK(Update(&g, 0, f).override);
    f[0].mm = 80;
    CHECK(Update(&g, 50, f).reverse_pwm > 0); /* middle is still too close */
    f[1].mm = 80;
    CHECK(!Update(&g, 60, f).reverse_pwm); /* unrelated missing sensor is ignored */
    CHECK(Update(&g, 180, f).state == CLEARANCE_DONE);

    FrontClearance_Rearm(&g);
    f[0].mm = 19;
    Update(&g, 200, f);
    f[0].valid = false;
    CHECK(Update(&g, 250, f).state == CLEARANCE_SENSOR_FAULT);
    CHECK(!Update(&g, 260, f).reverse_pwm);

    f[0].valid = true;
    CHECK(Update(&g, 300, f).state == CLEARANCE_SENSOR_FAULT);
    CHECK(!Update(&g, 499, f).reverse_pwm);
    CHECK(Update(&g, 500, f).state == CLEARANCE_BRAKE);
    CHECK(Update(&g, 541, f).reverse_pwm > 0); /* recovered automatically */
    f[0].valid = false;
    CHECK(Update(&g, 600, f).state == CLEARANCE_SENSOR_FAULT);
    uint32_t spent = g.reverse_ms;
    f[0].valid = true;
    Update(&g, 1000, f);
    Update(&g, 1200, f);
    CHECK(Update(&g, 1241, f).reverse_pwm > 0);
    CHECK(g.reverse_ms == spent); /* retries do not reset or spend stopped time */

    FrontClearance_Stop(&g);
    f[0].valid = true;
    CHECK(!Update(&g, 300, f).override); /* stop command cannot restart itself */
    FrontClearance_Rearm(&g);
    Update(&g, 300, f);
    CHECK(Update(&g, 341, f).reverse_pwm > 0);
    CHECK(Update(&g, 2341, f).state == CLEARANCE_LIMIT_FAULT);

    FrontClearance_Rearm(&g);
    Update(&g, 0, f);
    CHECK(FrontClearance_Update(&g, 100, f, CLEARANCE_MAX_COUNTS, 0).state == CLEARANCE_LIMIT_FAULT);
    FrontClearance_Rearm(&g);
    FrontClearance_Update(&g, 0, f, 32760, 0);
    CHECK(FrontClearance_Update(&g, 100, f, -32760, 0).state == CLEARANCE_REVERSE);

    FrontClearance_Rearm(&g);
    Update(&g, 0, f);
    f[0].mm = 72;
    Update(&g, 50, f);
    f[0].mm = 55;
    CHECK(!Update(&g, 70, f).reverse_pwm); /* no chatter in 50-70 mm band */
    f[0].mm = 19;
    CHECK(Update(&g, 90, f).state == CLEARANCE_BRAKE);
    CHECK(Update(&g, 140, f).reverse_pwm > 0);

    FrontClearance_Rearm(&g);
    f[0] = (ClearanceReading){50, true};
    f[1] = (ClearanceReading){50, true};
    f[2] = (ClearanceReading){50, true};
    CHECK(!Update(&g, 0, f).override); /* diagonals use 2 cm, center uses 3 cm */
    f[1].mm = 29;
    CHECK(Update(&g, 10, f).override); /* center begins protection below 3 cm */

    FrontClearance_Rearm(&g);
    f[0]=(ClearanceReading){19,true};
    f[1]=f[2]=(ClearanceReading){200,true};
    Update(&g,0,f);Update(&g,41,f);
    f[0].valid=false;
    CHECK(Update(&g,60,f).state==CLEARANCE_SENSOR_FAULT);
    CHECK(Update(&g,1059,f).state==CLEARANCE_SENSOR_FAULT);
    ClearanceOutput timeout=Update(&g,1060,f);
    CHECK(timeout.state==CLEARANCE_SENSOR_TIMEOUT && !timeout.reverse_pwm);
    f[0].valid=true;
    CHECK(Update(&g,1100,f).state==CLEARANCE_SENSOR_TIMEOUT); /* no blind restart */
    FrontClearance_Stop(&g);
    CHECK(!Update(&g,1110,f).override); /* app STOP/V releases ownership */
    puts("PASS: front sensors, hysteresis, recovery, bounded missing-sensor wait, stop, time/travel limits");
    return 0;
}
