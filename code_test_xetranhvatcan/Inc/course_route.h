#ifndef COURSE_ROUTE_H
#define COURSE_ROUTE_H
#include <stdint.h>

/* Route points are stored in centimetres in flash and expanded to mm in RAM.
 * This trades 432 bytes of flash data for 432 bytes of available SRAM. */
#ifdef NAV_TEST_START_X_MM
#define COURSE_ROUTE_START_X_MM NAV_TEST_START_X_MM
#else
#define COURSE_ROUTE_START_X_MM 300
#endif
#define COURSE_FIXED_LINE_TEST 1
#define COURSE_ROUTE_COUNT 73u
static int16_t course_route[COURSE_ROUTE_COUNT][3];
/* x is unsigned: the course reaches 1320 mm, which does not fit in int8_t.
 * Storing x in a signed byte silently wrapped the last six points to about
 * -1240 mm, so the car could never reach the goal. */
static const struct { uint8_t x_cm; int8_t y_cm; }
    course_route_cm[COURSE_ROUTE_COUNT] = {
    /* Immediately angle right from the new start x=300 mm and join the
     * original x=420 mm lane before reaching the T3 obstacle. */
    {42, 20},
    {42, 58},
    {42, 60},
    {42, 61},
    {42, 63},
    {43, 64},
    {43, 65},
    {43, 67},
    {44, 68},
    {44, 69},
    {47, 76},
    {48, 76},
    {48, 77},
    {48, 78},
    {49, 79},
    {49, 80},
    {49, 81},
    {49, 82},
    {50, 83},
    {50, 86},
    {50, 87},
    {50, 88},
    {51, 89},
    {51, 89},
    {52, 90},
    {53, 90},
    {53, 90},
    {54, 89},
    {68, 88},
    {69, 88},
    {69, 88},
    {70, 87},
    {71, 87},
    {72, 87},
    {73, 87},
    {73, 87},
    {74, 87},
    {77, 87},
    {78, 87},
    {78, 87},
    {79, 87},
    {80, 86},
    {81, 86},
    {82, 86},
    {83, 86},
    {83, 86},
    {90, 86},
    {91, 88},
    {93, 87},
    {94, 87},
    {95, 87},
    {97, 86},
    {98, 86},
    {99, 85},
    {100, 84},
    {108, 79},
    {109, 78},
    {110, 77},
    {112, 76},
    {113, 75},
    {115, 75},
    {116, 74},
    {118, 74},
    {120, 73},
    {123, 73},
    {126, 72},
    {128, 71},
    {130, 70},
    {131, 68},
    {132, 66},
    {132, 63},
    {132, 62},
    {132, -10},
};
static inline void CourseRoute_Init(void)
{
    for (unsigned i = 0; i < COURSE_ROUTE_COUNT; ++i) {
        course_route[i][0] = (int16_t)course_route_cm[i].x_cm * 10;
        course_route[i][1] = (int16_t)course_route_cm[i].y_cm * 10;
        course_route[i][2] = i < 15u ? 1 : i < 70u ? 2 : 3;
    }
}
#endif

