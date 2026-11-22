#include "tof_offset.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #c); exit(1); } } while (0)

int main(void)
{
    CHECK(ToF_CorrectDistance(55, 25) == 30);
    CHECK(ToF_CorrectDistance(30, 0) == 30);
    CHECK(ToF_CorrectDistance(85, 25) == 60);
    CHECK(ToF_CorrectDistance(55, TOF_OFFSET_LEFT_MM) == 0);
    CHECK(ToF_CorrectDistance(30, TOF_OFFSET_CENTER_MM) == 0);
    CHECK(ToF_CorrectDistance(85, TOF_OFFSET_RIGHT_MM) == 0);
    CHECK(ToF_CorrectDistance(54, TOF_OFFSET_LEFT_MM) == 0);
    CHECK(ToF_CorrectDistance(0, TOF_OFFSET_RIGHT_MM) == 0);
    CHECK(ToF_CorrectDistance(155, TOF_OFFSET_LEFT_MM) == 100);
    CHECK(ToF_CorrectDistance(130, TOF_OFFSET_CENTER_MM) == 100);
    CHECK(ToF_CorrectDistance(185, TOF_OFFSET_RIGHT_MM) == 100);
    const uint16_t invalid[] = {8000, 8190, 8191, 65535};
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
        CHECK(ToF_CorrectDistance(invalid[i], 2000) == invalid[i]);
    CHECK(ToF_ClampOffset(-5) == 0);
    CHECK(ToF_ClampOffset(2005) == 2000);
    CHECK(ToF_CorrectDistance(45, ToF_ClampOffset(45)) == 0);
    puts("PASS: measured floors -> zero, no underflow, linear offset, invalid ranges preserved, bounds");
    return 0;
}
