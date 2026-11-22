#ifndef TOF_OFFSET_H
#define TOF_OFFSET_H
#include <stdint.h>

/* Existing v2 offsets plus the remaining displayed floor reported by user.
 * Left: 25+30; center: 0+30; right: 25+60. Units: millimeters. */
#define TOF_OFFSET_LEFT_MM    55
#define TOF_OFFSET_CENTER_MM  30
#define TOF_OFFSET_RIGHT_MM   85
#define TOF_INVALID_MM        8000u
#define TOF_OFFSET_MAX_MM     2000

static inline int16_t ToF_ClampOffset(int32_t offset)
{
    return offset < 0 ? 0 : offset > TOF_OFFSET_MAX_MM ? TOF_OFFSET_MAX_MM : (int16_t)offset;
}

static inline uint16_t ToF_CorrectDistance(uint16_t raw, int16_t offset)
{
    if (raw >= TOF_INVALID_MM) return raw;
    int32_t corrected = (int32_t)raw - offset;
    if (corrected < 0) return 0;
    if (corrected >= (int32_t)TOF_INVALID_MM) return TOF_INVALID_MM - 1;
    return (uint16_t)corrected;
}
#endif
