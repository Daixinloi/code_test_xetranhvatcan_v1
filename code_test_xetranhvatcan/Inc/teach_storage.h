#ifndef TEACH_STORAGE_H
#define TEACH_STORAGE_H

#include <stdbool.h>
#include <stdint.h>

#define TEACH_MAX_SEGMENTS 200u

typedef struct {
    uint16_t target; /* E2 counts for F/B, tenths of a degree for L/R. */
    uint8_t direction;
    uint8_t reserved;
} TeachSegment;

typedef struct {
    uint16_t count;
    TeachSegment segments[TEACH_MAX_SEGMENTS];
} TeachRoute;

uint32_t TeachStorage_Load(TeachRoute *route);
bool TeachStorage_Save(const TeachRoute *route, uint32_t *run_id);

#endif
