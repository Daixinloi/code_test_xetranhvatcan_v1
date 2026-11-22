#ifndef FRONT_CLEARANCE_H
#define FRONT_CLEARANCE_H
#include <stdbool.h>
#include <stdint.h>

#define CLEARANCE_CENTER_MIN_MM        50
#define CLEARANCE_CENTER_RELEASE_MM    70
#define CLEARANCE_DIAGONAL_MIN_MM      50
#define CLEARANCE_DIAGONAL_RELEASE_MM  70
#define CLEARANCE_TIMEOUT_MS      2000u
#define CLEARANCE_MAX_COUNTS      1149u /* ~150 mm with 34 mm wheel, 818 counts/rev */

typedef struct { uint16_t mm; bool valid; } ClearanceReading;
/* Input order is diagonal-left, center, diagonal-right. */
static inline uint16_t FrontClearance_MinMm(unsigned index)
{
    return index == 1 ? CLEARANCE_CENTER_MIN_MM : CLEARANCE_DIAGONAL_MIN_MM;
}
static inline uint16_t FrontClearance_ReleaseMm(unsigned index)
{
    return index == 1 ? CLEARANCE_CENTER_RELEASE_MM : CLEARANCE_DIAGONAL_RELEASE_MM;
}
typedef enum {
    CLEARANCE_IDLE, CLEARANCE_BRAKE, CLEARANCE_REVERSE, CLEARANCE_SETTLE,
    CLEARANCE_DONE, CLEARANCE_SENSOR_FAULT, CLEARANCE_LIMIT_FAULT,
    CLEARANCE_SENSOR_TIMEOUT
} ClearanceState;

typedef struct {
    ClearanceState state;
    uint32_t started, phase_started;
    uint32_t last_update, reverse_ms, reverse_started;
    uint32_t sensor_fault_started;
    bool recovery_pending;
    int16_t start_left, start_right;
    uint8_t tracked;
    bool inhibited;
} FrontClearance;

typedef struct { ClearanceState state; bool override; int16_t reverse_pwm; } ClearanceOutput;

void FrontClearance_Stop(FrontClearance *guard);
void FrontClearance_Rearm(FrontClearance *guard);
ClearanceOutput FrontClearance_Update(FrontClearance *guard, uint32_t now,
    const ClearanceReading front[3], int16_t left, int16_t right);
#endif
