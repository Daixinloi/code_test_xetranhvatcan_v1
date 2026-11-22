#ifndef BOARD_DRIVE_H
#define BOARD_DRIVE_H
#include <stdint.h>
#include <stdbool.h>
/* Application output adapter. Keep motor.c/h byte-identical to working ZIP.
 * PB8/PB9 levels present in the old main must accompany EVERY motor path. */
void BoardDrive_Set(int16_t a, int16_t b);
void BoardDrive_Stop(void);
bool BoardDrive_IsStopped(void);
typedef struct {
    int32_t requested_a, requested_b;
    uint32_t aux_levels, updates;
} BoardDriveDiagnostics;
extern volatile BoardDriveDiagnostics board_drive_diag;
#endif
