#include "front_clearance.h"

void FrontClearance_Stop(FrontClearance *guard)
{
    *guard = (FrontClearance){.inhibited = true};
}

void FrontClearance_Rearm(FrontClearance *guard)
{
    *guard = (FrontClearance){0};
}

static uint32_t Travel(int16_t current, int16_t origin)
{
    int32_t delta = (int16_t)((uint16_t)current - (uint16_t)origin);
    return (uint32_t)(delta < 0 ? -delta : delta);
}

ClearanceOutput FrontClearance_Update(FrontClearance *guard, uint32_t now,
    const ClearanceReading front[3], int16_t left, int16_t right)
{
    ClearanceOutput out = {.state = guard->state};
    if (guard->inhibited) return out;
    if (guard->state == CLEARANCE_REVERSE)
        guard->reverse_ms += now - guard->last_update;
    guard->last_update = now;
    if (guard->state == CLEARANCE_DONE) guard->state = CLEARANCE_IDLE;
    uint8_t near = 0;
    for (unsigned i = 0; i < 3; ++i)
        if (front[i].valid && front[i].mm < FrontClearance_MinMm(i)) near |= 1u << i;

    if (guard->state == CLEARANCE_IDLE && near) {
        guard->tracked = near;
        guard->state = CLEARANCE_BRAKE;
        guard->started = guard->phase_started = now;
        guard->start_left = left;
        guard->start_right = right;
        guard->reverse_ms = 0;
        guard->recovery_pending = false;
    }
    if (guard->state == CLEARANCE_IDLE) return (ClearanceOutput){.state = CLEARANCE_IDLE};
    out.override = true;
    if (guard->state == CLEARANCE_LIMIT_FAULT || guard->state == CLEARANCE_SENSOR_TIMEOUT)
        goto done;
    if (guard->state == CLEARANCE_SENSOR_FAULT && now - guard->sensor_fault_started >= 1000u) {
        guard->state = CLEARANCE_SENSOR_TIMEOUT;
        goto done;
    }

    guard->tracked |= near;
    bool clear = true;
    uint16_t largest_gap = 0;
    for (unsigned i = 0; i < 3; ++i) {
        if (guard->tracked & (1u << i)) {
            /* Losing the sensor that triggered retreat is not proof of clearance. */
            if (!front[i].valid) {
                if (guard->state != CLEARANCE_SENSOR_FAULT) guard->sensor_fault_started = now;
                guard->state = CLEARANCE_SENSOR_FAULT;
                guard->recovery_pending = false;
                goto done;
            }
            uint16_t release = FrontClearance_ReleaseMm(i);
            if (front[i].mm < release) {
                clear = false;
                uint16_t gap = release - front[i].mm;
                if (gap > largest_gap) largest_gap = gap;
            }
        }
    }
    if (guard->state == CLEARANCE_SENSOR_FAULT) {
        /* A missing sample must stop the motors, but must not permanently
         * lock retreat once all relevant sensors recover. No travel/time
         * budget is reset here, so repeated I2C dropouts cannot extend travel. */
        if (!guard->recovery_pending) {
            guard->recovery_pending = true;
            guard->phase_started = now;
        }
        if (now - guard->phase_started < 200) goto done;
        guard->state = CLEARANCE_BRAKE;
        guard->phase_started = now;
        guard->recovery_pending = false;
    }
    if (clear) {
        if (guard->state != CLEARANCE_SETTLE) {
            guard->state = CLEARANCE_SETTLE;
            guard->phase_started = now;
        }
        if (now - guard->phase_started >= 120) {
            guard->state = CLEARANCE_DONE;
            out.override = false;
        }
        goto done;
    }
    if (guard->reverse_ms >= CLEARANCE_TIMEOUT_MS ||
        Travel(left, guard->start_left) >= CLEARANCE_MAX_COUNTS ||
        Travel(right, guard->start_right) >= CLEARANCE_MAX_COUNTS) {
        guard->state = CLEARANCE_LIMIT_FAULT;
        goto done;
    }
    if (guard->state == CLEARANCE_SETTLE) {
        /* Stay stopped in each sensor's hysteresis band; avoid chatter. */
        if (!near) {
            if (now - guard->phase_started >= 120) {
                guard->state = CLEARANCE_DONE;
                out.override = false;
            }
            goto done;
        }
        guard->state = CLEARANCE_BRAKE;
        guard->phase_started = now;
    }
    if (guard->state == CLEARANCE_BRAKE && now - guard->phase_started >= 40) {
        guard->state = CLEARANCE_REVERSE;
        guard->reverse_started = now;
    }
    if (guard->state == CLEARANCE_REVERSE) {
        int16_t pwm = 360 + largest_gap * 2;
        out.reverse_pwm = pwm > 460 ? 460 : pwm;
        if (now - guard->reverse_started < 60) out.reverse_pwm = 480;
    }
done:
    out.state = guard->state;
    return out;
}
