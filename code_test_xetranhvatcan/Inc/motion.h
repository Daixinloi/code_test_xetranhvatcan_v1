#ifndef MOTION_H
#define MOTION_H

#include "mpu6050.h"

/* Counts per WHEEL revolution include gearbox and timer quadrature x4.
 * Encoder 2 measured 807 counts for one hand-turned wheel revolution.
 * Use the selected floor calibration of 770 counts/rev.
 * Distance feedback uses encoder 2 only; encoder 1 has not been calibrated. */
#define MOTION_WHEEL_DIAMETER_MM       34.0f
#define MOTION_COUNTS_PER_WHEEL_REV    770.0f
#define MOTION_WHEEL_MM_PER_COUNT      (3.14159265f * MOTION_WHEEL_DIAMETER_MM / MOTION_COUNTS_PER_WHEEL_REV)
/* Continuous field travel uses encoder 2 and the calibrated wheel geometry. */
#define MOTION_FIELD_MM_PER_COUNT      MOTION_WHEEL_MM_PER_COUNT
#define MOTION_STEP_MM                50.0f
/* Manual forward cutoff before the chassis coasts to its 50 mm target. */
#define MOTION_MANUAL_FORWARD_ENCODER_MM  25.0f
#define MOTION_GYRO_SAMPLE_MS         10u
#define MOTION_CALIBRATION_SAMPLES   200u
#define MOTION_STEP_MIN_PWM           330
#define MOTION_ENCODER_STALL_MS       400u
#define MOTION_HEADING_KP             14.0f
#define MOTION_HEADING_KD              1.5f
#define MOTION_HEADING_KI              3.0f
#define MOTION_HEADING_I_LIMIT        60.0f
#define MOTION_HEADING_MAX_PWM       150.0f
#define MOTION_HEADING_SLEW          900.0f
#define MOTION_STEP_ACCEL_PWM_S      650.0f
#define MOTION_STEP_START_PWM        300.0f
/* MPU mounted flat, +Z upward: left turn gives positive gyro Z.
 * Change to -1.0f if MPU is mounted upside down. */
#define MOTION_GYRO_Z_SIGN            1.0f
#define MOTION_MIN_PWM                200
#define MOTION_COURSE_MIN_PWM         300
#define MOTION_FORWARD_ROLL_MAX_DEG   18.0f
#define MOTION_TURN_START_PWM         220
#define MOTION_TURN_MAX_DPS           220.0f
#define MOTION_TURN_ACCEL_DPS2        800.0f
#define MOTION_TURN_DECEL_DPS2        1000.0f
#define MOTION_TURN_PWM_SLEW_PER_SEC  2400.0f
#define MOTION_TURN_PWM_LIMIT         650.0f
#define MOTION_TURN_BRAKE_MS          100
#define MOTION_ANGLE_TOLERANCE_DEG     2.0f

typedef enum { MOTION_IDLE, MOTION_RUNNING, MOTION_DONE,
               MOTION_SENSOR_ERROR, MOTION_TIMEOUT, MOTION_ANGLE_ERROR,
               MOTION_ENCODER_ERROR } MotionStatus;

bool Motion_Init(I2C_HandleTypeDef *bus);
bool Motion_StartStep(int direction);
bool Motion_StartManualStep(int direction, int16_t speed);
void Motion_ResetManualHeading(void);
int16_t Motion_GetReportedEncoder2(void);
bool Motion_StartStepHeading(int direction, float heading_deg);
bool Motion_StartDistanceHeading(int direction, float distance_mm, float heading_deg);
/* Extend a running forward move from the current position, retaining PID and PWM. */
bool Motion_ExtendForward(float distance_mm, float heading_deg);
bool Motion_SteerForward(float heading_deg);
/* Replace the forward lookahead while keeping the current wheel control. */
bool Motion_RetargetForward(float distance_mm, float heading_deg);
void Motion_SetCruise(uint16_t mm_per_second);
float Motion_GetMeasuredForwardSpeed(void);
void Motion_SetCourseSteering(bool enabled);
bool Motion_BeginStraight(void);
bool Motion_BeginStraightHeading(float heading_deg);
bool Motion_DriveStraight(int direction, int16_t speed);
bool Motion_StartTurn(float angle_deg, bool curve);
bool Motion_StartTurnHeading(float heading_deg);
void Motion_Update(int16_t speed);
void Motion_Cancel(void);
MotionStatus Motion_GetStatus(void);
float Motion_GetAngle(void);
void Motion_PollIdle(void);
const char *Motion_GetMPUState(void);
bool Motion_IsMPUReady(void);
typedef struct {
    uint32_t state; /* 1=init error, 2=calibration read error, 3=not still,
                     * 4=ready, 5=runtime read error, 6=sample gap, 7=saturated,
                     * 8=waiting for stationary recalibration */
    int32_t gyro_z_raw;
    int32_t bias_mdps;
    int32_t variance_mdps2;
    int32_t angle_mdeg;
    uint32_t sample_gap_ms;
    int32_t accel_x, accel_y, accel_z;
    uint32_t calibration_samples, calibration_restarts;
    uint32_t last_fault, last_fault_tick, max_sample_gap_ms;
    int32_t turn_error_mdeg; /* Target minus yaw after the last turn settle. */
    int32_t manual_forward_stop_counts; /* E2 at PWM cutoff, before any coast. */
} MotionDiagnostics;
extern volatile MotionDiagnostics motion_diag;

#endif
