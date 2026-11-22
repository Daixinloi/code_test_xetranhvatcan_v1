#ifndef MOTION_H
#define MOTION_H

#include "mpu6050.h"

/* Counts per WHEEL revolution include gearbox and timer quadrature x4.
 * Encoder 2 measured 822 and 814 counts/rev, mean 818 (2026-11-16).
 * Distance feedback uses encoder 2 only; encoder 1 has not been calibrated. */
#define MOTION_WHEEL_DIAMETER_MM       34.0f
#define MOTION_COUNTS_PER_WHEEL_REV    818.0f
#define MOTION_STEP_MM                50.0f
#define MOTION_GYRO_SAMPLE_MS         10u
#define MOTION_STEP_MIN_PWM           360
#define MOTION_ENCODER_STALL_MS       400u
#define MOTION_HEADING_KP             14.0f
#define MOTION_HEADING_KD              1.5f
#define MOTION_HEADING_KI              3.0f
#define MOTION_HEADING_I_LIMIT        60.0f
#define MOTION_HEADING_MAX_PWM       150.0f
#define MOTION_HEADING_SLEW          900.0f
#define MOTION_STEP_ACCEL_PWM_S      700.0f
#define MOTION_STEP_START_PWM        300.0f
/* MPU mounted flat, +Z upward: left turn gives positive gyro Z.
 * Change to -1.0f if MPU is mounted upside down. */
#define MOTION_GYRO_Z_SIGN            1.0f
#define MOTION_MIN_PWM                200
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
bool Motion_StartStepHeading(int direction, float heading_deg);
bool Motion_StartDistanceHeading(int direction, float distance_mm, float heading_deg);
bool Motion_BeginStraight(void);
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
                     * 4=ready, 5=runtime read error, 6=sample gap, 7=saturated */
    int32_t gyro_z_raw;
    int32_t bias_mdps;
    int32_t variance_mdps2;
    int32_t angle_mdeg;
    uint32_t sample_gap_ms;
    int32_t accel_x, accel_y, accel_z;
} MotionDiagnostics;
extern volatile MotionDiagnostics motion_diag;

#endif
