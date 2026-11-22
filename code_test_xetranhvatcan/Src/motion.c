#include "motion.h"
#include "motor.h"
#include "board_drive.h"
#include <math.h>
#include <stdlib.h>

#define COURSE_OUTER_PWM_MAX 480
#define RIGHT_PWM_TRIM_PERCENT 2
#define COURSE_PWM_BEFORE_TRIM (COURSE_OUTER_PWM_MAX * 100 / (100 + RIGHT_PWM_TRIM_PERCENT))
#define MAP_TURN_MAX_DPS 110.0f
#define MAP_TURN_PWM_LIMIT 350.0f
#define TURN_TRIM_MAX_ERROR_DEG 25.0f
#define MAP_TURN_RETRY_MAX_ERROR_DEG 45.0f
#define TURN_TRIM_MAX_PULSES 20u
#define TURN_TRIM_PWM_INITIAL 210
#define TURN_TRIM_PWM_MIN 170
#define TURN_TRIM_PWM_MAX 240

static I2C_HandleTypeDef *mpu_bus;
static bool mpu_ready;
static float bias_z, angle, target, previous_rate;
static uint32_t started, sampled, settled;
static int direction;
static bool turning, curve_turn;
static int turn_sign;
typedef enum { TURN_APPROACH, TURN_BRAKE, TURN_TRIM_PULSE } TurnPhase;
static TurnPhase turn_phase;
static bool map_turn, approach_retried;
static uint8_t trim_pulses, trim_pulse_ms;
static int16_t trim_pwm;
static float trim_previous_error;
static uint32_t phase_started;
static float turn_rate_ref, turn_integral, turn_pwm;
static MotionStatus status;
static uint32_t last_encoder_progress;
static int32_t previous_distance_count;
volatile MotionDiagnostics motion_diag;
static bool mpu_connected;
static float heading_target, heading_pwm;
static uint32_t heading_tick;
static bool heading_active;
static float heading_integral, heading_rate, step_pwm;
static uint32_t step_pwm_tick;
static float step_distance_mm;
static uint16_t cruise_mm_s;
static bool course_steering;
static uint32_t velocity_tick;
static int32_t velocity_count;
static float velocity_ref, velocity_measured, velocity_i, velocity_pwm;
static uint32_t calibration_sampled, calibration_count;
static float calibration_mean, calibration_m2;
static bool manual_heading_valid;
static float manual_heading;
static bool manual_forward;
static bool manual_step, manual_trim;
static int16_t manual_end_encoder;

static void Finish(MotionStatus result);

void Motion_ResetManualHeading(void) { manual_heading_valid = manual_trim = false; }

int16_t Motion_GetReportedEncoder2(void)
{
    return manual_trim ? manual_end_encoder : Encoder_GetCount2();
}

static float HeadingError(float requested)
{
    return remainderf(requested - angle, 360.0f);
}

static float EncoderDistance(int32_t count)
{
    return (count < 0 ? -count : count) *
        (course_steering ? MOTION_FIELD_MM_PER_COUNT : MOTION_WHEEL_MM_PER_COUNT);
}

void Motion_SetCruise(uint16_t mm_per_second)
{
    cruise_mm_s = mm_per_second;
}

float Motion_GetMeasuredForwardSpeed(void) { return velocity_measured; }

void Motion_SetCourseSteering(bool enabled) { course_steering = enabled; }

bool Motion_BeginStraight(void)
{
    manual_trim = false;
    heading_active = mpu_ready && motion_diag.state == 4;
    heading_target = angle;
    heading_pwm = 0;
    heading_integral = 0;
    heading_rate = previous_rate;
    heading_tick = sampled;
    return heading_active;
}

bool Motion_BeginStraightHeading(float heading_deg)
{
    if (!Motion_BeginStraight())
        return false;
    heading_target = heading_deg;
    return true;
}

bool Motion_DriveStraight(int travel_direction, int16_t speed)
{
    if ((travel_direction != 1 && travel_direction != -1) || speed <= 0 ||
        !heading_active || !mpu_ready || motion_diag.state != 4 ||
        HAL_GetTick() - sampled > 100) {
        BoardDrive_Stop();
        return false;
    }
    bool course_forward = travel_direction > 0 && (cruise_mm_s || course_steering);
    if (course_steering && course_forward && speed < MOTION_COURSE_MIN_PWM)
        speed = MOTION_COURSE_MIN_PWM;
    float limit = fminf(MOTION_HEADING_MAX_PWM, speed * 0.35f);
    int16_t course_max = course_steering ? COURSE_PWM_BEFORE_TRIM : COURSE_OUTER_PWM_MAX;
    int16_t course_min = speed < MOTION_COURSE_MIN_PWM ? speed : MOTION_COURSE_MIN_PWM;
    if (course_forward)
        limit = fminf(limit, (course_max - course_min) * 0.5f);
    if (sampled != heading_tick) {
        float dt = (sampled - heading_tick) / 1000.0f;
        /* Filter vibration, retain gyro damping, and learn persistent wheel
         * imbalance without a large integral kick after wheel slip. */
        heading_rate += dt / (0.04f + dt) * (previous_rate - heading_rate);
        float error = HeadingError(heading_target);
        float raw = MOTION_HEADING_KP * error - MOTION_HEADING_KD * heading_rate + heading_integral;
        if ((raw < limit || error < 0) && (raw > -limit || error > 0))
            heading_integral = fmaxf(-MOTION_HEADING_I_LIMIT, fminf(MOTION_HEADING_I_LIMIT,
                heading_integral + MOTION_HEADING_KI * error * dt));
        float correction = MOTION_HEADING_KP * error -
                           MOTION_HEADING_KD * heading_rate + heading_integral;
        correction = fmaxf(-limit, fminf(limit, correction));
        float delta = MOTION_HEADING_SLEW * dt;
        heading_pwm += fmaxf(-delta, fminf(delta, correction - heading_pwm));
        heading_tick = sampled;
    }
    int16_t correction = (int16_t)fmaxf(-limit, fminf(limit, heading_pwm));
    /* Positive yaw torque is B > A for BOTH travel directions. Reduce the
     * common PWM at saturation so the heading difference is preserved. */
    int16_t base = (int16_t)fminf(speed, 1000 - abs(correction));
    int16_t left = travel_direction * base - correction;
    int16_t right = travel_direction * base + correction;
    if (course_forward) {
        /* Allocate the common PWM within the travel available after steering.
         * Near either limit, preserve the difference and move both outputs
         * together. The inner wheel keeps its breakaway torque. */
        int16_t differential = abs(correction);
        int16_t inner = (int16_t)fmaxf(course_min,
            fminf(base - differential, course_max - 2 * differential));
        int16_t outer = inner + 2 * differential;
        left = correction >= 0 ? inner : outer;
        right = correction >= 0 ? outer : inner;
    }
    if (travel_direction > 0 && (manual_forward || course_steering)) {
        right += (course_forward ? (left + right) / 2 : base) * RIGHT_PWM_TRIM_PERCENT / 100;
        if (right > MOTOR_PWM_MAX) right = MOTOR_PWM_MAX;
    }
    BoardDrive_Set(left, right);
    return true;
}

static void DriveTurn(int16_t pwm)
{
    /* Direction stays fixed until the gyro has settled. */
    if (curve_turn)
        BoardDrive_Set(turn_sign > 0 ? pwm * 35 / 100 : pwm,
                       turn_sign > 0 ? pwm : pwm * 35 / 100);
    else
        BoardDrive_Set(-turn_sign * pwm, turn_sign * pwm);
}

static void BrakeTurn(uint32_t now)
{
    BoardDrive_Stop();
    turn_phase = TURN_BRAKE;
    phase_started = now;
    settled = 0;
}

static void StartTurnApproach(void)
{
    turn_phase = TURN_APPROACH;
    turn_rate_ref = 0.0f;
    turn_integral = 0.0f;
    turn_pwm = MOTION_TURN_START_PWM;
    DriveTurn(MOTION_TURN_START_PWM);
}

static void StartTurnTrim(uint32_t now, float error)
{
    if (trim_pulses) {
        if (trim_previous_error * error < 0.0f && trim_pwm > TURN_TRIM_PWM_MIN)
            trim_pwm -= 5;
        else if (fabsf(trim_previous_error - error) < 0.75f && trim_pwm < TURN_TRIM_PWM_MAX)
            trim_pwm += 5;
    }
    trim_previous_error = error;
    ++trim_pulses;
    trim_pulse_ms = (uint8_t)fminf(50.0f, fmaxf(10.0f, fabsf(error) * 6.0f));
    turn_sign = error > 0.0f ? 1 : -1;
    turn_phase = TURN_TRIM_PULSE;
    phase_started = now;
    DriveTurn(trim_pwm);
}

static void UpdateTurn(uint32_t now, bool fresh_sample, float dt, int16_t speed)
{
    float rate = previous_rate;
    float error = target - angle;
    float remaining = error * turn_sign;
    if (turn_phase == TURN_TRIM_PULSE) {
        if (now - phase_started >= trim_pulse_ms ||
            (fresh_sample && remaining <= fmaxf(0.6f, rate * turn_sign * 0.030f)))
            BrakeTurn(now);
        return;
    }
    if (!fresh_sample) return;
    if (turn_phase == TURN_BRAKE) {
        BoardDrive_Stop();
        if (fabsf(rate) >= 5.0f) settled = 0;
        else if (!settled) settled = now;
        if (now - phase_started < MOTION_TURN_BRAKE_MS || !settled || now - settled < 60)
            return;
        motion_diag.turn_error_mdeg = (int32_t)(error * 1000.0f);
        if (fabsf(error) <= MOTION_ANGLE_TOLERANCE_DEG) {
            Finish(MOTION_DONE);
        } else if (map_turn && trim_pulses < TURN_TRIM_MAX_PULSES &&
                   fabsf(error) <= TURN_TRIM_MAX_ERROR_DEG) {
            StartTurnTrim(now, error);
        } else if (map_turn && !approach_retried && fabsf(error) <= MAP_TURN_RETRY_MAX_ERROR_DEG) {
            approach_retried = true;
            turn_sign = error > 0.0f ? 1 : -1;
            StartTurnApproach();
        } else {
            Finish(MOTION_ANGLE_ERROR);
        }
        return;
    }

    float toward_rate = fmaxf(0.0f, rate * turn_sign);
    float lead = map_turn ? toward_rate * 0.035f + toward_rate * toward_rate / 3000.0f :
        toward_rate * 0.015f + toward_rate * toward_rate / 4000.0f;
    if (remaining <= fmaxf(0.7f, lead)) {
        BrakeTurn(now);
        return;
    }

    /* Ramp the gyro rate smoothly, then taper as the target approaches. */
    float max_rate = map_turn ? MAP_TURN_MAX_DPS : MOTION_TURN_MAX_DPS;
    float desired_rate = fminf(max_rate,
        fminf(6.0f * remaining, sqrtf(2.0f * MOTION_TURN_DECEL_DPS2 * remaining)));
    float slew_rate = desired_rate > turn_rate_ref ? MOTION_TURN_ACCEL_DPS2 : MOTION_TURN_DECEL_DPS2;
    float rate_delta = slew_rate * dt;
    turn_rate_ref += fmaxf(-rate_delta, fminf(rate_delta, desired_rate - turn_rate_ref));
    float rate_error = turn_rate_ref - rate * turn_sign;
    float max_pwm = map_turn ? MAP_TURN_PWM_LIMIT : MOTION_TURN_PWM_LIMIT;
    float limit = fminf(max_pwm, fmaxf(260.0f, speed * 1.25f));
    float friction = map_turn ? 190.0f : 180.0f;
    float feedforward = friction + 1.5f * turn_rate_ref;
    float unsaturated = feedforward + 2.0f * rate_error + turn_integral;
    if ((unsaturated < limit || rate_error < 0.0f) &&
        (unsaturated > 0.0f || rate_error > 0.0f))
        turn_integral = fmaxf(-120.0f, fminf(120.0f,
            turn_integral + 4.0f * rate_error * dt));
    float request = fmaxf(0.0f, fminf(limit, feedforward + 2.0f * rate_error + turn_integral));
    float delta = MOTION_TURN_PWM_SLEW_PER_SEC * dt;
    turn_pwm += fmaxf(-delta, fminf(delta, request - turn_pwm));
    DriveTurn((int16_t)turn_pwm);
}

static void RecordRaw(const MPU6050_Data_t *data)
{
    motion_diag.gyro_z_raw = data->Gyro_Z;
    motion_diag.accel_x = data->Accel_X;
    motion_diag.accel_y = data->Accel_Y;
    motion_diag.accel_z = data->Accel_Z;
}

static void Finish(MotionStatus result)
{
    BoardDrive_Stop();
    status = result;
}

static void RestartCalibration(uint32_t now)
{
    if (calibration_count || motion_diag.state != 8) motion_diag.calibration_restarts++;
    calibration_count = 0;
    calibration_mean = calibration_m2 = 0;
    calibration_sampled = now;
    motion_diag.calibration_samples = 0;
    motion_diag.state = 8;
}

static void RetryStationaryCalibration(uint32_t now)
{
    if (!mpu_connected || mpu_ready || status == MOTION_RUNNING) return;
    if (motion_diag.state != 3 && motion_diag.state != 6 &&
        motion_diag.state != 7 && motion_diag.state != 8) return;
    if (motion_diag.state != 8) RestartCalibration(now);
    /* A bench/legacy-auto command can drive without a running Motion step.
     * Never learn gyro zero while any application motor output is active. */
    if (!BoardDrive_IsStopped()) { RestartCalibration(now); return; }
    if (now - calibration_sampled < MOTION_GYRO_SAMPLE_MS) return;
    MPU6050_Data_t data;
    if (!MPU6050_Read_All(mpu_bus, &data)) { motion_diag.state = 5; return; }
    now = HAL_GetTick();
    calibration_sampled = now;
    RecordRaw(&data);
    float rate = data.Gyro_Z / MPU6050_GYRO_LSB_PER_DPS;
    if (fabsf(rate) > 10.0f) { RestartCalibration(now); return; }
    ++calibration_count;
    float delta = rate - calibration_mean;
    calibration_mean += delta / calibration_count;
    calibration_m2 += delta * (rate - calibration_mean);
    float variance = fmaxf(0, calibration_m2 / calibration_count);
    motion_diag.calibration_samples = calibration_count;
    motion_diag.variance_mdps2 = (int32_t)(variance * 1000.0f);
    if (calibration_count >= 8 && variance > 1.0f) {
        RestartCalibration(now); return;
    }
    if (calibration_count < MOTION_CALIBRATION_SAMPLES) return;
    bias_z = calibration_mean;
    Motion_ResetManualHeading();
    motion_diag.bias_mdps = (int32_t)(bias_z * 1000.0f);
    angle = 0;
    motion_diag.angle_mdeg = 0;
    sampled = now;
    previous_rate = (rate - bias_z) * MOTION_GYRO_Z_SIGN;
    mpu_ready = true;
    motion_diag.state = 4;
    /* Do not restart or complete any old movement here. The application
     * cancels it and requires a fresh command after yaw has been reset. */
}

bool Motion_Init(I2C_HandleTypeDef *bus)
{
    MPU6050_Data_t data;
    Motion_Cancel();
    Motion_ResetManualHeading();
    mpu_bus = bus;
    mpu_ready = false;
    mpu_connected = false;
    calibration_count = 0;
    motion_diag.calibration_samples = motion_diag.calibration_restarts = 0;
    motion_diag.last_fault = motion_diag.last_fault_tick = motion_diag.max_sample_gap_ms = 0;
    motion_diag.state = 1;
    if (!MPU6050_Init(bus)) return false;
    mpu_connected = true;
    motion_diag.state = 2;
    HAL_Delay(200);
    /* Keep robot stationary during these 200 samples (~2 seconds). */
    float sum = 0.0f, sum_sq = 0.0f;
    for (unsigned i = 0; i < 200; ++i) {
        if (!MPU6050_Read_All(bus, &data)) return false;
        RecordRaw(&data);
        float rate = data.Gyro_Z / MPU6050_GYRO_LSB_PER_DPS;
        sum += rate;
        sum_sq += rate * rate;
        HAL_Delay(10);
    }
    bias_z = sum / 200.0f;
    motion_diag.bias_mdps = (int32_t)(bias_z * 1000.0f);
    float variance = fmaxf(0.0f, sum_sq / 200.0f - bias_z * bias_z);
    motion_diag.variance_mdps2 = (int32_t)(variance * 1000.0f);
    motion_diag.state = 3;
    if (variance > 1.0f || fabsf(bias_z) > 10.0f) {
        RestartCalibration(HAL_GetTick());
        return false;
    }
    mpu_ready = true;
    motion_diag.state = 4;
    angle = 0.0f;
    motion_diag.angle_mdeg = 0;
    sampled = HAL_GetTick();
    previous_rate = (data.Gyro_Z / MPU6050_GYRO_LSB_PER_DPS - bias_z) * MOTION_GYRO_Z_SIGN;
    return true;
}

void Motion_Cancel(void)
{
    manual_step = manual_forward = false;
    heading_active = false;
    Finish(MOTION_IDLE);
}

bool Motion_StartStep(int travel_direction)
{
    return Motion_StartManualStep(travel_direction, 420);
}

bool Motion_StartManualStep(int travel_direction, int16_t speed)
{
    if (!manual_heading_valid) { manual_heading = angle; manual_heading_valid = true; }
    if (fabsf(HeadingError(manual_heading)) > 15.0f) {
        Motion_Cancel(); Finish(MOTION_ANGLE_ERROR); return false;
    }
    float distance = travel_direction > 0 ? MOTION_MANUAL_FORWARD_ENCODER_MM : MOTION_STEP_MM;
    if (!Motion_StartDistanceHeading(travel_direction, distance, manual_heading)) return false;
    if (speed <= 0) { Finish(MOTION_SENSOR_ERROR); return false; }
    manual_step = true;
    manual_forward = travel_direction > 0;
    /* Restore the proven manual starting torque. Short app button presses
     * must not spend their whole duration below loaded motor breakaway PWM.
     * Autonomous distance moves retain their gradual acceleration profile. */
    step_pwm = fminf(speed, MOTION_STEP_MIN_PWM + MOTION_STEP_MM * 3.0f);
    Motion_Update(speed);
    return status == MOTION_RUNNING;
}

bool Motion_StartStepHeading(int travel_direction, float heading_deg)
{
    return Motion_StartDistanceHeading(travel_direction, MOTION_STEP_MM, heading_deg);
}

bool Motion_StartDistanceHeading(int travel_direction, float distance_mm, float heading_deg)
{
    Motion_Cancel();
    if ((travel_direction != 1 && travel_direction != -1) || !isfinite(distance_mm) ||
        distance_mm < 1.0f || distance_mm > 150.0f || !isfinite(heading_deg) ||
        !Motion_BeginStraight()) {
        Finish(MOTION_SENSOR_ERROR);
        return false;
    }
    heading_target = heading_deg;
    step_distance_mm = distance_mm;
    Encoder_Reset();
    direction = travel_direction;
    turning = false;
    started = HAL_GetTick();
    step_pwm_tick = started;
    velocity_tick = started;
    velocity_count = 0;
    velocity_ref = velocity_measured = velocity_i = 0.0f;
    velocity_pwm = MOTION_STEP_START_PWM;
    step_pwm = cruise_mm_s && travel_direction > 0 ?
        (course_steering ? MOTION_COURSE_MIN_PWM : MOTION_MIN_PWM) : MOTION_STEP_START_PWM;
    last_encoder_progress = started;
    previous_distance_count = 0;
    status = MOTION_RUNNING;
    return true;
}

bool Motion_ExtendForward(float distance_mm, float heading_deg)
{
    if (status != MOTION_RUNNING || turning || direction != 1 || !heading_active || !mpu_ready ||
        !isfinite(distance_mm) || distance_mm < 1.0f || distance_mm > 150.0f ||
        !isfinite(heading_deg) || fabsf(HeadingError(heading_deg)) >
            (course_steering ? MOTION_FORWARD_ROLL_MAX_DEG : 17.0f))
        return false;
    float distance = EncoderDistance(Encoder_GetCount2());
    float remaining = step_distance_mm - distance;
    if (remaining > 35.0f || distance_mm < remaining + 6.0f) return false;
    step_distance_mm = distance + distance_mm;
    heading_target = heading_deg;
    started = HAL_GetTick();
    return true;
}

bool Motion_SteerForward(float heading_deg)
{
    if (status != MOTION_RUNNING || turning || direction != 1 || !heading_active || !mpu_ready ||
        !isfinite(heading_deg) || fabsf(HeadingError(heading_deg)) > 14.0f)
        return false;
    heading_target = heading_deg;
    return true;
}

bool Motion_RetargetForward(float distance_mm, float heading_deg)
{
    if (!isfinite(distance_mm) || distance_mm < 1.0f || distance_mm > 150.0f ||
        !Motion_SteerForward(heading_deg)) return false;
    step_distance_mm = EncoderDistance(Encoder_GetCount2()) + distance_mm;
    started = HAL_GetTick();
    return true;
}

static bool StartTurn(float angle_deg, bool curve, bool heading_turn)
{
    Motion_Cancel();
    Motion_ResetManualHeading();
    MPU6050_Data_t data;
    if (!mpu_ready || motion_diag.state != 4 || !isfinite(angle_deg) || fabsf(angle_deg) < 0.1f) {
        Finish(MOTION_SENSOR_ERROR);
        return false;
    }
    if (!MPU6050_Read_All(mpu_bus, &data)) {
        motion_diag.state = 5;
        mpu_ready = false;
        Finish(MOTION_SENSOR_ERROR);
        return false;
    }
    RecordRaw(&data);
    /* Absolute yaw remains visible while stopped; each turn is relative. */
    target = angle + angle_deg;
    motion_diag.turn_error_mdeg = 0;
    curve_turn = curve;
    turn_sign = angle_deg > 0.0f ? 1 : -1;
    map_turn = heading_turn;
    trim_pulses = 0;
    approach_retried = false;
    trim_pwm = TURN_TRIM_PWM_INITIAL;
    turning = true;
    started = sampled = HAL_GetTick();
    settled = 0;
    previous_rate = (data.Gyro_Z / MPU6050_GYRO_LSB_PER_DPS - bias_z) * MOTION_GYRO_Z_SIGN;
    status = MOTION_RUNNING;
    if (map_turn && fabsf(angle_deg) <= TURN_TRIM_MAX_ERROR_DEG)
        BrakeTurn(started);
    else
        StartTurnApproach();
    return true;
}

bool Motion_StartTurn(float angle_deg, bool curve)
{
    return StartTurn(angle_deg, curve, false);
}

bool Motion_StartTurnHeading(float heading_deg)
{
    if (!mpu_ready || motion_diag.state != 4 || !isfinite(heading_deg)) {
        Motion_Cancel();
        Finish(MOTION_SENSOR_ERROR);
        return false;
    }
    float error = HeadingError(heading_deg);
    if (fabsf(error) <= 0.1f) {
        Motion_Cancel();
        Finish(MOTION_DONE);
        return true;
    }
    return StartTurn(error, false, true);
}

void Motion_Update(int16_t speed)
{
    uint32_t now = HAL_GetTick();
    RetryStationaryCalibration(now);
    now = HAL_GetTick();
    bool fresh_sample = false;
    float sample_seconds = 0.0f;
    if (mpu_ready && now - sampled >= MOTION_GYRO_SAMPLE_MS) {
        MPU6050_Data_t data;
        if (!MPU6050_Read_All(mpu_bus, &data)) {
            motion_diag.state = 5;
            motion_diag.last_fault = 5;
            motion_diag.last_fault_tick = HAL_GetTick();
            mpu_ready = false;
            if (status == MOTION_RUNNING) Finish(MOTION_SENSOR_ERROR);
        } else {
            now = HAL_GetTick();
            uint32_t elapsed = now - sampled;
            motion_diag.sample_gap_ms = elapsed;
            if (elapsed > motion_diag.max_sample_gap_ms) motion_diag.max_sample_gap_ms = elapsed;
            RecordRaw(&data);
            float rate = (data.Gyro_Z / MPU6050_GYRO_LSB_PER_DPS - bias_z) * MOTION_GYRO_Z_SIGN;
            if (elapsed > 100 || data.Gyro_Z > 32000 || data.Gyro_Z < -32000) {
                motion_diag.state = elapsed > 100 ? 6 : 7;
                motion_diag.last_fault = motion_diag.state;
                motion_diag.last_fault_tick = now;
                /* Lost rotation cannot be reconstructed from later samples.
                 * Latch invalid yaw until stationary recalibration. */
                mpu_ready = false;
                if (status == MOTION_RUNNING) Finish(MOTION_SENSOR_ERROR);
            } else {
                angle += (previous_rate + rate) * 0.5f * elapsed / 1000.0f;
                motion_diag.angle_mdeg = (int32_t)(angle * 1000.0f);
                fresh_sample = true;
                motion_diag.state = 4;
                sample_seconds = elapsed / 1000.0f;
            }
            sampled = now;
            previous_rate = rate;
        }
    }
    if (status != MOTION_RUNNING) return;
    if (now - started > (turning ? 8000u : 5000u)) {
        Finish(MOTION_TIMEOUT);
        return;
    }
    if (!turning) {
        int16_t encoder = Encoder_GetCount2();
        int32_t count = encoder;
        if (count < 0) count = -count;
        float distance = EncoderDistance(count);
        if (distance >= step_distance_mm) {
            if (manual_forward) motion_diag.manual_forward_stop_counts = count;
            if (manual_step) {
                float error = fabsf(HeadingError(manual_heading));
                if (error > MOTION_ANGLE_TOLERANCE_DEG) {
                    if (error > TURN_TRIM_MAX_ERROR_DEG) {
                        Finish(MOTION_ANGLE_ERROR);
                    } else if (Motion_StartTurnHeading(manual_heading)) {
                        manual_heading_valid = manual_trim = true;
                        manual_end_encoder = encoder;
                    }
                    return;
                }
            }
            Finish(MOTION_DONE);
            return;
        }
        if (count - previous_distance_count >= 2) {
            previous_distance_count = count;
            last_encoder_progress = now;
        }
        if (now - last_encoder_progress >= MOTION_ENCODER_STALL_MS) {
            Finish(MOTION_ENCODER_ERROR);
            return;
        }
        /* Both motors retain the working v2 PWM convention. Do not stop a
         * wheel using the uncalibrated encoder 1, which can leave only one
         * wheel running. Slow modestly near the end, then stop both together. */
        float remaining = step_distance_mm - distance;
        float requested = fminf(speed, MOTION_STEP_MIN_PWM + remaining * 3.0f);
        if (cruise_mm_s && direction > 0) {
            uint32_t elapsed = now - velocity_tick;
            if (elapsed >= 25u) {
                float dt = elapsed * 0.001f;
                float goal = fminf(cruise_mm_s, sqrtf(900.0f * remaining));
                velocity_ref += fmaxf(-450.0f * dt, fminf(300.0f * dt, goal - velocity_ref));
                float actual = EncoderDistance(count > velocity_count ? count - velocity_count : 0) / dt;
                velocity_measured += dt / (0.04f + dt) * (actual - velocity_measured);
                float error = velocity_ref - velocity_measured;
                float base = 300.0f + 0.34f * velocity_ref + 0.35f * error;
                if ((base + velocity_i < speed || error < 0.0f) &&
                    (base + velocity_i > 290.0f || error > 0.0f))
                    velocity_i = fmaxf(-32.0f, fminf(32.0f, velocity_i + error * dt));
                velocity_pwm = fmaxf(290.0f, base + velocity_i);
                velocity_count = count;
                velocity_tick = now;
            }
            requested = fminf(speed, velocity_pwm);
        }
        float delta = MOTION_STEP_ACCEL_PWM_S * (now - step_pwm_tick) / 1000.0f;
        step_pwm += fmaxf(-delta, fminf(delta, requested - step_pwm));
        step_pwm_tick = now;
        int16_t pwm = (int16_t)step_pwm;
        if (!Motion_DriveStraight(direction, pwm)) Finish(MOTION_SENSOR_ERROR);
        return;
    }
    UpdateTurn(now, fresh_sample, sample_seconds, speed);
}

MotionStatus Motion_GetStatus(void) { return status; }
float Motion_GetAngle(void) { return angle; }
bool Motion_IsMPUReady(void) { return mpu_ready; }

void Motion_PollIdle(void)
{
    if (!mpu_connected || status == MOTION_RUNNING) return;
    MPU6050_Data_t data;
    if (MPU6050_Read_All(mpu_bus, &data)) RecordRaw(&data);
    else {
        motion_diag.state = 5;
        mpu_ready = false;
    }
}

const char *Motion_GetMPUState(void)
{
    switch (motion_diag.state) {
      case 1: return mpu_diag.stage == 1 ? "NO_ACK_68_69" :
                     mpu_diag.stage == 2 ? "WRONG_ID" : "CONFIG_ERROR";
      case 2: return "CAL_READ_ERROR";
      case 3: return "NOT_STILL_OR_BIAS";
      case 4: return "OK";
      case 5: return "READ_ERROR_RECAL_C";
      case 6: return "SAMPLE_GAP";
      case 7: return "GYRO_SATURATED";
      case 8: return "MPU_GIU_YEN_2S";
      default: return "NOT_INITIALIZED";
    }
}
