#include "motion.h"
#include "motor.h"
#include <math.h>
#include <stdlib.h>

static I2C_HandleTypeDef *mpu_bus;
static bool mpu_ready;
static float bias_z, angle, target, previous_rate;
static uint32_t started, sampled, settled;
static int direction;
static bool turning, curve_turn;
static int turn_sign;
typedef enum { TURN_APPROACH, TURN_BRAKE } TurnPhase;
static TurnPhase turn_phase;
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

static float HeadingError(float requested)
{
    return remainderf(requested - angle, 360.0f);
}

bool Motion_BeginStraight(void)
{
    heading_active = mpu_ready && motion_diag.state == 4;
    heading_target = angle;
    heading_pwm = 0;
    heading_integral = 0;
    heading_rate = previous_rate;
    heading_tick = sampled;
    return heading_active;
}

bool Motion_DriveStraight(int travel_direction, int16_t speed)
{
    if ((travel_direction != 1 && travel_direction != -1) || speed <= 0 ||
        !heading_active || !mpu_ready || motion_diag.state != 4 ||
        HAL_GetTick() - sampled > 100) {
        Motor_Stop();
        return false;
    }
    float limit = fminf(MOTION_HEADING_MAX_PWM, speed * 0.35f);
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
    Motor_SetSpeed(travel_direction * base - correction,
                   travel_direction * base + correction);
    return true;
}

static void DriveTurn(int16_t pwm)
{
    /* Keep the requested direction for the entire turn. Never chase an
     * overshoot by reversing the wheels. */
    if (curve_turn)
        Motor_SetSpeed(turn_sign > 0 ? pwm * 35 / 100 : pwm,
                       turn_sign > 0 ? pwm : pwm * 35 / 100);
    else
        Motor_SetSpeed(-turn_sign * pwm, turn_sign * pwm);
}

static void BrakeTurn(uint32_t now)
{
    Motor_Stop();
    turn_phase = TURN_BRAKE;
    phase_started = now;
    settled = 0;
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
    Motor_Stop();
    status = result;
}

bool Motion_Init(I2C_HandleTypeDef *bus)
{
    MPU6050_Data_t data;
    Motion_Cancel();
    mpu_bus = bus;
    mpu_ready = false;
    mpu_connected = false;
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
    if (variance > 1.0f || fabsf(bias_z) > 10.0f)
        return false;
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
    heading_active = false;
    Finish(MOTION_IDLE);
}

bool Motion_StartStep(int travel_direction)
{
    return Motion_StartStepHeading(travel_direction, angle);
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
    step_pwm = MOTION_STEP_START_PWM;
    last_encoder_progress = started;
    previous_distance_count = 0;
    status = MOTION_RUNNING;
    return true;
}

bool Motion_StartTurn(float angle_deg, bool curve)
{
    Motion_Cancel();
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
    curve_turn = curve;
    turn_sign = angle_deg > 0.0f ? 1 : -1;
    turn_phase = TURN_APPROACH;
    turn_rate_ref = 0.0f;
    turn_integral = 0.0f;
    turn_pwm = MOTION_TURN_START_PWM;
    turning = true;
    started = sampled = HAL_GetTick();
    settled = 0;
    previous_rate = (data.Gyro_Z / MPU6050_GYRO_LSB_PER_DPS - bias_z) * MOTION_GYRO_Z_SIGN;
    status = MOTION_RUNNING;
    /* Apply starting torque in the command handler, before a following UART
     * byte or any telemetry. Feedback takes over on the next gyro sample. */
    DriveTurn(MOTION_TURN_START_PWM);
    return true;
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
    return Motion_StartTurn(error, false);
}

void Motion_Update(int16_t speed)
{
    uint32_t now = HAL_GetTick();
    bool fresh_sample = false;
    float sample_seconds = 0.0f;
    if (mpu_ready && now - sampled >= MOTION_GYRO_SAMPLE_MS) {
        MPU6050_Data_t data;
        if (!MPU6050_Read_All(mpu_bus, &data)) {
            motion_diag.state = 5;
            mpu_ready = false;
            if (status == MOTION_RUNNING) Finish(MOTION_SENSOR_ERROR);
        } else {
            now = HAL_GetTick();
            uint32_t elapsed = now - sampled;
            motion_diag.sample_gap_ms = elapsed;
            RecordRaw(&data);
            float rate = (data.Gyro_Z / MPU6050_GYRO_LSB_PER_DPS - bias_z) * MOTION_GYRO_Z_SIGN;
            if (elapsed > 100 || data.Gyro_Z > 32000 || data.Gyro_Z < -32000) {
                motion_diag.state = elapsed > 100 ? 6 : 7;
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
        float mm_per_count = 3.14159265f * MOTION_WHEEL_DIAMETER_MM /
                             MOTION_COUNTS_PER_WHEEL_REV;
        int32_t count = Encoder_GetCount2();
        if (count < 0) count = -count;
        float distance = count * mm_per_count;
        if (distance >= step_distance_mm) {
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
        float delta = MOTION_STEP_ACCEL_PWM_S * (now - step_pwm_tick) / 1000.0f;
        step_pwm += fmaxf(-delta, fminf(delta, requested - step_pwm));
        step_pwm_tick = now;
        int16_t pwm = (int16_t)step_pwm;
        if (!Motion_DriveStraight(direction, pwm)) Finish(MOTION_SENSOR_ERROR);
        return;
    }
    if (!fresh_sample) return;
    float rate = previous_rate;
    float error = target - angle;
    float remaining = error * turn_sign;

    if (turn_phase == TURN_BRAKE) {
        Motor_Stop();
        if (fabsf(rate) >= 5.0f) settled = 0;
        else if (!settled) settled = now;
        if (now - phase_started < MOTION_TURN_BRAKE_MS || !settled || now - settled < 60)
            return;
        if (fabsf(error) <= MOTION_ANGLE_TOLERANCE_DEG) {
            Finish(MOTION_DONE);
        } else {
            /* One final brake, no stop/start pulses or reverse corrections. */
            Finish(MOTION_ANGLE_ERROR);
        }
        return;
    }

    float toward_rate = fmaxf(0.0f, rate * turn_sign);
    /* Account for gyro/filter delay and residual rotation under active brake. */
    float lead = toward_rate * 0.015f + toward_rate * toward_rate / 4000.0f;
    if (remaining <= fmaxf(0.7f, lead)) {
        BrakeTurn(now);
        return;
    }

    /* Continuous velocity profile: accelerate smoothly, then taper as the
     * remaining angle shrinks. Gyro feedback compensates load/battery changes. */
    float desired_rate = fminf(MOTION_TURN_MAX_DPS,
        fminf(6.0f * remaining, sqrtf(2.0f * MOTION_TURN_DECEL_DPS2 * remaining)));
    float slew_rate = desired_rate > turn_rate_ref ? MOTION_TURN_ACCEL_DPS2 : MOTION_TURN_DECEL_DPS2;
    float rate_delta = slew_rate * sample_seconds;
    turn_rate_ref += fmaxf(-rate_delta, fminf(rate_delta, desired_rate - turn_rate_ref));
    float rate_error = turn_rate_ref - rate * turn_sign;
    float limit = fminf(MOTION_TURN_PWM_LIMIT, fmaxf(260.0f, speed * 1.25f));
    /* Keep friction compensation at low rate; tapering it to zero stalls a
     * loaded N20 just before the target and makes the final degrees crawl. */
    float friction = 180.0f;
    float feedforward = friction + 1.5f * turn_rate_ref;
    float unsaturated = feedforward + 2.0f * rate_error + turn_integral;
    /* Integrate only when it will not push a saturated output farther out. */
    if ((unsaturated < limit || rate_error < 0.0f) &&
        (unsaturated > 0.0f || rate_error > 0.0f))
        turn_integral = fmaxf(-120.0f, fminf(120.0f,
            turn_integral + 4.0f * rate_error * sample_seconds));
    float requested_pwm = fmaxf(0.0f,
        fminf(limit, feedforward + 2.0f * rate_error + turn_integral));
    float pwm_delta = MOTION_TURN_PWM_SLEW_PER_SEC * sample_seconds;
    turn_pwm += fmaxf(-pwm_delta, fminf(pwm_delta, requested_pwm - turn_pwm));
    DriveTurn((int16_t)turn_pwm);
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
      default: return "NOT_INITIALIZED";
    }
}
