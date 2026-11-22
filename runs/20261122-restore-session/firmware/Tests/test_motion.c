/* Host test: gcc -std=c11 -Wall -Wextra -ITests/fakes -IInc
 * Tests/test_motion.c Src/motion.c -lm -o build/test_motion.exe */
#include "motion.h"
#include "motor.h"
#include "board_drive.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#undef assert
#define assert(c) do { if (!(c)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #c); exit(1); } } while (0)

volatile MPU6050_Diagnostics mpu_diag;
static uint32_t tick;
static int16_t pwm_a, pwm_b, encoder_a, encoder_b, gyro_z;
static bool init_ok = true, read_ok = true;
static bool shaking;
static unsigned read_count;
static I2C_HandleTypeDef bus;

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
bool MPU6050_Init(I2C_HandleTypeDef *b) { (void)b; return init_ok; }
bool MPU6050_Read_All(I2C_HandleTypeDef *b, MPU6050_Data_t *d)
{
    (void)b;
    int16_t measured = shaking ? ((read_count++ & 1) ? 400 : -400) : gyro_z;
    *d = (MPU6050_Data_t){.Gyro_Z = measured, .Accel_Z = 16384};
    return read_ok;
}
void Motor_SetSpeed(int16_t a, int16_t b) { pwm_a = a; pwm_b = b; }
void Motor_Stop(void) { pwm_a = pwm_b = 0; }
int16_t Encoder_GetCount1(void) { return encoder_a; }
int16_t Encoder_GetCount2(void) { return encoder_b; }
void Encoder_Reset(void) { encoder_a = encoder_b = 0; }

static void SimulateTurnModel(float target, float gain, float response, float friction)
{
    float simulated_rate = gyro_z / MPU6050_GYRO_LSB_PER_DPS;
    uint32_t test_started = tick;
    int last_direction = (pwm_b > pwm_a) - (pwm_b < pwm_a);
    int previous_pwm = abs(pwm_b);
    for (unsigned n = 0; n < 1500 && Motion_GetStatus() == MOTION_RUNNING; ++n) {
        /* Motor inertia and residual rotation under braking; no instant stop. */
        float drive = (pwm_b - pwm_a) * 0.5f;
        float driven_rate = (drive > 0 ? 1 : -1) * fmaxf(0.0f, fabsf(drive) - friction) * gain;
        simulated_rate += (driven_rate - simulated_rate) * response;
        gyro_z = (int16_t)(simulated_rate * MPU6050_GYRO_LSB_PER_DPS);
        tick += 5;
        Motion_Update(420);
        int direction = (pwm_b > pwm_a) - (pwm_b < pwm_a);
        int current_pwm = abs(pwm_b);
        if (direction && last_direction && direction != last_direction)
            assert(!previous_pwm && fabsf(simulated_rate) < 3.0f);
        if (direction) last_direction = direction;
        if (previous_pwm && current_pwm) {
            assert(abs(current_pwm - previous_pwm) <=
                (int)ceilf(MOTION_TURN_PWM_SLEW_PER_SEC * MOTION_GYRO_SAMPLE_MS / 1000.0f) + 1);
        }
        previous_pwm = current_pwm;
    }
    printf("model gain=%.2f friction=%.0f: %lu ms, error=%.2f deg, status=%d\n",
        gain, friction, (unsigned long)(tick - test_started),
        Motion_GetAngle() - target, Motion_GetStatus());
    assert(Motion_GetStatus() == MOTION_DONE);
    assert(tick - test_started < 4500);
    assert(fabsf(Motion_GetAngle() - target) <= MOTION_ANGLE_TOLERANCE_DEG);
    assert(pwm_a == 0 && pwm_b == 0);
    Motion_Cancel();
    gyro_z = 0;
}

static void SimulateTurn(float target) { SimulateTurnModel(target, 0.45f, 0.2f, 0.0f); }

int main(void)
{
    const int16_t step_counts = (int16_t)ceilf(MOTION_STEP_MM * MOTION_COUNTS_PER_WHEEL_REV /
        (3.14159265f * MOTION_WHEEL_DIAMETER_MM));
    const int16_t forward_counts = (int16_t)ceilf(MOTION_MANUAL_FORWARD_ENCODER_MM *
        MOTION_COUNTS_PER_WHEEL_REV / (3.14159265f * MOTION_WHEEL_DIAMETER_MM));
    setvbuf(stdout, NULL, _IONBF, 0);
    assert(Motion_Init(&bus));
    assert(Motion_IsMPUReady());
    assert(Motion_StartManualStep(1, 420));
    assert(pwm_a == 420 && pwm_b == 428); /* Immediate torque with 2% right-wheel trim. */
    Motion_Cancel();
    assert(Motion_StartManualStep(-1, 420));
    assert(pwm_a == -420 && pwm_b == -420);
    Motion_Cancel();
    assert(Motion_StartDistanceHeading(1, 50, 0));
    Motion_Update(420);
    assert(pwm_a == 300 && pwm_b == 300); /* Autonomous profile remains gradual. */
    Motion_Cancel();
    assert(Motion_StartTurn(90, false));
    assert(pwm_a == -MOTION_TURN_START_PWM && pwm_b == MOTION_TURN_START_PWM); /* immediate starting torque */
    SimulateTurn(90);
    float previous = Motion_GetAngle();
    assert(Motion_StartTurn(-90, false));
    assert(pwm_a == MOTION_TURN_START_PWM && pwm_b == -MOTION_TURN_START_PWM);
    SimulateTurn(previous - 90); /* relative target without resetting yaw */

    assert(Motion_Init(&bus));
    assert(Motion_StartTurn(90, false));
    SimulateTurnModel(90, 0.7f, 0.12f, 140.0f);
    assert(Motion_Init(&bus));
    assert(Motion_StartTurn(-90, false));
    SimulateTurnModel(-90, 0.55f, 0.3f, 100.0f);
    assert(Motion_Init(&bus));
    assert(Motion_StartTurn(180, false));
    SimulateTurnModel(180, 0.55f, 0.2f, 100.0f);
    assert(Motion_Init(&bus));
    assert(Motion_StartTurn(45, true));
    assert(pwm_a > 0 && pwm_b > pwm_a);
    SimulateTurnModel(45, 0.55f, 0.2f, 30.0f);

    assert(Motion_Init(&bus));
    assert(Motion_StartTurnHeading(90));
    SimulateTurnModel(90, 0.45f, 0.2f, 140.0f);
    assert(Motion_Init(&bus));
    assert(Motion_StartTurnHeading(-22.1f));
    SimulateTurnModel(-22.1f, 0.55f, 0.2f, 140.0f);
    assert(Motion_Init(&bus));
    assert(Motion_StartTurnHeading(8));
    SimulateTurnModel(8, 0.9f, 0.2f, 30.0f);

    /* The turn brakes early at high angular speed. A forced overshoot
     * can be corrected after the gyro settles. */
    assert(Motion_Init(&bus));
    assert(Motion_StartTurnHeading(90));
    gyro_z = (int16_t)(100.0f * MPU6050_GYRO_LSB_PER_DPS);
    for (unsigned n = 0; n < 100 && pwm_b > 0; ++n) {
        tick += 10;
        Motion_Update(420);
    }
    assert(!pwm_a && !pwm_b && Motion_GetAngle() < 85.0f);
    for (unsigned n = 0; n < 60 && Motion_GetAngle() < 116.0f; ++n) {
        tick += 10;
        Motion_Update(420);
        assert(!pwm_a && !pwm_b);
    }
    assert(Motion_GetAngle() >= 116.0f);
    gyro_z = 0;
    for (unsigned n = 0; n < 6; ++n) {
        tick += 10;
        Motion_Update(420);
        assert(!pwm_a && !pwm_b);
    }
    tick += 10;
    Motion_Update(420);
    assert(motion_diag.turn_error_mdeg < -25000 && motion_diag.turn_error_mdeg > -30000);
    assert(Motion_GetStatus() == MOTION_RUNNING && pwm_a > 0 && pwm_b < 0);
    SimulateTurnModel(90, 0.55f, 0.2f, 120.0f);

    /* A command received during residual yaw waits for the body to settle. */
    assert(Motion_Init(&bus));
    gyro_z = (int16_t)(50.0f * MPU6050_GYRO_LSB_PER_DPS);
    assert(Motion_StartTurnHeading(8));
    for (unsigned n = 0; n < 7; ++n) {
        tick += 10;
        Motion_Update(420);
        assert(!pwm_a && !pwm_b);
    }
    gyro_z = 0;
    SimulateTurnModel(8, 0.55f, 0.2f, 30.0f);

    previous = Motion_GetAngle();
    gyro_z = 1476; /* rotate manually at 45 deg/s while stopped */
    for (unsigned n = 0; n < 40; ++n) { tick += 5; Motion_Update(420); }
    assert(Motion_GetAngle() > previous + 8.0f);
    gyro_z = 0;

    Motion_StartStep(1);
    Motion_Update(420);
    assert(pwm_a > 0 && pwm_b > 0);
    encoder_a = 800; /* uncalibrated encoder 1 must not stop either wheel */
    Motion_Update(420);
    assert(pwm_a > 0 && pwm_b > 0);
    encoder_b = -(forward_counts - 1);
    Motion_Update(420);
    assert(Motion_GetStatus() == MOTION_RUNNING && pwm_a > 0 && pwm_b > 0);
    encoder_b = -forward_counts;
    Motion_Update(420);
    assert(Motion_GetStatus() == MOTION_DONE);
    assert(motion_diag.manual_forward_stop_counts == forward_counts);
    assert(pwm_a == 0 && pwm_b == 0);
    assert(Motion_StartStepHeading(1, Motion_GetAngle()));
    assert(!Motion_ExtendForward(100, Motion_GetAngle()));
    encoder_b = step_counts - 10;
    Motion_Update(420);
    int16_t held_pwm_a = pwm_a, held_pwm_b = pwm_b;
    assert(Motion_ExtendForward(100, Motion_GetAngle()));
    assert(encoder_b == step_counts - 10 && pwm_a == held_pwm_a && pwm_b == held_pwm_b);
    encoder_b = step_counts;
    Motion_Update(420);
    assert(Motion_GetStatus() == MOTION_RUNNING && pwm_a > 0 && pwm_b > 0);
    Motion_Cancel();
    assert(Motion_Init(&bus));
    Motion_SetCruise(175);
    assert(Motion_StartDistanceHeading(1, 100, Motion_GetAngle()));
    Motion_Update(360);
    int16_t first_course_pwm = pwm_a;
    assert(pwm_a == MOTION_MIN_PWM && pwm_b == MOTION_MIN_PWM);
    for (unsigned n = 1; n <= 12; ++n) {
        encoder_b = (int16_t)(4 * n);
        tick += 10;
        Motion_Update(360);
        int16_t common_pwm = (pwm_a + pwm_b) / 2;
        assert(abs(common_pwm - first_course_pwm) <= 7);
        first_course_pwm = common_pwm;
    }
    assert(first_course_pwm > MOTION_MIN_PWM && first_course_pwm < 360);
    Motion_Cancel();
    assert(Motion_StartDistanceHeading(1, 100, Motion_GetAngle() + 15.0f));
    for (unsigned n = 1; n <= 45; ++n) {
        encoder_b = (int16_t)(9 * n);
        tick += 10;
        Motion_Update(360);
        assert(pwm_a > 0 && pwm_b > 0);
    }
    assert(pwm_a >= 280 && pwm_b > pwm_a && pwm_b <= 480);
    Motion_Cancel();
    Motion_SetCruise(0);
    Motion_SetCourseSteering(true);
    Motion_SetCruise(150);
    assert(Motion_StartDistanceHeading(1, 50.0f, Motion_GetAngle()));
    Motion_Update(200);
    assert(pwm_a == MOTION_COURSE_MIN_PWM && pwm_b > pwm_a);
    encoder_b = -20;
    held_pwm_a = pwm_a;
    held_pwm_b = pwm_b;
    assert(!Motion_SteerForward(Motion_GetAngle() + 16.0f));
    assert(Motion_SteerForward(Motion_GetAngle() + 7.0f));
    assert(encoder_b == -20 && pwm_a == held_pwm_a && pwm_b == held_pwm_b);
    for (unsigned sample = 0; sample < 10; ++sample) {
        encoder_b -= 4;
        tick += 10;
        Motion_Update(200);
        assert(pwm_a >= MOTION_COURSE_MIN_PWM && pwm_b >= MOTION_COURSE_MIN_PWM);
    }
    assert(pwm_b > pwm_a + 20);
    int16_t field_counts = (int16_t)ceilf(50.0f / MOTION_FIELD_MM_PER_COUNT);
    encoder_b = -(field_counts - 1);
    Motion_Update(360);
    assert(Motion_GetStatus() == MOTION_RUNNING && pwm_a > 0 && pwm_b > pwm_a);
    encoder_b = -field_counts;
    Motion_Update(360);
    assert(Motion_GetStatus() == MOTION_DONE && !pwm_a && !pwm_b);
    Motion_SetCruise(0);
    assert(Motion_StartDistanceHeading(1, 100.0f, Motion_GetAngle()));
    encoder_b = -80;
    tick += 10;
    Motion_Update(420);
    held_pwm_a = pwm_a;
    held_pwm_b = pwm_b;
    assert(!Motion_RetargetForward(0.0f, Motion_GetAngle()));
    assert(Motion_RetargetForward(35.0f, Motion_GetAngle() - 6.0f));
    assert(encoder_b == -80 && pwm_a == held_pwm_a && pwm_b == held_pwm_b);
    int16_t retarget_counts = 80 + (int16_t)ceilf(35.0f / MOTION_FIELD_MM_PER_COUNT);
    encoder_b = -(retarget_counts - 1);
    tick += 10;
    Motion_Update(420);
    assert(Motion_GetStatus() == MOTION_RUNNING && pwm_a > 0 && pwm_b > 0);
    encoder_b = -retarget_counts;
    Motion_Update(420);
    assert(Motion_GetStatus() == MOTION_DONE && !pwm_a && !pwm_b);

    /* Saturation must shift both outputs together to retain the correction. */
    for (int turn = -1; turn <= 1; turn += 2) {
        assert(Motion_StartDistanceHeading(1, 140.0f, Motion_GetAngle() + turn * 5.0f));
        for (unsigned sample = 1; sample <= 55; ++sample) {
            encoder_b = (int16_t)(2 * sample);
            tick += 10;
            Motion_Update(420);
            assert(pwm_a >= MOTION_COURSE_MIN_PWM && pwm_b >= MOTION_COURSE_MIN_PWM);
            assert(pwm_a <= 480 && pwm_b <= 480);
        }
        assert((pwm_b - pwm_a) * turn >= 140);
        Motion_Cancel();
    }
    Motion_SetCourseSteering(false);
    Motion_StartStep(-1);
    Motion_Update(420);
    assert(pwm_a < 0 && pwm_b < 0);
    encoder_b = step_counts;
    Motion_Update(420);
    assert(Motion_GetStatus() == MOTION_DONE && pwm_a == 0 && pwm_b == 0);
    Motion_StartStep(1);
    for (unsigned n = 0; n < MOTION_ENCODER_STALL_MS / 10; ++n) {
        tick += 10; Motion_Update(420);
    }
    assert(Motion_GetStatus() == MOTION_ENCODER_ERROR && pwm_a == 0 && pwm_b == 0);

    /* Even a forced large overshoot must brake and report an error; it must
     * never reverse to chase the target. */
    assert(Motion_Init(&bus));
    assert(Motion_StartTurn(90, false));
    gyro_z = 31000;
    for (unsigned n = 0; n < 4; ++n) { tick += 50; Motion_Update(420); }
    gyro_z = 0;
    for (unsigned n = 0; n < 100; ++n) {
        tick += 5; Motion_Update(420);
        assert(pwm_a <= 0 && pwm_b >= 0);
    }
    assert(Motion_GetStatus() == MOTION_ANGLE_ERROR && !pwm_a && !pwm_b);

    assert(Motion_StartTurn(90, false));
    read_ok = false;
    tick += MOTION_GYRO_SAMPLE_MS;
    Motion_Update(420);
    assert(Motion_GetStatus() == MOTION_SENSOR_ERROR && !pwm_a && !pwm_b);
    assert(!Motion_IsMPUReady());
    assert(!Motion_StartTurn(90, false));
    read_ok = true;
    assert(Motion_Init(&bus));
    assert(Motion_IsMPUReady());
    assert(Motion_StartTurn(90, false));
    tick += 150;
    Motion_Update(420);
    assert(Motion_GetStatus() == MOTION_SENSOR_ERROR && !pwm_a && !pwm_b);
    assert(motion_diag.last_fault==6 && motion_diag.max_sample_gap_ms>=150);

    /* The angle lost during a sampling gap must remain invalid even after
     * another good read, until a stationary calibration creates a new zero. */
    tick += 10; Motion_Update(420);
    assert(!Motion_IsMPUReady());
    assert(motion_diag.last_fault==6); /* automatic retry cannot erase fault evidence */
    assert(!Motion_StartStep(1));

    assert(Motion_Init(&bus));
    Motion_StartStep(1);
    for (unsigned n = 0; n < 51; ++n) {
        encoder_b = n;
        tick += 100;
        Motion_Update(420);
    }
    assert(Motion_GetStatus() == MOTION_TIMEOUT && !pwm_a && !pwm_b);
    init_ok = false;
    assert(!Motion_Init(&bus));
    assert(!Motion_StartTurn(90, false) && !pwm_a && !pwm_b);
    assert(!Motion_StartStep(1) && !pwm_a && !pwm_b);
    init_ok = true;
    assert(Motion_Init(&bus));
    for (int dir = -1; dir <= 1; dir += 2) {
        assert(Motion_StartStep(dir));
        int initial_difference = pwm_a - pwm_b;
        gyro_z = 328; /* unwanted positive yaw, must request negative torque */
        tick += 10; Motion_Update(420);
        assert(pwm_a - pwm_b > initial_difference);
        assert(pwm_a * dir > 0 && pwm_b * dir > 0);
        Motion_Cancel();
        gyro_z = -328;
        tick += 10; Motion_Update(420);
        assert(Motion_StartStep(dir));
        tick += 10; Motion_Update(420);
        assert(pwm_b > pwm_a);
        assert(pwm_a * dir > 0 && pwm_b * dir > 0);
        read_ok = false;
        tick += 10; Motion_Update(420);
        assert(Motion_GetStatus() == MOTION_SENSOR_ERROR && !pwm_a && !pwm_b);
        read_ok = true; gyro_z = 0;
        assert(Motion_Init(&bus));
    }
    /* Unequal wheel drive must converge in both forward and reverse. Exercise
     * the same heading helper used by continuous auto drive and ToF retreat. */
    for (int dir = -1; dir <= 1; dir += 2) {
        gyro_z = 0;
        assert(Motion_Init(&bus));
        assert(Motion_BeginStraight());
        float rate = 0;
        for (unsigned n = 0; n < 200; ++n) {
            assert(Motion_DriveStraight(dir, 420));
            assert(pwm_a * dir > 0 && pwm_b * dir > 0);
            float desired = (pwm_b - pwm_a) * 0.2f + dir * 12.0f;
            rate += (desired - rate) * 0.25f;
            gyro_z = (int16_t)(rate * MPU6050_GYRO_LSB_PER_DPS);
            tick += 10; Motion_Update(420);
        }
        assert(fabsf(rate) < 2.0f);
        assert(fabsf(Motion_GetAngle()) < 3.0f);
        Motion_Cancel();
    }
    /* Keep the requested map heading across consecutive steps instead of
     * capturing the disturbed current yaw as a new straight direction. */
    gyro_z = 0;
    assert(Motion_Init(&bus));
    gyro_z = 3280;
    for (unsigned i = 0; i < 10; ++i) { tick += 10; Motion_Update(420); }
    gyro_z = 0;
    for (unsigned i = 0; i < 5; ++i) { tick += 10; Motion_Update(420); }
    assert(Motion_GetAngle() > 9.0f);
    for (int dir = -1; dir <= 1; dir += 2) {
        assert(Motion_StartStepHeading(dir, 0));
        int previous_a = 0, previous_b = 0;
        for (unsigned i = 0; i < 20; ++i) {
            tick += 10; Motion_Update(420);
            assert(pwm_a > pwm_b && pwm_a * dir > 0 && pwm_b * dir > 0);
            if (i) {
                assert(abs(pwm_a - previous_a) <= 18 && abs(pwm_b - previous_b) <= 18);
            }
            previous_a = pwm_a; previous_b = pwm_b;
        }
        Motion_Cancel();
    }
    assert(!Motion_StartDistanceHeading(1, NAN, 0));
    assert(!Motion_StartDistanceHeading(0, 50, 0));
    assert(!Motion_StartTurnHeading(NAN));
    /* Startup motion must not permanently disable a connected MPU. Collect
     * a fresh stationary window without blocking sensor/UART servicing. */
    shaking = true;
    assert(!Motion_Init(&bus));
    assert(!Motion_IsMPUReady() && motion_diag.state == 8);
    assert(!pwm_a && !pwm_b);
    for (unsigned n = 0; n < 240; ++n) {
        tick += 10; Motion_Update(420);
        assert(!Motion_IsMPUReady() && !pwm_a && !pwm_b);
    }
    shaking = false; gyro_z = -24;
    for (unsigned n = 0; n < MOTION_CALIBRATION_SAMPLES-1; ++n) {
        tick += 10; Motion_Update(420);
        assert(!Motion_IsMPUReady());
    }
    tick += 10; Motion_Update(420);
    assert(Motion_IsMPUReady() && motion_diag.state == 4);
    assert(fabsf(Motion_GetAngle()) < 0.001f && !pwm_a && !pwm_b);
    assert(motion_diag.calibration_samples == MOTION_CALIBRATION_SAMPLES);
    /* Direct bench output must prevent calibration even though Motion idle. */
    shaking = true;
    assert(!Motion_Init(&bus));
    shaking = false; gyro_z = 0;
    BoardDrive_Set(500,500);
    for (unsigned n = 0; n < 250; ++n) { tick += 10; Motion_Update(420); }
    assert(!Motion_IsMPUReady() && motion_diag.calibration_samples == 0);
    BoardDrive_Stop();
    for (unsigned n = 0; n < MOTION_CALIBRATION_SAMPLES; ++n) { tick += 10; Motion_Update(420); }
    assert(Motion_IsMPUReady() && !pwm_a && !pwm_b);
    shaking = true;
    assert(!Motion_Init(&bus));
    shaking = false; read_ok = false;
    tick += 10; Motion_Update(420);
    assert(!Motion_IsMPUReady() && motion_diag.state == 5 && !pwm_a && !pwm_b);
    read_ok = true;
    /* Consecutive manual 5 cm commands retain the first heading, even after
     * the normal completion/cancel path. Explicit reset starts a new line. */
    for (int dir=-1; dir<=1; dir+=2) {
        gyro_z=0; assert(Motion_Init(&bus));
        assert(Motion_StartManualStep(dir,420));
        encoder_b=dir > 0 ? forward_counts : step_counts; Motion_Update(420);
        assert(Motion_GetStatus()==MOTION_DONE);
        Motion_Cancel();
        gyro_z=3280;
        for (unsigned n=0;n<4;++n) {tick+=10;Motion_Update(420);}
        gyro_z=0;tick+=10;Motion_Update(420);
        assert(Motion_GetAngle()>3.0f);
        assert(Motion_StartManualStep(dir,420));
        for (unsigned n=0;n<5;++n) {tick+=10;Motion_Update(420);}
        assert(pwm_a>pwm_b); /* correct toward the ORIGINAL heading */
        Motion_Cancel();Motion_ResetManualHeading();
        assert(Motion_StartManualStep(dir,420));
        tick+=10;Motion_Update(420);
        assert(pwm_b - pwm_a == (dir > 0 ? pwm_a * 2 / 100 : 0));
        Motion_Cancel();
    }
    for (int dir = -1; dir <= 1; dir += 2) {
        assert(Motion_Init(&bus));
        assert(Motion_StartManualStep(dir, 420));
        gyro_z = dir * 3280;
        for (unsigned n = 0; n < 8; ++n) { tick += 10; Motion_Update(420); }
        gyro_z = 0;
        tick += 10;
        encoder_b = dir > 0 ? forward_counts : -step_counts;
        int16_t end_encoder = encoder_b;
        Motion_Update(420);
        assert(Motion_GetStatus() == MOTION_RUNNING && !pwm_a && !pwm_b);
        encoder_b += 30;
        SimulateTurn(0);
        assert(Motion_GetReportedEncoder2() == end_encoder);

        float target_angle = Motion_GetAngle() + dir * 10.0f;
        assert(Motion_StartTurnHeading(target_angle));
        assert(Motion_GetReportedEncoder2() == encoder_b);
        SimulateTurn(target_angle);
    }
    puts("PASS: gyro heading hold forward/reverse, smooth turns, distance, sensor/encoder faults");
    return 0;
}
