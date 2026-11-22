/* Host test: gcc -std=c11 -Wall -Wextra -ITests/fakes -IInc
 * Tests/test_motion.c Src/motion.c -lm -o build/test_motion.exe */
#include "motion.h"
#include "motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

volatile MPU6050_Diagnostics mpu_diag;
static uint32_t tick;
static int16_t pwm_a, pwm_b, encoder_a, encoder_b, gyro_z;
static bool init_ok = true, read_ok = true;
static I2C_HandleTypeDef bus;

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
bool MPU6050_Init(I2C_HandleTypeDef *b) { (void)b; return init_ok; }
bool MPU6050_Read_All(I2C_HandleTypeDef *b, MPU6050_Data_t *d)
{
    (void)b;
    *d = (MPU6050_Data_t){.Gyro_Z = gyro_z, .Accel_Z = 16384};
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
    int requested_sign = target > Motion_GetAngle() ? 1 : -1;
    uint32_t test_started = tick;
    bool braked = false;
    int previous_pwm = pwm_b < 0 ? -pwm_b : pwm_b;
    for (unsigned n = 0; n < 1500 && Motion_GetStatus() == MOTION_RUNNING; ++n) {
        /* Motor inertia and residual rotation under braking; no instant stop. */
        float drive = (pwm_b - pwm_a) * 0.5f;
        float driven_rate = (drive > 0 ? 1 : -1) * fmaxf(0.0f, fabsf(drive) - friction) * gain;
        if (pwm_a || pwm_b) assert(driven_rate * requested_sign >= 0.0f);
        simulated_rate += (driven_rate - simulated_rate) * response;
        gyro_z = (int16_t)(simulated_rate * MPU6050_GYRO_LSB_PER_DPS);
        tick += 5;
        Motion_Update(420);
        int current_pwm = pwm_b < 0 ? -pwm_b : pwm_b;
        if (!current_pwm) braked = true;
        else {
            assert(!braked); /* no restart pulses near the destination */
            assert(abs(current_pwm - previous_pwm) <=
                (int)ceilf(MOTION_TURN_PWM_SLEW_PER_SEC * 0.005f) + 1);
        }
        previous_pwm = current_pwm;
    }
    printf("model gain=%.2f friction=%.0f: %lu ms, error=%.2f deg, status=%d\n",
        gain, friction, (unsigned long)(tick - test_started),
        Motion_GetAngle() - target, Motion_GetStatus());
    assert(Motion_GetStatus() == MOTION_DONE);
    assert(tick - test_started < 2500);
    assert(fabsf(Motion_GetAngle() - target) <= MOTION_ANGLE_TOLERANCE_DEG);
    assert(pwm_a == 0 && pwm_b == 0);
    Motion_Cancel();
    gyro_z = 0;
}

static void SimulateTurn(float target) { SimulateTurnModel(target, 0.45f, 0.2f, 0.0f); }

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    assert(Motion_Init(&bus));
    assert(Motion_IsMPUReady());
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
    encoder_b = -382;
    Motion_Update(420);
    assert(Motion_GetStatus() == MOTION_RUNNING && pwm_a > 0 && pwm_b > 0);
    encoder_b = -383; /* ~50 mm at 818 counts/rev, diameter 34 mm */
    Motion_Update(420);
    assert(Motion_GetStatus() == MOTION_DONE);
    assert(pwm_a == 0 && pwm_b == 0);
    Motion_StartStep(-1);
    Motion_Update(420);
    assert(pwm_a < 0 && pwm_b < 0);
    encoder_b = 383;
    Motion_Update(420);
    assert(Motion_GetStatus() == MOTION_DONE && pwm_a == 0 && pwm_b == 0);
    Motion_StartStep(1);
    tick += MOTION_ENCODER_STALL_MS;
    Motion_Update(420);
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
    tick += 5;
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

    Motion_StartStep(1);
    tick += 5001;
    Motion_Update(420);
    assert(Motion_GetStatus() == MOTION_TIMEOUT && !pwm_a && !pwm_b);
    init_ok = false;
    assert(!Motion_Init(&bus));
    assert(!Motion_StartTurn(90, false) && !pwm_a && !pwm_b);
    puts("PASS: inertial left/right turns without reversal, overshoot braking, live yaw, distance, faults");
    return 0;
}
