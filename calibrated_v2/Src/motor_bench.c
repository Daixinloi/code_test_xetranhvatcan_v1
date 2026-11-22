#include "motor_bench.h"
#include "motor.h"
#include "board_drive.h"
void MotorBench_Start(MotorBench *test, uint32_t now)
{
    test->started = now; test->active = true;
    Encoder_Reset();
    BoardDrive_Set(MOTOR_BENCH_PWM, MOTOR_BENCH_PWM);
}
void MotorBench_Cancel(MotorBench *test)
{
    if (test->active) BoardDrive_Stop();
    test->active = false;
}
bool MotorBench_Update(MotorBench *test, uint32_t now)
{
    if (!test->active || now - test->started < MOTOR_BENCH_MS) return false;
    MotorBench_Cancel(test);
    return true;
}
