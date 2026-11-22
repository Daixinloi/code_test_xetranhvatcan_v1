#ifndef MOTOR_BENCH_H
#define MOTOR_BENCH_H
#include <stdint.h>
#include <stdbool.h>
#define MOTOR_BENCH_PWM 500
#define MOTOR_BENCH_MS 700u
typedef struct { bool active; uint32_t started; } MotorBench;
/* Explicit operator test ONLY, wheels raised. No gyro, encoder stop or ToF
 * correction, one equal pulse to both channels, no automatic restart. */
void MotorBench_Start(MotorBench *test, uint32_t now);
void MotorBench_Cancel(MotorBench *test);
bool MotorBench_Update(MotorBench *test, uint32_t now);
#endif
