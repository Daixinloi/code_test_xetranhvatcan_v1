#include "motor_bench.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(c) do { if (!(c)) { fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c); exit(1); } } while (0)
static int16_t a,b;
static unsigned resets;
void Encoder_Reset(void) { resets++; }
void Motor_SetSpeed(int16_t x,int16_t y) { a=x; b=y; }
void Motor_Stop(void) { a=b=0; }
int main(void)
{
    MotorBench test={0};
    CHECK(!MotorBench_Update(&test,0));
    MotorBench_Start(&test,100);
    CHECK(test.active && a==500 && b==500 && resets==1);
    CHECK(!MotorBench_Update(&test,799) && a==500 && b==500);
    CHECK(MotorBench_Update(&test,800) && !test.active && a==0 && b==0);
    CHECK(!MotorBench_Update(&test,1000) && a==0 && b==0);
    MotorBench_Start(&test,1100); MotorBench_Cancel(&test);
    CHECK(!test.active && !a && !b && !MotorBench_Update(&test,1200));
    MotorBench_Start(&test,UINT32_MAX-100);
    CHECK(!MotorBench_Update(&test,598));
    CHECK(MotorBench_Update(&test,599) && !a && !b);
    puts("PASS: equal motor pulse, 700 ms stop, cancellation, no restart, tick wrap");
    return 0;
}
