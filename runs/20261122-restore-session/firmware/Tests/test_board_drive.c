#include "board_drive.h"
#include "motor.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(c) do { if (!(c)) { fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c); exit(1); } } while (0)
static int16_t out_a,out_b;
void Motor_SetSpeed(int16_t a,int16_t b)
{
    /* Check board enables BEFORE the proven low-level driver is invoked. */
    uint32_t expected = b > 0 ? GPIO_PIN_9 : b < 0 ? GPIO_PIN_8 : 0;
    CHECK((fake_gpiob.ODR & (GPIO_PIN_8|GPIO_PIN_9)) == expected);
    out_a=a;out_b=b;
}
void Motor_Stop(void) { out_a=out_b=0; }
int main(void)
{
    fake_gpiob.ODR=1u<<5; /* An unrelated GPIO must never be modified. */
    CHECK(BoardDrive_IsStopped());
    BoardDrive_Set(420,420); CHECK(out_a==420 && out_b==420);
    CHECK(!BoardDrive_IsStopped());
    BoardDrive_Set(-420,-420); CHECK(out_a==-420 && out_b==-420);
    BoardDrive_Set(-220,220); CHECK(out_a==-220 && out_b==220);
    BoardDrive_Set(220,-220); CHECK(out_a==220 && out_b==-220);
    BoardDrive_Set(360,500); CHECK(out_a==360 && out_b==500);
    BoardDrive_Set(500,0); CHECK(out_a==500 && out_b==0);
    BoardDrive_Stop(); CHECK(!out_a && !out_b && fake_gpiob.ODR==(1u<<5));
    CHECK(BoardDrive_IsStopped());
    puts("PASS: baseline PB8/PB9 outputs for forward, reverse, turns, corrections, stop; other pins preserved");
    return 0;
}
