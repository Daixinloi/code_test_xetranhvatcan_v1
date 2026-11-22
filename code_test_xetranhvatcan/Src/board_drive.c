#include "board_drive.h"
#include "motor.h"
volatile BoardDriveDiagnostics board_drive_diag;
static bool output_active;
void BoardDrive_Set(int16_t a, int16_t b)
{
    /* Baseline forward: PB9=1 PB8=0; reverse: PB9=0 PB8=1.
     * Keep levels consistent with electrical channel B during in-place turns
     * too. Their physical role on this assembled board is not inferred from
     * the schematic: restoring these outputs was verified with both wheels. */
    if (b > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    } else if (b < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    } else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
    Motor_SetSpeed(a,b);
    output_active = a || b;
    if (a || b) {
        board_drive_diag.requested_a = a;
        board_drive_diag.requested_b = b;
        board_drive_diag.aux_levels = GPIOB->ODR & (GPIO_PIN_8|GPIO_PIN_9);
        board_drive_diag.updates++;
    }
}
void BoardDrive_Stop(void)
{
    Motor_Stop();
    output_active = false;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
}
bool BoardDrive_IsStopped(void) { return !output_active; }
