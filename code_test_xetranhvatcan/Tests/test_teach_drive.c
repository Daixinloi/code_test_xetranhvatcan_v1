#include <assert.h>
#include <string.h>
#include "stm32f1xx_hal.h"

/* Emulate the two STM32 pages and inject a failure before the commit word. */
#define FLASH_PAGE_SIZE 1024u
#define FLASH_TYPEERASE_PAGES 0u
#define FLASH_TYPEPROGRAM_HALFWORD 0u
#define __HAL_FLASH_CLEAR_FLAG(flags) ((void)0)
typedef struct { uint32_t TypeErase, PageAddress, NbPages; } FLASH_EraseInitTypeDef;
_Alignas(FLASH_PAGE_SIZE) uint8_t _teach_flash_start[2 * FLASH_PAGE_SIZE];
static unsigned writes_left;

static uint8_t *FlashPointer(uint32_t address)
{
    uint32_t offset = address - (uint32_t)(uintptr_t)_teach_flash_start;
    assert(offset < sizeof(_teach_flash_start));
    return &_teach_flash_start[offset];
}

static HAL_StatusTypeDef HAL_FLASH_Unlock(void) { return HAL_OK; }
static HAL_StatusTypeDef HAL_FLASH_Lock(void) { return HAL_OK; }

static HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *failed)
{
    assert(erase->NbPages == 1);
    *failed = UINT32_MAX;
    memset(FlashPointer(erase->PageAddress), 0xFF, FLASH_PAGE_SIZE);
    return HAL_OK;
}

static HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t value)
{
    assert(type == FLASH_TYPEPROGRAM_HALFWORD);
    if (!writes_left)
        return HAL_ERROR;
    --writes_left;
    uint8_t *bytes = FlashPointer(address);
    assert(bytes[0] == 0xFF && bytes[1] == 0xFF);
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
    return HAL_OK;
}

#include "../Src/teach_storage.c"
#include "../Src/teach_drive.c"

static int16_t encoder, left_pwm, right_pwm;
static float angle;
static float straight_heading;
volatile MotionDiagnostics motion_diag = {.state = 4};

bool Motion_IsMPUReady(void) { return motion_diag.state == 4; }
float Motion_GetAngle(void) { return angle; }
bool Motion_BeginStraight(void) { return Motion_IsMPUReady(); }
bool Motion_BeginStraightHeading(float heading)
{
    straight_heading = heading;
    return Motion_BeginStraight();
}
bool Motion_Init(I2C_HandleTypeDef *bus) { (void)bus; return true; }
void Motion_Update(int16_t speed) { (void)speed; }
bool SoftUART_ReadByte(uint8_t *command) { (void)command; return false; }
void SoftUART_TransmitByte(uint8_t value) { (void)value; }
uint32_t HAL_GetTick(void) { return 0; }
void HAL_Delay(uint32_t ms) { (void)ms; }
int16_t Encoder_GetCount2(void) { return encoder; }
int16_t Encoder_GetCount1(void) { return 0; }
void BoardDrive_Set(int16_t left, int16_t right) { left_pwm = left; right_pwm = right; }
void BoardDrive_Stop(void) { BoardDrive_Set(0, 0); }
bool Motion_DriveStraight(int direction, int16_t speed)
{
    BoardDrive_Set(direction * speed, direction * speed);
    return true;
}

int main(void)
{
    const char ranges_off[5][8] = {"---", "---", "---", "---", "---"};
    char frame[BLUETOOTH_STATUS_SIZE];
    BluetoothStatus_Format(frame, sizeof(frame), false, 0, ranges_off,
                           "DUNG:-", "CHO_MPU", 0, 0);
    assert(strcmp(frame, "Goc:--- do | ToF:HT=---,TL=---,G=---,TR=---,HP=--- mm"
                         " | Lenh:DUNG:-:CHO_MPU | E1:0,E2:0*53\r\n") == 0);

    memset(_teach_flash_start, 0xFF, sizeof(_teach_flash_start));
    assert(TeachStorage_Load(&route) == 0);

    Command('Z', 0);
    Command('F', 10);
    UpdateMove(10);
    assert(recording && !active && left_pwm == 0);
    Command('F', 100);
    UpdateMove(100);
    assert(recording && left_pwm == STRAIGHT_PWM && right_pwm == STRAIGHT_PWM);
    encoder = 1000;
    UpdateMove(150);
    assert(fabsf(y_mm - (navigation_config.start_y_mm + 1000 * MOTION_FIELD_MM_PER_COUNT)) < 0.01f);
    Command('R', 150);
    assert(left_pwm == 0);
    Command('R', 250);
    UpdateMove(150 + SETTLE_MS);
    UpdateMove(151 + SETTLE_MS);
    assert(route.count == 1 && route.segments[0].target == 1000);
    assert(left_pwm == PIVOT_PWM && right_pwm == -PIVOT_PWM);
    angle = -90;
    Command('B', 420);
    UpdateMove(420 + SETTLE_MS);
    UpdateMove(421 + SETTLE_MS);
    assert(route.count == 2 && route.segments[1].target == 900);
    assert(left_pwm == -STRAIGHT_PWM && right_pwm == -STRAIGHT_PWM);
    encoder = 900;
    UpdateMove(680);
    Command('x', 690);
    assert(left_pwm == 0 && recording);
    UpdateMove(690 + SETTLE_MS);
    assert(route.count == 3 && route.segments[2].target == 100);
    Command('U', 875);
    assert(recording && !replaying && route.count == 3);
    Command('V', 880);
    assert(save_pending);
    Command('U', 881);
    assert(!replaying && strcmp(notice, "DANG_LUU") == 0);
    writes_left = 1000;
    SaveIfStopped();
    assert(!recording && !save_pending && saved_id == 1);
    assert(strcmp(notice, "DA_LUU:ID1:SEG3") == 0);
    writes_left = 7;
    assert(!TeachStorage_Save(&route, &saved_id));
    assert(TeachStorage_Load(&route) == 1 && route.count == 3);

    angle = encoder = 0; /* Car returned to the start pose. */
    assert(!at_start);
    Command('U', 1010);
    assert(replaying && active == 'F' && y_mm == navigation_config.start_y_mm);
    UpdateMove(1010);
    encoder = 1000;
    UpdateMove(1100);
    assert(settling && left_pwm == 0);
    UpdateMove(1100 + SETTLE_MS);
    assert(active == 'R');
    angle = -90;
    UpdateMove(1300);
    UpdateMove(1300 + SETTLE_MS);
    assert(active == 'B');
    encoder = 900;
    UpdateMove(1500);
    UpdateMove(1500 + SETTLE_MS);
    assert(!replaying && !active && left_pwm == 0);

    Command('Y', 1800);
    Command('P', 1810);
    Command('x', 1820);
    UpdateMove(1820 + SETTLE_MS);
    assert(!replaying && !active && left_pwm == 0);

    Command('Z', 2100);
    Command('L', 2110);
    motion_diag.state = 5;
    UpdateMove(2120);
    assert(!recording && settling && left_pwm == 0);
    motion_diag.state = 4;
    UpdateMove(2120 + SETTLE_MS);
    Command('Y', 2400);
    Command('B', 2410);
    encoder -= 1000;
    UpdateMove(2510);
    assert(settling && left_pwm == 0 && strcmp(notice, "GIOI_HAN_SAN") == 0);

    UpdateMove(2510 + SETTLE_MS);
    angle = encoder = 0;
    Command('Y', 2800);
    Command('L', 2810);
    UpdateMove(2810);
    assert(left_pwm == -PIVOT_PWM && right_pwm == PIVOT_PWM);
    angle = 6.8f;
    UpdateMove(2860);
    assert(left_pwm == -PIVOT_PWM);
    angle = 7.0f;
    UpdateMove(2900);
    assert(settling && left_pwm == 0 && cutoff == TAP_TURN_TENTHS);
    UpdateMove(2900 + SETTLE_MS);
    Command('R', 3100);
    Command('R', 3200);
    angle = -5.0f;
    UpdateMove(3210);
    assert(left_pwm == PIVOT_PWM && right_pwm == -PIVOT_PWM);
    Command('R', 3300);
    angle = -10.0f;
    UpdateMove(3300);
    UpdateMove(3300 + HOLD_TIMEOUT_MS);
    assert(settling && left_pwm == 0);
    UpdateMove(3300 + HOLD_TIMEOUT_MS + SETTLE_MS);

    Command('F', 3900);
    Command('F', 4000);
    encoder += 20;
    UpdateMove(4100);
    assert(left_pwm == STRAIGHT_PWM);
    UpdateMove(4000 + HOLD_TIMEOUT_MS);
    assert(settling && left_pwm == 0);
    UpdateMove(4000 + HOLD_TIMEOUT_MS + SETTLE_MS);

    Command('J', 4500);
    assert(replaying && course_playback && active == 'F' && straight_heading == angle);
    encoder += teach_course[0].target;
    UpdateMove(4600);
    assert(settling);
    UpdateMove(4600 + SETTLE_MS);
    assert(active == 'R' && fabsf(course_yaw - (start_yaw - 28.3f)) < 0.01f);
    angle = course_yaw - 1.0f; /* Turn coast beyond the nominal heading. */
    UpdateMove(4800);
    UpdateMove(4800 + SETTLE_MS);
    assert(active == 'F' && straight_heading == course_yaw);
    Command('x', 5000);
    UpdateMove(5000 + SETTLE_MS);
    assert(!active && !replaying && !left_pwm && TeachStorage_Load(&route) == 1);

    puts("PASS: teach tap/hold, timeout, flash fallback, replay, T14 route, stop, stale MPU, field boundary");
    return 0;
}
