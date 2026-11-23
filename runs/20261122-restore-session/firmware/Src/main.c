/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : CHUONG TRINH ROBOT TU HANH TRONG VE TRANH VAT CAN VA DO LINE
 *                   - 3 Cam bien ToF dau xe (T-L, T-C, T-R) de ne vat can
 *                   - 2 Cam bien ToF duoi xe (S-L, S-R) xac nhan da qua vat can
 *                   - 2 Cam bien hong ngoai IR (PA4, PA5) de do line den
 *                   - Cam bien goc nghieng MPU-6050 tinh goc Yaw
 *                   - Dieu khien dong co TB6612 va doc xung Encoder
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "software_uart.h"
#include "bluetooth_status.h"
#include "vl53l0x.h"
#include "tof_offset.h"
#include "motor.h"
#include "board_drive.h"
#include "motor_bench.h"
#include "mpu6050.h"
#include "motion.h"
#include "front_clearance.h"
#include "navigation.h"
#include "navigation_config.h"
#ifdef TEACH_DRIVE
#include "teach_drive.h"
#endif
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  const char *name;
  GPIO_TypeDef *port;
  uint16_t pin;
  const char *pin_name;
  uint8_t addr;
  bool ok;
  uint16_t dist;
  int16_t offset_mm; // Bu tru sai so cu ly gan (Offset)
  uint16_t raw_dist; // Last uncorrected sample, never reconstruct from clamped output
  uint32_t raw_tick;
} Sensor_Info_t;

typedef enum
{
  ROBOT_MODE_MANUAL = 0,
  ROBOT_MODE_AUTO = 1
} RobotMode_t;

typedef enum
{
  ACTION_STOP = 0,
  ACTION_FORWARD,
  ACTION_BACKWARD,
  ACTION_TURN_LEFT_90,
  ACTION_TURN_RIGHT_90,
  ACTION_CURVE_LEFT_45,
  ACTION_CURVE_RIGHT_45,
  ACTION_PIVOT_LEFT_10,
  ACTION_PIVOT_RIGHT_10,
  ACTION_FOLLOW_LINE,
  ACTION_AVOID_LEFT_45,
  ACTION_AVOID_RIGHT_45,
  ACTION_RETURN_TO_LINE,
  ACTION_EMERGENCY_REVERSE,
  ACTION_MOTOR_BENCH
} RobotAction_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_RED_PIN GPIO_PIN_8
#define LED_RED_PORT GPIOB
#define LED_GREEN_PIN GPIO_PIN_9
#define LED_GREEN_PORT GPIOB

/* Chan cam bien do Line IR */
#define IR_LEFT_PIN GPIO_PIN_4
#define IR_LEFT_PORT GPIOA
#define IR_RIGHT_PIN GPIO_PIN_5
#define IR_RIGHT_PORT GPIOA

/* Nguong khoang cach tranh vat can ToF (mm) */
#define DIST_TRIGGER 240 // Phat hien vat can phia truoc (<24cm) -> re 45 do de ne
#define DIST_CLEARED 260 // ToF sau xac nhan da vuot qua hoan toan vat can (>26cm)

#define TOF_POLL_INTERVAL_MS 10u
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
VL53L0X_Dev dev[5];

/*
 * Danh sach 5 cam bien ToF (Kem offset giup do sat cự ly gan):
 * - cb[0]: Sau-Trai   (PB0 ) -> Offset 0mm
 * - cb[1]: Truoc-Trai (PB14) -> Offset 55mm = 25 cu + 30 con du tren man hinh
 * - cb[2]: Truoc-Giua (PB13) -> Offset 30mm = 0 cu + 30 con du tren man hinh
 * - cb[3]: Truoc-Phai (PB12) -> Offset 85mm = 25 cu + 60 con du tren man hinh
 * - cb[4]: Sau-Phai   (PB1 ) -> Offset 0mm
 */
Sensor_Info_t cb[5] = {
    {"CB1 Hong-Trai", GPIOB, GPIO_PIN_0, "PB0", 0x30, false, 8190, 0, 8190, 0},
    {"CB2 Truoc-Trai", GPIOB, GPIO_PIN_14, "PB14", 0x32, false, 8190, TOF_OFFSET_LEFT_MM, 8190, 0},
    {"CB3 Truoc-Giua", GPIOB, GPIO_PIN_13, "PB13", 0x34, false, 8190, TOF_OFFSET_CENTER_MM, 8190, 0},
    {"CB4 Truoc-Phai", GPIOB, GPIO_PIN_12, "PB12", 0x36, false, 8190, TOF_OFFSET_RIGHT_MM, 8190, 0},
    {"CB5 Hong-Phai", GPIOB, GPIO_PIN_1, "PB1", 0x38, false, 8190, 0, 8190, 0}};

/* MPU6050 / MPU6500, yaw managed by the shared motion controller. */
bool mpu_ok = false;
float yaw_angle = 0.0f;
static const char *motion_error;
static bool i2c_recovery_pending;
static FrontClearance clearance;
static bool clearance_active, auto_straight;
static MotorBench motor_bench;
typedef struct
{
  uint32_t state, request, attempts, recoveries, last_error, fault_tick, last_attempt, ready_mask;
} I2CRecoveryDiagnostics;
volatile I2CRecoveryDiagnostics i2c_diag;
typedef struct
{
  uint32_t rx_count, last_rx, last_rx_tick, movement_accepted;
  uint32_t blocked_i2c, blocked_clearance, last_motion_result, last_motion_tick;
  uint8_t recent_rx[32]; /* Most recent non-newline command bytes, ring. */
} ControlDiagnostics;
volatile ControlDiagnostics control_diag;
typedef struct
{
  uint32_t state; /* 0=waiting, 1=valid, 2=invalid range, 3=stale, 4=I2C, 5=init */
  uint32_t samples, valid_samples, invalid_samples, valid_tick, i2c_errors, last_error;
} SensorDiagnostics;
volatile SensorDiagnostics tof_diag[5];

/* Cam bien Line IR */
uint8_t ir_left_val = 0;
uint8_t ir_right_val = 0;
uint8_t ir_black_level = 1; // 1 neu line den ra muc HIGH, 0 neu line den ra muc LOW

char msg[190];
bool auto_stream = true;
unsigned selected_tof = 1;
RobotMode_t current_mode = ROBOT_MODE_MANUAL;
RobotAction_t current_action = ACTION_STOP;
static uint8_t held_drive;
static uint32_t held_at;
static bool held_blocked;

int16_t base_speed = 420; // Toc do mac dinh (thang do 0 - 1000)
uint32_t state_timer = 0;
RobotAction_t avoid_direction = ACTION_STOP;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void I2C_Bus_Recovery(void);
void XSHUT_Set_All(GPIO_PinState state);
void Run_Full_Initialization(void);
void MPU6050_Calibrate(void);
void Update_MPU_Yaw(void);
void Read_Line_Sensors(void);
void Process_Auto_Navigation(void);
void Set_Robot_Action(RobotAction_t action);
const char *Get_Action_Name(RobotAction_t action);
void Print_Menu(void);
static void Request_I2C_Recovery(uint32_t error);
static void Service_I2C_Recovery(void);
static bool Service_Front_Clearance(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void I2C_Bus_Recovery(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_I2C2_CLK_ENABLE();
  __HAL_RCC_I2C2_FORCE_RESET();
  HAL_Delay(10);
  __HAL_RCC_I2C2_RELEASE_RESET();

  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
  HAL_Delay(5);

  for (int i = 0; i < 9; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(1);
  }

  /* Generate checked SCL/SDA transitions before returning to alternate
   * function; HAL_I2C_Init then resets the peripheral's analog-filter state. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
  HAL_Delay(5);
}

void XSHUT_Set_All(GPIO_PinState state)
{
  for (int i = 0; i < 5; i++)
  {
    HAL_GPIO_WritePin(cb[i].port, cb[i].pin, state);
  }
}

void Run_Full_Initialization(void)
{
  /* 1. Khoi tao 5 cam bien ToF VL53L0X */
  XSHUT_Set_All(GPIO_PIN_RESET);
  for (int i = 0; i < 5; i++)
  {
    cb[i].ok = false;
    tof_diag[i].state = 5;
  }
  HAL_Delay(80);

  uint8_t count_ok = 0;
  for (int i = 0; i < 5; i++)
  {
    HAL_GPIO_WritePin(cb[i].port, cb[i].pin, GPIO_PIN_SET);
    HAL_Delay(35);

    if (HAL_I2C_IsDeviceReady(&hi2c2, 0x52, 3, 10) != HAL_OK)
    {
      cb[i].ok = false;
      HAL_GPIO_WritePin(cb[i].port, cb[i].pin, GPIO_PIN_RESET);
      continue;
    }

    bool initialized = false;
    for (unsigned attempt = 0; attempt < 2 && !initialized; ++attempt)
    {
      if (attempt)
      {
        HAL_GPIO_WritePin(cb[i].port, cb[i].pin, GPIO_PIN_RESET);
        HAL_Delay(5);
        HAL_GPIO_WritePin(cb[i].port, cb[i].pin, GPIO_PIN_SET);
        HAL_Delay(35);
      }
      initialized = VL53L0X_Init(&dev[i], &hi2c2, 0x52);
    }
    if (!initialized)
    {
      cb[i].ok = false;
      HAL_GPIO_WritePin(cb[i].port, cb[i].pin, GPIO_PIN_RESET);
      continue;
    }

    if (!VL53L0X_SetAddress(&dev[i], cb[i].addr))
    {
      cb[i].ok = false;
      HAL_GPIO_WritePin(cb[i].port, cb[i].pin, GPIO_PIN_RESET);
      continue;
    }

    if (!VL53L0X_StartContinuous(&dev[i]))
    {
      cb[i].ok = false;
      HAL_GPIO_WritePin(cb[i].port, cb[i].pin, GPIO_PIN_RESET);
      continue;
    }
    dev[i].timeout = 20;
    cb[i].ok = true;
    tof_diag[i].state = 0;
    count_ok++;
  }

  snprintf(msg, sizeof(msg), "ToF: %u/5\r\n", count_ok);
  SoftUART_TransmitString(msg);

  /* Initialize the actual MPU6500 as well as MPU6050, then calibrate. */
  MPU6050_Calibrate();
}

/* Hieu chuan diem 0 (Offset) cho truc Gyro Z de tinh goc quay Yaw chinh xac */
void MPU6050_Calibrate(void)
{
  current_mode = ROBOT_MODE_MANUAL;
  Set_Robot_Action(ACTION_STOP);
  SoftUART_TransmitString("MPU: GIU XE DUNG YEN\r\n");
  mpu_ok = Motion_Init(&hi2c2);
  yaw_angle = Motion_GetAngle();
  motion_error = mpu_ok ? NULL : Motion_GetMPUState();
  uint8_t discard;
  while (SoftUART_ReadByte(&discard))
  {
  }
}

void Update_MPU_Yaw(void)
{
  if (i2c_recovery_pending)
    return;
  bool was_ready = mpu_ok;
  int16_t motion_speed = base_speed;
  if (Navigation_IsActive())
  {
    int16_t limit = Navigation_IsEmptyFloorTest() ? 400 : 360;
    if (Navigation_IsRetreating())
      limit = 350;
    if (motion_speed > limit)
      motion_speed = limit;
  }
  Motion_Update(motion_speed);
  mpu_ok = Motion_IsMPUReady();
  yaw_angle = Motion_GetAngle();
  if (!mpu_ok && motion_diag.state == 8 && current_action == ACTION_STOP)
    motion_error = Motion_GetMPUState();
  if (!was_ready && mpu_ok)
  {
    /* A newly calibrated zero invalidates old map/command references. */
    current_mode = ROBOT_MODE_MANUAL;
    Set_Robot_Action(ACTION_STOP);
    motion_error = NULL;
  }
  if (mpu_diag.stage == 5)
    Request_I2C_Recovery(mpu_diag.last_read_error);
}

static void Request_I2C_Recovery(uint32_t error)
{
  if (i2c_recovery_pending)
    return;
  i2c_recovery_pending = true;
  i2c_diag.state = 1;
  i2c_diag.last_error = error;
  i2c_diag.fault_tick = HAL_GetTick();
  i2c_diag.last_attempt = HAL_GetTick() - 2000u;
  current_mode = ROBOT_MODE_MANUAL;
  Set_Robot_Action(ACTION_STOP);
  motion_error = "I2C_PHUC_HOI";
  mpu_ok = false;
}

static void Service_I2C_Recovery(void)
{
  if (i2c_diag.request)
  {
    i2c_diag.request = 0;
    Request_I2C_Recovery(HAL_I2C_GetError(&hi2c2));
  }
  if (!i2c_recovery_pending || HAL_GetTick() - i2c_diag.last_attempt < 2000)
    return;
  i2c_diag.last_attempt = HAL_GetTick();
  i2c_diag.attempts++;
  Set_Robot_Action(ACTION_STOP);
  HAL_I2C_DeInit(&hi2c2);
  I2C_Bus_Recovery();
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) != GPIO_PIN_SET ||
      HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) != GPIO_PIN_SET)
  {
    /* Release any ToF slave holding SDA, then try the recovery clocks again. */
    XSHUT_Set_All(GPIO_PIN_RESET);
    HAL_Delay(5);
    I2C_Bus_Recovery();
  }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) != GPIO_PIN_SET ||
      HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) != GPIO_PIN_SET)
  {
    i2c_diag.state = 2;
    motion_error = "I2C_DAY_BI_GIU";
    return;
  }
  MX_I2C2_Init();
  for (unsigned i = 0; i < 5; ++i)
  {
    cb[i].dist = cb[i].raw_dist = 8190;
    cb[i].raw_tick = 0;
  }
  /* Keep offsets, reassign each ToF's address and recalibrate gyro while
   * stopped. Never restart the command interrupted by the bus fault. */
  Run_Full_Initialization();
  i2c_diag.ready_mask = 0;
  for (unsigned i = 0; i < 5; ++i)
    if (cb[i].ok)
      i2c_diag.ready_mask |= 1u << i;
  uint32_t error = HAL_I2C_GetError(&hi2c2);
  if (error & (HAL_I2C_ERROR_TIMEOUT | HAL_I2C_WRONG_START | HAL_I2C_ERROR_BERR))
  {
    i2c_diag.state = 3;
    i2c_diag.last_error = error;
    return;
  }
  /* Let responding devices stream even if another sensor is unplugged. */
  i2c_recovery_pending = false;
  i2c_diag.recoveries++;
  i2c_diag.state = mpu_ok && i2c_diag.ready_mask == 0x1F ? 0 : 3;
  motion_error = !mpu_ok ? Motion_GetMPUState() : i2c_diag.ready_mask == 0x1F ? NULL
                                                                              : "LOI_MOT_TOF";
}

static void Update_One_ToF(void)
{
  static unsigned next;
  static const uint8_t order[] = {1, 2, 3, 0, 1, 2, 3, 4};
  unsigned i = order[next];
  next = (next + 1) % sizeof(order);
  if (!cb[i].ok)
  {
    cb[i].dist = 8190;
    tof_diag[i].state = 5;
    return;
  }
  uint16_t d = VL53L0X_ReadRangeContinuousMillimeters(&dev[i]);
  if (dev[i].last_error)
  {
    tof_diag[i].state = 4;
    tof_diag[i].i2c_errors++;
    tof_diag[i].last_error = dev[i].last_error;
    Request_I2C_Recovery(dev[i].last_error);
    return;
  }
  if (d > 0)
  {
    tof_diag[i].samples++;
    if (d < 8000)
    {
      tof_diag[i].state = 1;
      tof_diag[i].valid_samples++;
      tof_diag[i].valid_tick = HAL_GetTick();
    }
    else
    {
      tof_diag[i].state = 2;
      tof_diag[i].invalid_samples++;
    }
    cb[i].raw_dist = d;
    cb[i].raw_tick = HAL_GetTick();
    cb[i].dist = i >= 1 && i <= 3 ? ToF_CorrectDistance(d, cb[i].offset_mm) : (d < 15 ? 15 : d);
  }
  else if (HAL_GetTick() - cb[i].raw_tick > 500)
  {
    cb[i].dist = 8190;
    tof_diag[i].state = 3;
  }
}

static void Format_Status(char *buffer, size_t size)
{
  char ranges[5][8];
  for (unsigned i = 0; i < 5; ++i)
  {
    if (!cb[i].ok || cb[i].dist >= 8000)
      strcpy(ranges[i], "---");
    else
      snprintf(ranges[i], sizeof(ranges[i]), "%u", cb[i].dist);
  }
  BluetoothStatus_Format(buffer, size, mpu_ok, yaw_angle, ranges,
                         Navigation_IsActive() ? Navigation_GetLabel() : Get_Action_Name(current_action),
                         motion_error, Encoder_GetCount1(), Motion_GetReportedEncoder2());
}

void Read_Line_Sensors(void)
{
  uint8_t raw_l = (HAL_GPIO_ReadPin(IR_LEFT_PORT, IR_LEFT_PIN) == GPIO_PIN_SET) ? 1 : 0;
  uint8_t raw_r = (HAL_GPIO_ReadPin(IR_RIGHT_PORT, IR_RIGHT_PIN) == GPIO_PIN_SET) ? 1 : 0;

  /* Chuan hoa ve 1 neu dung line den */
  ir_left_val = (raw_l == ir_black_level) ? 1 : 0;
  ir_right_val = (raw_r == ir_black_level) ? 1 : 0;
}

const char *Get_Action_Name(RobotAction_t action)
{
  switch (action)
  {
  case ACTION_STOP:
    return "DUNG";
  case ACTION_FORWARD:
    return "TIEN";
  case ACTION_BACKWARD:
    return "LUI";
  case ACTION_TURN_LEFT_90:
    return "RE-T90";
  case ACTION_TURN_RIGHT_90:
    return "RE-P90";
  case ACTION_CURVE_LEFT_45:
    return "CUA-T45";
  case ACTION_CURVE_RIGHT_45:
    return "CUA-P45";
  case ACTION_PIVOT_LEFT_10:
    return "XOAY-T10";
  case ACTION_PIVOT_RIGHT_10:
    return "XOAY-P10";
  case ACTION_FOLLOW_LINE:
    return "BAM-LINE";
  case ACTION_AVOID_LEFT_45:
    return "NE-T45";
  case ACTION_AVOID_RIGHT_45:
    return "NE-P45";
  case ACTION_RETURN_TO_LINE:
    return "VE-LINE";
  case ACTION_EMERGENCY_REVERSE:
    return "LUI-KHAN";
  case ACTION_MOTOR_BENCH:
    return "TEST-2BANH-500";
  default:
    return "---";
  }
}

void Set_Robot_Action(RobotAction_t action)
{
  held_drive = 0;
  if (current_mode != ROBOT_MODE_MANUAL ||
      (action != ACTION_FORWARD && action != ACTION_BACKWARD && action != ACTION_STOP))
    Motion_ResetManualHeading();
  MotorBench_Cancel(&motor_bench);
  /* Manual/legacy-auto commands take ownership and invalidate map pose. */
  Navigation_Cancel();
  if (action == ACTION_STOP)
  {
    FrontClearance_Stop(&clearance);
    clearance_active = false;
  }
  if (action == current_action && Motion_GetStatus() == MOTION_RUNNING)
    return;
  Motion_Cancel();
  auto_straight = false;
  current_action = action;
  /* All in-place 90-degree turns use gyro, including emergency auto turns. */
  if (action == ACTION_TURN_LEFT_90 || action == ACTION_TURN_RIGHT_90)
  {
    Motion_StartTurn(action == ACTION_TURN_LEFT_90 ? 90.0f : -90.0f, false);
    return;
  }
  if (current_mode == ROBOT_MODE_MANUAL)
  {
    switch (action)
    {
    case ACTION_FORWARD:
      Motion_StartManualStep(1, base_speed);
      return;
    case ACTION_BACKWARD:
      Motion_StartManualStep(-1, base_speed);
      return;
    case ACTION_CURVE_LEFT_45:
      Motion_StartTurn(45.0f, true);
      return;
    case ACTION_CURVE_RIGHT_45:
      Motion_StartTurn(-45.0f, true);
      return;
    case ACTION_PIVOT_LEFT_10:
      Motion_StartTurnHeading(Motion_GetAngle() + 10.0f);
      return;
    case ACTION_PIVOT_RIGHT_10:
      Motion_StartTurnHeading(Motion_GetAngle() - 10.0f);
      return;
    default:
      break;
    }
  }
  switch (action)
  {
  case ACTION_FORWARD:
    BoardDrive_Set(base_speed, base_speed);
    break;

  case ACTION_BACKWARD:
    BoardDrive_Set(-base_speed, -base_speed);
    break;

  case ACTION_CURVE_LEFT_45:
  case ACTION_AVOID_LEFT_45:
    /* Cua mem / Ne 45 do sang trai: Banh trai 35%, banh phai 100% */
    BoardDrive_Set((base_speed * 35) / 100, base_speed);
    break;

  case ACTION_CURVE_RIGHT_45:
  case ACTION_AVOID_RIGHT_45:
    /* Cua mem / Ne 45 do sang phai: Banh trai 100%, banh phai 35% */
    BoardDrive_Set(base_speed, (base_speed * 35) / 100);
    break;

  case ACTION_RETURN_TO_LINE:
    /* Bẻ lái 45 độ ngược lại để tìm lại vạch line đen */
    if (avoid_direction == ACTION_AVOID_RIGHT_45)
    {
      BoardDrive_Set((base_speed * 40) / 100, base_speed); // Lượn trái lại tìm line
    }
    else
    {
      BoardDrive_Set(base_speed, (base_speed * 40) / 100); // Lượn phải lại tìm line
    }
    state_timer = HAL_GetTick();
    break;

  case ACTION_EMERGENCY_REVERSE:
    BoardDrive_Set(-base_speed, -base_speed);
    state_timer = HAL_GetTick();
    break;

  case ACTION_FOLLOW_LINE:
  case ACTION_STOP:
  default:
    BoardDrive_Stop();
    break;
  }
}

static void HoldDrive(uint8_t command)
{
  if (held_drive != command)
  {
    Set_Robot_Action(ACTION_STOP);
    current_mode = ROBOT_MODE_MANUAL;
    held_drive = command;
    held_blocked = false;
    motion_error = NULL;
    Encoder_Reset();
    Motion_BeginStraight();
    current_action = command == 'W' ? ACTION_FORWARD : command == 'A' ? ACTION_TURN_LEFT_90
                                                                      : ACTION_TURN_RIGHT_90;
  }
  held_at = HAL_GetTick();
}

static void UpdateHeldDrive(void)
{
  if (!held_drive)
    return;
  uint32_t now = HAL_GetTick();
  if (now - held_at >= 600u)
  {
    Set_Robot_Action(ACTION_STOP);
    return;
  }
  if (held_blocked)
    return;
  bool safe = mpu_ok && !i2c_recovery_pending;
  for (unsigned i = 1; i <= 3 && safe; ++i)
  {
    Sensor_Info_t *sensor = &cb[i];
    safe = sensor->ok && now - sensor->raw_tick <= 200u &&
           (sensor->dist < 8000 ? sensor->dist >= (held_drive == 'W' ? 50 : 30) : (sensor->dist == 8190 || sensor->dist == 8191) && !dev[i].last_error);
  }
  if (safe && held_drive != 'W')
  {
    int16_t pwm = held_drive == 'A' ? 280 : -280;
    BoardDrive_Set(-pwm, pwm);
  }
  else if (!safe || !Motion_DriveStraight(1, base_speed))
  {
    BoardDrive_Stop();
    held_blocked = true;
    current_action = ACTION_STOP;
    motion_error = "GIU-5CM";
  }
}

/* Runs for manual and auto. Cancel the interrupted move; never resume its
 * old encoder target after retreat. User STOP always inhibits retreat until
 * the next movement command. Only the calibrated encoder 2 limits travel. */
static bool Service_Front_Clearance(void)
{
  ClearanceReading front[3];
  uint32_t now = HAL_GetTick();
  for (unsigned i = 0; i < 3; ++i)
  {
    Sensor_Info_t *s = &cb[i + 1];
    front[i] = (ClearanceReading){s->dist,
                                  s->ok && s->dist < 8000 && now - s->raw_tick <= 200};
  }
  if (current_mode == ROBOT_MODE_MANUAL && !Navigation_IsActive())
  {
    if (Motion_GetStatus() == MOTION_RUNNING && current_action != ACTION_BACKWARD)
    {
      uint16_t limit = current_action == ACTION_FORWARD ? 50 : 30;
      for (unsigned i = 0; i < 3; ++i)
      {
        if (front[i].valid && front[i].mm < limit)
        {
          Motion_ResetManualHeading();
          Set_Robot_Action(ACTION_STOP);
          motion_error = "GIU-5CM";
          return true;
        }
      }
    }
    return false;
  }
  /* Navigation owns obstacle response while autonomous. Let it keep the
   * surveyed route pose and execute its local sidestep; the emergency reverse
   * guard invalidates pose and switches to manual, preventing route rejoin. */
  if (Navigation_IsActive())
    return false;
  int16_t encoder = Encoder_GetCount2();
  ClearanceReading guard_front[3] = {front[0], front[1], front[2]};
  if (Navigation_UsesCenteredField())
  {
    /* On the centered route Navigation owns all ToF response. The generic
     * reverse guard must not invalidate its waypoint pose. */
    guard_front[0].valid = false;
    guard_front[2].valid = false;
  }
  ClearanceOutput out = FrontClearance_Update(&clearance, now, guard_front, encoder, encoder);
  if (!out.override && !clearance_active)
    return false;
  if (out.state == CLEARANCE_LIMIT_FAULT || out.state == CLEARANCE_SENSOR_TIMEOUT)
  {
    /* A completed retreat budget must not own the command interface forever.
     * Keep stopped and inhibit automatic retries; a NEW user command rearms
     * the guard, just as after an explicit STOP. Never auto-resume navigation. */
    current_mode = ROBOT_MODE_MANUAL;
    Set_Robot_Action(ACTION_STOP);
    motion_error = out.state == CLEARANCE_SENSOR_TIMEOUT ? "TOF_DAU_MAT_MAU" : "GIOI_HAN_LUI";
    return true;
  }
  if (!clearance_active)
  {
    Motion_ResetManualHeading();
    Navigation_InvalidatePose();
    Motion_Cancel();
    Motion_BeginStraight();
    auto_straight = false;
    clearance_active = true;
    current_action = ACTION_EMERGENCY_REVERSE;
  }
  if (out.state == CLEARANCE_DONE)
  {
    Set_Robot_Action(ACTION_STOP);
    FrontClearance_Rearm(&clearance);
    motion_error = NULL;
    return true;
  }
  if (out.reverse_pwm)
  {
    if (!Motion_DriveStraight(-1, out.reverse_pwm))
    {
      current_mode = ROBOT_MODE_MANUAL;
      Set_Robot_Action(ACTION_STOP);
      motion_error = Motion_GetMPUState();
      return true;
    }
  }
  else
    BoardDrive_Stop();
  motion_error = out.state == CLEARANCE_SENSOR_FAULT ? "LOI_TOF_DAU" : "GIU-5CM";
  return true;
}

/* THUAT TOAN TU DONG: KET HOP 3 ToF TRUOC + 2 ToF SAU + 2 IR LINE + MPU */
void Process_Auto_Navigation(void)
{
  uint16_t d_sl = cb[0].ok ? cb[0].dist : 9999;
  uint16_t d_tl = cb[1].ok ? cb[1].dist : 9999;
  uint16_t d_tc = cb[2].ok ? cb[2].dist : 9999;
  uint16_t d_tr = cb[3].ok ? cb[3].dist : 9999;
  uint16_t d_sr = cb[4].ok ? cb[4].dist : 9999;

  if (current_action == ACTION_TURN_LEFT_90 || current_action == ACTION_TURN_RIGHT_90)
  {
    MotionStatus result = Motion_GetStatus();
    if (result == MOTION_RUNNING)
      return;
    if (result == MOTION_DONE)
      Set_Robot_Action(ACTION_FORWARD);
    else
    {
      current_mode = ROBOT_MODE_MANUAL;
      Set_Robot_Action(ACTION_STOP);
      motion_error = result == MOTION_TIMEOUT ? "QUA_THOI_GIAN" : result == MOTION_ANGLE_ERROR ? "GOC_LECH"
                                                                                               : Motion_GetMPUState();
    }
    return;
  }

  /* 3. Xu ly khi dang VE LAI LINE DEN sau khi vuot qua vat can */
  if (current_action == ACTION_RETURN_TO_LINE)
  {
    // Neu mot trong hai mat IR da bat lai duoc line den:
    if (ir_left_val == 1 || ir_right_val == 1)
    {
      Set_Robot_Action(ACTION_FORWARD);
      avoid_direction = ACTION_STOP;
      return;
    }
    // Neu qua 800ms ma chua cham line: tra ve tien thang de tranh xoay tron
    if (HAL_GetTick() - state_timer >= 800)
    {
      Set_Robot_Action(ACTION_FORWARD);
      avoid_direction = ACTION_STOP;
      return;
    }
    return;
  }

  /* 4. DANG NE SANG PHAI 45 DO: Theo doi ToF Sau-Trai (S-L) */
  if (current_action == ACTION_AVOID_RIGHT_45)
  {
    /* ToF truoc da thoang va ToF Sau-Trai (S-L) > 260mm:
       => Than va duoi xe da vuot qua hoan toan vat can ben trai! */
    if (d_tl > DIST_TRIGGER && d_sl > DIST_CLEARED)
    {
      avoid_direction = ACTION_AVOID_RIGHT_45;
      Set_Robot_Action(ACTION_RETURN_TO_LINE);
      return;
    }
    return;
  }

  /* 5. DANG NE SANG TRAI 45 DO: Theo doi ToF Sau-Phai (S-R) */
  if (current_action == ACTION_AVOID_LEFT_45)
  {
    /* ToF Sau-Phai (S-R) > 260mm:
       => Da vuot qua hoan toan vat can ben phai! */
    if (d_tr > DIST_TRIGGER && d_sr > DIST_CLEARED)
    {
      avoid_direction = ACTION_AVOID_LEFT_45;
      Set_Robot_Action(ACTION_RETURN_TO_LINE);
      return;
    }
    return;
  }

  /* 6. KIEM TRA 3 TOF PHIA TRUOC (T-L, T-C, T-R) DE KICH HOAT NE */
  bool obst_left = (d_tl < DIST_TRIGGER);
  bool obst_right = (d_tr < DIST_TRIGGER);
  bool obst_center = (d_tc < DIST_TRIGGER);

  if (obst_center || (obst_left && obst_right))
  {
    if (d_tl > d_tr)
    {
      Set_Robot_Action(ACTION_AVOID_LEFT_45);
    }
    else
    {
      Set_Robot_Action(ACTION_AVOID_RIGHT_45);
    }
    return;
  }
  else if (obst_left)
  {
    Set_Robot_Action(ACTION_AVOID_RIGHT_45);
    return;
  }
  else if (obst_right)
  {
    Set_Robot_Action(ACTION_AVOID_LEFT_45);
    return;
  }

  /* 7. KHI KHONG CO VAT CAN PHIA TRUOC: BAM THEO LINE DEN BANG 2 MAT IR */
  if (current_action != ACTION_FOLLOW_LINE)
    auto_straight = false;
  current_action = ACTION_FOLLOW_LINE;
  if (ir_left_val == 1 && ir_right_val == 0)
  {
    /* Xe lech sang phai vach line -> Banh phai quay nhanh hon, banh trai giam de chinh sang trai */
    auto_straight = false;
    BoardDrive_Set((base_speed * 40) / 100, base_speed);
  }
  else if (ir_left_val == 0 && ir_right_val == 1)
  {
    /* Xe lech sang trai vach line -> Banh trai quay nhanh hon, banh phai giam de chinh sang phai */
    auto_straight = false;
    BoardDrive_Set(base_speed, (base_speed * 40) / 100);
  }
  else
  {
    /* Capture a new heading after IR has finished steering. */
    if (!auto_straight)
      auto_straight = Motion_BeginStraight();
    if (!Motion_DriveStraight(1, base_speed))
    {
      current_mode = ROBOT_MODE_MANUAL;
      Set_Robot_Action(ACTION_STOP);
      motion_error = Motion_GetMPUState();
      return;
    }
  }
}

void Print_Menu(void)
{
  SoftUART_TransmitString(
      "\r\nw/s tien/lui, a/d 90, q/e cua 45, u/p 10, x dung.\r\n"
      "b test (nhac 2 banh), v xuat phat, g san U, j san trong, +/- toc do.\r\n"
      "i/o dao motor, m bam line, c yaw, 1/2/3 ToF, z can, [] TR, {} TL, k IR, t log.\r\n");
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  SystemCoreClockUpdate();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* Giai phong bus I2C2 truoc khi khoi dong ngoai vi */
  I2C_Bus_Recovery();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */

  /* 1. Khoi tao Dong co TB6612 va Encoder */
  Motor_Init();
  Encoder_Init();
  BoardDrive_Stop();

  /* 2. Khoi tao Software UART (co ngat EXTI11 tren PA11) */
  SoftUART_Init();

  HAL_Delay(100);

#ifdef TEACH_DRIVE
  XSHUT_Set_All(GPIO_PIN_RESET);
  TeachDrive_Run(&hi2c2);
#endif
  /* Build occupancy before gyro calibration; map construction must not
   * create a long sampling gap after a valid yaw reference is established. */
  Navigation_Init(&navigation_config);
  /* 3. Khoi tao 5x ToF va MPU-6050 */
  Run_Full_Initialization();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_print = HAL_GetTick(), last_tof = 0;
  char tx_line[BLUETOOTH_STATUS_SIZE] = {0};
  size_t tx_pos = 0;

  FrontClearance_Rearm(&clearance);
  while (1)
  {
    Service_I2C_Recovery();
    if (MotorBench_Update(&motor_bench, HAL_GetTick()))
    {
      Set_Robot_Action(ACTION_STOP);
      motion_error = "TEST-MOTOR-XONG";
    }
    Update_MPU_Yaw();
    uint8_t rx_char;
    if (SoftUART_ReadByte(&rx_char) && rx_char && rx_char != '\r' && rx_char != '\n')
    {
      control_diag.recent_rx[control_diag.rx_count % 32] = rx_char;
      control_diag.rx_count++;
      control_diag.last_rx = rx_char;
      control_diag.last_rx_tick = HAL_GetTick();
      if (rx_char == 'W' || rx_char == 'A' || rx_char == 'D')
      {
        HoldDrive(rx_char);
        continue;
      }
      RobotAction_t requested = ACTION_STOP;
      bool movement = true;
      switch (rx_char)
      {
      case 'w':
      case 'F':
        requested = ACTION_FORWARD;
        break;
      case 's':
      case 'B':
        requested = ACTION_BACKWARD;
        break;
      case 'a':
      case 'L':
        requested = ACTION_TURN_LEFT_90;
        break;
      case 'd':
      case 'R':
        requested = ACTION_TURN_RIGHT_90;
        break;
      case 'q':
      case 'Q':
        requested = ACTION_CURVE_LEFT_45;
        break;
      case 'e':
      case 'E':
        requested = ACTION_CURVE_RIGHT_45;
        break;
      case 'u':
      case 'U':
        requested = ACTION_PIVOT_LEFT_10;
        break;
      case 'p':
      case 'P':
        requested = ACTION_PIVOT_RIGHT_10;
        break;
      case 'x':
      case 'X':
      case ' ':
      case 'S':
        break;
      default:
        movement = false;
        break;
      }
      if (movement)
      {
        if (requested == ACTION_STOP)
          Motion_ResetManualHeading();
        if (i2c_recovery_pending && requested != ACTION_STOP)
        {
          control_diag.blocked_i2c++;
          continue;
        }
        if (clearance_active && requested != ACTION_STOP)
        {
          control_diag.blocked_clearance++;
          continue;
        }
        control_diag.movement_accepted++;
        if (requested != ACTION_STOP && clearance.inhibited)
          FrontClearance_Rearm(&clearance);
        current_mode = ROBOT_MODE_MANUAL;
        motion_error = NULL;
        Set_Robot_Action(requested);
      }
      else if (rx_char == 'b')
      {
        if (i2c_recovery_pending)
          continue;
        current_mode = ROBOT_MODE_MANUAL;
        Set_Robot_Action(ACTION_STOP);
        Motion_ResetManualHeading();
        current_action = ACTION_MOTOR_BENCH;
        motion_error = NULL;
        MotorBench_Start(&motor_bench, HAL_GetTick());
      }
      else if (rx_char == '+' || rx_char == '=')
      {
        base_speed += 50;
        if (base_speed > 950)
          base_speed = 950;
      }
      else if (rx_char == '-' || rx_char == '_')
      {
        base_speed -= 50;
        if (base_speed < 200)
          base_speed = 200;
      }
      else if (rx_char == 't' || rx_char == 'T')
      {
        auto_stream = !auto_stream;
      }
      else if (rx_char == '1' || rx_char == '2' || rx_char == '3')
      {
        selected_tof = rx_char - '0';
      }
      else if (rx_char == 'k' || rx_char == 'K')
      {
        ir_black_level = !ir_black_level;
      }
      else if (rx_char == 'v' || rx_char == 'V')
      {
        if (i2c_recovery_pending)
          continue;
        /* Explicit start confirmation cancels a stopped clearance fault. */
        Motion_ResetManualHeading();
        current_mode = ROBOT_MODE_MANUAL;
        Set_Robot_Action(ACTION_STOP);
        Navigation_ConfirmStart();
        motion_error = Navigation_GetLabel();
      }
      else if (rx_char == 'g' || rx_char == 'G' || rx_char == 'j' || rx_char == 'J')
      {
        if (i2c_recovery_pending)
          continue;
        if (clearance_active)
        {
          motion_error = "MAP_GUI_V_LAI";
          continue;
        }
        Motion_ResetManualHeading();
        if (motor_bench.active)
          Set_Robot_Action(ACTION_STOP);
        current_mode = ROBOT_MODE_MANUAL;
        current_action = ACTION_STOP;
        motion_error = NULL;
        bool floor_test = rx_char == 'j' || rx_char == 'J';
        /* Route navigation handles close returns without losing pose to retreat. */
        if (floor_test || navigation_config.route_u)
          FrontClearance_Stop(&clearance);
        else
          FrontClearance_Rearm(&clearance);
        bool started = floor_test ? Navigation_StartEmptyFloorTest() : Navigation_Start();
        if (!started)
        {
          Motion_Cancel();
          motion_error = Navigation_GetLabel();
        }
      }
      else if (rx_char == 'm' || rx_char == 'M')
      {
        if (i2c_recovery_pending)
          continue;
        Set_Robot_Action(ACTION_STOP);
        current_mode = current_mode == ROBOT_MODE_MANUAL ? ROBOT_MODE_AUTO : ROBOT_MODE_MANUAL;
        if (current_mode == ROBOT_MODE_AUTO)
        {
          FrontClearance_Rearm(&clearance);
          Set_Robot_Action(ACTION_FORWARD);
        }
      }
      else if (rx_char == 'c' || rx_char == 'C')
      {
        Request_I2C_Recovery(HAL_I2C_GetError(&hi2c2));
      }
      else if (rx_char == 'i' || rx_char == 'I' || rx_char == 'o' || rx_char == 'O')
      {
        current_mode = ROBOT_MODE_MANUAL;
        Set_Robot_Action(ACTION_STOP);
        if (rx_char == 'i' || rx_char == 'I')
          Motor_ToggleInvertA();
        else
          Motor_ToggleInvertB();
      }
      else if (rx_char == 'z' || rx_char == 'Z')
      {
        current_mode = ROBOT_MODE_MANUAL;
        Set_Robot_Action(ACTION_STOP);
        unsigned k = selected_tof;
        if (cb[k].ok && cb[k].raw_dist < 400 && HAL_GetTick() - cb[k].raw_tick <= 500)
        {
          cb[k].offset_mm = ToF_ClampOffset(cb[k].raw_dist);
          cb[k].dist = ToF_CorrectDistance(cb[k].raw_dist, cb[k].offset_mm);
          motion_error = NULL;
        }
        else
          motion_error = "LOI_ZERO_TOF";
      }
      else if (rx_char == '[' || rx_char == ']' || rx_char == '{' || rx_char == '}')
      {
        unsigned k = rx_char == '[' || rx_char == ']' ? 3 : 1;
        int delta = rx_char == '[' || rx_char == '{' ? 5 : -5;
        cb[k].offset_mm = ToF_ClampOffset((int32_t)cb[k].offset_mm + delta);
      }
      else if (rx_char == 'h' || rx_char == 'H' || rx_char == '?')
      {
        current_mode = ROBOT_MODE_MANUAL;
        Set_Robot_Action(ACTION_STOP);
        Print_Menu();
      }
    }

    Update_MPU_Yaw();
    UpdateHeldDrive();
    if (current_mode == ROBOT_MODE_MANUAL && !Navigation_IsActive())
    {
      MotionStatus result = Motion_GetStatus();
      if (result != MOTION_IDLE && result != MOTION_RUNNING)
      {
        if (result != MOTION_DONE)
          Motion_ResetManualHeading();
        control_diag.last_motion_result = result;
        control_diag.last_motion_tick = HAL_GetTick();
        bool was_armed = !clearance.inhibited;
        Set_Robot_Action(ACTION_STOP);
        if (result == MOTION_DONE && was_armed)
          FrontClearance_Rearm(&clearance);
        motion_error = result == MOTION_DONE ? NULL : result == MOTION_ENCODER_ERROR ? "LOI_ENCODER2"
                                                  : result == MOTION_TIMEOUT         ? "QUA_THOI_GIAN"
                                                  : result == MOTION_ANGLE_ERROR     ? "GOC_LECH"
                                                                                     : Motion_GetMPUState();
      }
    }
    if (!i2c_recovery_pending && HAL_GetTick() - last_tof >= TOF_POLL_INTERVAL_MS)
    {
      last_tof = HAL_GetTick();
      Update_One_ToF();
      Update_MPU_Yaw();
      if (!i2c_recovery_pending && !Service_Front_Clearance())
      {
        if (Navigation_IsActive())
        {
          NavigationRange front[3];
          for (unsigned i = 0; i < 3; ++i)
          {
            Sensor_Info_t *s = &cb[i + 1];
            front[i] = (NavigationRange){.mm = s->dist, .valid = s->ok && s->dist < 8000, .tick = s->raw_tick, .no_echo = s->ok && (s->dist == 8190 || s->dist == 8191) && dev[i + 1].last_error == 0};
          }
          Navigation_Update(front, base_speed);
          if (!Navigation_IsActive())
          {
            current_action = ACTION_STOP;
            motion_error = Navigation_GetLabel();
          }
        }
        else if (current_mode == ROBOT_MODE_AUTO)
        {
          Read_Line_Sensors();
          Process_Auto_Navigation();
        }
      }
    }
    if (!tx_line[tx_pos] && auto_stream && HAL_GetTick() - last_print >= BLUETOOTH_STATUS_INTERVAL_MS)
    {
      last_print = HAL_GetTick();
      Format_Status(tx_line, sizeof(tx_line));
      tx_pos = 0;
    }
    if (tx_line[tx_pos])
    {
      SoftUART_TransmitByte((uint8_t)tx_line[tx_pos++]);
      Update_MPU_Yaw();
    }
    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C2 Initialization Function (100kHz Standard Mode)
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* 1. LED kit BluePill (PC13) */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* 2. LED Do (PB8), LED Xanh (PB9) */
  HAL_GPIO_WritePin(GPIOB, LED_RED_PIN | LED_GREEN_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LED_RED_PIN | LED_GREEN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 3. 5 Chân XSHUT của VL53L0X: PB0, PB12, PB1, PB13, PB14 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_12 | GPIO_PIN_1 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_12 | GPIO_PIN_1 | GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 4. 2 Chan cam bien do Line IR: PA4 (IR Left), PA5 (IR Right) */
  GPIO_InitStruct.Pin = IR_LEFT_PIN | IR_RIGHT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
