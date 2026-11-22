/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CHUONG TRINH DIEU KHIEN ROBOT TOAN DIEN
  *                   - Da sua chieu quay: Tien chay Tien, Lui chay Lui
  *                   - Tich hop: Tien, Lui, Re trai 90, Re phai 90, Cua/Re 45 do
  *                   - Ngat EXTI11 tren RX: Nhan lenh 100% tuc thi chi sau 1 lan bam
  *                   - Tu dong tranh vat can: Ne mem 45 do bang ToF truoc,
  *                     dung ToF hong trai/phai de xac nhan da vuot qua vat can
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "software_uart.h"
#include "vl53l0x.h"
#include "motor.h"
#include "motion.h"
#include "front_clearance.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  const char    *name;
  GPIO_TypeDef  *port;
  uint16_t      pin;
  const char    *pin_name;
  uint8_t       addr;
  bool          ok;
  uint16_t      dist;
} Sensor_Info_t;

typedef enum {
  TOF_SIDE_LEFT = 0,
  TOF_DIAGONAL_LEFT,
  TOF_CENTER,
  TOF_DIAGONAL_RIGHT,
  TOF_SIDE_RIGHT
} ToF_Position_t;

typedef enum {
  ROBOT_MODE_MANUAL = 0,
  ROBOT_MODE_AUTO   = 1
} RobotMode_t;

typedef enum {
  ACTION_STOP = 0,
  ACTION_FORWARD,
  ACTION_BACKWARD,
  ACTION_TURN_LEFT_90,
  ACTION_TURN_RIGHT_90,
  ACTION_CURVE_LEFT_45,
  ACTION_CURVE_RIGHT_45,
  ACTION_PIVOT_LEFT_45,
  ACTION_PIVOT_RIGHT_45,
  ACTION_AVOID_LEFT_45,
  ACTION_AVOID_RIGHT_45,
  ACTION_RETURN_ORIGIN,
  ACTION_CLEARANCE_REVERSE,
  ACTION_MOTOR_TEST
} RobotAction_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_RED_PIN       GPIO_PIN_8
#define LED_RED_PORT      GPIOB
#define LED_GREEN_PIN     GPIO_PIN_9
#define LED_GREEN_PORT    GPIOB

/* Nguong khoang cach tranh vat can ToF (mm) */
#define DIST_TRIGGER     240   // Phat hien vat can (<24cm) -> bat dau re 45 do de ne
#define DIST_CLEARED     260   // ToF hong xac nhan da qua khoi vat can (>26cm)

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
VL53L0X_Dev dev[5];

Sensor_Info_t cb[5] = {
  [TOF_SIDE_LEFT]      = {"Hong-Trai",  GPIOB, GPIO_PIN_0,  "PB0",  0x30, false, 8190},
  [TOF_DIAGONAL_LEFT]  = {"Cheo-Trai",  GPIOB, GPIO_PIN_14, "PB14", 0x38, false, 8190},
  [TOF_CENTER]         = {"Giua",       GPIOB, GPIO_PIN_13, "PB13", 0x36, false, 8190},
  /* PB12 confirmed at front-right diagonal: 61-63 mm after sensor swap/reset. */
  [TOF_DIAGONAL_RIGHT] = {"Cheo-Phai",  GPIOB, GPIO_PIN_12, "PB12", 0x32, false, 8190},
  [TOF_SIDE_RIGHT]     = {"Hong-Phai",  GPIOB, GPIO_PIN_1,  "PB1",  0x34, false, 8190}
};

char msg[180];
bool auto_stream = true;
RobotMode_t current_mode = ROBOT_MODE_MANUAL;
RobotAction_t current_action = ACTION_STOP;

int16_t base_speed = 420; // Toc do mac dinh (thang do 0 - 1000)
uint32_t state_timer = 0;
RobotAction_t last_avoid_side = ACTION_STOP;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void I2C_Bus_Recovery(void);
void XSHUT_Set_All(GPIO_PinState state);
void Run_Full_Initialization(void);
void Process_Auto_Navigation(void);
void Set_Robot_Action(RobotAction_t action);
const char* Get_Action_Name(RobotAction_t action);

static void Init_MPU(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static const char *motion_error = NULL;
static uint32_t last_mpu_attempt;
static FrontClearance clearance;
static bool clearance_owned;
static uint16_t tof_guard_mm[5];
static uint32_t tof_good_tick[5];
static bool tof_has_valid[5];
static bool avoid_saw_side;
static uint32_t side_clear_since;
volatile uint32_t command_history[16];
volatile uint32_t command_count;

/* Bounded bench diagnostic, requested only through debugger RAM:
 * 1=A forward, 2=A reverse, 3=B forward, 4=B reverse.
 * Always stops itself after 250 ms; leaves clearance inhibited afterward. */
typedef struct {
  uint32_t request, state, channel, started;
  int32_t delta_a, delta_b;
} MotorTestDiagnostics;
volatile MotorTestDiagnostics motor_test;
static int16_t motor_test_start_a, motor_test_start_b;

static void Motor_Test_Finish(void)
{
  Motor_Stop();
  motor_test.delta_a = (int16_t)((uint16_t)Encoder_GetCount1() - (uint16_t)motor_test_start_a);
  motor_test.delta_b = (int16_t)((uint16_t)Encoder_GetCount2() - (uint16_t)motor_test_start_b);
  motor_test.state = 2;
  current_action = ACTION_STOP;
}

static void Motor_Test_Update(void)
{
  if (motor_test.state == 1) {
    if (HAL_GetTick() - motor_test.started >= 250) Motor_Test_Finish();
    return;
  }
  uint32_t request = motor_test.request;
  if (request < 1 || request > 4) return;
  motor_test.request = 0;
  FrontClearance_Stop(&clearance);
  clearance_owned = false;
  current_mode = ROBOT_MODE_MANUAL;
  Set_Robot_Action(ACTION_STOP);
  motion_error = NULL;
  motor_test.channel = request;
  motor_test_start_a = Encoder_GetCount1();
  motor_test_start_b = Encoder_GetCount2();
  motor_test.delta_a = motor_test.delta_b = 0;
  motor_test.started = HAL_GetTick();
  motor_test.state = 1;
  current_action = ACTION_MOTOR_TEST;
  int16_t pwm = request == 2 || request == 4 ? -600 : 600;
  Motor_SetSpeed(request <= 2 ? pwm : 0, request >= 3 ? pwm : 0);
}

static void Init_MPU(void)
{
  current_mode = ROBOT_MODE_MANUAL;
  Set_Robot_Action(ACTION_STOP);
  motion_error = Motion_Init(&hi2c2) ? NULL : Motion_GetMPUState();
  last_mpu_attempt = HAL_GetTick();
  uint8_t discarded;
  while (SoftUART_ReadByte(&discarded)) {}
}

/* One sensor per slot keeps gyro sampling and commands responsive. */
static void Update_One_ToF(void)
{
  static unsigned index;
  static const uint8_t order[] = {
    TOF_DIAGONAL_LEFT, TOF_CENTER, TOF_DIAGONAL_RIGHT, TOF_SIDE_LEFT,
    TOF_DIAGONAL_LEFT, TOF_CENTER, TOF_DIAGONAL_RIGHT, TOF_SIDE_RIGHT
  };
  static uint32_t updated[5];
  unsigned i = order[index];
  index = (index + 1) % sizeof(order);
  if (!cb[i].ok) { cb[i].dist = 8190; return; }
  uint16_t d;
  if (VL53L0X_ReadMeasurement(&dev[i], &d)) {
    cb[i].dist = d;
    updated[i] = HAL_GetTick();
    if (d < 8000) {
      tof_guard_mm[i] = d;
      tof_good_tick[i] = HAL_GetTick();
      tof_has_valid[i] = true;
    }
  }
  else if (HAL_GetTick() - updated[i] > 500) cb[i].dist = 8190;
}

static bool ToF_ValidRecent(unsigned i)
{
  return cb[i].ok && tof_has_valid[i] && HAL_GetTick() - tof_good_tick[i] <= 150;
}

static void Process_Front_Clearance(void)
{
  const unsigned positions[] = {TOF_DIAGONAL_LEFT, TOF_CENTER, TOF_DIAGONAL_RIGHT};
  ClearanceReading front[3];
  for (unsigned n = 0; n < 3; ++n) {
    unsigned i = positions[n];
    front[n] = (ClearanceReading){.mm = tof_guard_mm[i], .valid = ToF_ValidRecent(i)};
  }
  ClearanceOutput out = FrontClearance_Update(&clearance, HAL_GetTick(), front,
      Encoder_GetCount1(), Encoder_GetCount2());
  if (out.override) {
    if (!clearance_owned) {
      Set_Robot_Action(ACTION_STOP);
      current_mode = ROBOT_MODE_MANUAL;
      clearance_owned = true;
    }
    if (out.state == CLEARANCE_SENSOR_FAULT || out.state == CLEARANCE_LIMIT_FAULT) {
      current_action = ACTION_STOP;
      motion_error = out.state == CLEARANCE_SENSOR_FAULT ? "LOI_TOF_DAU" : "GIOI_HAN_LUI";
    } else {
      current_action = ACTION_CLEARANCE_REVERSE;
      motion_error = NULL;
    }
    Motor_SetSpeed(-out.reverse_pwm, -out.reverse_pwm);
  } else if (clearance_owned) {
    clearance_owned = false;
    Set_Robot_Action(ACTION_STOP);
    if (out.state == CLEARANCE_DONE) motion_error = NULL;
  }
}

static bool Passed_Side(unsigned i, uint16_t front_mm)
{
  if (!ToF_ValidRecent(i)) { side_clear_since = 0; return false; }
  uint16_t side_mm = tof_guard_mm[i];
  if (side_mm < DIST_TRIGGER) avoid_saw_side = true;
  if (avoid_saw_side && side_mm > DIST_CLEARED && front_mm > DIST_TRIGGER) {
    if (!side_clear_since) side_clear_since = HAL_GetTick();
    return HAL_GetTick() - side_clear_since >= 120;
  }
  side_clear_since = 0;
  return false;
}

static void Format_Status(char *buffer, size_t size)
{
  char ranges[5][8];
  for (unsigned i = 0; i < 5; ++i) {
    if (!cb[i].ok || cb[i].dist >= 8000) strcpy(ranges[i], "---");
    else snprintf(ranges[i], sizeof(ranges[i]), "%u", cb[i].dist);
  }
  int32_t tenth = (int32_t)(Motion_GetAngle() * 10.0f);
  uint32_t magnitude = tenth < 0 ? (uint32_t)(-tenth) : (uint32_t)tenth;
  char angle_text[24];
  if (motion_diag.state == 4)
    snprintf(angle_text, sizeof(angle_text), "%s%lu.%lu",
      tenth < 0 ? "-" : "", (unsigned long)(magnitude / 10),
      (unsigned long)(magnitude % 10));
  else strcpy(angle_text, "---");
  snprintf(buffer, size, "Goc:%s do | ToF:%s,%s,%s,%s,%s mm | Lenh:%s%s%s\r\n",
    angle_text, ranges[0], ranges[1], ranges[2], ranges[3], ranges[4],
    Get_Action_Name(current_action), motion_error ? ":" : "",
    motion_error ? motion_error : "");
}

void I2C_Bus_Recovery(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

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

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
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

  XSHUT_Set_All(GPIO_PIN_RESET);
  for (int i = 0; i < 5; i++) cb[i].ok = false;
  HAL_Delay(80);


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

    if (!VL53L0X_Init(&dev[i], &hi2c2, 0x52))
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

    if (!VL53L0X_StartContinuous(&dev[i])) {
      cb[i].ok = false;
      HAL_GPIO_WritePin(cb[i].port, cb[i].pin, GPIO_PIN_RESET);
      continue;
    }
    dev[i].timeout = 5;
    cb[i].ok = true;
  }

}

const char* Get_Action_Name(RobotAction_t action)
{
  switch (action)
  {
    case ACTION_STOP:              return "DUNG";
    case ACTION_FORWARD:           return "TIEN";
    case ACTION_BACKWARD:          return "LUI";
    case ACTION_TURN_LEFT_90:      return "RE-T90";
    case ACTION_TURN_RIGHT_90:     return "RE-P90";
    case ACTION_CURVE_LEFT_45:     return "CUA-T45";
    case ACTION_CURVE_RIGHT_45:    return "CUA-P45";
    case ACTION_PIVOT_LEFT_45:     return "XOAY-T45";
    case ACTION_PIVOT_RIGHT_45:    return "XOAY-P45";
    case ACTION_AVOID_LEFT_45:     return "NE-T45";
    case ACTION_AVOID_RIGHT_45:    return "NE-P45";
    case ACTION_RETURN_ORIGIN:     return "TRA-LAI";
    case ACTION_CLEARANCE_REVERSE: return "LUI-GIU-KC";
    case ACTION_MOTOR_TEST: return "THU-MOTOR";
    default:                       return "---";
  }
}

void Set_Robot_Action(RobotAction_t action)
{
  if (action != current_action &&
      (action == ACTION_AVOID_LEFT_45 || action == ACTION_AVOID_RIGHT_45)) {
    avoid_saw_side = false;
    side_clear_since = 0;
  }
  /* Repeated packets must not reset distance/angle while a step is running. */
  if (current_mode == ROBOT_MODE_MANUAL && action == current_action &&
      Motion_GetStatus() == MOTION_RUNNING) return;
  Motion_Cancel();
  current_action = action;
  if (current_mode == ROBOT_MODE_MANUAL && action != ACTION_STOP)
  {
    switch (action)
    {
      case ACTION_FORWARD: Motion_StartStep(1); break;
      case ACTION_BACKWARD: Motion_StartStep(-1); break;
      case ACTION_TURN_LEFT_90: Motion_StartTurn(90.0f, false); break;
      case ACTION_TURN_RIGHT_90: Motion_StartTurn(-90.0f, false); break;
      case ACTION_PIVOT_LEFT_45: Motion_StartTurn(45.0f, false); break;
      case ACTION_PIVOT_RIGHT_45: Motion_StartTurn(-45.0f, false); break;
      case ACTION_CURVE_LEFT_45: Motion_StartTurn(45.0f, true); break;
      case ACTION_CURVE_RIGHT_45: Motion_StartTurn(-45.0f, true); break;
      default: break;
    }
    /* Manual distance/gyro controller owns motor output until completion. */
    return;
  }
  switch (action)
  {
    case ACTION_FORWARD:
      /* 2 banh quay tien dong bo */
      Motor_SetSpeed(base_speed, base_speed);
      HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
      break;

    case ACTION_BACKWARD:
      /* 2 banh quay lui dong bo */
      Motor_SetSpeed(-base_speed, -base_speed);
      HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
      break;

    case ACTION_TURN_LEFT_90:
      /* Quay tai cho 90 do sang trai: banh trai lui, banh phai tien */
      Motor_SetSpeed(-base_speed, base_speed);
      HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
      break;

    case ACTION_TURN_RIGHT_90:
      /* Quay tai cho 90 do sang phai: banh trai tien, banh phai lui */
      Motor_SetSpeed(base_speed, -base_speed);
      HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
      break;

    case ACTION_CURVE_LEFT_45:
    case ACTION_AVOID_LEFT_45:
      /* Cua mem / Ne 45 do sang trai: Banh trai 35%, banh phai 100% */
      Motor_SetSpeed((base_speed * 35) / 100, base_speed);
      HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
      break;

    case ACTION_CURVE_RIGHT_45:
    case ACTION_AVOID_RIGHT_45:
      /* Cua mem / Ne 45 do sang phai: Banh trai 100%, banh phai 35% */
      Motor_SetSpeed(base_speed, (base_speed * 35) / 100);
      HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
      break;

    case ACTION_PIVOT_LEFT_45:
      /* Xoay tai cho 45 do sang trai */
      Motor_SetSpeed(-base_speed, base_speed);
      state_timer = HAL_GetTick();
      break;

    case ACTION_PIVOT_RIGHT_45:
      /* Xoay tai cho 45 do sang phai */
      Motor_SetSpeed(base_speed, -base_speed);
      state_timer = HAL_GetTick();
      break;

    case ACTION_RETURN_ORIGIN:
      /* Tra lai huong thang sau khi vuot qua vat can */
      if (last_avoid_side == ACTION_AVOID_RIGHT_45)
      {
        Motor_SetSpeed((base_speed * 40) / 100, base_speed);
      }
      else
      {
        Motor_SetSpeed(base_speed, (base_speed * 40) / 100);
      }
      state_timer = HAL_GetTick();
      break;

    case ACTION_CLEARANCE_REVERSE:
      /* Lui khan cap khi qua sat vat can */
      Motor_SetSpeed(-base_speed, -base_speed);
      HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
      state_timer = HAL_GetTick();
      break;

    case ACTION_STOP:
    default:
      Motor_Stop();
      HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
      break;
  }
}

/* THUAT TOAN TU DONG NE VAT CAN THEO TOF DAU VA HONG */
void Process_Auto_Navigation(void)
{
  uint16_t d_tl = cb[TOF_DIAGONAL_LEFT].ok ? cb[TOF_DIAGONAL_LEFT].dist : 9999;
  uint16_t d_tc = cb[TOF_CENTER].ok ? cb[TOF_CENTER].dist : 9999;
  uint16_t d_tr = cb[TOF_DIAGONAL_RIGHT].ok ? cb[TOF_DIAGONAL_RIGHT].dist : 9999;

  /* 3. Neu dang xoay sau lui khan cap */
  if (current_action == ACTION_TURN_LEFT_90 || current_action == ACTION_TURN_RIGHT_90)
  {
    if (HAL_GetTick() - state_timer < 280)
    {
      return;
    }
    Set_Robot_Action(ACTION_FORWARD);
    return;
  }

  /* 4. Neu dang o trang thai TRA LAI HUONG THANG */
  if (current_action == ACTION_RETURN_ORIGIN)
  {
    if (HAL_GetTick() - state_timer < 220)
    {
      return;
    }
    Set_Robot_Action(ACTION_FORWARD);
    last_avoid_side = ACTION_STOP;
    return;
  }

  /* 5. Ne sang phai: theo doi hong trai de xac nhan da qua vat can. */
  if (current_action == ACTION_AVOID_RIGHT_45)
  {
    if (Passed_Side(TOF_SIDE_LEFT, d_tl))
    {
      last_avoid_side = ACTION_AVOID_RIGHT_45;
      Set_Robot_Action(ACTION_RETURN_ORIGIN);
      return;
    }
    return;
  }

  /* 6. Ne sang trai: theo doi hong phai de xac nhan da qua vat can. */
  if (current_action == ACTION_AVOID_LEFT_45)
  {
    if (Passed_Side(TOF_SIDE_RIGHT, d_tr))
    {
      last_avoid_side = ACTION_AVOID_LEFT_45;
      Set_Robot_Action(ACTION_RETURN_ORIGIN);
      return;
    }
    return;
  }

  /* 7. Khi dang chay tien thang binh thuong: Kiem tra ToF truoc de kich hoat ne 45 do */
  bool obst_left   = (d_tl < DIST_TRIGGER);
  bool obst_right  = (d_tr < DIST_TRIGGER);
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
  }
  else if (obst_left)
  {
    Set_Robot_Action(ACTION_AVOID_RIGHT_45);
  }
  else if (obst_right)
  {
    Set_Robot_Action(ACTION_AVOID_LEFT_45);
  }
  else
  {
    if (current_action != ACTION_FORWARD)
    {
      Set_Robot_Action(ACTION_FORWARD);
    }
  }
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
  Motor_Stop();

  /* 2. Khoi tao Software UART (co ngat EXTI11 de bat phim bam tuc thi 100%) */
  SoftUART_Init();

  HAL_Delay(100);

  /* 3. Khoi tao va gan dia chi 5 cam bien ToF */
  Run_Full_Initialization();

  Init_MPU();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_print = HAL_GetTick(), last_tof = 0;
  char tx_line[180] = {0};
  size_t tx_pos = 0;

  while (1)
  {
    Motor_Test_Update();
    Process_Front_Clearance();
    /* One command per iteration: start PWM before handling a following byte. */
    uint8_t rx_char;
    if (SoftUART_ReadByte(&rx_char) && rx_char != '\r' && rx_char != '\n' && rx_char != 0)
    {
      command_history[command_count++ % 16] = (HAL_GetTick() << 8) | rx_char;
      if (motor_test.state == 1) {
        if (rx_char == 'x' || rx_char == 'X' || rx_char == 'S' || rx_char == ' ')
          Motor_Test_Finish();
        else continue;
      }
      RobotAction_t requested = ACTION_STOP;
      bool movement = true;
      switch (rx_char) {
        case 'w': case 'W': case 'F': requested = ACTION_FORWARD; break;
        case 's': case 'B': requested = ACTION_BACKWARD; break;
        case 'a': case 'A': case 'L': requested = ACTION_TURN_LEFT_90; break;
        case 'd': case 'D': case 'R': requested = ACTION_TURN_RIGHT_90; break;
        case 'q': case 'Q': requested = ACTION_CURVE_LEFT_45; break;
        case 'e': case 'E': requested = ACTION_CURVE_RIGHT_45; break;
        case 'u': case 'U': requested = ACTION_PIVOT_LEFT_45; break;
        case 'p': case 'P': requested = ACTION_PIVOT_RIGHT_45; break;
        case 'x': case 'X': case ' ': case 'S': break;
        default: movement = false; break;
      }
      if (movement) {
        if (requested == ACTION_STOP) {
          FrontClearance_Stop(&clearance);
          clearance_owned = false;
          current_mode = ROBOT_MODE_MANUAL;
          Set_Robot_Action(ACTION_STOP);
          motion_error = NULL;
        } else if (!clearance_owned) {
          FrontClearance_Rearm(&clearance);
          current_mode = ROBOT_MODE_MANUAL;
          motion_error = NULL;
          Process_Front_Clearance();
          if (!clearance_owned) Set_Robot_Action(requested);
        }
      } else {
        switch (rx_char) {
          case '+': case '=': base_speed += 50; break;
          case '-': case '_': base_speed -= 50; break;
          case 'c': case 'C':
            FrontClearance_Stop(&clearance); clearance_owned = false;
            Init_MPU(); break;
          case 'm': case 'M':
            if (clearance_owned) break;
            FrontClearance_Rearm(&clearance);
            Set_Robot_Action(ACTION_STOP);
            current_mode = current_mode == ROBOT_MODE_MANUAL ? ROBOT_MODE_AUTO : ROBOT_MODE_MANUAL;
            Process_Front_Clearance();
            if (!clearance_owned && current_mode == ROBOT_MODE_AUTO) Set_Robot_Action(ACTION_FORWARD);
            break;
          case 't': case 'T': auto_stream = !auto_stream; break;
          case 'i': case 'I':
            FrontClearance_Stop(&clearance); clearance_owned = false;
            Set_Robot_Action(ACTION_STOP); Motor_ToggleInvertA(); break;
          case 'o': case 'O':
            FrontClearance_Stop(&clearance); clearance_owned = false;
            Set_Robot_Action(ACTION_STOP); Motor_ToggleInvertB(); break;
          default: break;
        }
        if (base_speed > 950) base_speed = 950;
        if (base_speed < 200) base_speed = 200;
      }
    }

    Motion_Update(base_speed);
    if (!Motion_IsMPUReady() && !motion_error)
      motion_error = Motion_GetMPUState();
    if (current_mode == ROBOT_MODE_MANUAL) {
      MotionStatus result = Motion_GetStatus();
      if (result != MOTION_IDLE && result != MOTION_RUNNING) {
        Set_Robot_Action(ACTION_STOP);
        motion_error = result == MOTION_DONE ? NULL :
          result == MOTION_TIMEOUT ? "QUA_THOI_GIAN" :
          result == MOTION_ANGLE_ERROR ? "GOC_LECH" : Motion_GetMPUState();
      }
    }

    /* A persistent read fault stops the turn. Retry only while stopped; never
     * resume a cancelled movement. Recalibration establishes a new zero yaw. */
    if (!Motion_IsMPUReady() && current_mode == ROBOT_MODE_MANUAL &&
        current_action == ACTION_STOP && HAL_GetTick() - last_mpu_attempt >= 5000) {
      motion_error = "DANG_HIEU_CHUAN_MPU";
      /* Finish any partial line before reporting the calibration state. */
      while (tx_line[tx_pos]) SoftUART_TransmitByte((uint8_t)tx_line[tx_pos++]);
      Format_Status(tx_line, sizeof(tx_line));
      SoftUART_TransmitString(tx_line);
      tx_line[0] = 0;
      tx_pos = 0;
      HAL_I2C_DeInit(&hi2c2);
      I2C_Bus_Recovery();
      MX_I2C2_Init();
      Init_MPU();
    }

    if (HAL_GetTick() - last_tof >= 5) {
      last_tof = HAL_GetTick();
      Update_One_ToF();
      Motion_Update(base_speed);
      Process_Front_Clearance();
      if (!clearance_owned && current_mode == ROBOT_MODE_AUTO) Process_Auto_Navigation();
    }

    /* Transmit one byte per loop. No stream or acknowledgement blocks a turn. */
    if (!tx_line[tx_pos] && auto_stream && HAL_GetTick() - last_print >= 250) {
      last_print = HAL_GetTick();
      Format_Status(tx_line, sizeof(tx_line));
      tx_pos = 0;
    }
    if (tx_line[tx_pos]) {
      SoftUART_TransmitByte((uint8_t)tx_line[tx_pos++]);
      Motion_Update(base_speed);
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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
