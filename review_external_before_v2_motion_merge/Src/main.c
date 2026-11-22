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
#include "vl53l0x.h"
#include "tof_offset.h"
#include "motor.h"
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
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
  int16_t       offset_mm; // Bu tru sai so cu ly gan (Offset)
  uint16_t      raw_dist;  // Last uncorrected sample, never reconstruct from clamped output
  uint32_t      raw_tick;
} Sensor_Info_t;

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
  ACTION_FOLLOW_LINE,
  ACTION_AVOID_LEFT_45,
  ACTION_AVOID_RIGHT_45,
  ACTION_RETURN_TO_LINE,
  ACTION_EMERGENCY_REVERSE
} RobotAction_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_RED_PIN       GPIO_PIN_8
#define LED_RED_PORT      GPIOB
#define LED_GREEN_PIN     GPIO_PIN_9
#define LED_GREEN_PORT    GPIOB

/* Chan cam bien do Line IR */
#define IR_LEFT_PIN       GPIO_PIN_4
#define IR_LEFT_PORT      GPIOA
#define IR_RIGHT_PIN      GPIO_PIN_5
#define IR_RIGHT_PORT     GPIOA

/* Nguong khoang cach tranh vat can ToF (mm) */
#define DIST_EMERGENCY    90   // Qua sat (<9cm) -> phanh va lui khan cap
#define DIST_TRIGGER     240   // Phat hien vat can phia truoc (<24cm) -> re 45 do de ne
#define DIST_CLEARED     260   // ToF sau xac nhan da vuot qua hoan toan vat can (>26cm)

#define TIME_PIVOT_45MS   180
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
  {"CB1 Sau-Trai",   GPIOB, GPIO_PIN_0,  "PB0",  0x30, false, 8190, 0, 8190, 0},
  {"CB2 Truoc-Trai", GPIOB, GPIO_PIN_14, "PB14", 0x32, false, 8190, TOF_OFFSET_LEFT_MM, 8190, 0},
  {"CB3 Truoc-Giua", GPIOB, GPIO_PIN_13, "PB13", 0x34, false, 8190, TOF_OFFSET_CENTER_MM, 8190, 0},
  {"CB4 Truoc-Phai", GPIOB, GPIO_PIN_12, "PB12", 0x36, false, 8190, TOF_OFFSET_RIGHT_MM, 8190, 0},
  {"CB5 Sau-Phai",   GPIOB, GPIO_PIN_1,  "PB1",  0x38, false, 8190, 0, 8190, 0}
};

/* MPU-6050 */
MPU6050_Data_t mpu_data;
bool mpu_ok = false;
float gyro_z_offset = 0.0f;
float yaw_angle = 0.0f;
uint32_t last_mpu_time = 0;

/* Cam bien Line IR */
uint8_t ir_left_val = 0;
uint8_t ir_right_val = 0;
uint8_t ir_black_level = 1; // 1 neu line den ra muc HIGH, 0 neu line den ra muc LOW

char msg[190];
bool auto_stream = true;
unsigned selected_tof = 1;
RobotMode_t current_mode = ROBOT_MODE_MANUAL;
RobotAction_t current_action = ACTION_STOP;

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
const char* Get_Action_Name(RobotAction_t action);
void Print_Menu(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  SoftUART_TransmitString("\r\n=================================================\r\n");
  SoftUART_TransmitString("   KHOI TAO HE THONG 5x TOF, MPU6050 & IR LINE   \r\n");
  SoftUART_TransmitString("=================================================\r\n");

  /* 1. Khoi tao 5 cam bien ToF VL53L0X */
  XSHUT_Set_All(GPIO_PIN_RESET);
  for (int i = 0; i < 5; i++) cb[i].ok = false;
  HAL_Delay(80);

  uint8_t count_ok = 0;
  for (int i = 0; i < 5; i++)
  {
    snprintf(msg, sizeof(msg), "[%u/5] Bat %s (%s)...", i + 1, cb[i].name, cb[i].pin_name);
    SoftUART_TransmitString(msg);

    HAL_GPIO_WritePin(cb[i].port, cb[i].pin, GPIO_PIN_SET);
    HAL_Delay(35);

    if (HAL_I2C_IsDeviceReady(&hi2c2, 0x52, 3, 10) != HAL_OK)
    {
      SoftUART_TransmitString(" -> [ERR: Khong thay tai 0x29!]\r\n");
      cb[i].ok = false;
      continue;
    }

    if (!VL53L0X_Init(&dev[i], &hi2c2, 0x52))
    {
      SoftUART_TransmitString(" -> [ERR: Loi khoi tao tai 0x29!]\r\n");
      cb[i].ok = false;
      continue;
    }

    if (!VL53L0X_SetAddress(&dev[i], cb[i].addr))
    {
      SoftUART_TransmitString(" -> [ERR: Loi doi dia chi!]\r\n");
      cb[i].ok = false;
      continue;
    }

    VL53L0X_StartContinuous(&dev[i]);
    cb[i].ok = true;
    count_ok++;
    snprintf(msg, sizeof(msg), " -> [OK! Dia chi: 0x%02X]\r\n", cb[i].addr);
    SoftUART_TransmitString(msg);
  }

  snprintf(msg, sizeof(msg), "=> ToF San sang: %u / 5 cam bien hoat dong!\r\n", count_ok);
  SoftUART_TransmitString(msg);

  /* 2. Khoi tao MPU-6050 */
  SoftUART_TransmitString("[*] Khoi tao cam bien goc MPU-6050...");
  if (MPU6050_Init(&hi2c2))
  {
    mpu_ok = true;
    SoftUART_TransmitString(" -> [OK!]\r\n");
    MPU6050_Calibrate();
  }
  else
  {
    mpu_ok = false;
    SoftUART_TransmitString(" -> [ERR: Khong tim thay MPU-6050 tai 0x68!]\r\n");
  }

  SoftUART_TransmitString("=================================================\r\n\r\n");
}

/* Hieu chuan diem 0 (Offset) cho truc Gyro Z de tinh goc quay Yaw chinh xac */
void MPU6050_Calibrate(void)
{
  SoftUART_TransmitString("  -> Dang calib diem 0 MPU-6050 (giu xe yen tinh)...");
  int32_t sum_gz = 0;
  uint16_t samples = 100;

  for (uint16_t i = 0; i < samples; i++)
  {
    if (MPU6050_Read_All(&hi2c2, &mpu_data))
    {
      sum_gz += mpu_data.Gyro_Z;
    }
    HAL_Delay(3);
  }

  gyro_z_offset = (float)sum_gz / (float)samples;
  yaw_angle = 0.0f;
  last_mpu_time = HAL_GetTick();
  SoftUART_TransmitString(" [XONG!]\r\n");
}

/* Tich phan Gyro Z de cap nhat goc quay Yaw lien tuc */
void Update_MPU_Yaw(void)
{
  if (!mpu_ok) return;

  uint32_t now = HAL_GetTick();
  uint32_t elapsed = now - last_mpu_time;
  if (elapsed < 10) return; // Cap nhat moi 10ms (100 Hz)
  
  float dt = (float)elapsed / 1000.0f;
  last_mpu_time = now;

  if (MPU6050_Read_All(&hi2c2, &mpu_data))
  {
    /* Do nhay Gyro Z ±250 dps la 131.0 LSB / (deg/s) */
    float rate_z = ((float)mpu_data.Gyro_Z - gyro_z_offset) / 131.0f;
    if (fabsf(rate_z) > 0.3f) // Deadband loc nhieu rung nhe
    {
      yaw_angle += rate_z * dt;
    }
  }
}

/* Doc trang thai 2 cam bien Line IR */
void Read_Line_Sensors(void)
{
  uint8_t raw_l = (HAL_GPIO_ReadPin(IR_LEFT_PORT, IR_LEFT_PIN) == GPIO_PIN_SET) ? 1 : 0;
  uint8_t raw_r = (HAL_GPIO_ReadPin(IR_RIGHT_PORT, IR_RIGHT_PIN) == GPIO_PIN_SET) ? 1 : 0;

  /* Chuan hoa ve 1 neu dung line den */
  ir_left_val  = (raw_l == ir_black_level) ? 1 : 0;
  ir_right_val = (raw_r == ir_black_level) ? 1 : 0;
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
    case ACTION_FOLLOW_LINE:       return "BAM-LINE";
    case ACTION_AVOID_LEFT_45:     return "NE-T45";
    case ACTION_AVOID_RIGHT_45:    return "NE-P45";
    case ACTION_RETURN_TO_LINE:    return "VE-LINE";
    case ACTION_EMERGENCY_REVERSE: return "LUI-KHAN";
    default:                       return "---";
  }
}

void Set_Robot_Action(RobotAction_t action)
{
  current_action = action;
  switch (action)
  {
    case ACTION_FORWARD:
      Motor_SetSpeed(base_speed, base_speed);
      HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
      break;

    case ACTION_BACKWARD:
      Motor_SetSpeed(-base_speed, -base_speed);
      HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
      break;

    case ACTION_TURN_LEFT_90:
      /* Quay tai cho 90 do sang trai */
      Motor_SetSpeed(-base_speed, base_speed);
      HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
      break;

    case ACTION_TURN_RIGHT_90:
      /* Quay tai cho 90 do sang phai */
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
      Motor_SetSpeed(-base_speed, base_speed);
      state_timer = HAL_GetTick();
      break;

    case ACTION_PIVOT_RIGHT_45:
      Motor_SetSpeed(base_speed, -base_speed);
      state_timer = HAL_GetTick();
      break;

    case ACTION_RETURN_TO_LINE:
      /* Bẻ lái 45 độ ngược lại để tìm lại vạch line đen */
      if (avoid_direction == ACTION_AVOID_RIGHT_45)
      {
        Motor_SetSpeed((base_speed * 40) / 100, base_speed); // Lượn trái lại tìm line
      }
      else
      {
        Motor_SetSpeed(base_speed, (base_speed * 40) / 100); // Lượn phải lại tìm line
      }
      state_timer = HAL_GetTick();
      break;

    case ACTION_EMERGENCY_REVERSE:
      Motor_SetSpeed(-base_speed, -base_speed);
      HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
      state_timer = HAL_GetTick();
      break;

    case ACTION_FOLLOW_LINE:
    case ACTION_STOP:
    default:
      Motor_Stop();
      HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
      break;
  }
}

/* THUAT TOAN TU DONG: KET HOP 3 ToF TRUOC + 2 ToF SAU + 2 IR LINE + MPU */
void Process_Auto_Navigation(void)
{
  uint16_t d_sl = cb[0].ok ? cb[0].dist : 9999;
  uint16_t d_tl = cb[1].ok ? cb[1].dist : 9999;
  uint16_t d_tc = cb[2].ok ? cb[2].dist : 9999;
  uint16_t d_tr = cb[3].ok ? cb[3].dist : 9999;
  uint16_t d_sr = cb[4].ok ? cb[4].dist : 9999;

  /* 1. Tinh huong nguy cap: Vat can qua sat (< 90mm) */
  if (d_tl < DIST_EMERGENCY || d_tr < DIST_EMERGENCY || d_tc < DIST_EMERGENCY)
  {
    if (current_action != ACTION_EMERGENCY_REVERSE)
    {
      Set_Robot_Action(ACTION_EMERGENCY_REVERSE);
    }
    return;
  }

  /* 2. Xu ly lui khan cap */
  if (current_action == ACTION_EMERGENCY_REVERSE)
  {
    if (HAL_GetTick() - state_timer < 300)
    {
      return;
    }
    // Lui xong -> xoay ve ben thoang
    if (d_tl > d_tr)
    {
      Set_Robot_Action(ACTION_TURN_LEFT_90);
    }
    else
    {
      Set_Robot_Action(ACTION_TURN_RIGHT_90);
    }
    state_timer = HAL_GetTick();
    return;
  }

  if (current_action == ACTION_TURN_LEFT_90 || current_action == ACTION_TURN_RIGHT_90)
  {
    if (HAL_GetTick() - state_timer < 280)
    {
      return;
    }
    Set_Robot_Action(ACTION_FORWARD);
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
  current_action = ACTION_FOLLOW_LINE;
  if (ir_left_val == 1 && ir_right_val == 0)
  {
    /* Xe lech sang phai vach line -> Banh phai quay nhanh hon, banh trai giam de chinh sang trai */
    Motor_SetSpeed((base_speed * 40) / 100, base_speed);
    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
  }
  else if (ir_left_val == 0 && ir_right_val == 1)
  {
    /* Xe lech sang trai vach line -> Banh trai quay nhanh hon, banh phai giam de chinh sang phai */
    Motor_SetSpeed(base_speed, (base_speed * 40) / 100);
    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
  }
  else
  {
    /* Ca 2 mat IR nam ngoai (line o giua xe) hoac ca 2 deu thay line -> Chay thang tien */
    Motor_SetSpeed(base_speed, base_speed);
    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
  }
}

void Print_Menu(void)
{
  SoftUART_TransmitString("\r\n================ [MENU DIEU KHIEN ROBOT] ================\r\n");
  SoftUART_TransmitString("  [DIEU KHIEN THU CONG]:\r\n");
  SoftUART_TransmitString("  - 'w' / 's': Tien / Lui thang (2 banh dong bo)\r\n");
  SoftUART_TransmitString("  - 'a' / 'd': Re trai / phai 90 do\r\n");
  SoftUART_TransmitString("  - 'q' / 'e': Cua mem 45 do sang trai / phai\r\n");
  SoftUART_TransmitString("  - 'u' / 'p': Xoay tai cho 45 do trai / phai\r\n");
  SoftUART_TransmitString("  - 'x' / ' ': Dung xe\r\n");
  SoftUART_TransmitString("  - '+' / '-': Tang / Giam toc do chay\r\n");
  SoftUART_TransmitString("  - 'i' / 'o': Dao chieu Motor A / Motor B\r\n");
  SoftUART_TransmitString("\r\n  [CAM BIEN & TU DONG]:\r\n");
  SoftUART_TransmitString("  - 'm': Bat / Tat Che Do Tu Dong (Bam Line + Ne ToF Truoc/Sau)\r\n");
  SoftUART_TransmitString("  - 'c': Calib lai diem 0 goc Yaw MPU-6050\r\n");
  SoftUART_TransmitString("  - '1'/'2'/'3': Chon ToF dau trai/giua/phai de calib\r\n");
  SoftUART_TransmitString("  - 'z': Dung xe, calib ToF da chon ve 0mm (can mau moi hop le)\r\n");
  SoftUART_TransmitString("  - '[' / ']': Tang / Giam bu tru cho Truoc-Phai (TR do sat hon)\r\n");
  SoftUART_TransmitString("  - '{' / '}': Tang / Giam bu tru cho Truoc-Trai (TL do sat hon)\r\n");
  SoftUART_TransmitString("  - 'k': Dao cuc tinh do Line den (Muc 1 <-> Muc 0)\r\n");
  SoftUART_TransmitString("  - 't': Bat / Tat in du lieu len man hinh\r\n");
  SoftUART_TransmitString("  - 'h': In lai Menu nay\r\n");
  SoftUART_TransmitString("=========================================================\r\n\r\n");
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

  /* 2. Khoi tao Software UART (co ngat EXTI11 tren PA11) */
  SoftUART_Init();

  HAL_Delay(100);

  Print_Menu();

  /* 3. Khoi tao 5x ToF va MPU-6050 */
  Run_Full_Initialization();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_print = HAL_GetTick();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* 1. Cap nhat goc quay Yaw tu MPU-6050 */
    Update_MPU_Yaw();

    /* 2. Doc gia tri 2 cam bien Line IR */
    Read_Line_Sensors();

    /* 3. Xu ly phim bam dieu khien tu Bluetooth App qua hang doi ngat */
    uint8_t rx_char = 0;
    while (SoftUART_ReadByte(&rx_char))
    {
      if (rx_char == '\r' || rx_char == '\n') continue;

      switch (rx_char)
      {
        case 'w': case 'W': case 'F':
          current_mode = ROBOT_MODE_MANUAL;
          Set_Robot_Action(ACTION_FORWARD);
          SoftUART_TransmitString("-> [TIEN THANG]\r\n");
          break;

        case 's': case 'S': case 'B':
          current_mode = ROBOT_MODE_MANUAL;
          Set_Robot_Action(ACTION_BACKWARD);
          SoftUART_TransmitString("-> [LUI THANG]\r\n");
          break;

        case 'a': case 'A': case 'L':
          current_mode = ROBOT_MODE_MANUAL;
          Set_Robot_Action(ACTION_TURN_LEFT_90);
          SoftUART_TransmitString("-> [RE TRAI 90]\r\n");
          break;

        case 'd': case 'D': case 'R':
          current_mode = ROBOT_MODE_MANUAL;
          Set_Robot_Action(ACTION_TURN_RIGHT_90);
          SoftUART_TransmitString("-> [RE PHAI 90]\r\n");
          break;

        case 'q': case 'Q':
          current_mode = ROBOT_MODE_MANUAL;
          Set_Robot_Action(ACTION_CURVE_LEFT_45);
          SoftUART_TransmitString("-> [CUA TRAI 45]\r\n");
          break;

        case 'e': case 'E':
          current_mode = ROBOT_MODE_MANUAL;
          Set_Robot_Action(ACTION_CURVE_RIGHT_45);
          SoftUART_TransmitString("-> [CUA PHAI 45]\r\n");
          break;

        case 'u': case 'U':
          current_mode = ROBOT_MODE_MANUAL;
          Set_Robot_Action(ACTION_PIVOT_LEFT_45);
          SoftUART_TransmitString("-> [XOAY TRAI 45]\r\n");
          break;

        case 'p': case 'P':
          current_mode = ROBOT_MODE_MANUAL;
          Set_Robot_Action(ACTION_PIVOT_RIGHT_45);
          SoftUART_TransmitString("-> [XOAY PHAI 45]\r\n");
          break;

        case 'x': case 'X': case ' ':
          current_mode = ROBOT_MODE_MANUAL;
          Set_Robot_Action(ACTION_STOP);
          SoftUART_TransmitString("-> [DUNG XE]\r\n");
          break;

        case '+': case '=':
          base_speed += 50;
          if (base_speed > 950) base_speed = 950;
          snprintf(msg, sizeof(msg), "-> [TOC DO]: %d\r\n", base_speed);
          SoftUART_TransmitString(msg);
          if (current_action != ACTION_STOP) Set_Robot_Action(current_action);
          break;

        case '-': case '_':
          base_speed -= 50;
          if (base_speed < 200) base_speed = 200;
          snprintf(msg, sizeof(msg), "-> [TOC DO]: %d\r\n", base_speed);
          SoftUART_TransmitString(msg);
          if (current_action != ACTION_STOP) Set_Robot_Action(current_action);
          break;

        case 'i': case 'I':
          Motor_ToggleInvertA();
          snprintf(msg, sizeof(msg), "-> [INVERT A]: %d\r\n", invert_motor_a);
          SoftUART_TransmitString(msg);
          break;

        case 'o': case 'O':
          Motor_ToggleInvertB();
          snprintf(msg, sizeof(msg), "-> [INVERT B]: %d\r\n", invert_motor_b);
          SoftUART_TransmitString(msg);
          break;

        case 'c': case 'C':
          MPU6050_Calibrate();
          break;

        case '1': case '2': case '3':
          selected_tof = rx_char - '0';
          snprintf(msg, sizeof(msg), "-> [CHON TOF]: %s\r\n", cb[selected_tof].name);
          SoftUART_TransmitString(msg);
          break;

        case 'z': case 'Z':
        {
          current_mode = ROBOT_MODE_MANUAL;
          Set_Robot_Action(ACTION_STOP);
          unsigned k = selected_tof;
          if (cb[k].ok && cb[k].raw_dist < 400 && HAL_GetTick() - cb[k].raw_tick <= 500)
          {
            cb[k].offset_mm = ToF_ClampOffset(cb[k].raw_dist);
            cb[k].dist = ToF_CorrectDistance(cb[k].raw_dist, cb[k].offset_mm);
            snprintf(msg, sizeof(msg), "-> [ZERO %s]: raw=%u offset=%d mm\r\n",
                     cb[k].name, cb[k].raw_dist, cb[k].offset_mm);
            SoftUART_TransmitString(msg);
          }
          else SoftUART_TransmitString("-> [LOI ZERO]: Can mau ToF moi hop le, duoi 400mm\r\n");
          break;
        }

        case '[':
          cb[3].offset_mm = ToF_ClampOffset((int32_t)cb[3].offset_mm + 5);
          snprintf(msg, sizeof(msg), "-> [TR Offset]: %d mm\r\n", cb[3].offset_mm);
          SoftUART_TransmitString(msg);
          break;

        case ']':
          cb[3].offset_mm = ToF_ClampOffset((int32_t)cb[3].offset_mm - 5);
          snprintf(msg, sizeof(msg), "-> [TR Offset]: %d mm\r\n", cb[3].offset_mm);
          SoftUART_TransmitString(msg);
          break;

        case '{':
          cb[1].offset_mm = ToF_ClampOffset((int32_t)cb[1].offset_mm + 5);
          snprintf(msg, sizeof(msg), "-> [TL Offset]: %d mm\r\n", cb[1].offset_mm);
          SoftUART_TransmitString(msg);
          break;

        case '}':
          cb[1].offset_mm = ToF_ClampOffset((int32_t)cb[1].offset_mm - 5);
          snprintf(msg, sizeof(msg), "-> [TL Offset]: %d mm\r\n", cb[1].offset_mm);
          SoftUART_TransmitString(msg);
          break;

        case 'k': case 'K':
          ir_black_level = !ir_black_level;
          snprintf(msg, sizeof(msg), "-> [DAO CUC TINH LINE DEN]: Black = Muc %u\r\n", ir_black_level);
          SoftUART_TransmitString(msg);
          break;

        case 'm': case 'M':
          if (current_mode == ROBOT_MODE_MANUAL)
          {
            current_mode = ROBOT_MODE_AUTO;
            SoftUART_TransmitString("\r\n>>> [KICH HOAT: TU DONG BAM LINE & TRANH VAT CAN TOF] <<<\r\n");
            Set_Robot_Action(ACTION_FORWARD);
          }
          else
          {
            current_mode = ROBOT_MODE_MANUAL;
            Set_Robot_Action(ACTION_STOP);
            SoftUART_TransmitString("\r\n>>> [CHUYEN VE: CHE DO THU CONG (DA DUNG)] <<<\r\n");
          }
          break;

        case 't': case 'T':
          auto_stream = !auto_stream;
          snprintf(msg, sizeof(msg), "-> [STREAM]: %s\r\n", auto_stream ? "BAT" : "TAT");
          SoftUART_TransmitString(msg);
          break;

        case 'h': case 'H': case '?':
          Print_Menu();
          break;

        default:
          break;
      }
    }

    /* Xu ly hoan tat xoay 45 do tai cho trong che do thu cong */
    if (current_mode == ROBOT_MODE_MANUAL)
    {
      if (current_action == ACTION_PIVOT_LEFT_45 || current_action == ACTION_PIVOT_RIGHT_45)
      {
        if (HAL_GetTick() - state_timer >= TIME_PIVOT_45MS)
        {
          Set_Robot_Action(ACTION_STOP);
          SoftUART_TransmitString("-> [XONG 45 DO]\r\n");
        }
      }
    }

    /* 4. Cap nhat khoang cach 5x ToF (Non-blocking continuous kem bu Offset cu ly gan) */
    for (int i = 0; i < 5; i++)
    {
      if (cb[i].ok)
      {
        uint16_t d = VL53L0X_ReadRangeContinuousMillimeters(&dev[i]);
        if (d > 0)
        {
          cb[i].raw_dist = d;
          cb[i].raw_tick = HAL_GetTick();
          if (d < 8000)
          {
            if (i >= 1 && i <= 3)
              cb[i].dist = ToF_CorrectDistance(d, cb[i].offset_mm);
            else
              cb[i].dist = d < 15 ? 15 : d;
          }
          else
          {
            cb[i].dist = d;
          }
        }
      }
      else
      {
        cb[i].dist = 8190;
      }
    }

    /* 5. Dieu huong tu dong: Bam line & Ne vat can ToF truoc/sau */
    if (current_mode == ROBOT_MODE_AUTO)
    {
      Process_Auto_Navigation();
    }

    /* 6. In trang thai tong the xe len App moi 220ms */
    if (auto_stream && (HAL_GetTick() - last_print >= 220))
    {
      last_print = HAL_GetTick();

      char d_str[5][8];
      for (int i = 0; i < 5; i++)
      {
        if (!cb[i].ok)
        {
          strcpy(d_str[i], "ERR");
        }
        else if (cb[i].dist >= 8000)
        {
          strcpy(d_str[i], "---");
        }
        else
        {
          snprintf(d_str[i], sizeof(d_str[i]), "%u", cb[i].dist);
        }
      }

      char yaw_str[16];
      if (!mpu_ok)
      {
        strcpy(yaw_str, "ERR");
      }
      else
      {
        int32_t y_int = (int32_t)yaw_angle;
        int32_t y_dec = (int32_t)(fabsf(yaw_angle) * 10.0f) % 10;
        if (yaw_angle < 0.0f && y_int == 0)
        {
          snprintf(yaw_str, sizeof(yaw_str), "-0.%ddo", (int)y_dec);
        }
        else
        {
          snprintf(yaw_str, sizeof(yaw_str), "%ld.%ddo", (long)y_int, (int)y_dec);
        }
      }

      snprintf(msg, sizeof(msg), "[%s|%s] ToF:[SL:%s|TL:%s|TC:%s|TR:%s|SR:%s] IR:[%u,%u] Yaw:%s\r\n",
               (current_mode == ROBOT_MODE_AUTO ? "AUTO" : "MANUAL"),
               Get_Action_Name(current_action),
               d_str[0], d_str[1], d_str[2], d_str[3], d_str[4],
               ir_left_val, ir_right_val,
               yaw_str);
      SoftUART_TransmitString(msg);

      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }

    HAL_Delay(2);
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

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
