/* USER CODE BEGIN Header */

/**

 ******************************************************************************

 * @file : main.c

 * @brief : Main program body

 ******************************************************************************

 * @attention

 *

 * Copyright (c) 2026 STMicroelectronics.

 * All rights reserved.

 *

 * This software is licensed under terms that can be found in the LICENSE file

 * in the root directory of this software component.

 * If no LICENSE file comes with this software, it is provided AS-IS.

 *

 ******************************************************************************

 */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "main.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include "pca9685.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

typedef struct {
  int current_count;
  int min_count;
  int center_count;
  int max_count;
  uint8_t reversed;
  const char* name;
} ServoConfig;

typedef struct {
  float base_deg;
  float shoulder_deg;
  float elbow_deg;
} IKAngles;

typedef struct {
  int base_count;
  int shoulder_count;
  int elbow_count;
} JointCounts;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Servo channels on PCA9685 */
#define BASE_SERVO_MOTOR 0
#define ARM_SERVO_MOTOR1 1 /* Shoulder */
#define ARM_SERVO_MOTOR2 2 /* Elbow */
#define CLAW_SERVO_MOTOR 3

/* Final ROM values */
#define BASE_MIN_COUNT 200
#define BASE_CENTER_COUNT 450
#define BASE_MAX_COUNT 620

#define ARM1_MIN_COUNT 220
#define ARM1_CENTER_COUNT 410
#define ARM1_MAX_COUNT 600

#define ARM2_MIN_COUNT 220
#define ARM2_CENTER_COUNT 410
#define ARM2_MAX_COUNT 620

#define CLAW_MIN_COUNT 290 /* close */
#define CLAW_CENTER_COUNT 410
#define CLAW_MAX_COUNT 470 /* open */

/* Smooth motion timing */
#define MOTION_DT_MS 20.0f
#define DEFAULT_MAX_SPEED_CPS 120.0f
#define DEFAULT_ACCEL_CPS2 240.0f

/* Pause times */
#define SHORT_SETTLE_MS 600
#define LONG_SETTLE_MS 1200

/* Workspace / geometry */
#define PAPER_WIDTH_CM 25.0f
#define PAPER_HEIGHT_CM 20.0f

/* Robot base axis in workspace coordinates */
#define BASE_X_CM 13.0f
#define BASE_Y_CM -12.8f
#define BASE_Z_CM 3.0f

/* Shoulder pivot height above paper */
#define SHOULDER_Z_CM 15.0f

/* Effective link lengths */
#define LINK1_CM 8.3f
#define LINK2_CM 13.0f

/* Hardcoded test points */
#define PICK_X_CM 3.0f
#define PICK_Y_CM 3.0f
#define DROP_X_CM 21.0f
#define DROP_Y_CM 3.0f

/* Heights above paper */
#define APPROACH_Z_CM 3.0f
#define GRASP_Z_CM 0.5f

/* First-pass logical angle spans */
#define BASE_MIN_DEG -90.0f
#define BASE_MAX_DEG 90.0f

#define SHOULDER_MIN_DEG -70.0f
#define SHOULDER_MAX_DEG 70.0f

#define ELBOW_MIN_DEG -80.0f
#define ELBOW_MAX_DEG 80.0f

/* Neutral absolute/reference assumptions */
#define BASE_FORWARD_ABS_DEG 0.0f
#define SHOULDER_NEUTRAL_ABS_DEG -70.0f
#define ELBOW_NEUTRAL_REL_DEG 110.0f

#define REACH_MARGIN_CM 0.3f
#define CLAMP_UNREACHABLE_TARGETS 1

#define BASE_SERVO_MOTOR 0

#define ARM_SERVO_MOTOR1 1

#define ARM_SERVO_MOTOR2 2

#define CLAW_SERVO_MOTOR 3

/* USER CODE END PD */

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart3;

ADC_HandleTypeDef hadc1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

ServoConfig servos[4] = {{BASE_CENTER_COUNT, BASE_MIN_COUNT, BASE_CENTER_COUNT,
                          BASE_MAX_COUNT, 0, "Base Servo"},
                         {ARM1_CENTER_COUNT, ARM1_MIN_COUNT, ARM1_CENTER_COUNT,
                          ARM1_MAX_COUNT, 0, "Shoulder Servo"},
                         {ARM2_CENTER_COUNT, ARM2_MIN_COUNT, ARM2_CENTER_COUNT,
                          ARM2_MAX_COUNT, 0, "Elbow Servo"},
                         {CLAW_CENTER_COUNT, CLAW_MIN_COUNT, CLAW_CENTER_COUNT,
                          CLAW_MAX_COUNT, 0, "Claw Servo"}};

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Joystick code
#define J1_X_CHANNEL ADC_CHANNEL_0  // PA0
#define J1_Y_CHANNEL ADC_CHANNEL_1  // PA1
#define J2_X_CHANNEL ADC_CHANNEL_4  // PA4
#define J2_Y_CHANNEL ADC_CHANNEL_5  // PA5

#define JOY_CENTER 2048
#define JOY_DEADZONE 180
#define JOY_MIN 0
#define JOY_MAX 4095

typedef struct {
  uint16_t x_raw;
  uint16_t y_raw;
  int x_norm;
  int y_norm;
  uint8_t button;  // 1 = pressed, 0 = not pressed
} JoystickState;

uint16_t Joystick_ReadADC(uint32_t channel);
int Joystick_Normalize(uint16_t raw);
void Joystick_Read(JoystickState* js, uint32_t x_channel, uint32_t y_channel,
                   GPIO_TypeDef* sw_port, uint16_t sw_pin);
void Joystick_Print(const JoystickState* j1, const JoystickState* j2);

uint16_t Joystick_ReadADC(uint32_t channel) {
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    return 0;
  }

  HAL_ADC_Start(&hadc1);

  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
    HAL_ADC_Stop(&hadc1);
    return 0;
  }

  uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  return value;
}

int Joystick_Normalize(uint16_t raw) {
  int val = (int)raw - JOY_CENTER;

  if (abs(val) < JOY_DEADZONE) {
    return 0;
  }

  return val;
}

void Joystick_Read(JoystickState* js, uint32_t x_channel, uint32_t y_channel,
                   GPIO_TypeDef* sw_port, uint16_t sw_pin) {
  js->x_raw = Joystick_ReadADC(x_channel);
  js->y_raw = Joystick_ReadADC(y_channel);

  js->x_norm = Joystick_Normalize(js->x_raw);
  js->y_norm = Joystick_Normalize(js->y_raw);

  // Active low button because GPIO pull-up is enabled
  js->button = (HAL_GPIO_ReadPin(sw_port, sw_pin) == GPIO_PIN_RESET) ? 1 : 0;
}

void Joystick_Print(const JoystickState* j1, const JoystickState* j2) {
  char msg[200];

  int len = snprintf(msg, sizeof(msg),
                     "J1: X=%4u Y=%4u Xn=%5d Yn=%5d Btn=%d | "
                     "J2: X=%4u Y=%4u Xn=%5d Yn=%5d Btn=%d\r\n",
                     j1->x_raw, j1->y_raw, j1->x_norm, j1->y_norm, j1->button,
                     j2->x_raw, j2->y_raw, j2->x_norm, j2->y_norm, j2->button);

  HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, HAL_MAX_DELAY);
}

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_I2C2_Init(void);

static void MX_USART3_UART_Init(void);

static void MX_USB_OTG_FS_PCD_Init(void);

static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

void print_status_and_halt(const char* msg);
int clamp_int(int value, int min_value, int max_value);
float clamp_float(float value, float min_value, float max_value);
float deg_to_rad(float deg);
float rad_to_deg(float rad);

int servo_limit_count(int channel, int raw_count);
int servo_apply_direction(int channel, int logical_count);
HAL_StatusTypeDef servo_write_count(int channel, int logical_count);
void servo_write_all_centers(void);

void move_servo_smooth_trapezoid(int channel, int target_count,
                                 float max_speed_cps, float accel_cps2);

int logical_angle_deg_to_count(int channel, float logical_angle_deg);
float count_to_logical_angle_deg(int channel, int count);

void move_arm_counts(int base_count, int shoulder_count, int elbow_count);
void claw_open(void);
void claw_close(void);

/* USER CODE BEGIN 0 */

void print_msg(char* msg) {
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
}

void print_status_and_halt(const char* msg) {
  print_msg(msg);
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
  __disable_irq();
  while (1) {
  }
}

int clamp_int(int value, int min_value, int max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

float clamp_float(float value, float min_value, float max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

float deg_to_rad(float deg) { return deg * ((float)M_PI / 180.0f); }

float rad_to_deg(float rad) { return rad * (180.0f / (float)M_PI); }

int servo_limit_count(int channel, int raw_count) {
  return clamp_int(raw_count, servos[channel].min_count,
                   servos[channel].max_count);
}

/* If a servo goes the wrong way on hardware, set reversed = 1 in servos[] */
int servo_apply_direction(int channel, int logical_count) {
  ServoConfig* s = &servos[channel];
  int limited = servo_limit_count(channel, logical_count);

  if (!s->reversed) {
    return limited;
  }

  return s->center_count - (limited - s->center_count);
}

HAL_StatusTypeDef servo_write_count(int channel, int logical_count) {
  int physical_count = servo_apply_direction(channel, logical_count);
  return PCA9685_SetServoPulseCounts(&hi2c2, channel, physical_count);
}

void move_servo_smooth_trapezoid(int channel, int target_count,
                                 float max_speed_cps, float accel_cps2) {
  char msg[128];

  ServoConfig* s = &servos[channel];
  const float dt_s = MOTION_DT_MS / 1000.0f;

  const int start_count = s->current_count;
  const int final_target = servo_limit_count(channel, target_count);
  const int delta = final_target - start_count;

  if (delta == 0) {
    return;
  }

  const float direction = (delta > 0) ? 1.0f : -1.0f;
  const float distance = fabsf((float)delta);

  if (max_speed_cps <= 0.0f) max_speed_cps = DEFAULT_MAX_SPEED_CPS;
  if (accel_cps2 <= 0.0f) accel_cps2 = DEFAULT_ACCEL_CPS2;

  float t_accel = max_speed_cps / accel_cps2;
  float d_accel = 0.5f * accel_cps2 * t_accel * t_accel;

  float t_cruise = 0.0f;
  float total_time = 0.0f;
  float peak_speed = max_speed_cps;

  if (2.0f * d_accel >= distance) {
    t_accel = sqrtf(distance / accel_cps2);
    d_accel = 0.5f * accel_cps2 * t_accel * t_accel;
    t_cruise = 0.0f;
    peak_speed = accel_cps2 * t_accel;
    total_time = 2.0f * t_accel;
  } else {
    float d_cruise = distance - 2.0f * d_accel;
    t_cruise = d_cruise / max_speed_cps;
    total_time = 2.0f * t_accel + t_cruise;
  }

  snprintf(msg, sizeof(msg), "\r\n%s: %d -> %d\r\n", s->name, start_count,
           final_target);
  print_msg(msg);

  float t = 0.0f;
  int last_sent = start_count;

  while (t < total_time) {
    float pos = 0.0f;

    if (t < t_accel) {
      pos = 0.5f * accel_cps2 * t * t;
    } else if (t < (t_accel + t_cruise)) {
      float tc = t - t_accel;
      pos = d_accel + peak_speed * tc;
    } else {
      float td = t - t_accel - t_cruise;
      pos = d_accel + peak_speed * t_cruise + peak_speed * td -
            0.5f * accel_cps2 * td * td;
    }

    if (pos > distance) pos = distance;

    int desired_count = start_count + (int)lroundf(direction * pos);
    desired_count = servo_limit_count(channel, desired_count);

    if (desired_count != last_sent) {
      if (servo_write_count(channel, desired_count) != HAL_OK) {
        print_status_and_halt("ERROR: PCA9685 write failed during motion\r\n");
      }
      last_sent = desired_count;
    }

    HAL_Delay((uint32_t)MOTION_DT_MS);
    t += dt_s;
  }

  if (servo_write_count(channel, final_target) != HAL_OK) {
    print_status_and_halt("ERROR: PCA9685 final write failed\r\n");
  }

  s->current_count = final_target;
}

void servo_write_all_centers(void) {
  for (int i = 0; i < 4; i++) {
    servos[i].current_count = servos[i].center_count;
    if (servo_write_count(i, servos[i].center_count) != HAL_OK) {
      print_status_and_halt("ERROR: Failed to write center position\r\n");
    }
  }
}

/* Piecewise linear logical angle -> count.
   center count = 0 deg
   min side = negative angle
   max side = positive angle */
int logical_angle_deg_to_count(int channel, float logical_angle_deg) {
  ServoConfig* s = &servos[channel];
  float min_deg = 0.0f;
  float max_deg = 0.0f;
  float count_f = (float)s->center_count;

  if (channel == BASE_SERVO_MOTOR) {
    min_deg = BASE_MIN_DEG;
    max_deg = BASE_MAX_DEG;
  } else if (channel == ARM_SERVO_MOTOR1) {
    min_deg = SHOULDER_MIN_DEG;
    max_deg = SHOULDER_MAX_DEG;
  } else if (channel == ARM_SERVO_MOTOR2) {
    min_deg = ELBOW_MIN_DEG;
    max_deg = ELBOW_MAX_DEG;
  } else {
    return s->center_count;
  }

  if (logical_angle_deg >= 0.0f) {
    float a = clamp_float(logical_angle_deg, 0.0f, max_deg);
    float t = (max_deg == 0.0f) ? 0.0f : (a / max_deg);
    count_f = (float)s->center_count +
              t * ((float)s->max_count - (float)s->center_count);
  } else {
    float a = clamp_float(logical_angle_deg, min_deg, 0.0f);
    float t = (min_deg == 0.0f) ? 0.0f : (a / min_deg);
    count_f = (float)s->center_count +
              t * ((float)s->min_count - (float)s->center_count);
  }

  return servo_limit_count(channel, (int)lroundf(count_f));
}

float count_to_logical_angle_deg(int channel, int count) {
  ServoConfig* s = &servos[channel];
  int limited = servo_limit_count(channel, count);

  if (limited >= s->center_count) {
    float t = ((float)limited - (float)s->center_count) /
              ((float)s->max_count - (float)s->center_count);
    if (channel == BASE_SERVO_MOTOR) return t * BASE_MAX_DEG;
    if (channel == ARM_SERVO_MOTOR1) return t * SHOULDER_MAX_DEG;
    if (channel == ARM_SERVO_MOTOR2) return t * ELBOW_MAX_DEG;
  } else {
    float t = ((float)limited - (float)s->center_count) /
              ((float)s->center_count - (float)s->min_count);
    if (channel == BASE_SERVO_MOTOR) return t * (-BASE_MIN_DEG);
    if (channel == ARM_SERVO_MOTOR1) return t * (-SHOULDER_MIN_DEG);
    if (channel == ARM_SERVO_MOTOR2) return t * (-ELBOW_MIN_DEG);
  }

  return 0.0f;
}

void move_arm_counts(int base_count, int shoulder_count, int elbow_count) {
  move_servo_smooth_trapezoid(BASE_SERVO_MOTOR, base_count,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(200);

  move_servo_smooth_trapezoid(ARM_SERVO_MOTOR1, shoulder_count,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(200);

  move_servo_smooth_trapezoid(ARM_SERVO_MOTOR2, elbow_count,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(300);
}

void claw_open(void) {
  move_servo_smooth_trapezoid(CLAW_SERVO_MOTOR, CLAW_MAX_COUNT,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(350);
}

void claw_close(void) {
  move_servo_smooth_trapezoid(CLAW_SERVO_MOTOR, CLAW_MIN_COUNT,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(350);
}

/* USER CODE END 0 */

/**

 * @brief The application entry point.

 * @retval int

 */

int main(void)

{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  MX_GPIO_Init();

  MX_I2C2_Init();

  MX_USART3_UART_Init();

  MX_USB_OTG_FS_PCD_Init();

  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */

  HAL_StatusTypeDef status;

  // Optional: slow I2C to 100 kHz first if you want easier bring-up

  // see note below in MX_I2C2_Init()

  print_msg("Starting test\n");

  // 1) Check PCA9685 is present on the bus

  status = PCA9685_IsReady(&hi2c2);

  if (status != HAL_OK) {
    // LD1 = error

    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);

    while (1);
  }

  print_msg("PCA9685 is present on the bus\n");

  // LD3 = found device

  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

  // 2) Initialize PCA9685 registers

  status = PCA9685_Init(&hi2c2);

  if (status != HAL_OK) {
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);

    while (1);
  }

  print_msg("PCA9685 registers success\n");

  // 3) Set PWM frequency to 50 Hz for servos

  status = PCA9685_SetPWMFreq(&hi2c2, 50.0f);

  if (status != HAL_OK) {
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);

    while (1);
  }

  print_msg("PCA9685 PWM frequency set to 50 Hz for servos\r\n");

  // To track current counts for each motor so we can do smooth transitions
  int current_counts[4] = {BASE_CENTER_COUNT, ARM1_CENTER_COUNT,
                           ARM2_CENTER_COUNT, CLAW_CENTER_COUNT};

  // Step 0: Ensure we start perfectly at center instantly so we have a known
  // baseline

  PCA9685_SetServoPulseCounts(&hi2c2, 0, current_counts[0]);

  PCA9685_SetServoPulseCounts(&hi2c2, 1, current_counts[1]);

  PCA9685_SetServoPulseCounts(&hi2c2, 2, current_counts[2]);

  PCA9685_SetServoPulseCounts(&hi2c2, 3, current_counts[3]);

  print_msg("Servos locked securely at neutral position\r\n");

  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */

  /* USER CODE BEGIN WHILE */

  JoystickState joy1 = {0};
  JoystickState joy2 = {0};  // after testing, joy2 x and y should be flipped in
                             // usage to match expected directions

  while (1)

  {
    // // test elbow motor again
    // move_servo_smooth_trapezoid(ARM_SERVO_MOTOR2, ARM2_MAX_COUNT,
    //                             DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
    // HAL_Delay(1000);
    // move_servo_smooth_trapezoid(ARM_SERVO_MOTOR2, ARM2_MIN_COUNT,
    //                             DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
    // HAL_Delay(1000);
    // move_servo_smooth_trapezoid(ARM_SERVO_MOTOR2, ARM2_CENTER_COUNT,
    //                             DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
    // HAL_Delay(1000);

    // Manual mode (joystick controls motors directly)

    // 1. Read joystick states
    Joystick_Read(&joy1, J1_X_CHANNEL, J1_Y_CHANNEL, J1_SW_GPIO_Port,
                  J1_SW_Pin);
    Joystick_Read(&joy2, J2_X_CHANNEL, J2_Y_CHANNEL, J2_SW_GPIO_Port,
                  J2_SW_Pin);

    //    Joystick_Print(&joy1, &joy2);

    // 2. Map joystick inputs to servo counts

    // base and shoulder on joy1, elbow and claw on joy2

    // for now if X > 0, go towards max count, if X < 0 go towards min count,
    // if Y > 0 go towards max count, if Y < 0 go towards min count
    int step_size =
        70;  // counts to move per joystick press - adjust for sensitivity

    // scale step by joystick deflection
    int base_step =
        step_size * (float)abs(joy1.x_norm) / (float)(JOY_MAX - JOY_DEADZONE);
    int shoulder_step =
        step_size * (float)abs(joy1.y_norm) / (float)(JOY_MAX - JOY_DEADZONE);
    int elbow_step =
        step_size * (float)abs(joy2.x_norm) / (float)(JOY_MAX - JOY_DEADZONE);
    int claw_step =
        step_size * (float)abs(joy2.y_norm) / (float)(JOY_MAX - JOY_DEADZONE);

    // print base step and current count for debugging
    //    char msg[100];
    //    snprintf(msg, sizeof(msg), "Base step: %d, Current count: %d\r\n",
    //             base_step, servos[BASE_SERVO_MOTOR].current_count);
    //    print_msg(msg);

    if (joy1.x_norm > JOY_DEADZONE) {
      //   servos[BASE_SERVO_MOTOR].current_count += base_step;
      move_servo_smooth_trapezoid(
          BASE_SERVO_MOTOR, servos[BASE_SERVO_MOTOR].current_count + base_step,
          DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
    } else if (joy1.x_norm < -JOY_DEADZONE) {
      //   servos[BASE_SERVO_MOTOR].current_count -= base_step;
      move_servo_smooth_trapezoid(
          BASE_SERVO_MOTOR, servos[BASE_SERVO_MOTOR].current_count - base_step,
          DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
    }

    if (joy1.y_norm > JOY_DEADZONE) {
      //   servos[ARM_SERVO_MOTOR1].current_count += shoulder_step;
      move_servo_smooth_trapezoid(
          ARM_SERVO_MOTOR1,
          servos[ARM_SERVO_MOTOR1].current_count + shoulder_step,
          DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
    } else if (joy1.y_norm < -JOY_DEADZONE) {
      //   servos[ARM_SERVO_MOTOR1].current_count -= shoulder_step;
      move_servo_smooth_trapezoid(
          ARM_SERVO_MOTOR1,
          servos[ARM_SERVO_MOTOR1].current_count - shoulder_step,
          DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
    }

    if (joy2.x_norm > JOY_DEADZONE) {
      //   servos[ARM_SERVO_MOTOR2].current_count += elbow_step;
      move_servo_smooth_trapezoid(
          ARM_SERVO_MOTOR2, servos[ARM_SERVO_MOTOR2].current_count + elbow_step,
          DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
    } else if (joy2.x_norm < -JOY_DEADZONE) {
      //   servos[ARM_SERVO_MOTOR2].current_count -= elbow_step;
      move_servo_smooth_trapezoid(
          ARM_SERVO_MOTOR2, servos[ARM_SERVO_MOTOR2].current_count - elbow_step,
          DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
    }

    if (joy2.y_norm > JOY_DEADZONE) {
      //   servos[CLAW_SERVO_MOTOR].current_count += claw_step;
      move_servo_smooth_trapezoid(
          CLAW_SERVO_MOTOR, servos[CLAW_SERVO_MOTOR].current_count + claw_step,
          DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
    } else if (joy2.y_norm < -JOY_DEADZONE) {
      //   servos[CLAW_SERVO_MOTOR].current_count -= claw_step;
      move_servo_smooth_trapezoid(
          CLAW_SERVO_MOTOR, servos[CLAW_SERVO_MOTOR].current_count - claw_step,
          DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
    }

    // HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage

  */

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters

  * in the RCC_OscInitTypeDef structure.

  */

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;

  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 4;

  RCC_OscInitStruct.PLL.PLLN = 168;

  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;

  RCC_OscInitStruct.PLL.PLLQ = 7;

  RCC_OscInitStruct.PLL.PLLR = 2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)

  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks

  */

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK

                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;

  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;

  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)

  {
    Error_Handler();
  }
}

/**

 * @brief I2C2 Initialization Function

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

  hi2c2.Init.ClockSpeed = 400000;

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

 * @brief USART3 Initialization Function

 * @param None

 * @retval None

 */

static void MX_USART3_UART_Init(void)

{
  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */

  huart3.Instance = USART3;

  huart3.Init.BaudRate = 115200;

  huart3.Init.WordLength = UART_WORDLENGTH_8B;

  huart3.Init.StopBits = UART_STOPBITS_1;

  huart3.Init.Parity = UART_PARITY_NONE;

  huart3.Init.Mode = UART_MODE_TX_RX;

  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;

  huart3.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart3) != HAL_OK)

  {
    Error_Handler();
  }

  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIG_EDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;

  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

/**

 * @brief USB_OTG_FS Initialization Function

 * @param None

 * @retval None

 */

static void MX_USB_OTG_FS_PCD_Init(void)

{
  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */

  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;

  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;

  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;

  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;

  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;

  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;

  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;

  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;

  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;

  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;

  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)

  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */
}

/**

 * @brief GPIO Initialization Function

 * @param None

 * @retval None

 */

static void MX_GPIO_Init(void)

{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */

  __HAL_RCC_GPIOC_CLK_ENABLE();

  __HAL_RCC_GPIOF_CLK_ENABLE();

  __HAL_RCC_GPIOH_CLK_ENABLE();

  __HAL_RCC_GPIOB_CLK_ENABLE();

  __HAL_RCC_GPIOD_CLK_ENABLE();

  __HAL_RCC_GPIOG_CLK_ENABLE();

  __HAL_RCC_GPIOA_CLK_ENABLE();

  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */

  HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */

  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */

  GPIO_InitStruct.Pin = USER_Btn_Pin;

  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */

  GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */

  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */

  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : J1_SW_Pin J2_SW_Pin */
  GPIO_InitStruct.Pin = J1_SW_Pin | J2_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**

 * @brief This function is executed in case of error occurrence.

 * @retval None

 */

void Error_Handler(void)

{
  /* USER CODE BEGIN Error_Handler_Debug */

  /* User can add his own implementation to report the HAL error return state */

  __disable_irq();

  while (1)

  {
  }

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**

 * @brief Reports the name of the source file and the source line number

 * where the assert_param error has occurred.

 * @param file: pointer to the source file name

 * @param line: assert_param error line source number

 * @retval None

 */

void assert_failed(uint8_t* file, uint32_t line)

{
  /* USER CODE BEGIN 6 */

  /* User can add his own implementation to report the file name and line
  number,

  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */
