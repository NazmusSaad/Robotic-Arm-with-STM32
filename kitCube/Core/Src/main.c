/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file : main.c
 * @brief : Merged manual joystick + UART CV auto pick/place controller
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pca9685.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  int current_count;
  int min_count;
  int center_count;
  int max_count;
  uint8_t reversed;
  const char* name;
} ServoConfig;

typedef struct {
  int shoulder_count;
  int elbow_count;
  float x_cm;
  float y_cm;
  float z_cm;
} EmpiricalArmPose;

typedef struct {
  uint16_t x_raw;
  uint16_t y_raw;
  int x_norm;
  int y_norm;
  uint8_t button;
} JoystickState;

typedef struct {
  float x_cm;
  float y_cm;
  float z_cm;
  uint8_t valid;
} CartesianTarget;

typedef enum { ROBOT_MODE_MANUAL = 0, ROBOT_MODE_AUTO = 1 } RobotMode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BASE_SERVO_MOTOR 0
#define ARM_SERVO_MOTOR1 1
#define ARM_SERVO_MOTOR2 2
#define CLAW_SERVO_MOTOR 3

#define BASE_MIN_COUNT 200
#define BASE_CENTER_COUNT 450
#define BASE_MAX_COUNT 620

#define ARM1_MIN_COUNT 220
#define ARM1_CENTER_COUNT 410
#define ARM1_MAX_COUNT 600

#define ARM2_MIN_COUNT 220
#define ARM2_CENTER_COUNT 410
#define ARM2_MAX_COUNT 620

#define CLAW_MIN_COUNT 290
#define CLAW_CENTER_COUNT 410
#define CLAW_MAX_COUNT 470

#define CLAW_OPEN_HOLD_COUNT 410
#define CLAW_CLOSE_COUNT 280

#define HOME_BASE_COUNT BASE_CENTER_COUNT
#define HOME_SHOULDER_COUNT ARM1_CENTER_COUNT
#define HOME_ELBOW_COUNT ARM2_CENTER_COUNT
#define HOME_CLAW_COUNT CLAW_OPEN_HOLD_COUNT

#define MOTION_DT_MS 20.0f
#define DEFAULT_MAX_SPEED_CPS 120.0f
#define DEFAULT_ACCEL_CPS2 240.0f

#define HOME_RETURN_MAX_SPEED_CPS 45.0f
#define HOME_RETURN_ACCEL_CPS2 90.0f

#define AUTO_WAIT_BETWEEN_CYCLES_MS 5000U
#define AUTO_LIFT_DELTA_Z_CM 1.5f
#define AUTO_DROP_X_CM 20.0f
#define AUTO_DROP_Y_CM 13.0f
#define AUTO_DROP_Z_CM 5.0f

#define BASE_COUNTS_PER_CM_X 9.5f
#define CAL_Y_MIN_CM 10.7f
#define CAL_Y_MAX_CM 16.7f
#define CAL_Z_MIN_CM 3.0f
#define CAL_Z_MAX_CM 6.5f

#define J1_X_CHANNEL ADC_CHANNEL_0
#define J1_Y_CHANNEL ADC_CHANNEL_1
#define J2_X_CHANNEL ADC_CHANNEL_4
#define J2_Y_CHANNEL ADC_CHANNEL_5

#define JOY_CENTER 2048
#define JOY_DEADZONE 180
#define JOY_MAX 4095

#define JOYSTICK_STEP_SIZE_COUNTS 70

/* UART Interrupt RX Buffer Size */
#define RX_BUFFER_SIZE 64

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart3;
ADC_HandleTypeDef hadc1;
PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
static ServoConfig servos[4] = {
    {BASE_CENTER_COUNT, BASE_MIN_COUNT, BASE_CENTER_COUNT, BASE_MAX_COUNT, 0,
     "Base Servo"},
    {ARM1_CENTER_COUNT, ARM1_MIN_COUNT, ARM1_CENTER_COUNT, ARM1_MAX_COUNT, 0,
     "Shoulder Servo"},
    {ARM2_CENTER_COUNT, ARM2_MIN_COUNT, ARM2_CENTER_COUNT, ARM2_MAX_COUNT, 0,
     "Elbow Servo"},
    {CLAW_CENTER_COUNT, CLAW_MIN_COUNT, CLAW_CENTER_COUNT, CLAW_MAX_COUNT, 0,
     "Claw Servo"}};

static const EmpiricalArmPose arm_pose_table[] = {
    {220, 565, 12.5f, 12.9f, 4.2f}, {220, 570, 11.6f, 13.2f, 3.5f},
    {280, 565, 11.4f, 14.2f, 5.5f}, {280, 575, 11.5f, 14.5f, 4.0f},
    {280, 590, 11.4f, 14.2f, 3.5f}, {330, 590, 11.9f, 15.5f, 4.5f},
    {330, 610, 12.0f, 15.6f, 4.3f}, {380, 600, 11.8f, 16.5f, 4.4f},
    {380, 610, 11.8f, 16.0f, 3.0f}};
#define ARM_POSE_TABLE_SIZE (sizeof(arm_pose_table) / sizeof(arm_pose_table[0]))

static volatile uint8_t user_button_event = 0;
static volatile uint8_t auto_abort_requested = 0;

static RobotMode g_mode = ROBOT_MODE_MANUAL;
static CartesianTarget g_latest_target = {0.0f, 0.0f, 0.0f, 0};

/* --- UART Interrupt Variables --- */
char rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;
uint8_t rx_data;

volatile uint8_t new_cv_data_ready = 0;
volatile uint8_t cv_parse_error = 0;
float latest_cv_x = 0.0f, latest_cv_y = 0.0f, latest_cv_z = 0.0f;
char debug_raw_rx_string[RX_BUFFER_SIZE];  // Used to print what the MCU saw
/* -------------------------------- */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
void print_msg(const char* msg);
void print_status_and_halt(const char* msg);

int clamp_int(int value, int min_value, int max_value);
float clamp_float(float value, float min_value, float max_value);

int servo_limit_count(int channel, int raw_count);
int servo_apply_direction(int channel, int logical_count);
HAL_StatusTypeDef servo_write_count(int channel, int logical_count);

void write_home_instant(void);
void move_servo_smooth_trapezoid(int channel, int target_count,
                                 float max_speed_cps, float accel_cps2);
void move_arm_counts(int base_count, int shoulder_count, int elbow_count,
                     float max_speed_cps, float accel_cps2);
void claw_open(void);
void claw_close(void);
void return_home_slow(void);

uint16_t Joystick_ReadADC(uint32_t channel);
int Joystick_Normalize(uint16_t raw);
void Joystick_Read(JoystickState* js, uint32_t x_channel, uint32_t y_channel,
                   GPIO_TypeDef* sw_port, uint16_t sw_pin);
void manual_mode_step(void);

const EmpiricalArmPose* choose_best_arm_pose(float target_y_cm,
                                             float target_z_cm,
                                             float* out_cost);
int estimate_base_count_from_target_x(float target_x_cm,
                                      const EmpiricalArmPose* pose);
int move_to_workspace_point_empirical(float wx_cm, float wy_cm, float wz_cm,
                                      const char* label);

void service_background(void);
void delay_with_service(uint32_t delay_ms);

uint8_t take_user_button_event(void);
void enter_auto_mode(void);
void leave_auto_mode(void);
int auto_run_one_cycle(const CartesianTarget* target);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void print_msg(const char* msg) {
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

int servo_limit_count(int channel, int raw_count) {
  return clamp_int(raw_count, servos[channel].min_count,
                   servos[channel].max_count);
}

int servo_apply_direction(int channel, int logical_count) {
  ServoConfig* s = &servos[channel];

  // First get a "limited" count within the servo's physical range
  int limited = servo_limit_count(channel, logical_count);
  // For non-reversed servos, the logical count maps directly to physical count
  if (!s->reversed) return limited;
  // For reversed servos, we mirror the logical count around the center count
  return s->center_count - (limited - s->center_count);
}

// function that should be used to move the motors in the rest of the code, so that the direction reversal and limits are always applied
HAL_StatusTypeDef servo_write_count(int channel, int logical_count) {
  int physical_count = servo_apply_direction(channel, logical_count);
  return PCA9685_SetServoPulseCounts(&hi2c2, channel, physical_count);
}

/* ---------------- Background & Interrupt Services ---------------- */

void service_background(void) {
  char msg[128];

  // Safely check if the interrupt caught new valid coordinates
  if (new_cv_data_ready) {
    g_latest_target.x_cm = latest_cv_x;
    g_latest_target.y_cm = latest_cv_y;
    g_latest_target.z_cm = latest_cv_z;
    g_latest_target.valid = 1U;

    new_cv_data_ready = 0;  // Clear flag

    snprintf(msg, sizeof(msg),
             "[DEBUG UART] Parsed valid target -> X:%.2f Y:%.2f Z:%.2f\r\n",
             latest_cv_x, latest_cv_y, latest_cv_z);
    print_msg(msg);
  }

  // Safely check if the interrupt caught a string but couldn't parse it
  if (cv_parse_error) {
    cv_parse_error = 0;  // Clear flag
    snprintf(msg, sizeof(msg), "[DEBUG UART] Failed to parse string: '%s'\r\n",
             debug_raw_rx_string);
    print_msg(msg);
  }
}

void delay_with_service(uint32_t delay_ms) {
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < delay_ms) {
    service_background();
    HAL_Delay(1);
  }
}

/* ---------------- Robot Movement Logic ---------------- */

// instantly move all servos to the home position (without any smooth motion) - used for initialization and emergency abort
void write_home_instant(void) {
  servos[BASE_SERVO_MOTOR].current_count = HOME_BASE_COUNT;
  servos[ARM_SERVO_MOTOR1].current_count = HOME_SHOULDER_COUNT;
  servos[ARM_SERVO_MOTOR2].current_count = HOME_ELBOW_COUNT;
  servos[CLAW_SERVO_MOTOR].current_count = HOME_CLAW_COUNT;

  servo_write_count(BASE_SERVO_MOTOR, HOME_BASE_COUNT);
  servo_write_count(ARM_SERVO_MOTOR1, HOME_SHOULDER_COUNT);
  servo_write_count(ARM_SERVO_MOTOR2, HOME_ELBOW_COUNT);
  servo_write_count(CLAW_SERVO_MOTOR, HOME_CLAW_COUNT);
}

void move_servo_smooth_trapezoid(int channel, int target_count,
                                 float max_speed_cps, float accel_cps2) {
  ServoConfig* s = &servos[channel];
  const float dt_s = MOTION_DT_MS / 1000.0f;
  const int start_count = s->current_count;
  const int final_target = servo_limit_count(channel, target_count);
  const int delta = final_target - start_count;

  if (delta == 0) return;

  const float direction = (delta > 0) ? 1.0f : -1.0f;
  const float distance = fabsf((float)delta);

  if (max_speed_cps <= 0.0f) max_speed_cps = DEFAULT_MAX_SPEED_CPS;
  if (accel_cps2 <= 0.0f) accel_cps2 = DEFAULT_ACCEL_CPS2;

  float t_accel = max_speed_cps / accel_cps2;
  float d_accel = 0.5f * accel_cps2 * t_accel * t_accel;
  float t_cruise = 0.0f;
  float total_time = 0.0f;
  float peak_speed = max_speed_cps;

  // Check if we can reach max speed before needing to decelerate
  if (2.0f * d_accel >= distance) {
    t_accel = sqrtf(distance / accel_cps2);
    d_accel = 0.5f * accel_cps2 * t_accel * t_accel;
    peak_speed = accel_cps2 * t_accel;
    total_time = 2.0f * t_accel;
  } else {
    // cruise phase at max speed
    float d_cruise = distance - 2.0f * d_accel;
    t_cruise = d_cruise / max_speed_cps;
    total_time = 2.0f * t_accel + t_cruise;
  }

  float t = 0.0f;
  int last_sent = start_count;

  // loop to update servo position at regular intervals based on trapezoidal trajectory 
  while (t < total_time) {
    if (auto_abort_requested && (g_mode == ROBOT_MODE_AUTO)) {
      s->current_count = last_sent;
      return;
    }

    float pos = 0.0f;
    if (t < t_accel) {
      pos = 0.5f * accel_cps2 * t * t;
    } else if (t < (t_accel + t_cruise)) {
      pos = d_accel + peak_speed * (t - t_accel);
    } else {
      float td = t - t_accel - t_cruise;
      pos = d_accel + peak_speed * t_cruise + peak_speed * td -
            0.5f * accel_cps2 * td * td;
    }

    if (pos > distance) pos = distance;
    int desired_count = start_count + (int)lroundf(direction * pos);
    desired_count = servo_limit_count(channel, desired_count);

    if (desired_count != last_sent) {
      servo_write_count(channel, desired_count);
      last_sent = desired_count;
    }

    // delay for a short interval while allowing background services to run (like checking for auto abort or new CV data) instead of a blind HAL_Delay
    delay_with_service((uint32_t)MOTION_DT_MS);
    t += dt_s;
  }

  servo_write_count(channel, final_target);
  s->current_count = final_target;
}

// moves the arm to the specified servo counts with a smooth trapezoidal velocity
void move_arm_counts(int base_count, int shoulder_count, int elbow_count,
                     float max_speed_cps, float accel_cps2) {
  move_servo_smooth_trapezoid(BASE_SERVO_MOTOR, base_count, max_speed_cps,
                              accel_cps2);
  if (auto_abort_requested && (g_mode == ROBOT_MODE_AUTO)) return;
  delay_with_service(200);

  move_servo_smooth_trapezoid(ARM_SERVO_MOTOR1, shoulder_count, max_speed_cps,
                              accel_cps2);
  if (auto_abort_requested && (g_mode == ROBOT_MODE_AUTO)) return;
  delay_with_service(200);

  move_servo_smooth_trapezoid(ARM_SERVO_MOTOR2, elbow_count, max_speed_cps,
                              accel_cps2);
  delay_with_service(300);
}

void claw_open(void) {
  print_msg("[DEBUG ARM] Opening claw...\r\n");
  move_servo_smooth_trapezoid(CLAW_SERVO_MOTOR, CLAW_OPEN_HOLD_COUNT,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  delay_with_service(350);
}

void claw_close(void) {
  print_msg("[DEBUG ARM] Closing claw...\r\n");
  move_servo_smooth_trapezoid(CLAW_SERVO_MOTOR, CLAW_CLOSE_COUNT,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  delay_with_service(350);
}

void return_home_slow(void) {
  print_msg("[DEBUG ARM] Moving back to home configuration slowly...\r\n");
  move_servo_smooth_trapezoid(CLAW_SERVO_MOTOR, HOME_CLAW_COUNT,
                              HOME_RETURN_MAX_SPEED_CPS,
                              HOME_RETURN_ACCEL_CPS2);
  delay_with_service(200);

  move_arm_counts(HOME_BASE_COUNT, HOME_SHOULDER_COUNT, HOME_ELBOW_COUNT,
                  HOME_RETURN_MAX_SPEED_CPS, HOME_RETURN_ACCEL_CPS2);
  print_msg("[DEBUG ARM] Arrived at home configuration.\r\n");
}

/* ---------------- Joystick Logic ---------------- */

uint16_t Joystick_ReadADC(uint32_t channel) {
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) return 0;
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
    HAL_ADC_Stop(&hadc1);
    return 0;
  }
  uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return value;
}

// Convert raw ADC value to a normalized joystick value with deadzone handling
int Joystick_Normalize(uint16_t raw) {
  int val = (int)raw - JOY_CENTER;
  if (abs(val) < JOY_DEADZONE) return 0;
  return val;
}

void Joystick_Read(JoystickState* js, uint32_t x_channel, uint32_t y_channel,
                   GPIO_TypeDef* sw_port, uint16_t sw_pin) {
  js->x_raw = Joystick_ReadADC(x_channel);
  js->y_raw = Joystick_ReadADC(y_channel);
  js->x_norm = Joystick_Normalize(js->x_raw);
  js->y_norm = Joystick_Normalize(js->y_raw);
  js->button = (HAL_GPIO_ReadPin(sw_port, sw_pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

// Read joystick state and move the arm in manual mode based on joystick input, with smooth motion
void manual_mode_step(void) {
  JoystickState joy1 = {0};
  JoystickState joy2 = {0};

  Joystick_Read(&joy1, J1_X_CHANNEL, J1_Y_CHANNEL, J1_SW_GPIO_Port, J1_SW_Pin);
  Joystick_Read(&joy2, J2_X_CHANNEL, J2_Y_CHANNEL, J2_SW_GPIO_Port, J2_SW_Pin);

  int base_step = (int)(JOYSTICK_STEP_SIZE_COUNTS * (float)abs(joy1.x_norm) /
                        (float)(JOY_MAX - JOY_DEADZONE));
  int shoulder_step =
      (int)(JOYSTICK_STEP_SIZE_COUNTS * (float)abs(joy1.y_norm) /
            (float)(JOY_MAX - JOY_DEADZONE));
  int elbow_step = (int)(JOYSTICK_STEP_SIZE_COUNTS * (float)abs(joy2.x_norm) /
                         (float)(JOY_MAX - JOY_DEADZONE));
  int claw_step = (int)(JOYSTICK_STEP_SIZE_COUNTS * (float)abs(joy2.y_norm) /
                        (float)(JOY_MAX - JOY_DEADZONE));

  if (joy1.x_norm > JOY_DEADZONE) {
    move_servo_smooth_trapezoid(
        BASE_SERVO_MOTOR, servos[BASE_SERVO_MOTOR].current_count + base_step,
        DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  } else if (joy1.x_norm < -JOY_DEADZONE) {
    move_servo_smooth_trapezoid(
        BASE_SERVO_MOTOR, servos[BASE_SERVO_MOTOR].current_count - base_step,
        DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  }

  if (joy1.y_norm > JOY_DEADZONE) {
    move_servo_smooth_trapezoid(
        ARM_SERVO_MOTOR1,
        servos[ARM_SERVO_MOTOR1].current_count + shoulder_step,
        DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  } else if (joy1.y_norm < -JOY_DEADZONE) {
    move_servo_smooth_trapezoid(
        ARM_SERVO_MOTOR1,
        servos[ARM_SERVO_MOTOR1].current_count - shoulder_step,
        DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  }

  if (joy2.x_norm > JOY_DEADZONE) {
    move_servo_smooth_trapezoid(
        ARM_SERVO_MOTOR2, servos[ARM_SERVO_MOTOR2].current_count + elbow_step,
        DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  } else if (joy2.x_norm < -JOY_DEADZONE) {
    move_servo_smooth_trapezoid(
        ARM_SERVO_MOTOR2, servos[ARM_SERVO_MOTOR2].current_count - elbow_step,
        DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  }

  if (joy2.y_norm > JOY_DEADZONE) {
    move_servo_smooth_trapezoid(
        CLAW_SERVO_MOTOR, servos[CLAW_SERVO_MOTOR].current_count + claw_step,
        DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  } else if (joy2.y_norm < -JOY_DEADZONE) {
    move_servo_smooth_trapezoid(
        CLAW_SERVO_MOTOR, servos[CLAW_SERVO_MOTOR].current_count - claw_step,
        DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  }

  delay_with_service(20);
}

/* ---------------- Empirical Move Logic ---------------- */

static float sqrf(float x) { return x * x; }

// Given a target Y and Z in cm, find the best empirical arm pose from the table and return it along with the cost of that pose (lower is better)
const EmpiricalArmPose* choose_best_arm_pose(float target_y_cm,
                                             float target_z_cm,
                                             float* out_cost) {
  const EmpiricalArmPose* best_pose = NULL;
  float best_cost = 1e9f;

  for (uint32_t i = 0; i < ARM_POSE_TABLE_SIZE; i++) {
    const EmpiricalArmPose* p = &arm_pose_table[i];
    float dy = target_y_cm - p->y_cm;
    float dz = target_z_cm - p->z_cm;
    // We can weight Y error more than Z error since Y is generally more critical for successful grasping, and also because our calibration data shows more variance in Y than Z
    float cost = 1.0f * sqrf(dy) + 1.6f * sqrf(dz);

    if (cost < best_cost) {
      best_cost = cost;
      best_pose = p;
    }
  }
  if (out_cost) *out_cost = best_cost;
  return best_pose;
}

int estimate_base_count_from_target_x(float target_x_cm,
                                      const EmpiricalArmPose* pose) {
  if (pose == NULL) return BASE_CENTER_COUNT;
  float delta_x = target_x_cm - pose->x_cm;
  float raw_base = 450.0f + BASE_COUNTS_PER_CM_X * delta_x;
  return servo_limit_count(BASE_SERVO_MOTOR, (int)lroundf(raw_base));
}


int move_to_workspace_point_empirical(float wx_cm, float wy_cm, float wz_cm,
                                      const char* label) {
  char msg[256];

  print_msg("[DEBUG ARM] Calculating empirical trajectory...\r\n");

  float clamped_y = clamp_float(wy_cm, CAL_Y_MIN_CM, CAL_Y_MAX_CM);
  float clamped_z = clamp_float(wz_cm, CAL_Z_MIN_CM, CAL_Z_MAX_CM);

  float best_cost = 0.0f;
  const EmpiricalArmPose* best_pose =
      choose_best_arm_pose(clamped_y, clamped_z, &best_cost);

  if (best_pose == NULL) {
    print_msg(
        "[DEBUG ARM] ERROR: Empirical pose lookup failed. Out of bounds?\r\n");
    return 0;
  }

  int base_count = estimate_base_count_from_target_x(wx_cm, best_pose);

  snprintf(msg, sizeof(msg),
           ">>> Moving to Target '%s': W=(%.2f, %.2f, %.2f)\r\n", label, wx_cm,
           wy_cm, wz_cm);
  print_msg(msg);

  /* Move all non-claw joints (claw forcing logic removed so it stays grasped!)
   */
  move_arm_counts(base_count, best_pose->shoulder_count, best_pose->elbow_count,
                  DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  if (auto_abort_requested && (g_mode == ROBOT_MODE_AUTO)) return 0;
  delay_with_service(400);

  print_msg("[DEBUG ARM] Empirical movement complete.\r\n");
  return 1;
}

/* ---------------- State Control ---------------- */

uint8_t take_user_button_event(void) {
  uint8_t had_event = user_button_event;
  user_button_event = 0U;
  return had_event;
}

void enter_auto_mode(void) {
  auto_abort_requested = 0U;
  g_mode = ROBOT_MODE_AUTO;
  g_latest_target.valid = 0U;  // Clear old targets so we wait for a fresh one
  print_msg("\r\n[DEBUG MAIN] Mode switched to: AUTO MODE\r\n");
  print_msg(
      "[DEBUG MAIN] The robot will now wait for a valid coordinate from "
      "CV.\r\n");
}

void leave_auto_mode(void) {
  auto_abort_requested = 0U;
  print_msg("\r\n[DEBUG MAIN] AUTO MODE ABORTED! Canceling movements...\r\n");
  return_home_slow();
  g_mode = ROBOT_MODE_MANUAL;
  print_msg("\r\n[DEBUG MAIN] Mode switched to: MANUAL MODE\r\n");
}

int auto_run_one_cycle(const CartesianTarget* target) {
  float lift_z = 0.0f;

  if ((target == NULL) || (!target->valid)) return 0;

  print_msg("\r\n[DEBUG AUTO] --- STARTING NEW PICK & PLACE CYCLE ---\r\n");

  /* Ensure claw is open BEFORE approaching the object */
  claw_open();
  if (auto_abort_requested) return 0;

  /* 1. Go to pickup */
  if (!move_to_workspace_point_empirical(target->x_cm, target->y_cm,
                                         target->z_cm, "Pickup Object"))
    return 0;
  if (auto_abort_requested) return 0;

  /* 2. Grab it */
  claw_close();
  if (auto_abort_requested) return 0;

  /* 3. Lift */
  print_msg("[DEBUG AUTO] Lifting object vertically...\r\n");
  lift_z = clamp_float(target->z_cm + AUTO_LIFT_DELTA_Z_CM, CAL_Z_MIN_CM,
                       CAL_Z_MAX_CM);
  if (!move_to_workspace_point_empirical(target->x_cm, target->y_cm, lift_z,
                                         "Lift Object"))
    return 0;
  if (auto_abort_requested) return 0;

  /* 4. Drop Point */
  if (!move_to_workspace_point_empirical(AUTO_DROP_X_CM, AUTO_DROP_Y_CM,
                                         AUTO_DROP_Z_CM, "Drop Zone"))
    return 0;
  if (auto_abort_requested) return 0;

  /* 5. Release */
  claw_open();
  if (auto_abort_requested) return 0;

  /* 6. Return */
  return_home_slow();
  print_msg("[DEBUG AUTO] --- CYCLE FINISHED SUCCESSFULLY ---\r\n");

  return 1;
}

/* --- Hardware Interrupt Callbacks --- */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == USER_Btn_Pin) {
    user_button_event = 1U;
    if (g_mode == ROBOT_MODE_AUTO) {
      auto_abort_requested = 1U;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART3) {
    if (rx_data == '\n') {
      rx_buffer[rx_index] = '\0';

      // Try to parse. Supports "12.3,4.5,6.7" or "X=12.3,Y=4.5,Z=6.7"
      if (sscanf(rx_buffer, "%f,%f,%f", &latest_cv_x, &latest_cv_y,
                 &latest_cv_z) == 3 ||
          sscanf(rx_buffer, "X=%f,Y=%f,Z=%f", &latest_cv_x, &latest_cv_y,
                 &latest_cv_z) == 3) {
        new_cv_data_ready = 1;
      } else {
        // Save the failed string to print it safely later
        strncpy(debug_raw_rx_string, rx_buffer, RX_BUFFER_SIZE - 1);
        cv_parse_error = 1;
      }
      rx_index = 0;
    } else if (rx_data != '\r') {
      if (rx_index < RX_BUFFER_SIZE - 1) {
        rx_buffer[rx_index++] = rx_data;
      }
    }

    // Re-arm interrupt for next byte
    HAL_UART_Receive_IT(&huart3, &rx_data, 1);
  }
}

/* USER CODE END 0 */

int main(void) {
  HAL_StatusTypeDef status;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();

  print_msg("\r\n==========================================\r\n");
  print_msg("[DEBUG MAIN] System Init Started...\r\n");

  status = PCA9685_IsReady(&hi2c2);
  if (status != HAL_OK) {
    print_status_and_halt(
        "[DEBUG MAIN] ERROR: PCA9685 not found on I2C bus\r\n");
  }
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  print_msg("[DEBUG MAIN] PCA9685 found on I2C bus\r\n");

  status = PCA9685_Init(&hi2c2);
  if (status != HAL_OK)
    print_status_and_halt("[DEBUG MAIN] ERROR: PCA9685 init failed\r\n");

  status = PCA9685_SetPWMFreq(&hi2c2, 50.0f);
  if (status != HAL_OK)
    print_status_and_halt("[DEBUG MAIN] ERROR: PWM setup failed\r\n");

  print_msg("[DEBUG MAIN] Moving to home pose instantly...\r\n");
  write_home_instant();
  delay_with_service(1200);

  print_msg("[DEBUG MAIN] --- INIT COMPLETE ---\r\n");
  print_msg("Current Mode: MANUAL (Joystick)\r\n");
  print_msg("Press Blue USER Button to switch to AUTO mode.\r\n");
  print_msg("==========================================\r\n");

  // starting UART interrupt listener here
  HAL_UART_Receive_IT(&huart3, &rx_data, 1);

  uint32_t last_waiting_print = 0;

  while (1) {
    service_background();

    /* Check if User Pressed the Blue Button */
    if (take_user_button_event()) {
      print_msg("\r\n[DEBUG MAIN] Detected Blue USER Button Press!\r\n");
      if (g_mode == ROBOT_MODE_MANUAL) {
        enter_auto_mode();
      } else {
        print_msg(
            "[DEBUG MAIN] Interrupting AUTO sequence. Requesting abort...\r\n");
      }
    }

    /* MANUAL MODE LOOP */
    if (g_mode == ROBOT_MODE_MANUAL) {
      manual_mode_step();
      continue;
    }

    /* AUTO MODE LOOP */
    if (auto_abort_requested) {
      leave_auto_mode();
      continue;
    }

    if (!g_latest_target.valid) {
      // Print a waiting message every 2 seconds so it doesn't flood the
      // terminal
      if (HAL_GetTick() - last_waiting_print > 2000) {
        print_msg(
            "[DEBUG AUTO] Waiting for valid coordinate from Python CV...\r\n");
        last_waiting_print = HAL_GetTick();
      }
      delay_with_service(100);
      continue;
    }

    /* Target is valid, execute Pick and Place */
    {
      CartesianTarget target_snapshot = g_latest_target;
      auto_run_one_cycle(&target_snapshot);
    }

    if (auto_abort_requested) {
      leave_auto_mode();
      continue;
    }

    print_msg(
        "[DEBUG AUTO] Cycle Complete. Waiting 5 seconds before checking next "
        "target...\r\n");
    g_latest_target.valid =
        0;  // Invalidate so it requires a fresh target next cycle
    delay_with_service(AUTO_WAIT_BETWEEN_CYCLES_MS);

    if (auto_abort_requested) {
      leave_auto_mode();
      continue;
    }
  }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    Error_Handler();
}

static void MX_I2C2_Init(void) {
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
}

static void MX_USART3_UART_Init(void) {
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  } /* --- MANUALLY ENABLE USART3 INTERRUPT --- */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(
      USART3_IRQn); /* ---------------------------------------- */
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
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_USB_OTG_FS_PCD_Init(void) {
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
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
                    GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = J1_SW_Pin | J2_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* Blue button EXTI */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {}
#endif /* USE_FULL_ASSERT */
