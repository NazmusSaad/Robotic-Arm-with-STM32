/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : IK pick-and-place demo using PCA9685
  ******************************************************************************
  * @attention
  *
  * First-pass assumptions:
  * - Workspace frame is from robot perspective:
  *     origin = paper corner closest to robot
  *     +x = toward left edge of paper
  *     +y = toward top edge of paper
  * - Paper plane is z = 0
  * - Neutral servo counts correspond to logical angle 0
  * - Min side corresponds to negative angle
  * - Max side corresponds to positive angle
  * - Linear count <-> angle mapping
  *
  * Joint mapping:
  * - Channel 0 = Base
  * - Channel 1 = Shoulder
  * - Channel 2 = Elbow
  * - Channel 3 = Claw
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pca9685.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  int current_count;
  int min_count;
  int center_count;
  int max_count;
  uint8_t reversed;
  const char *name;
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

typedef struct {
  int shoulder_count;
  int elbow_count;
  float x_cm;
  float y_cm;
  float z_cm;
} EmpiricalArmPose;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Servo channels on PCA9685 */
#define CLAW_OPEN_HOLD_COUNT   410
#define CLAW_CLOSE_COUNT       320


#define BASE_SERVO_MOTOR   0
#define ARM_SERVO_MOTOR1   1   /* Shoulder */
#define ARM_SERVO_MOTOR2   2   /* Elbow */
#define CLAW_SERVO_MOTOR   3

/* Final ROM values */
#define BASE_MIN_COUNT     200
#define BASE_CENTER_COUNT  450
#define BASE_MAX_COUNT     620

#define ARM1_MIN_COUNT     220
#define ARM1_CENTER_COUNT  410
#define ARM1_MAX_COUNT     600

#define ARM2_MIN_COUNT     220
#define ARM2_CENTER_COUNT  410
#define ARM2_MAX_COUNT     620

#define CLAW_MIN_COUNT     290   /* close */
#define CLAW_CENTER_COUNT  410
#define CLAW_MAX_COUNT     470   /* open */

/* Smooth motion timing */
#define MOTION_DT_MS               20.0f
#define DEFAULT_MAX_SPEED_CPS      120.0f
#define DEFAULT_ACCEL_CPS2         240.0f

/* Pause times */
#define SHORT_SETTLE_MS            600
#define LONG_SETTLE_MS             1200

/* Workspace / geometry */
#define PAPER_WIDTH_CM             25.0f
#define PAPER_HEIGHT_CM            20.0f

/* Robot base axis in workspace coordinates */
#define BASE_X_CM                  9.2f
#define BASE_Y_CM                 -4.0f
#define BASE_Z_CM                   3.0f

/* Shoulder pivot height above paper */
#define SHOULDER_Z_CM             13.0f

/* Effective link lengths */
#define LINK1_CM                   8.3f
#define LINK2_CM                  13.0f

/* Hardcoded test points */
#define PICK_X_CM                  3.0f
#define PICK_Y_CM                  3.0f
#define DROP_X_CM                 21.0f
#define DROP_Y_CM                  3.0f

/* Heights above paper */
#define APPROACH_Z_CM              3.0f
#define GRASP_Z_CM                 0.5f

/* First-pass logical angle spans */
#define BASE_MIN_DEG             -90.0f
#define BASE_MAX_DEG              90.0f

#define SHOULDER_MIN_DEG         -70.0f
#define SHOULDER_MAX_DEG          70.0f

#define ELBOW_MIN_DEG            -80.0f
#define ELBOW_MAX_DEG             80.0f

/* Neutral absolute/reference assumptions */
#define BASE_FORWARD_ABS_DEG       0.0f
#define SHOULDER_NEUTRAL_ABS_DEG   18.0f
#define ELBOW_NEUTRAL_REL_DEG     112.0f
#define REACH_MARGIN_CM            0.3f
#define CLAMP_UNREACHABLE_TARGETS  1



/* Slower return-to-home motion */
#define HOME_RETURN_MAX_SPEED_CPS   45.0f
#define HOME_RETURN_ACCEL_CPS2      90.0f

#define BASE_COUNTS_PER_CM_X       9.5f

/* Expanded calibrated XY/Z region from measurements */
#define CAL_Y_MIN_CM               10.7f
#define CAL_Y_MAX_CM               16.7f
#define CAL_Z_MIN_CM                3.0f
#define CAL_Z_MAX_CM                6.5f

#define MANUAL_BASE_COUNT      450
#define MANUAL_SHOULDER_COUNT  280
#define MANUAL_ELBOW_COUNT     575
#define MANUAL_CLAW_COUNT      340 //higher means more open


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart3;
PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
ServoConfig servos[4] = {
    {BASE_CENTER_COUNT, BASE_MIN_COUNT, BASE_CENTER_COUNT, BASE_MAX_COUNT, 0, "Base Servo"},
    {ARM1_CENTER_COUNT, ARM1_MIN_COUNT, ARM1_CENTER_COUNT, ARM1_MAX_COUNT, 0, "Shoulder Servo"},
    {ARM2_CENTER_COUNT, ARM2_MIN_COUNT, ARM2_CENTER_COUNT, ARM2_MAX_COUNT, 0, "Elbow Servo"},
    {CLAW_CENTER_COUNT, CLAW_MIN_COUNT, CLAW_CENTER_COUNT, CLAW_MAX_COUNT, 0, "Claw Servo"}
};

static const EmpiricalArmPose arm_pose_table[] = {
    /* Near / lower-y band */
    {220, 565, 12.5f, 12.9f, 4.2f},
    {220, 570, 11.6f, 13.2f, 3.5f},

    /* Middle-near band */
    {280, 565, 11.4f, 14.2f, 5.5f},
    {280, 575, 11.5f, 14.5f, 4.0f},
    {280, 590, 11.4f, 14.2f, 3.5f},

    /* Best flat mid band */
    {330, 590, 11.9f, 15.5f, 4.5f},
    {330, 610, 12.0f, 15.6f, 4.3f},

    /* Far-center band */
    {380, 600, 11.8f, 16.5f, 4.4f},
    {380, 610, 11.8f, 16.0f, 3.0f}
};

#define ARM_POSE_TABLE_SIZE (sizeof(arm_pose_table) / sizeof(arm_pose_table[0]))
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);

/* USER CODE BEGIN PFP */
void print_msg(const char *msg);
void print_status_and_halt(const char *msg);
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

int solve_ik_workspace(float wx_cm, float wy_cm, float wz_cm,
                       IKAngles *out_angles, uint8_t *was_clamped);
int ik_angles_to_counts(const IKAngles *angles, JointCounts *counts);
int move_to_workspace_point(float wx_cm, float wy_cm, float wz_cm, const char *label);
void perform_pick_and_place_demo(void);

const EmpiricalArmPose* choose_best_arm_pose(float target_y_cm, float target_z_cm, float *out_cost);
int estimate_base_count_from_target_x(float target_x_cm, const EmpiricalArmPose *pose);
int move_to_workspace_point_empirical(float wx_cm, float wy_cm, float wz_cm, const char *label);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void print_msg(const char *msg) {
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
}

void print_status_and_halt(const char *msg) {
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

float deg_to_rad(float deg) {
  return deg * ((float)M_PI / 180.0f);
}

float rad_to_deg(float rad) {
  return rad * (180.0f / (float)M_PI);
}

int servo_limit_count(int channel, int raw_count) {
  return clamp_int(raw_count, servos[channel].min_count, servos[channel].max_count);
}

/* If a servo goes the wrong way on hardware, set reversed = 1 in servos[] */
int servo_apply_direction(int channel, int logical_count) {
  ServoConfig *s = &servos[channel];
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

  ServoConfig *s = &servos[channel];
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

  snprintf(msg, sizeof(msg), "\r\n%s: %d -> %d\r\n", s->name, start_count, final_target);
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
      pos = d_accel + peak_speed * t_cruise +
            peak_speed * td - 0.5f * accel_cps2 * td * td;
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
  ServoConfig *s = &servos[channel];
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
  ServoConfig *s = &servos[channel];
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

/* Solve IK for workspace point (wx, wy, wz), cm */
int solve_ik_workspace(float wx_cm, float wy_cm, float wz_cm,
                       IKAngles *out_angles, uint8_t *was_clamped) {
  if (out_angles == NULL) return 0;
  if (was_clamped) *was_clamped = 0;

  /* Relative to base axis projection */
  float x_rel = wx_cm - BASE_X_CM;
  float y_rel = wy_cm - BASE_Y_CM;

  /* Neutral base points straight toward paper, i.e. +y */
  float base_deg = rad_to_deg(atan2f(x_rel, y_rel));

  /* Shoulder plane coordinates */
  float r = sqrtf(x_rel * x_rel + y_rel * y_rel);
  float z = wz_cm - SHOULDER_Z_CM;

  float d = sqrtf(r * r + z * z);
  float max_reach = (LINK1_CM + LINK2_CM) - REACH_MARGIN_CM;
  float min_reach = fabsf(LINK1_CM - LINK2_CM) + REACH_MARGIN_CM;

  if ((d > max_reach) || (d < min_reach)) {
#if CLAMP_UNREACHABLE_TARGETS
    float d_safe = clamp_float(d, min_reach, max_reach);
    if (d > 1e-6f) {
      float scale = d_safe / d;
      r *= scale;
      z *= scale;
    } else {
      r = min_reach;
      z = 0.0f;
    }
    if (was_clamped) *was_clamped = 1;
#else
    return 0;
#endif
  }

  float D = (r * r + z * z - LINK1_CM * LINK1_CM - LINK2_CM * LINK2_CM) /
            (2.0f * LINK1_CM * LINK2_CM);
  D = clamp_float(D, -1.0f, 1.0f);

  /* Geometric elbow bend */
  float beta_rad = acosf(D);
  float beta_deg = rad_to_deg(beta_rad);

  /* Geometric shoulder absolute angle relative to +r axis */
  float phi_rad = atan2f(z, r) -
                  atan2f(LINK2_CM * sinf(beta_rad),
                         LINK1_CM + LINK2_CM * cosf(beta_rad));
  float phi_deg = rad_to_deg(phi_rad);

  /* Convert geometric angles to logical angles relative to neutral counts */
  out_angles->base_deg = base_deg - BASE_FORWARD_ABS_DEG;
  out_angles->shoulder_deg = phi_deg - SHOULDER_NEUTRAL_ABS_DEG;

  /* Positive logical elbow = extend outward. Geometric beta gets smaller when extending. */
  out_angles->elbow_deg = ELBOW_NEUTRAL_REL_DEG - beta_deg;

  return 1;
}

int ik_angles_to_counts(const IKAngles *angles, JointCounts *counts) {
  if ((angles == NULL) || (counts == NULL)) return 0;

  counts->base_count = logical_angle_deg_to_count(BASE_SERVO_MOTOR, angles->base_deg);
  counts->shoulder_count = logical_angle_deg_to_count(ARM_SERVO_MOTOR1, angles->shoulder_deg);
  counts->elbow_count = logical_angle_deg_to_count(ARM_SERVO_MOTOR2, angles->elbow_deg);

  return 1;
}

int move_to_workspace_point(float wx_cm, float wy_cm, float wz_cm, const char *label) {
  char msg[256];
  IKAngles angles;
  JointCounts counts;
  uint8_t was_clamped = 0;

  snprintf(msg, sizeof(msg),
           "\r\nTarget %s: W=(%.2f, %.2f, %.2f cm)\r\n",
           label, wx_cm, wy_cm, wz_cm);
  print_msg(msg);

  if (!solve_ik_workspace(wx_cm, wy_cm, wz_cm, &angles, &was_clamped)) {
    print_msg("IK solve failed\r\n");
    return 0;
  }

  if (!ik_angles_to_counts(&angles, &counts)) {
    print_msg("IK angle->count conversion failed\r\n");
    return 0;
  }

  snprintf(msg, sizeof(msg),
           "IK angles (deg): base=%.2f shoulder=%.2f elbow=%.2f%s\r\n",
           angles.base_deg, angles.shoulder_deg, angles.elbow_deg,
           was_clamped ? " [CLAMPED]" : "");
  print_msg(msg);

  snprintf(msg, sizeof(msg),
           "Counts: base=%d shoulder=%d elbow=%d\r\n",
           counts.base_count, counts.shoulder_count, counts.elbow_count);
  print_msg(msg);

  move_arm_counts(counts.base_count, counts.shoulder_count, counts.elbow_count);
  HAL_Delay(400);

  return 1;
}
//void perform_pick_and_place_demo(void) {
//  print_msg("\r\n===== EMPIRICAL CALIBRATION-BASED MOVE TEST =====\r\n");
//
//  servo_write_all_centers();
//  HAL_Delay(1500);
//
//  move_servo_smooth_trapezoid(CLAW_SERVO_MOTOR, EMPIRICAL_CLAW_COUNT,
//                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
//  HAL_Delay(400);
//
//  /* Try targets that are actually inside your calibrated region first. */
//
//  if (!move_to_workspace_point_empirical(11.5f, 13.0f, 4.5f, "test 1")) {
//    print_msg("Empirical move failed\r\n");
//    return;
//  }
//  HAL_Delay(2000);
//
//  if (!move_to_workspace_point_empirical(7.0f, 15.0f, 4.5f, "test 2")) {
//    print_msg("Empirical move failed\r\n");
//    return;
//  }
//  HAL_Delay(2000);
//
//  if (!move_to_workspace_point_empirical(16.0f, 15.0f, 4.5f, "test 3")) {
//    print_msg("Empirical move failed\r\n");
//    return;
//  }
//  HAL_Delay(2000);
//
//  print_msg("\r\nHolding final empirical pose...\r\n");
//}

void perform_pick_and_place_demo(void) {
  print_msg("\r\n===== EMPIRICAL PICK TEST =====\r\n");

  servo_write_all_centers();
  HAL_Delay(1500);

  /* Start open */
  move_servo_smooth_trapezoid(CLAW_SERVO_MOTOR, CLAW_OPEN_HOLD_COUNT,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(400);

  /* Move to one target point using empirical solver */
  if (!move_to_workspace_point_empirical(4.0f, 12.0f, 4.5f, "pickup target")) {
    print_msg("Empirical move failed\r\n");
  }
  move_servo_smooth_trapezoid(CLAW_SERVO_MOTOR, CLAW_CLOSE_COUNT,
                                DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(5000);

  if (!move_to_workspace_point_empirical(7.0f, 15.0f, 4.5f, "pickup target")) {
      print_msg("Empirical move failed\r\n");
    }
  move_servo_smooth_trapezoid(CLAW_SERVO_MOTOR, CLAW_CLOSE_COUNT,
                                DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
    HAL_Delay(5000);

    if (!move_to_workspace_point_empirical(3.5f, 11.0f, 4.5f, "pickup target")) {
        print_msg("Empirical move failed\r\n");
      }
    move_servo_smooth_trapezoid(CLAW_SERVO_MOTOR, CLAW_CLOSE_COUNT,
                                  DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
      HAL_Delay(5000);

      if (!move_to_workspace_point_empirical(20.5f, 12.0f, 4.5f, "pickup target")) {
          print_msg("Empirical move failed\r\n");
        }
      move_servo_smooth_trapezoid(CLAW_SERVO_MOTOR, CLAW_CLOSE_COUNT,
                                    DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
        HAL_Delay(5000);

  /* Close only after arm has fully moved */
  HAL_Delay(500);

  print_msg("\r\nHolding final grasp pose...\r\n");
}


void manual_touch_pose_test(void) {
  print_msg("\r\n===== MANUAL TOUCH POSE TEST =====\r\n");

  /* Optional: comment this out if you do not want auto-centering first */
  // servo_write_all_centers();
  // HAL_Delay(1000);

  /* IMPORTANT:
     We assume servos[].current_count already roughly matches the arm's
     physical current pose. If startup jerk is a problem, do NOT call
     servo_write_all_centers() before this.
  */

  move_servo_smooth_trapezoid(BASE_SERVO_MOTOR, MANUAL_BASE_COUNT,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(300);

  move_servo_smooth_trapezoid(ARM_SERVO_MOTOR1, MANUAL_SHOULDER_COUNT,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(300);

  move_servo_smooth_trapezoid(ARM_SERVO_MOTOR2, MANUAL_ELBOW_COUNT,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(300);

  move_servo_smooth_trapezoid(CLAW_SERVO_MOTOR, MANUAL_CLAW_COUNT,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(300);

  char msg[200];
  snprintf(msg, sizeof(msg),
           "Holding manual pose: base=%d shoulder=%d elbow=%d claw=%d\r\n",
           MANUAL_BASE_COUNT, MANUAL_SHOULDER_COUNT,
           MANUAL_ELBOW_COUNT, MANUAL_CLAW_COUNT);
  print_msg(msg);
}


static float sqrf(float x) {
  return x * x;
}

const EmpiricalArmPose* choose_best_arm_pose(float target_y_cm, float target_z_cm, float *out_cost) {
  const EmpiricalArmPose *best_pose = NULL;
  float best_cost = 1e9f;

  for (uint32_t i = 0; i < ARM_POSE_TABLE_SIZE; i++) {
    const EmpiricalArmPose *p = &arm_pose_table[i];

    float dy = target_y_cm - p->y_cm;
    float dz = target_z_cm - p->z_cm;

    /* Make z matter slightly more now because you want grab height near 4.5 */
    float cost = 1.0f * sqrf(dy) + 1.6f * sqrf(dz);

    if (cost < best_cost) {
      best_cost = cost;
      best_pose = p;
    }
  }

  if (out_cost) {
    *out_cost = best_cost;
  }

  return best_pose;
}

int estimate_base_count_from_target_x(float target_x_cm, const EmpiricalArmPose *pose) {
  if (pose == NULL) {
    return BASE_CENTER_COUNT;
  }

  /* At base=450, this pose was measured at pose->x_cm.
     Shift base count so the arm moves from the measured x to the desired x. */
  float delta_x = target_x_cm - pose->x_cm;
  float raw_base = 450.0f + BASE_COUNTS_PER_CM_X * delta_x;

  int base_count = (int)lroundf(raw_base);

  /* Keep inside safe ROM */
  return servo_limit_count(BASE_SERVO_MOTOR, base_count);
}

int move_to_workspace_point_empirical(float wx_cm, float wy_cm, float wz_cm, const char *label) {
  char msg[256];

  float clamped_y = clamp_float(wy_cm, CAL_Y_MIN_CM, CAL_Y_MAX_CM);
  float clamped_z = clamp_float(wz_cm, CAL_Z_MIN_CM, CAL_Z_MAX_CM);

  float best_cost = 0.0f;
  const EmpiricalArmPose *best_pose = choose_best_arm_pose(clamped_y, clamped_z, &best_cost);
  if (best_pose == NULL) {
    print_msg("Empirical pose lookup failed\r\n");
    return 0;
  }

  int base_count = estimate_base_count_from_target_x(wx_cm, best_pose);

  snprintf(msg, sizeof(msg),
           "\r\nEmpirical target %s: W=(%.2f, %.2f, %.2f)\r\n",
           label, wx_cm, wy_cm, wz_cm);
  print_msg(msg);

  if ((wy_cm != clamped_y) || (wz_cm != clamped_z)) {
    snprintf(msg, sizeof(msg),
             "WARNING: target outside calibrated Y/Z region, using nearest in-range Y/Z = (%.2f, %.2f)\r\n",
             clamped_y, clamped_z);
    print_msg(msg);
  }

  snprintf(msg, sizeof(msg),
           "Chosen arm pose: shoulder=%d elbow=%d -> approx actual (x=%.2f, y=%.2f, z=%.2f), cost=%.2f\r\n",
           best_pose->shoulder_count, best_pose->elbow_count,
           best_pose->x_cm, best_pose->y_cm, best_pose->z_cm, best_cost);
  print_msg(msg);

  snprintf(msg, sizeof(msg),
           "Estimated counts: base=%d shoulder=%d elbow=%d, claw(open)=%d\r\n",
           base_count, best_pose->shoulder_count, best_pose->elbow_count, CLAW_OPEN_HOLD_COUNT);
  print_msg(msg);

  /* KEEP CLAW OPEN FIRST */
  move_servo_smooth_trapezoid(CLAW_SERVO_MOTOR, CLAW_OPEN_HOLD_COUNT,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(250);

  /* MOVE ALL OTHER JOINTS */
  move_arm_counts(base_count, best_pose->shoulder_count, best_pose->elbow_count);
  HAL_Delay(400);

  return 1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();

  /* USER CODE BEGIN 2 */
  HAL_StatusTypeDef status;

  print_msg("\r\nStarting IK pick-and-place demo...\r\n");

  status = PCA9685_IsReady(&hi2c2);
  if (status != HAL_OK) {
    print_status_and_halt("ERROR: PCA9685 not found on I2C bus\r\n");
  }

  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  print_msg("PCA9685 found on I2C bus\r\n");

  status = PCA9685_Init(&hi2c2);
  if (status != HAL_OK) {
    print_status_and_halt("ERROR: PCA9685 init failed\r\n");
  }
  print_msg("PCA9685 init OK\r\n");

  status = PCA9685_SetPWMFreq(&hi2c2, 50.0f);
  if (status != HAL_OK) {
    print_status_and_halt("ERROR: PCA9685 PWM frequency setup failed\r\n");
  }
  print_msg("PCA9685 PWM set to 50 Hz\r\n");

  servo_write_all_centers();
  print_msg("All servos moved to center\r\n");
  HAL_Delay(1500);

  perform_pick_and_place_demo();
  //manual_touch_pose_test();


  print_msg("\r\nDemo complete. Holding position.\r\n");
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  while (1) {
    HAL_Delay(1000);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
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

  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
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
  }
}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
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

  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

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
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
}
#endif /* USE_FULL_ASSERT */








