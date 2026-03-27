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

#define SERVO_LEFT_COUNT 250

#define SERVO_CENTER_COUNT 410

#define SERVO_RIGHT_COUNT 550

#define BASE_SERVO_MOTOR 0

#define ARM_SERVO_MOTOR1 1

#define ARM_SERVO_MOTOR2 2

#define CLAW_SERVO_MOTOR 3

/* USER CODE END PD */

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart3;

ADC_HandleTypeDef hadc1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

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

/* USER CODE BEGIN 0 */

void print_msg(char* msg) {
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
}

// function to get the offcount value for the pwm based on the desired servo
// angle

int angle_to_count(int angle) { return angle * 3.28 + 409; }

int count_to_angle(int count) { return (count - 409) / 3.28; }

// function to move a motor to a specific angle in a smooth manner
void move_motor_to_angle(int motor_channel, int angle, int current_count) {
  int target_count =
      angle * 3.28 +
      409;  // Convert angle to count using the same formula as angle_to_count

  if (target_count < current_count) {
    // Move smoothly downwards

    for (int i = current_count; i >= target_count; i--) {
      PCA9685_SetServoPulseCounts(&hi2c2, motor_channel, i);

      HAL_Delay(25);  // Adjust delay for smoother or faster movement
    }

  } else if (target_count > current_count) {
    // Move smoothly upwards

    for (int i = current_count; i <= target_count; i++) {
      PCA9685_SetServoPulseCounts(&hi2c2, motor_channel, i);

      HAL_Delay(25);  // Adjust delay for smoother or faster movement
    }
  }
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
  int current_counts[4] = {SERVO_CENTER_COUNT + 65, SERVO_CENTER_COUNT,
                           SERVO_CENTER_COUNT, SERVO_CENTER_COUNT};

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
  JoystickState joy2 = {0};

  while (1)

  {
    Joystick_Read(&joy1, J1_X_CHANNEL, J1_Y_CHANNEL, J1_SW_GPIO_Port,
                  J1_SW_Pin);
    Joystick_Read(&joy2, J2_X_CHANNEL, J2_Y_CHANNEL, J2_SW_GPIO_Port,
                  J2_SW_Pin);

    Joystick_Print(&joy1, &joy2);

    HAL_Delay(80000);

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
