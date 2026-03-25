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

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_I2C2_Init(void);

static void MX_USART3_UART_Init(void);

static void MX_USB_OTG_FS_PCD_Init(void);

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
  int current_counts[4] = {SERVO_CENTER_COUNT, SERVO_CENTER_COUNT,
                           SERVO_CENTER_COUNT, SERVO_CENTER_COUNT};

  // Step 0: Ensure we start perfectly at center instantly so we have a known
  // baseline

  PCA9685_SetServoPulseCounts(&hi2c2, current_counts[0], SERVO_CENTER_COUNT);

  PCA9685_SetServoPulseCounts(&hi2c2, current_counts[1], SERVO_CENTER_COUNT);

  PCA9685_SetServoPulseCounts(&hi2c2, current_counts[2], SERVO_CENTER_COUNT);

  PCA9685_SetServoPulseCounts(&hi2c2, current_counts[3], SERVO_CENTER_COUNT);

  print_msg("Servos locked securely at neutral position\r\n");

  HAL_Delay(1000);

  //   // Test Claw motor by opening and closing it a few times
  //   for (int i = 0; i < 2; i++) {
  //     move_motor_to_angle(CLAW_SERVO_MOTOR,
  //     count_to_angle(SERVO_CENTER_COUNT),
  //                         current_counts[CLAW_SERVO_MOTOR]);  // Open claw

  //     current_counts[CLAW_SERVO_MOTOR] = (SERVO_CENTER_COUNT);

  //     HAL_Delay(500);

  //     move_motor_to_angle(CLAW_SERVO_MOTOR, count_to_angle(SERVO_LEFT_COUNT),
  //                         current_counts[CLAW_SERVO_MOTOR]);  // Close claw

  //     current_counts[CLAW_SERVO_MOTOR] = (SERVO_LEFT_COUNT);

  //     HAL_Delay(500);
  //   }

  //   // test base motor by moving it to left, center, right, center with
  //   smooth
  //   // transitions
  //   move_motor_to_angle(BASE_SERVO_MOTOR, count_to_angle(SERVO_LEFT_COUNT -
  //   120),
  //                       current_counts[BASE_SERVO_MOTOR]);  // Move to left

  //   current_counts[BASE_SERVO_MOTOR] = (SERVO_LEFT_COUNT - 120);
  //   HAL_Delay(500);

  //   //   center is 345 for this motor
  //   move_motor_to_angle(BASE_SERVO_MOTOR, count_to_angle(SERVO_CENTER_COUNT -
  //   65),
  //                       current_counts[BASE_SERVO_MOTOR]);  // Move to center
  //   current_counts[BASE_SERVO_MOTOR] = (SERVO_CENTER_COUNT - 65);
  //   HAL_Delay(500);
  //   move_motor_to_angle(BASE_SERVO_MOTOR, count_to_angle(SERVO_RIGHT_COUNT +
  //   50),
  //                       current_counts[BASE_SERVO_MOTOR]);  // Move to right
  //   current_counts[BASE_SERVO_MOTOR] = (SERVO_RIGHT_COUNT + 50);
  //   HAL_Delay(500);
  //   move_motor_to_angle(BASE_SERVO_MOTOR, count_to_angle(SERVO_CENTER_COUNT -
  //   65),
  //                       current_counts[BASE_SERVO_MOTOR]);  // Move to center
  //   current_counts[BASE_SERVO_MOTOR] = (SERVO_CENTER_COUNT - 65);
  //   HAL_Delay(500);

  //   test arm motor 1 by moving it to left, center, right, center with smooth
  //   transitions
  //   move_motor_to_angle(ARM_SERVO_MOTOR1, count_to_angle(SERVO_LEFT_COUNT),
  //                       current_counts[ARM_SERVO_MOTOR1]);  // Move to left
  //   current_counts[ARM_SERVO_MOTOR1] = (SERVO_LEFT_COUNT);
  //   HAL_Delay(500);
  //   move_motor_to_angle(ARM_SERVO_MOTOR1, count_to_angle(SERVO_CENTER_COUNT),
  //                       current_counts[ARM_SERVO_MOTOR1]);  // Move to center
  //   current_counts[ARM_SERVO_MOTOR1] = (SERVO_CENTER_COUNT);
  //   HAL_Delay(500);
  //   move_motor_to_angle(ARM_SERVO_MOTOR1, count_to_angle(SERVO_RIGHT_COUNT),
  //                       current_counts[ARM_SERVO_MOTOR1]);  // Move to right
  //   current_counts[ARM_SERVO_MOTOR1] = (SERVO_RIGHT_COUNT);
  //   HAL_Delay(500);
  //   move_motor_to_angle(ARM_SERVO_MOTOR1, count_to_angle(SERVO_CENTER_COUNT),
  //                       current_counts[ARM_SERVO_MOTOR1]);  // Move to center
  //   current_counts[ARM_SERVO_MOTOR1] = (SERVO_CENTER_COUNT);
  //   HAL_Delay(500);

  //   //   test arm motor 2 by moving it to left, center, right, center with
  //   smooth
  //   //   transitions
  //   move_motor_to_angle(ARM_SERVO_MOTOR2, count_to_angle(SERVO_LEFT_COUNT),
  //                       current_counts[ARM_SERVO_MOTOR2]);  // Move to left
  //   current_counts[ARM_SERVO_MOTOR2] = (SERVO_LEFT_COUNT);
  //   HAL_Delay(500);
  //   move_motor_to_angle(ARM_SERVO_MOTOR2, count_to_angle(SERVO_CENTER_COUNT),
  //                       current_counts[ARM_SERVO_MOTOR2]);  // Move to center
  //   current_counts[ARM_SERVO_MOTOR2] = (SERVO_CENTER_COUNT);
  //   HAL_Delay(500);
  //   move_motor_to_angle(ARM_SERVO_MOTOR2, count_to_angle(SERVO_RIGHT_COUNT),
  //                       current_counts[ARM_SERVO_MOTOR2]);  // Move to right
  //   current_counts[ARM_SERVO_MOTOR2] = (SERVO_RIGHT_COUNT);
  //   HAL_Delay(500);
  //   move_motor_to_angle(ARM_SERVO_MOTOR2, count_to_angle(SERVO_CENTER_COUNT),
  //                       current_counts[ARM_SERVO_MOTOR2]);  // Move to center
  //   current_counts[ARM_SERVO_MOTOR2] = (SERVO_CENTER_COUNT);
  //   HAL_Delay(500);

  // see how far down the arm can go and what we should limit
  // motor 1 down is 250
  //   move_motor_to_angle(ARM_SERVO_MOTOR1, count_to_angle(SERVO_LEFT_COUNT),
  //                       current_counts[ARM_SERVO_MOTOR1]);  // Move to left
  //   current_counts[ARM_SERVO_MOTOR1] = (SERVO_LEFT_COUNT);
  //   HAL_Delay(500);
  //   // motor 2 down is 550
  //   move_motor_to_angle(ARM_SERVO_MOTOR2, count_to_angle(SERVO_RIGHT_COUNT),
  //                       current_counts[ARM_SERVO_MOTOR2]);  // Move to right
  //   current_counts[ARM_SERVO_MOTOR2] = (SERVO_RIGHT_COUNT);
  //   HAL_Delay(500);

  // simulate fsm.
  // 1. recieve dummmy IK input (angles from IK)
  // first target is directly down
  int target_motor_counts[4] = {SERVO_CENTER_COUNT, SERVO_LEFT_COUNT,
                                SERVO_RIGHT_COUNT, SERVO_CENTER_COUNT};
  // 2. move motors to position
  move_motor_to_angle(BASE_SERVO_MOTOR,
                      count_to_angle(target_motor_counts[BASE_SERVO_MOTOR]),
                      current_counts[BASE_SERVO_MOTOR]);  // Move to left
  current_counts[BASE_SERVO_MOTOR] = (target_motor_counts[BASE_SERVO_MOTOR]);
  HAL_Delay(500);
  move_motor_to_angle(ARM_SERVO_MOTOR1,
                      count_to_angle(target_motor_counts[ARM_SERVO_MOTOR1]),
                      current_counts[ARM_SERVO_MOTOR1]);  // Move to center
  current_counts[ARM_SERVO_MOTOR1] = (target_motor_counts[ARM_SERVO_MOTOR1]);
  HAL_Delay(500);
  move_motor_to_angle(ARM_SERVO_MOTOR2,
                      count_to_angle(target_motor_counts[ARM_SERVO_MOTOR2]),
                      current_counts[ARM_SERVO_MOTOR2]);  // Move to right
  current_counts[ARM_SERVO_MOTOR2] = (target_motor_counts[ARM_SERVO_MOTOR2]);
  HAL_Delay(500);
  // 3. close claw
  move_motor_to_angle(CLAW_SERVO_MOTOR, count_to_angle(SERVO_LEFT_COUNT),
                      current_counts[CLAW_SERVO_MOTOR]);  // Close claw

  current_counts[CLAW_SERVO_MOTOR] = (SERVO_LEFT_COUNT);

  HAL_Delay(500);
  // 4. move to neutral position
  move_motor_to_angle(BASE_SERVO_MOTOR, count_to_angle(SERVO_CENTER_COUNT),
                      current_counts[BASE_SERVO_MOTOR]);  // Move to left
  current_counts[BASE_SERVO_MOTOR] = (SERVO_CENTER_COUNT);
  HAL_Delay(500);
  move_motor_to_angle(ARM_SERVO_MOTOR1, count_to_angle(SERVO_CENTER_COUNT),
                      current_counts[ARM_SERVO_MOTOR1]);  // Move to center
  current_counts[ARM_SERVO_MOTOR1] = (SERVO_CENTER_COUNT);
  HAL_Delay(500);
  move_motor_to_angle(ARM_SERVO_MOTOR2, count_to_angle(SERVO_CENTER_COUNT),
                      current_counts[ARM_SERVO_MOTOR2]);  // Move to right
  current_counts[ARM_SERVO_MOTOR2] = (SERVO_CENTER_COUNT);
  HAL_Delay(500);
  // 5. move to position above dropoff

  // first dropoff will be to the side and as high as possible to show we can do
  // different positions than the pickup position
  move_motor_to_angle(BASE_SERVO_MOTOR, count_to_angle(SERVO_RIGHT_COUNT),
                      current_counts[BASE_SERVO_MOTOR]);  // Move to left
  current_counts[BASE_SERVO_MOTOR] = (SERVO_RIGHT_COUNT);
  HAL_Delay(500);
  move_motor_to_angle(ARM_SERVO_MOTOR1, count_to_angle(SERVO_RIGHT_COUNT),
                      current_counts[ARM_SERVO_MOTOR1]);  // Move to the top
  current_counts[ARM_SERVO_MOTOR1] = (SERVO_RIGHT_COUNT);
  HAL_Delay(500);
  move_motor_to_angle(ARM_SERVO_MOTOR2, count_to_angle(SERVO_LEFT_COUNT),
                      current_counts[ARM_SERVO_MOTOR2]);  // Move to the top
  current_counts[ARM_SERVO_MOTOR2] = (SERVO_LEFT_COUNT);
  HAL_Delay(500);
  // 6. open claw to dropoff
  move_motor_to_angle(CLAW_SERVO_MOTOR, count_to_angle(SERVO_CENTER_COUNT),
                      current_counts[CLAW_SERVO_MOTOR]);  // Open claw

  current_counts[CLAW_SERVO_MOTOR] = (SERVO_CENTER_COUNT);

  HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */

  /* USER CODE BEGIN WHILE */

  while (1)

  {
    HAL_Delay(1000);

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
