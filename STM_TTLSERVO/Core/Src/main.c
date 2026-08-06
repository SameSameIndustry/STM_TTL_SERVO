/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "as5600.h"
#include "bldc.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 5° in AS5600 raw counts: 5 / 360 * 4096 ≈ 57 */
#define AS5600_ZONE_MARGIN  57u

/* Target position: 90° mechanical from the position read at power-on. */
#define TARGET_OFFSET_COUNTS   (BLDC_ENCODER_COUNTS / 4u)   /* 1024 = 90° */

/* "On target" tolerance for the status LED (reuses the 5° margin above). */
#define POS_TOLERANCE_COUNTS   AS5600_ZONE_MARGIN
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  AS5600_Init();
  BLDC_Init();

  /* Position read before the rotor is driven anywhere: the control target
   * is defined relative to this power-on position. */
  uint16_t initial_angle = 0u;
  AS5600_ReadAngle(&initial_angle);

  /* Settle the rotor at a known starting field angle (0°) before the
   * velocity-cascade loop begins. Unlike the earlier torque-mode design,
   * the rotor's electrical phase no longer needs to be tracked afterward —
   * commutation below is open-loop (a free-running field angle, same
   * mechanism as BLDC_StartupRamp/BLDC_Run), and only its rotation SPEED is
   * closed-loop controlled. */
  BLDC_Commutate(0u, BLDC_ALIGN_DUTY);
  HAL_Delay(BLDC_ALIGN_TIME_MS);
  uint16_t settle_angle = 0u;
  AS5600_ReadAngle(&settle_angle);

  /* Direction calibration: step the commanded electrical angle forward a
   * bit and measure which way the encoder moves. Without this, advancing
   * the free-running field "forward" below would only push toward the
   * target by coincidence — the correspondence between "increasing
   * electrical angle" and "increasing encoder count" depends on motor
   * winding / encoder mounting and is not otherwise known. */
  BLDC_Commutate((uint8_t)BLDC_DIR_CAL_ELEC_STEP, BLDC_ALIGN_DUTY);
  HAL_Delay(BLDC_ALIGN_TIME_MS);
  uint16_t dir_cal_angle = 0u;
  AS5600_ReadAngle(&dir_cal_angle);

  int32_t dir_cal_delta = (int32_t)dir_cal_angle - (int32_t)settle_angle;
  dir_cal_delta = ((dir_cal_delta + (int32_t)(BLDC_ENCODER_COUNTS / 2u))
                    & (int32_t)(BLDC_ENCODER_COUNTS - 1u))
                   - (int32_t)(BLDC_ENCODER_COUNTS / 2u);
  /* +1 if increasing electrical angle increases the encoder count, -1 if
   * the motor/encoder are mounted the other way around. */
  int32_t elec_dir_sign = (dir_cal_delta >= 0) ? 1 : -1;

  uint16_t target_angle = (uint16_t)((initial_angle + TARGET_OFFSET_COUNTS)
                                      & (BLDC_ENCODER_COUNTS - 1u));

  /* Free-running commanded electrical field angle [°], wrapped to
   * [0, 360). Starts where the direction-calibration step left it, so the
   * velocity loop below doesn't begin with a discontinuous jump. Only its
   * rate of advance is closed-loop controlled (velocity cascade); duty
   * stays fixed at BLDC_POS_RUN_DUTY throughout. */
  float field_elec_deg = (float)BLDC_DIR_CAL_ELEC_STEP * (360.0f / 256.0f);

  /* Actual (rate-limited) commanded speed, ramped toward the desired speed
   * by at most BLDC_POS_MAX_ACCEL_DEG_S2 per second — see loop below. */
  float commanded_speed_deg_s = 0.0f;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint16_t angle;
    if (AS5600_ReadAngle(&angle) == AS5600_OK) {
        /* Shortest-path signed error in encoder counts (wraps through 0/4095). */
        int32_t err = (int32_t)target_angle - (int32_t)angle;
        err = ((err + (int32_t)(BLDC_ENCODER_COUNTS / 2u))
               & (int32_t)(BLDC_ENCODER_COUNTS - 1u))
              - (int32_t)(BLDC_ENCODER_COUNTS / 2u);
        float error_deg = (float)err * (360.0f / (float)BLDC_ENCODER_COUNTS);
        uint8_t on_target = (err > -(int32_t)POS_TOLERANCE_COUNTS)
                          && (err <  (int32_t)POS_TOLERANCE_COUNTS);

        /* Velocity cascade: position error sets the field's commanded
         * rotation SPEED, not its duty/torque — duty stays fixed at
         * BLDC_POS_RUN_DUTY (below). This reuses the same continuously-
         * advancing open-loop commutation already proven to spin this motor
         * reliably, instead of scaling duty with error (which was too weak
         * to break static friction for small errors).
         *
         * Trapezoidal profile: the desired speed is the fastest speed that
         * can still decelerate to exactly 0 at the target using at most
         * BLDC_POS_MAX_ACCEL_DEG_S2 (from v² = 2·a·d), clamped to
         * BLDC_POS_MAX_SPEED_DEG_S. Far from the target this saturates at
         * max speed (cruise); close to the target it naturally traces the
         * physically-correct braking curve down to 0 instead of an
         * arbitrary proportional droop — accelerate, cruise, decelerate,
         * converge, with no overshoot. */
        float dist_deg = fabsf(error_deg);
        float desired_speed_mag = sqrtf(2.0f * BLDC_POS_MAX_ACCEL_DEG_S2 * dist_deg);
        if (desired_speed_mag > BLDC_POS_MAX_SPEED_DEG_S) desired_speed_mag = BLDC_POS_MAX_SPEED_DEG_S;
        float desired_speed_deg_s = (error_deg >= 0.0f) ? desired_speed_mag : -desired_speed_mag;

        /* Acceleration limit: slew commanded_speed_deg_s toward
         * desired_speed_deg_s by at most BLDC_POS_MAX_ACCEL_DEG_S2 per
         * second, instead of jumping straight to it. Without this, a large
         * position error would instantly command BLDC_POS_MAX_SPEED_DEG_S
         * from a standstill, risking the rotor losing sync (the same
         * failure mode seen when testing higher open-loop approach speeds
         * without a ramp earlier). */
        float dt_s = (float)BLDC_POS_LOOP_PERIOD_MS / 1000.0f;
        float max_speed_delta = BLDC_POS_MAX_ACCEL_DEG_S2 * dt_s;
        float speed_error = desired_speed_deg_s - commanded_speed_deg_s;
        if (speed_error > max_speed_delta) {
            commanded_speed_deg_s += max_speed_delta;
        } else if (speed_error < -max_speed_delta) {
            commanded_speed_deg_s -= max_speed_delta;
        } else {
            commanded_speed_deg_s = desired_speed_deg_s;
        }

        /* elec_dir_sign corrects for which physical direction "increasing
         * electrical angle" actually turns this motor (see calibration
         * above), so commanded_speed_deg_s > 0 always advances the field
         * toward increasing mechanical angle regardless of winding /
         * encoder mounting. */
        field_elec_deg += (float)elec_dir_sign * commanded_speed_deg_s * dt_s * MOTOR_POLE_PAIRS;
        field_elec_deg = fmodf(field_elec_deg, 360.0f);
        if (field_elec_deg < 0.0f) field_elec_deg += 360.0f;

        uint8_t comm_angle = (uint8_t)(field_elec_deg * (256.0f / 360.0f));
        BLDC_Commutate(comm_angle, BLDC_POS_RUN_DUTY);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, on_target ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    HAL_Delay(BLDC_POS_LOOP_PERIOD_MS);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x60E12C36;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
