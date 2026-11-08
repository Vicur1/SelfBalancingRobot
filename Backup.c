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
#include <stdio.h> // For printf
#include <string.h> // Required for strlen()
#include <math.h>
#include <stdlib.h>  // abs()
#include <stdbool.h> // bool, true, false

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
#define SETPOINT_DEG   1.5f
#define KP             2000.0f
#define KD             150.0f
#define MIN_PWM        100
#define MAX_PWM        999
#define MAX_ANGLE_RAD  0.43f  // 30 degrees

#define GYRO_SCALE     0.00875f
#define DEG_TO_RAD     0.01745329252f


float angle = 0.0f;
float motorOutput = 0.0f;

//For motor
uint32_t M1_speed = 0; // 0 to 999 (Counter Period value)
uint32_t M2_speed = 0; // 0 to 999 (Counter Period value)

/* I2C Addresses - LSM9DS1 */
#define LSM9DS1_ACCEL_GYRO_ADDRESS  (0x6B << 1)
#define LSM9DS1_MAG_ADDRESS         (0x1C << 1)

/* LSM9DS1 Register Addresses */
#define WHO_AM_I_XG   0x0F
#define WHO_AM_I_M    0x0F

/* Control Registers */
#define CTRL_REG6_XL  0x20  // Accelerometer
#define CTRL_REG1_G   0x10  // Gyroscope
#define CTRL_REG3_M   0x22  // Magnetometer operating mode
#define CTRL_REG2_M   0x21  // Magnetometer data rate and scale

/* Accelerometer Data Registers */
#define OUT_X_L_XL    0x28
#define OUT_X_H_XL    0x29
#define OUT_Y_L_XL    0x2A
#define OUT_Y_H_XL    0x2B
#define OUT_Z_L_XL    0x2C
#define OUT_Z_H_XL    0x2D

/* Gyroscope Data Registers */
#define OUT_X_L_G     0x18
#define OUT_X_H_G     0x19
#define OUT_Y_L_G     0x1A
#define OUT_Y_H_G     0x1B
#define OUT_Z_L_G     0x1C
#define OUT_Z_H_G     0x1D

/* Magnetometer Data Registers */
#define OUT_X_L_M     0x28
#define OUT_X_H_M     0x29
#define OUT_Y_L_M     0x2A
#define OUT_Y_H_M     0x2B
#define OUT_Z_L_M     0x2C
#define OUT_Z_H_M     0x2D

/* Raw Sensor Data Variables */
int16_t accel_x_raw, accel_y_raw, accel_z_raw;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
int16_t mag_x_raw, mag_y_raw, mag_z_raw;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SetMotorSpeed(int16_t pwm)
{
    // Limit PWM
    if (pwm > MAX_PWM)  pwm = MAX_PWM;
    if (pwm < -MAX_PWM) pwm = -MAX_PWM;

    // Dead-zone compensation
    if (-MIN_PWM < pwm < MIN_PWM)
    {
      pwm = 0.0;
    }

    // Set motor direction
    if (pwm >= 0)
    {
        HAL_GPIO_WritePin(M1Direction_GPIO_Port,
                          M1Direction_Pin, GPIO_PIN_SET);

        HAL_GPIO_WritePin(M2Direction_GPIO_Port,
                          M2Direction_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(M1Direction_GPIO_Port,
                          M1Direction_Pin, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(M2Direction_GPIO_Port,
                          M2Direction_Pin, GPIO_PIN_SET);
    }

    uint16_t duty = (uint16_t)abs((int)pwm);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
}
/* Redirect printf to UART */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}


/* I2C Write Function */
HAL_StatusTypeDef LSM9DS1_I2C_Write(
    uint8_t device_addr,
    uint8_t reg_addr,
    uint8_t *data,
    uint16_t len)
{
    return HAL_I2C_Mem_Write(
        &hi2c1,
        device_addr,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        data,
        len,
        100
    );
}


/* I2C Read Function */
HAL_StatusTypeDef LSM9DS1_I2C_Read(
    uint8_t device_addr,
    uint8_t reg_addr,
    uint8_t *data,
    uint16_t len)
{
    return HAL_I2C_Mem_Read(
        &hi2c1,
        device_addr,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        data,
        len,
        100
    );
}


/* Initialize LSM9DS1 */
void LSM9DS1_Init(void)
{
    uint8_t reg_data = 0;
    HAL_StatusTypeDef status;

    /* Check WHO_AM_I - Accelerometer/Gyroscope */
    status = LSM9DS1_I2C_Read(
        LSM9DS1_ACCEL_GYRO_ADDRESS,
        WHO_AM_I_XG,
        &reg_data,
        1
    );

    if (status != HAL_OK || reg_data != 0x68)
    {
        printf("Accel/Gyro WHO_AM_I failed! Status: %d, Data: 0x%X\r\n",
               status, reg_data);
    }
    else
    {
        printf("Accel/Gyro WHO_AM_I: 0x%X (OK)\r\n", reg_data);
    }

    /* Check WHO_AM_I - Magnetometer */
    reg_data = 0;

    status = LSM9DS1_I2C_Read(
        LSM9DS1_MAG_ADDRESS,
        WHO_AM_I_M,
        &reg_data,
        1
    );

    if (status != HAL_OK || reg_data != 0x3D)
    {
        printf("Magnetometer WHO_AM_I failed! Status: %d, Data: 0x%X\r\n",
               status, reg_data);
    }
    else
    {
        printf("Magnetometer WHO_AM_I: 0x%X (OK)\r\n", reg_data);
    }

    /* Configure Accelerometer */
    /* ODR = 119 Hz, FS = +/-2g */
    reg_data = 0x60;

    LSM9DS1_I2C_Write(
        LSM9DS1_ACCEL_GYRO_ADDRESS,
        CTRL_REG6_XL,
        &reg_data,
        1
    );

    printf("Configured Accel: 119Hz, +/-2g\r\n");

    /* Configure Gyroscope */
    /* ODR = 119 Hz, FS = +/-245 dps */
    reg_data = 0x60;

    LSM9DS1_I2C_Write(
        LSM9DS1_ACCEL_GYRO_ADDRESS,
        CTRL_REG1_G,
        &reg_data,
        1
    );

    printf("Configured Gyro: 119Hz, +/-245dps\r\n");

    /* Configure Magnetometer */
    /* Continuous-conversion mode */
    reg_data = 0x00;

    LSM9DS1_I2C_Write(
        LSM9DS1_MAG_ADDRESS,
        CTRL_REG3_M,
        &reg_data,
        1
    );

    printf("Configured Mag: Continuous mode\r\n");

    /* Magnetometer ODR = 10 Hz */
    /* CTRL_REG1_M = 0x20 */
    reg_data = 0x10;

    LSM9DS1_I2C_Write(
        LSM9DS1_MAG_ADDRESS,
        0x20,
        &reg_data,
        1
    );

    printf("Configured Mag: 10Hz\r\n");
}


/* Read Accelerometer Data */
void LSM9DS1_ReadAccel(void)
{
    uint8_t buffer[6];

    if (LSM9DS1_I2C_Read(
            LSM9DS1_ACCEL_GYRO_ADDRESS,
            OUT_X_L_XL | 0x80,
            buffer,
            6) != HAL_OK)
    {
        return;
    }

    accel_x_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    accel_y_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
    accel_z_raw = (int16_t)((buffer[5] << 8) | buffer[4]);
}


/* Read Gyroscope Data */
void LSM9DS1_ReadGyro(void)
{
    uint8_t buffer[6];

    if (LSM9DS1_I2C_Read(
            LSM9DS1_ACCEL_GYRO_ADDRESS,
            OUT_X_L_G | 0x80,
            buffer,
            6) != HAL_OK)
    {
        return;
    }

    gyro_x_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    gyro_y_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
    gyro_z_raw = (int16_t)((buffer[5] << 8) | buffer[4]);
}


/* Read Magnetometer Data */
void LSM9DS1_ReadMag(void)
{
    uint8_t buffer[6];

    if (LSM9DS1_I2C_Read(
            LSM9DS1_MAG_ADDRESS,
            OUT_X_L_M | 0x80,
            buffer,
            6) != HAL_OK)
    {
        return;
    }

    mag_x_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    mag_y_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
    mag_z_raw = (int16_t)((buffer[5] << 8) | buffer[4]);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  //For motor:
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  LSM9DS1_Init();

  // Define the message to send
  char *tx_data = "Hello world from HAL_UART_Transmit!\r\n";
  uint16_t len = strlen(tx_data);
  uint32_t timeout = 100; // Timeout in milliseconds for blocking transmit

  HAL_UART_Transmit(&huart3, (uint8_t*)tx_data, len, timeout);

uint32_t lastControl = HAL_GetTick();
uint32_t lastPrint   = HAL_GetTick();

float angleAcc = 0.0f;
float gyroRate = 0.0f;
float gyroOffset = 0.0f;
float dt;

bool angleInitialized = false;

while (1)
{
    uint32_t now = HAL_GetTick();

    // PD regulation - 100 Hz
    if ((uint32_t)(now - lastControl) >= 10)
    {
        dt = (now - lastControl) * 0.001f;
        lastControl = now;

        // Read sensors
        LSM9DS1_ReadAccel();
        LSM9DS1_ReadGyro();
        LSM9DS1_ReadMag();

        // Angle from accelerometer [rad]
        angleAcc = atan2f(
            (float)accel_x_raw,
            -(float)accel_z_raw
        );

        // Angular velocity [rad/s]
        gyroRate = -((float)gyro_y_raw - gyroOffset)
                   * GYRO_SCALE * DEG_TO_RAD;
                   
        // Initialize filter with actual angle
        if (!angleInitialized)
        {
            angle = angleAcc;
            angleInitialized = true;
        }

        // Complementary filter
        angle = 0.98f * (angle + gyroRate * dt)
              + 0.02f * angleAcc;


        // Calculate error relative to balance point
        float error = SETPOINT_DEG*DEG_TO_RAD - angle;


        // PD controller
        motorOutput = -KP * error - KD * gyroRate;

        // Limit PWM
        if (motorOutput > MAX_PWM)
            motorOutput = MAX_PWM;

        if (motorOutput < -MAX_PWM)
            motorOutput = -MAX_PWM;

// Stop motors outside +/-15 degrees
if (fabsf(angle) > MAX_ANGLE_RAD)
{
    motorOutput = 0.0f;
}

SetMotorSpeed((int16_t)motorOutput);
// Motor control
SetMotorSpeed((int16_t)motorOutput);
    }

    // UART output - 1 Hz
    if ((uint32_t)(now - lastPrint) >= 10)
    {
        lastPrint = now;

        char tx_data[250];

        int len = snprintf(
            tx_data,
            sizeof(tx_data),
            "Accel: X=%d Y=%d Z=%d | "
            "Gyro: X=%d Y=%d Z=%d | "
            "Mag: X=%d Y=%d Z=%d\r\n"
            "Angle: Acc=%.2f Gyro=%.2f Angle=%.2f\r\n" 
            "Motor: PWM=%.0f\r\n",
            accel_x_raw, accel_y_raw, accel_z_raw,
            gyro_x_raw, gyro_y_raw, gyro_z_raw,
            mag_x_raw, mag_y_raw, mag_z_raw,
            angleAcc, gyroRate, angle,
            motorOutput
        );

        if (len > 0 && len < sizeof(tx_data))
        {
            HAL_UART_Transmit(
                &huart3,
                (uint8_t *)tx_data,
                (uint16_t)len,
                100
            );
        }
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x6000030D;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 215;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|M1Direction_Pin|M2Direction_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD1_Pin M1Direction_Pin M2Direction_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|M1Direction_Pin|M2Direction_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
