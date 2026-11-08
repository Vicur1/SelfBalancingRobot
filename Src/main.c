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
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STOP,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
} MovementState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define KP                 9000.0f 
#define KI                 0.0f
#define KD                 0.0f

#define MIN_PWM            20
#define MAX_PWM            999
#define MAX_ANGLE_RAD      0.43f
#define SETPOINT           0.0f
#define PWM_DEADBAND       10

#define GYRO_SCALE         0.00875f
#define DEG_TO_RAD         0.01745329252f
#define CALIBRATION_CYCLES 200

#define LSM9DS1_ACCEL_GYRO_ADDRESS  (0x6B << 1)
#define LSM9DS1_MAG_ADDRESS         (0x1C << 1)

#define WHO_AM_I_XG    0x0F
#define WHO_AM_I_M     0x0F

#define CTRL_REG6_XL   0x20
#define CTRL_REG1_G    0x10
#define CTRL_REG3_M    0x22
#define CTRL_REG2_M    0x21

#define OUT_X_L_XL     0x28
#define OUT_X_H_XL     0x29
#define OUT_Y_L_XL     0x2A
#define OUT_Y_H_XL     0x2B
#define OUT_Z_L_XL     0x2C
#define OUT_Z_H_XL     0x2D

#define OUT_X_L_G      0x18
#define OUT_X_H_G      0x19
#define OUT_Y_L_G      0x1A
#define OUT_Y_H_G      0x1B
#define OUT_Z_L_G      0x1C
#define OUT_Z_H_G      0x1D

#define OUT_X_L_M      0x28
#define OUT_X_H_M      0x29
#define OUT_Y_L_M      0x2A
#define OUT_Y_H_M      0x2B
#define OUT_Z_L_M      0x2C
#define OUT_Z_H_M      0x2D
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t rx_data;
MovementState current_state = STOP;

int16_t accel_x_raw, accel_y_raw, accel_z_raw;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
int16_t mag_x_raw, mag_y_raw, mag_z_raw;
float angle = 0.0f;
float angleAcc = 0.0f; 
float gyroRate = 0.0f; 
float gyroangle = 0.0f;
float accelBias = 0;
float gyroBias = 0;

float current_setpoint = 0.0f;
float steering_bias = 0.0f;

float PI = 3.14159265359;

float P = 0.0f;
float I = 0.0f;
float D = 0.0f;
float lastangle = 0.0f;
float motorOutput = 0.0f;

int32_t pwm_deadband = PWM_DEADBAND;
uint32_t M1_speed = 0;
uint32_t M2_speed = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void Motor_Control(float base_pwm, float bias);
HAL_StatusTypeDef LSM9DS1_I2C_Write(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
HAL_StatusTypeDef LSM9DS1_I2C_Read(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
void LSM9DS1_Init(void);
void LSM9DS1_ReadAccel(void);
void LSM9DS1_ReadGyro(void);
void LSM9DS1_ReadMag(void);
void IMU_Calibrate(void);
void Get_Angle(float dt);
void PID_Calculate(float dt);
void Update_Movement_State(void);
void Send_Telemetry(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Motor_Control(float base_pwm, float bias)
{
    if (fabsf(angle) > MAX_ANGLE_RAD)
    {
        base_pwm = 0.0f;
        bias = 0.0f;
    }

    float pwm1 = base_pwm - bias; // Motor 1 (Left)
    float pwm2 = base_pwm + bias; // Motor 2 (Right)

    if (pwm1 > MAX_PWM)  pwm1 = MAX_PWM;
    if (pwm1 < -MAX_PWM) pwm1 = -MAX_PWM;
    if (pwm2 > MAX_PWM)  pwm2 = MAX_PWM;
    if (pwm2 < -MAX_PWM) pwm2 = -MAX_PWM;

    // --- Motor 1 Control ---
    if (pwm1 <= 0)
        HAL_GPIO_WritePin(M1Direction_GPIO_Port, M1Direction_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(M1Direction_GPIO_Port, M1Direction_Pin, GPIO_PIN_RESET);

    int32_t abs_m1 = abs((int)pwm1);
    uint16_t duty1 = 0;
    if (abs_m1 > pwm_deadband)
    {
        duty1 = (uint16_t)(MIN_PWM + (abs_m1 * (int32_t)(MAX_PWM - MIN_PWM)) / MAX_PWM);
    }

    // --- Motor 2 Control ---
    if (pwm2 <= 0)
        HAL_GPIO_WritePin(M2Direction_GPIO_Port, M2Direction_Pin, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(M2Direction_GPIO_Port, M2Direction_Pin, GPIO_PIN_SET);

    int32_t abs_m2 = abs((int)pwm2);
    uint16_t duty2 = 0;
    if (abs_m2 > pwm_deadband)
    {
        duty2 = (uint16_t)(MIN_PWM + (abs_m2 * (int32_t)(MAX_PWM - MIN_PWM)) / MAX_PWM);
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty2);
}

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

HAL_StatusTypeDef LSM9DS1_I2C_Write(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Write(&hi2c1, device_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

HAL_StatusTypeDef LSM9DS1_I2C_Read(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, device_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

void LSM9DS1_Init(void)
{
    uint8_t reg_data = 0;
    HAL_StatusTypeDef status;

    status = LSM9DS1_I2C_Read(LSM9DS1_ACCEL_GYRO_ADDRESS, WHO_AM_I_XG, &reg_data, 1);
    if (status != HAL_OK || reg_data != 0x68)
    {
        printf("Accel/Gyro WHO_AM_I failed! Status: %d, Data: 0x%X\r\n", status, reg_data);
    }
    else
    {
        printf("Accel/Gyro WHO_AM_I: 0x%X (OK)\r\n", reg_data);
    }

    reg_data = 0;
    status = LSM9DS1_I2C_Read(LSM9DS1_MAG_ADDRESS, WHO_AM_I_M, &reg_data, 1);
    if (status != HAL_OK || reg_data != 0x3D)
    {
        printf("Magnetometer WHO_AM_I failed! Status: %d, Data: 0x%X\r\n", status, reg_data);
    }
    else
    {
        printf("Magnetometer WHO_AM_I: 0x%X (OK)\r\n", reg_data);
    }

    reg_data = 0x60;
    LSM9DS1_I2C_Write(LSM9DS1_ACCEL_GYRO_ADDRESS, CTRL_REG6_XL, &reg_data, 1);
    printf("Configured Accel: 119Hz, +/-2g\r\n");

    reg_data = 0x60;
    LSM9DS1_I2C_Write(LSM9DS1_ACCEL_GYRO_ADDRESS, CTRL_REG1_G, &reg_data, 1);
    printf("Configured Gyro: 119Hz, +/-245dps\r\n");

    reg_data = 0x00;
    LSM9DS1_I2C_Write(LSM9DS1_MAG_ADDRESS, CTRL_REG3_M, &reg_data, 1);
    printf("Configured Mag: Continuous mode\r\n");

    reg_data = 0x10;
    LSM9DS1_I2C_Write(LSM9DS1_MAG_ADDRESS, 0x20, &reg_data, 1);
    printf("Configured Mag: 10Hz\r\n");
}

void LSM9DS1_ReadAccel(void)
{
    uint8_t buffer[6];
    if (LSM9DS1_I2C_Read(LSM9DS1_ACCEL_GYRO_ADDRESS, OUT_X_L_XL | 0x80, buffer, 6) != HAL_OK)
    {
        return;
    }
    accel_x_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    accel_y_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
    accel_z_raw = (int16_t)((buffer[5] << 8) | buffer[4]);
}

void LSM9DS1_ReadGyro(void)
{
    uint8_t buffer[6];
    if (LSM9DS1_I2C_Read(LSM9DS1_ACCEL_GYRO_ADDRESS, OUT_X_L_G | 0x80, buffer, 6) != HAL_OK)
    {
        return;
    }
    gyro_x_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    gyro_y_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
    gyro_z_raw = (int16_t)((buffer[5] << 8) | buffer[4]);
}

void LSM9DS1_ReadMag(void)
{
    uint8_t buffer[6];
    if (LSM9DS1_I2C_Read(LSM9DS1_MAG_ADDRESS, OUT_X_L_M | 0x80, buffer, 6) != HAL_OK)
    {
        return;
    }
    mag_x_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    mag_y_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
    mag_z_raw = (int16_t)((buffer[5] << 8) | buffer[4]);
}

void IMU_Calibrate(void)
{
    /*Calibrate Acceleromter and Gyro on startup*/
    for (int i = 0; i < CALIBRATION_CYCLES; ++i)
    {
        LSM9DS1_ReadAccel();
        LSM9DS1_ReadGyro();
        accelBias += atan2f((float)accel_x_raw, -(float)accel_z_raw);
        gyroBias += (float)gyro_y_raw;
    }
    accelBias /= CALIBRATION_CYCLES;
    gyroBias /= CALIBRATION_CYCLES;
}

void Get_Angle(float dt)
{
    static bool angleInitialized = false;

    LSM9DS1_ReadAccel();
    LSM9DS1_ReadGyro();
    LSM9DS1_ReadMag();

    /*Calculate angle from accelerometer*/
    angleAcc = atan2f((float)accel_x_raw, -(float)accel_z_raw) - accelBias + PI/2 +0.01f;

    /*Initialize complementary filter*/
    if (!angleInitialized)
    {
        angle = angleAcc;
        angleInitialized = true;
    }

    /*Calculate angle from gyro*/
    gyroRate = -((float)gyro_y_raw - gyroBias) * GYRO_SCALE * DEG_TO_RAD;
    gyroangle = angle + gyroRate * dt;
    
    /*Complementary filter for angle*/
    angle = 0.99f * gyroangle + 0.01f * angleAcc;
}

void PID_Calculate(float dt)
{
    static float error_integral = 0.0f;
    static float filtered_gyroRate = 0.0f;
    float error = current_setpoint - angle;
    float error_squared = error * error;
    if (error < 0) {
    error_squared = -error_squared;
    }

    /* Proportinal control*/
    P = KP*error;

    /*Integral term with clamping*/
    error_integral += error * dt;
    if (error_integral > 200.0f)  error_integral = 200.0f;
    if (error_integral < -200.0f) error_integral = -200.0f;
    I = KI * error_integral;

    /* Low pass filter for derivate rate */ 
    float alpha = 0.99f; // Filter weighting (higher alpha, more filtered)
    filtered_gyroRate = (alpha * filtered_gyroRate) + ((1.0f - alpha) * gyroRate);

    /*Derivative term*/
    D = -KD * filtered_gyroRate;
    lastangle = angle;

    motorOutput = P + I + D;

    if (motorOutput > (float)MAX_PWM)
        motorOutput = (float)MAX_PWM;
    if (motorOutput < -(float)MAX_PWM)
        motorOutput = -(float)MAX_PWM;
}

void Update_Movement_State(void)
{
    switch (current_state) {
        case FORWARD:
            current_setpoint = 0.05f;  // Lean forward
            steering_bias = 0.0f;
            break;
            
        case BACKWARD:
            current_setpoint = -0.05f; // Lean backward
            steering_bias = 0.0f;
            break;
            
        case LEFT:
            current_setpoint = 0.0f;   // Balance
            steering_bias = -250.0f;   // Turn left bias
            break;
            
        case RIGHT:
            current_setpoint = 0.0f;   // Balance
            steering_bias = 250.0f;    // Turn right bias
            break;
            
        case STOP:
        default:
            current_setpoint = 0.0f;   // Reset balance
            steering_bias = 0.0f;
            break;
    }
}

void Send_Telemetry(void)
{
    static uint32_t bt_timer_counter = 0;
    char tx_data[128];
    const char *state_str;

    switch (current_state) {
        case FORWARD:  state_str = "FWD"; break;
        case BACKWARD: state_str = "REV"; break;
        case LEFT:     state_str = "LFT"; break;
        case RIGHT:    state_str = "RGT"; break;
        case STOP:
        default:       state_str = "STP"; break;
    }

    int len = snprintf(
        tx_data,
        sizeof(tx_data),
        "T:%s,%.4f,%.4f,%.4f,%.4f,%.0f,%.1f,%.1f,%.1f\r\n",
        state_str,
        angle, angleAcc, gyroangle, gyroRate,
        motorOutput, P, I, D
    );

    if (len > 0 && len < (int)sizeof(tx_data))
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)tx_data, (uint16_t)len, 50);

        bt_timer_counter++;
        if (bt_timer_counter >= 4)
        {
            bt_timer_counter = 0;
            HAL_UART_Transmit(&huart2, (uint8_t *)tx_data, (uint16_t)len, 100);
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (rx_data == 'w' || rx_data == 'W') {
            current_state = FORWARD;
        } 
        else if (rx_data == 's' || rx_data == 'S') {
            current_state = BACKWARD;
        } 
        else if (rx_data == 'a' || rx_data == 'A') {
            current_state = LEFT;
        } 
        else if (rx_data == 'd' || rx_data == 'D') {
            current_state = RIGHT;
        } 
        else if (rx_data == 'x' || rx_data == 'X') {
            current_state = STOP;
        }
        
        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    }
}

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  MPU_Config();

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  LSM9DS1_Init();

  static uint32_t lastControl = 0;
  static uint32_t lastPrint   = 0;

  IMU_Calibrate();

  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - lastControl) >= 2)
    {
      float dt = (now - lastControl) * 0.001f;
      lastControl = now;

      Get_Angle(dt);
      Update_Movement_State();
      PID_Calculate(dt);
      Motor_Control(motorOutput, steering_bias);
    }

    if ((uint32_t)(now - lastPrint) >= 100)
    {
      lastPrint = now;
      Send_Telemetry();
    }
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

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

static void MX_I2C1_Init(void)
{
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

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

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
  HAL_TIM_MspPostInit(&htim1);
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART3_UART_Init(void)
{
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
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, LD1_Pin|M1Direction_Pin|M2Direction_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LD1_Pin|M1Direction_Pin|M2Direction_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();

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
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif