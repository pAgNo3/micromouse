/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Path struct definition
#define MAX_PATH_LEN      1024
typedef struct {
    uint16_t len;
    uint8_t  x[MAX_PATH_LEN];
    uint8_t  y[MAX_PATH_LEN];
} Path;

typedef struct { int dir; uint16_t d; } Candidate;

typedef enum {
    MODE_DRY_RUN,
    MODE_MAIN_RUN
} RunMode_t;

typedef enum {
    STATE_IDLE,
    STATE_RUNNING

} RobotState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N                 16
#define MAX_QUEUE         (N*N)
#define MAX_MOVES         1000

#define MAX_STORED_PATHS  10

#define NORTH 0
#define EAST  1
#define SOUTH 2
#define WEST  3

// Define your encoder's parameters
#define MPU6050_ADDR 0xD0
#define WHO_AM_I_REG 0x75
#define GYRO_CONFIG_REG 0x1B



#define ENCODER_PPR 7
#define ENCODER_CPR 28
#define MS_PER_SEC 1000.0f
#define TWO_PI 6.28318530718f // 2 * pi
#define KP 1.0f // Proportional gain
#define KI 0.1f // Integral gain
#define KD 0.05f
#define CONTROL_LOOP_FREQ_HZ 500.0f
#define CONTROL_LOOP_PERIOD_MS (1000.0f / CONTROL_LOOP_FREQ_HZ)
#define KP_LEFT 10.0f
#define KI_LEFT 0.5f
#define KD_LEFT 0.1f
#define KP_RIGHT 10.0f
#define KI_RIGHT 0.5f
#define KD_RIGHT 0.1f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const int8_t dx[4] = { 0, 1, 0, -1 };
static const int8_t dy[4] = { 1, 0, -1, 0 };

static uint16_t dist_map[N][N];

static bool vWalls[N+1][N];  /* vertical grid lines: between (x-1,y) & (x,y) */
static bool hWalls[N][N+1];  /* horizontal grid lines: between (x,y-1) & (x,y) */

static bool visited[N][N];

/* Goals list (up to 4 center cells) */
static uint8_t goals_xy[4][2];
static uint8_t goal_count = 0;

/* Queue for BFS */
static uint8_t qx[MAX_QUEUE], qy[MAX_QUEUE];
static uint16_t q_front = 0, q_rear = 0;

/* Robot pose */
static int robotX = 0, robotY = 0, robotHeading = NORTH;

/* Paths */
static Path currentPath;
static Path pathToGoal;
static Path pathToStart;
static Path exploredPaths[MAX_STORED_PATHS];
static uint8_t exploredCount = 0;

/* Motor and encoder variables */
volatile int32_t leftEncoderCount = 0;
volatile int32_t rightEncoderCount = 0;
volatile uint32_t leftLastTick = 0;
volatile uint32_t rightLastTick = 0;
volatile uint32_t leftLastCount = 0;
volatile uint32_t rightLastCount = 0;

volatile float leftAngularVelocity = 0.0f;
volatile float rightAngularVelocity = 0.0f;
volatile float leftDistancem = 0.0f;
volatile float rightDistancem = 0.0f;
volatile float lastError = 0.0f;
volatile float integral = 0.0f;
volatile float derivative = 0.0f;
volatile float Gy0 = 0.0f;

volatile float leftSetpoint = 0.0f; // Target speed for left motor in m/s
volatile float rightSetpoint = 0.0f; // Target speed for right motor in m/s
volatile float leftError = 0.0f;
volatile float leftLastError = 0.0f;
volatile float leftIntegral = 0.0f;
volatile float rightError = 0.0f;
volatile float rightLastError = 0.0f;
volatile float rightIntegral = 0.0f;
volatile float Gyro_X_RAW = 0.0f;
volatile float Gyro_Y_RAW = 0.0f;
volatile float Gyro_Z_RAW = 0.0f;
volatile float Gx = 0.0f;
volatile float Gy = 0.0f;
volatile float Gz = 0.0f;
volatile float leftVelocity = 0.0f;
volatile float rightVelocity = 0.0f;
volatile float error = 0.0f;

// IR sensor variables
volatile uint32_t IR1 = 0;
volatile uint32_t IR2 = 0;
volatile uint32_t IR3 = 0;
volatile uint32_t IR4 = 0;

// Control variables
int backLength;
float tR = 0.125f;
int speed;
int Rspeed;
int Lspeed;
int ThresholdFront = 600;
int ThresholdSide = 600;
int PWM_RANGE = 4095;
int c = 0;
float r = 0.022f;

// Utility for random numbers
static uint32_t rng_state = 1u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/* Function prototypes */


void MPU6050_Read_Gyro(void);
void MPU6050_Init(void);
void RMotorSpeed(int Rspeed);
void LMotorSpeed(int Lspeed);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void calculateAndSetSpeeds(float a, float b);
int16_t calculateLeftPID(float leftSetPoint);
int16_t calculateRightPID(float rightSetPoint);
void setMotorPWM(int16_t leftPWM, int16_t rightPWM);
void Actuator_MoveForwardOneCell(void);
void Actuator_TurnLeft(void);
void Actuator_TurnRight(int t);
void readAllIRSensors(void);
void getEncoderValue(void);

// Utility functions
static inline void UART_Print(const char* s);
static inline bool in_bounds(int x, int y);
static inline void queue_reset(void);
static inline bool queue_empty(void);
static inline void enqueue(uint8_t x, uint8_t y);
static inline void dequeue(uint8_t* x, uint8_t* y);
static inline uint32_t rng_uniform(uint32_t m);
static inline bool hasWallBetween(int x, int y, int dir);
static inline void setWall(int x, int y, int dir);
static inline void clearWall(int x, int y, int dir);
static void reset_all(void);
static void resetVisited(void);
static void setGoals_center(void);
static inline bool isGoal(int x, int y);
static void updateDistances(void);
static void displayDistances(void);
static int getNextDirection(int x, int y);
static bool Sensor_WallFront(void);
static bool Sensor_WallLeft(void);
static bool Sensor_WallRight(void);
static void executeTurn(int newDir);
static bool moveForward(void);
static void updateWallsFromSensors(void);
static int selectDirectionWithVariation(Candidate* cands, int count);
static bool isInPreviousPaths(int nx, int ny);
static void storeCurrentPath(void);
static int getFallbackMoves(Candidate* cands);
static void exploreWithVariation(bool avoidPreviousPaths);
static void returnToStart(void);
static void followShortestPathToGoal(void);

// Random number functions
static inline void rng_seed(uint32_t s) { rng_state = (s ? s : 1u); }
static inline uint32_t rng_next(void) { rng_state = 1664525u * rng_state + 1013904223u; return rng_state; }
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
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  UART_Print("\r\nMicromouse Flood-Fill (STM32)\r\n");
  MPU6050_Init();
  RunMode_t runMode = MODE_DRY_RUN; // Default to Dry Run
  RobotState_t robotState = STATE_IDLE;
  reset_all();
  setGoals_center();
  updateDistances();
  displayDistances();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	  while (1)
	  {
	      // --- Switch Reading ---
	      // A low signal (GPIO_PIN_RESET) means the switch is ON (closed to ground)
	      GPIO_PinState dryMainSwitch = HAL_GPIO_ReadPin(DRY_MAIN_BUTTON_GPIO_Port, DRY_MAIN_BUTTON_Pin);
	      GPIO_PinState goHaltSwitch = HAL_GPIO_ReadPin(GO_HALT_BUTTON_GPIO_Port, GO_HALT_BUTTON_Pin);

	      // --- Determine Run Mode from Switch State ---
	      if (dryMainSwitch == GPIO_PIN_RESET) {
	          runMode = MODE_MAIN_RUN;
	      } else {
	          runMode = MODE_DRY_RUN;
	      }

	      // --- Determine Robot State from Switch State ---
	      if (goHaltSwitch == GPIO_PIN_RESET) {
	          // Switch is ON, so the robot should be in the RUNNING state
	          if (robotState == STATE_IDLE) {
	              robotState = STATE_RUNNING;
	              UART_Print("Robot State: RUNNING\r\n");

	              // Execute the run based on the selected mode
	              if (runMode == MODE_DRY_RUN) {
	                  UART_Print("Starting a new dry run...\r\n");
	                  exploreWithVariation(true);
	                  returnToStart();

	                  setGoals_center(); // Re-center for the next run
	                  exploreWithVariation(true);
	                  returnToStart();
	              } else { // MODE_MAIN_RUN
	                  UART_Print("Starting a new main run...\r\n");
	                  setGoals_center();
	                  followShortestPathToGoal();
	              }

	              // After the run completes, move back to IDLE
	              robotState = STATE_IDLE;
	              UART_Print("Run complete. Robot is IDLE.\r\n");
	          }
	      } else {
	          // Switch is OFF, so the robot should be in the IDLE state
	          robotState = STATE_IDLE;
	      }

	      // The main loop is not a busy loop, it just waits for the next tick
	      // if the robot is idle.
	      HAL_Delay(10);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4095;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4095;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LMOTOR_IN1_GPIO_Port, LMOTOR_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LMOTOR_IN2_Pin|RMOTOR_IN1_Pin|RMOTOR_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LC1_Pin */
  GPIO_InitStruct.Pin = LC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LC1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LMOTOR_IN1_Pin */
  GPIO_InitStruct.Pin = LMOTOR_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LMOTOR_IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LMOTOR_IN2_Pin RMOTOR_IN1_Pin RMOTOR_IN2_Pin */
  GPIO_InitStruct.Pin = LMOTOR_IN2_Pin|RMOTOR_IN1_Pin|RMOTOR_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RC1_Pin */
  GPIO_InitStruct.Pin = RC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RC1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_Pin DRY_MAIN_BUTTON_Pin GO_HALT_BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin|DRY_MAIN_BUTTON_Pin|GO_HALT_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void readAllIRSensors(void) {
    HAL_ADC_Start(&hadc1);

    HAL_ADC_PollForConversion(&hadc1, 100);
    IR1 = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_PollForConversion(&hadc1, 100);
    IR2 = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_PollForConversion(&hadc1, 100);
    IR3 = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_PollForConversion(&hadc1, 100);
    IR4 = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);
}

void calculateAndSetSpeeds(float a, float b) {
    static uint32_t lastControlTick = 0;
    if (HAL_GetTick() - lastControlTick >= CONTROL_LOOP_PERIOD_MS) {
        lastControlTick = HAL_GetTick();

        leftSetpoint = a;
        rightSetpoint = b;

        Lspeed = calculateLeftPID(a);
        Rspeed = calculateRightPID(b);

        setMotorPWM(Lspeed, Rspeed);
    }
}

void setMotorPWM(int16_t leftPWM, int16_t rightPWM) {
    LMotorSpeed(leftPWM);
    RMotorSpeed(rightPWM);
}

int16_t calculateLeftPID(float leftSetPoint) {
    static int32_t leftLastCount = 0;
    float leftVelocityMps = 0.0f;
    float leftDeltaCount = (float)(leftEncoderCount - leftLastCount);
    float leftDeltaTime_s = CONTROL_LOOP_PERIOD_MS / 1000.0f;
    float wheelCircumference = r * 3.14159f;

    leftVelocityMps = (leftDeltaCount / ENCODER_CPR) * (wheelCircumference / leftDeltaTime_s);
    leftLastCount = leftEncoderCount;

    leftError = leftSetpoint - leftVelocityMps;
    leftIntegral += leftError;

    if (leftIntegral > PWM_RANGE) leftIntegral = PWM_RANGE;
    if (leftIntegral < -PWM_RANGE) leftIntegral = -PWM_RANGE;

    float proportional = KP_LEFT * leftError;
    float integral_term = KI_LEFT * leftIntegral;
    float derivative_term = KD_LEFT * (leftError - leftLastError);

    int16_t outputPWM = (int16_t)(proportional + integral_term + derivative_term);
    if (outputPWM > PWM_RANGE) outputPWM = PWM_RANGE;
    if (outputPWM < -PWM_RANGE) outputPWM = -PWM_RANGE;

    leftLastError = leftError;
    return outputPWM;
}

int16_t calculateRightPID(float rightSetPoint) {
    static int32_t rightLastCount = 0;
    float rightVelocityMps = 0.0f;
    float rightDeltaCount = (float)(rightEncoderCount - rightLastCount);
    float rightDeltaTime_s = CONTROL_LOOP_PERIOD_MS / 1000.0f;
    float wheelCircumference = r * 3.14159f;

    rightVelocityMps = (rightDeltaCount / ENCODER_CPR) * (wheelCircumference / rightDeltaTime_s);
    rightLastCount = rightEncoderCount;

    rightError = rightSetpoint - rightVelocityMps;
    rightIntegral += rightError;

    if (rightIntegral > PWM_RANGE) rightIntegral = PWM_RANGE;
    if (rightIntegral < -PWM_RANGE) rightIntegral = -PWM_RANGE;

    float proportional = KP_RIGHT * rightError;
    float integral_term = KI_RIGHT * rightIntegral;
    float derivative_term = KD_RIGHT * (rightError - rightLastError);

    int16_t outputPWM = (int16_t)(proportional + integral_term + derivative_term);
    if (outputPWM > PWM_RANGE) outputPWM = PWM_RANGE;
    if (outputPWM < -PWM_RANGE) outputPWM = -PWM_RANGE;

    rightLastError = rightError;
    return outputPWM;
}

void LMotorSpeed(int Lspeed_param) {
    if (Lspeed_param > 4095) {
        Lspeed_param = 4095;
    } else if (Lspeed_param < -4095) {
        Lspeed_param = -4095;
    }

    if (Lspeed_param > 0) {
        HAL_GPIO_WritePin(LMOTOR_IN1_GPIO_Port, LMOTOR_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LMOTOR_IN2_GPIO_Port, LMOTOR_IN2_Pin, GPIO_PIN_RESET);
    } else if (Lspeed_param < 0) {
        HAL_GPIO_WritePin(LMOTOR_IN1_GPIO_Port, LMOTOR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LMOTOR_IN2_GPIO_Port, LMOTOR_IN2_Pin, GPIO_PIN_SET);
        Lspeed_param = -Lspeed_param;
    } else {
        HAL_GPIO_WritePin(LMOTOR_IN1_GPIO_Port, LMOTOR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LMOTOR_IN2_GPIO_Port, LMOTOR_IN2_Pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Lspeed_param);
}

void RMotorSpeed(int Rspeed_param) {
    if (Rspeed_param > 4095) {
        Rspeed_param = 4095;
    } else if (Rspeed_param < -4095) {
        Rspeed_param = -4095;
    }

    if (Rspeed_param > 0) {
        HAL_GPIO_WritePin(RMOTOR_IN1_GPIO_Port, RMOTOR_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RMOTOR_IN2_GPIO_Port, RMOTOR_IN2_Pin, GPIO_PIN_RESET);
    } else if (Rspeed_param < 0) {
        HAL_GPIO_WritePin(RMOTOR_IN1_GPIO_Port, RMOTOR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RMOTOR_IN2_GPIO_Port, RMOTOR_IN2_Pin, GPIO_PIN_SET);
        Rspeed_param = -Rspeed_param;
    } else {
        HAL_GPIO_WritePin(RMOTOR_IN1_GPIO_Port, RMOTOR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RMOTOR_IN2_GPIO_Port, RMOTOR_IN2_Pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Rspeed_param);
}

void MPU6050_Init(void)
{
    uint8_t check;
    uint8_t Data;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
    if (check == 0x68) {
        Data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &Data, 1, 1000);
        Data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x19, 1, &Data, 1, 1000);
        Data = 0x01;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
    }
}

void MPU6050_Read_Gyro(void)
{
    uint8_t Rec_Data[6];

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, 1000);

    Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    Gx = (float)Gyro_X_RAW / 65.50f;
    Gy = (float)Gyro_Y_RAW / 65.50f;
    Gz = (float)Gyro_Z_RAW / 65.50f;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LC1_Pin) {
        if (HAL_GPIO_ReadPin(LC2_GPIO_Port, LC2_Pin) == GPIO_PIN_SET) {
            leftEncoderCount++;
        } else {
            leftEncoderCount--;
        }
    } else if (GPIO_Pin == LC2_Pin) {
        if (HAL_GPIO_ReadPin(LC1_GPIO_Port, LC1_Pin) == GPIO_PIN_RESET) {
            leftEncoderCount++;
        } else {
            leftEncoderCount--;
        }
    } else if (GPIO_Pin == RC1_Pin) {
        if (HAL_GPIO_ReadPin(RC2_GPIO_Port, RC2_Pin) == GPIO_PIN_SET) {
            rightEncoderCount++;
        } else {
            rightEncoderCount--;
        }
    } else if (GPIO_Pin == RC2_Pin) {
        if (HAL_GPIO_ReadPin(RC1_GPIO_Port, RC1_Pin) == GPIO_PIN_RESET) {
            rightEncoderCount++;
        } else {
            rightEncoderCount--;
        }
    }
}

void getEncoderValue(void) {
    if (HAL_GetTick() - leftLastTick >= 10) {
        float leftDeltaCount = (float)(leftEncoderCount - leftLastCount);
        float leftDeltaTime = (float)(HAL_GetTick() - leftLastTick) / MS_PER_SEC;

        leftVelocity = r * (leftDeltaCount / ENCODER_PPR) * (TWO_PI / leftDeltaTime);
        leftDistancem = ((float)leftEncoderCount / ENCODER_CPR) * r;

        leftLastCount = leftEncoderCount;
        leftLastTick = HAL_GetTick();
    }

    if (HAL_GetTick() - rightLastTick >= 10) {
        float rightDeltaCount = (float)(rightEncoderCount - rightLastCount);
        float rightDeltaTime = (float)(HAL_GetTick() - rightLastTick) / MS_PER_SEC;

        rightVelocity = r * (rightDeltaCount / ENCODER_PPR) * (TWO_PI / rightDeltaTime);
        rightDistancem = ((float)rightEncoderCount / ENCODER_CPR) * r;

        rightLastCount = rightEncoderCount;
        rightLastTick = HAL_GetTick();
    }
}

void Actuator_MoveForwardOneCell(void) {
    float target_distance = (c == 1) ? 0.125f : 0.25f;

    // Reset encoder distances
    leftDistancem = 0.0f;
    rightDistancem = 0.0f;
    leftEncoderCount = 0;
    rightEncoderCount = 0;

    while (rightDistancem < target_distance) {
        readAllIRSensors();
        error = (float)(IR1 - IR4);
        getEncoderValue();

        float base_speed = 0.5f;
        float left_adjustment = 0.0f;
        float right_adjustment = 0.0f;

        if (IR1 > IR4) {
            left_adjustment = error * KP;
            right_adjustment = -error * KP;
        } else if (IR1 < IR4) {
            left_adjustment = -error * KP;
            right_adjustment = error * KP;
        }

        calculateAndSetSpeeds(base_speed + left_adjustment, base_speed + right_adjustment);
        HAL_Delay(1);
    }

    // Stop motors
    calculateAndSetSpeeds(0.0f, 0.0f);
    c++;
}

void Actuator_TurnRight(int t) {
    MPU6050_Read_Gyro();
    Gy0 = Gy;
    float target_angle = (t == 1) ? 90.0f : 180.0f;

    while (fabsf(Gy - Gy0) < target_angle) {
        MPU6050_Read_Gyro();
        if (t == 1) {
            calculateAndSetSpeeds(0.64f, 0.36f);
        } else {
            calculateAndSetSpeeds(-0.5f, 0.5f);
        }
        HAL_Delay(10);
    }

    // Stop motors
    calculateAndSetSpeeds(0.0f, 0.0f);
}

void Actuator_TurnLeft(void) {
    MPU6050_Read_Gyro();
    Gy0 = Gy;

    while (fabsf(Gy - Gy0) < 90.0f) {
        MPU6050_Read_Gyro();
        calculateAndSetSpeeds(0.36f, 0.64f);
        HAL_Delay(10);
    }

    // Stop motors
    calculateAndSetSpeeds(0.0f, 0.0f);
}

/* ======= UTILS ======= */
static inline void UART_Print(const char* s) {
    HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 100);
}

static inline bool in_bounds(int x, int y) {
    return (x >= 0 && x < N && y >= 0 && y < N);
}

static inline void queue_reset(void) { q_front = q_rear = 0; }
static inline bool queue_empty(void) { return q_front == q_rear; }
static inline void enqueue(uint8_t x, uint8_t y) {
    qx[q_rear] = x; qy[q_rear] = y;
    q_rear = (uint16_t)((q_rear + 1) % MAX_QUEUE);
}
static inline void dequeue(uint8_t* x, uint8_t* y) {
    *x = qx[q_front]; *y = qy[q_front];
    q_front = (uint16_t)((q_front + 1) % MAX_QUEUE);
}

static inline uint32_t rng_uniform(uint32_t m) { return (m ? (rng_next() % m) : 0u); }

/* ======= WALL HELPERS ======= */
static inline bool hasWallBetween(int x, int y, int dir) {
    if (!in_bounds(x,y)) return true;
    switch (dir) {
        case NORTH: return (y + 1 <= N) ? hWalls[x][y + 1] : true;
        case SOUTH: return (y > 0)      ? hWalls[x][y]       : true;
        case EAST:  return (x + 1 <= N) ? vWalls[x + 1][y] : true;
        case WEST:  return (x > 0)      ? vWalls[x][y]       : true;
        default:    return true;
    }
}

static inline void setWall(int x, int y, int dir) {
    if (!in_bounds(x,y)) return;
    switch (dir) {
        case NORTH: if (y + 1 < N + 1) hWalls[x][y + 1] = true; break;
        case SOUTH: if (y > 0)         hWalls[x][y]     = true; break;
        case EAST:  if (x + 1 < N + 1) vWalls[x + 1][y] = true; break;
        case WEST:  if (x > 0)         vWalls[x][y]     = true; break;
    }
}

static inline void clearWall(int x, int y, int dir) {
    if (!in_bounds(x,y)) return;
    switch (dir) {
        case NORTH: if (y + 1 < N + 1) hWalls[x][y + 1] = false; break;
        case SOUTH: if (y > 0)         hWalls[x][y]     = false; break;
        case EAST:  if (x + 1 < N + 1) vWalls[x + 1][y] = false; break;
        case WEST:  if (x > 0)         vWalls[x][y]     = false; break;
    }
}

/* ======= MAP/DIST INITIALIZATION ======= */
static void reset_all(void) {
    for (int x = 0; x < N; x++) {
        for (int y = 0; y < N; y++) {
            dist_map[x][y] = 0xFFFF;
            visited[x][y] = false;
        }
    }

    for (int x = 0; x <= N; x++) {
        for (int y = 0; y < N; y++){
            vWalls[x][y] = (x == 0 || x == N);
        }
    }

    for (int x = 0; x < N; x++){
        for (int y = 0; y <= N; y++){
            hWalls[x][y] = (y == 0 || y == N);
        }
    }

    currentPath.len = 0;
    pathToGoal.len = 0;
    pathToStart.len = 0;
    exploredCount = 0;

    robotX = 0; robotY = 0; robotHeading = NORTH;
    rng_seed(HAL_GetTick());
}

static void resetVisited(void) {
    for (int x = 0; x < N; x++){
        for (int y = 0; y < N; y++){
            visited[x][y] = false;
        }
    }
}

/* ======= GOALS ======= */
static void setGoals_center(void) {
    goals_xy[0][0] = 7; goals_xy[0][1] = 7;
    goals_xy[1][0] = 7; goals_xy[1][1] = 8;
    goals_xy[2][0] = 8; goals_xy[2][1] = 7;
    goals_xy[3][0] = 8; goals_xy[3][1] = 8;
    goal_count = 4;
}

static inline bool isGoal(int x, int y) {
    for (uint8_t i = 0; i < goal_count; i++) {
        if (goals_xy[i][0] == x && goals_xy[i][1] == y) return true;
    }
    return false;
}

/* ======= BFS FLOOD-FILL ======= */
static void updateDistances(void) {
    for (int x = 0; x < N; x++){
        for (int y = 0; y < N; y++){
            dist_map[x][y] = 0xFFFF;
        }
    }

    queue_reset();

    for (uint8_t i = 0; i < goal_count; i++) {
        uint8_t gx = goals_xy[i][0], gy = goals_xy[i][1];
        if (in_bounds(gx, gy)) {
            dist_map[gx][gy] = 0;
            enqueue(gx, gy);
        }
    }

    while (!queue_empty()) {
        uint8_t cx, cy;
        dequeue(&cx, &cy);

        uint16_t cd = dist_map[cx][cy];
        for (int dir = 0; dir < 4; dir++) {
            int nx = cx + dx[dir];
            int ny = cy + dy[dir];
            if (!in_bounds(nx, ny)) continue;
            if (hasWallBetween(cx, cy, dir)) continue;

            uint16_t nd = (uint16_t)(cd + 1);
            if (dist_map[nx][ny] > nd) {
                dist_map[nx][ny] = nd;
                enqueue((uint8_t)nx, (uint8_t)ny);
            }
        }
    }
}

static void displayDistances(void) {
    char buf[16];
    for (int y = N-1; y >= 0; y--) {
        for (int x = 0; x < N; x++) {
            uint16_t d = dist_map[x][y];
            if (d == 0xFFFF) {
                strcpy(buf, "INF ");
            } else {
                snprintf(buf, sizeof(buf), "%3u ", (unsigned)d);
            }
            UART_Print(buf);
        }
        UART_Print("\r\n");
    }
    UART_Print("----\r\n");
}

/* ======= MOVE SELECTION ======= */
static int getNextDirection(int x, int y) {
    uint16_t bestDist = 0xFFFF;
    int bestDir = -1;
    for (int dir = 0; dir < 4; dir++) {
        int nx = x + dx[dir];
        int ny = y + dy[dir];
        if (!in_bounds(nx, ny)) continue;
        if (hasWallBetween(x, y, dir)) continue;
        if (dist_map[nx][ny] < bestDist) {
            bestDist = dist_map[nx][ny];
            bestDir = dir;
        }
    }
    return bestDir;
}

/* ======= SENSORS & ACTUATORS ======= */
static bool Sensor_WallFront(void) {
    readAllIRSensors();
    if(IR1 > (uint32_t)ThresholdFront || IR4 > (uint32_t)ThresholdFront){
        return true;
    }
    return false;
}

static bool Sensor_WallLeft(void) {
    readAllIRSensors();
    if(IR2 > (uint32_t)ThresholdSide){
        return true;
    }
    return false;
}

static bool Sensor_WallRight(void) {
    readAllIRSensors();
    if(IR3 > (uint32_t)ThresholdSide){
        return true;
    }
    return false;
}

/* ======= POSE/CONTROL HELPERS ======= */
static void executeTurn(int newDir) {
    int turn = (newDir - robotHeading + 4) % 4;
    switch (turn) {
        case 1:  Actuator_TurnRight(1); break;
        case 2:  Actuator_TurnRight(2); break;
        case 3:  Actuator_TurnLeft();  break;
        case 0:  default: break;
    }
    robotHeading = newDir;
}

static bool moveForward(void) {
    int nx = robotX + dx[robotHeading];
    int ny = robotY + dy[robotHeading];

    if (!in_bounds(nx, ny)) {
        UART_Print("Invalid move (OOB)\r\n");
        return false;
    }
    if (hasWallBetween(robotX, robotY, robotHeading)) {
        UART_Print("Blocked by wall\r\n");
        return false;
    }

    Actuator_MoveForwardOneCell();

    robotX = nx; robotY = ny;

    if (currentPath.len < MAX_PATH_LEN) {
        currentPath.x[currentPath.len] = (uint8_t)robotX;
        currentPath.y[currentPath.len] = (uint8_t)robotY;
        currentPath.len++;
    }
    return true;
}

/* ======= SENSOR FUSION TO WALL MAP ======= */
static void updateWallsFromSensors(void) {
    int x = robotX, y = robotY, h = robotHeading;

    if (Sensor_WallFront()) setWall(x, y, h);
    else                    clearWall(x, y, h);

    int leftDir = (h + 3) & 3;
    if (Sensor_WallLeft()) setWall(x, y, leftDir);
    else                   clearWall(x, y, leftDir);

    int rightDir = (h + 1) & 3;
    if (Sensor_WallRight()) setWall(x, y, rightDir);
    else                    clearWall(x, y, rightDir);
}

/* ======= VARIATION / TIE-BREAKING ======= */
static int selectDirectionWithVariation(Candidate* cands, int count) {
    if (count <= 0) return -1;

    uint16_t minD = 0xFFFF;
    for (int i = 0; i < count; i++) {
        if (cands[i].d < minD) minD = cands[i].d;
    }

    int picks[4]; int pc = 0;
    for (int i = 0; i < count; i++) {
        if (cands[i].d <= (uint16_t)(minD + 1)) {
            if (pc < 4) picks[pc++] = cands[i].dir;
        }
    }
    if (pc == 0) return -1;
    return picks[rng_uniform((uint32_t)pc)];
}

static bool isInPreviousPaths(int nx, int ny) {
    for (uint8_t p = 0; p < exploredCount; p++) {
        for (uint16_t i = 0; i < exploredPaths[p].len; i++) {
            if (exploredPaths[p].x[i] == nx && exploredPaths[p].y[i] == ny) return true;
        }
    }
    return false;
}

static void storeCurrentPath(void) {
    if (exploredCount >= MAX_STORED_PATHS) return;
    exploredPaths[exploredCount] = currentPath;
    exploredCount++;
}

static int getFallbackMoves(Candidate* cands) {
    int cc = 0;
    for (int dir = 0; dir < 4; dir++) {
        int nx = robotX + dx[dir];
        int ny = robotY + dy[dir];
        if (!in_bounds(nx, ny)) continue;
        if (hasWallBetween(robotX, robotY, dir)) continue;
        if (cc < 4) { cands[cc].dir = dir; cands[cc].d = dist_map[nx][ny]; cc++; }
    }
    return cc;
}

/* ======= HIGH-LEVEL RUNS ======= */
static void exploreWithVariation(bool avoidPreviousPaths) {
    resetVisited();
    currentPath.len = 0;

    if (currentPath.len < MAX_PATH_LEN) {
        currentPath.x[currentPath.len] = (uint8_t)robotX;
        currentPath.y[currentPath.len] = (uint8_t)robotY;
        currentPath.len++;
    }

    int moveCount = 0;
    while (!isGoal(robotX, robotY) && moveCount < MAX_MOVES) {
        updateWallsFromSensors();
        updateDistances();
        visited[robotX][robotY] = true;

        Candidate cands[4]; int cc = 0;

        for (int dir = 0; dir < 4; dir++) {
            int nx = robotX + dx[dir];
            int ny = robotY + dy[dir];
            if (!in_bounds(nx, ny)) continue;
            if (hasWallBetween(robotX, robotY, dir)) continue;

            if (avoidPreviousPaths && visited[nx][ny]) continue;
            if (avoidPreviousPaths && isInPreviousPaths(nx, ny)) continue;

            if (cc < 4) { cands[cc].dir = dir; cands[cc].d = dist_map[nx][ny]; cc++; }
        }

        if (cc == 0) {
            if (avoidPreviousPaths) {
                cc = getFallbackMoves(cands);
            } else {
                cc = getFallbackMoves(cands);
            }
            if (cc == 0) {
                UART_Print("No candidates; stuck.\r\n");
                break;
            }
        }

        int nextDir = selectDirectionWithVariation(cands, cc);
        if (nextDir < 0) break;

        executeTurn(nextDir);
        if(nextDir == 2){
            if (!moveForward()) {
                updateWallsFromSensors();
                updateDistances();
            }
        }
        else{
            updateWallsFromSensors();
            updateDistances();
        }
        moveCount++;
    }

    if (isGoal(robotX, robotY)) {
        pathToGoal = currentPath;
        storeCurrentPath();
        currentPath.len = 0;
        UART_Print("Exploration reached goal.\r\n");
    } else {
        UART_Print("Exploration terminated.\r\n");
    }
}

static void returnToStart(void) {
    currentPath.len = 0;
    if (currentPath.len < MAX_PATH_LEN) {
        currentPath.x[currentPath.len] = (uint8_t)robotX;
        currentPath.y[currentPath.len] = (uint8_t)robotY;
        currentPath.len++;
    }

    goals_xy[0][0] = 0; goals_xy[0][1] = 0; goal_count = 1;

    int moveCount = 0;
    while ((robotX != 0 || robotY != 0) && moveCount < MAX_MOVES) {
        updateWallsFromSensors();
        updateDistances();

        int nextDir = getNextDirection(robotX, robotY);
        if (nextDir < 0) break;

        executeTurn(nextDir);
        if (!moveForward()) {
            updateWallsFromSensors();
            updateDistances();
        }
        moveCount++;
    }

    if (robotX == 0 && robotY == 0) {
        pathToStart = currentPath;
        storeCurrentPath();
        currentPath.len = 0;
        UART_Print("Returned to start.\r\n");
    } else {
        UART_Print("Return-to-start terminated.\r\n");
    }
}

static void followShortestPathToGoal(void) {
    resetVisited();
    currentPath.len = 0;
    if (currentPath.len < MAX_PATH_LEN) {
        currentPath.x[currentPath.len] = (uint8_t)robotX;
        currentPath.y[currentPath.len] = (uint8_t)robotY;
        currentPath.len++;
    }

    int moveCount = 0;
    while (!isGoal(robotX, robotY) && moveCount < MAX_MOVES) {
        updateWallsFromSensors();
        updateDistances();

        int nextDir = getNextDirection(robotX, robotY);
        if (nextDir < 0) {
            UART_Print("No path to goal.\r\n");
            break;
        }

        executeTurn(nextDir);
        if (!moveForward()) {
            updateWallsFromSensors();
            updateDistances();
        }
        moveCount++;
    }

    if (isGoal(robotX, robotY)) {
        pathToGoal = currentPath;
        storeCurrentPath();
        currentPath.len = 0;
        UART_Print("Shortest path run done.\r\n");
    }
}

// Add the missing peripheral initialization functions




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
#endif /* USE_FULL_ASSERT */
