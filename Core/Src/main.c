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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_MAX 999
#define DEADZONE 50
#define MODE_MANUAL (1 << 0)
#define MODE_AUTO   (1 << 1)

#define AUTO_STOP_CM     15.0f
#define AUTO_SLOW_CM     30.0f

#define AUTO_FAST_SPEED  0.6f
#define AUTO_SLOW_SPEED  0.3f
#define AUTO_TURN_SPEED  0.4f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TestTask01 */
osThreadId_t TestTask01Handle;
const osThreadAttr_t TestTask01_attributes = {
  .name = "TestTask01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PWM */
osThreadId_t PWMHandle;
const osThreadAttr_t PWM_attributes = {
  .name = "PWM",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SensorUTask01 */
osThreadId_t SensorUTask01Handle;
const osThreadAttr_t SensorUTask01_attributes = {
  .name = "SensorUTask01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for myQueue02 */
osMessageQueueId_t myQueue02Handle;
const osMessageQueueAttr_t myQueue02_attributes = {
  .name = "myQueue02"
};
/* Definitions for controlMutex */
osMutexId_t controlMutexHandle;
const osMutexAttr_t controlMutex_attributes = {
  .name = "controlMutex"
};
/* Definitions for sensorMutex */
osMutexId_t sensorMutexHandle;
const osMutexAttr_t sensorMutex_attributes = {
  .name = "sensorMutex"
};
/* Definitions for modeFlags */
osEventFlagsId_t modeFlagsHandle;
const osEventFlagsAttr_t modeFlags_attributes = {
  .name = "modeFlags"
};
/* USER CODE BEGIN PV */
volatile uint8_t rx_byte;
uint8_t joy_x=50;   // 0–100
uint8_t joy_y=50;   // 0–100

volatile uint32_t ic_rise = 0;
volatile uint32_t ic_fall = 0;
float Distance_cm = 50.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void *argument);
void TestTask(void *argument);
void PWMTask(void *argument);
void SensorUTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline uint32_t map_float_to_pwm(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return (uint32_t)(v * PWM_MAX);
}

void DriveFromJoystick(uint8_t x, uint8_t y)
{
    float speed, turn;
    float left, right;

    if (x == 50 && y == 50)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
		return;
	}

    if (y > DEADZONE)
		speed = (float)(y - DEADZONE) / (100.0f - DEADZONE); // 0..1
	else
		speed = 0.0f;


    if (speed < 0.05f && fabs(turn) > 0.05f)
    {
		// Giro en casi parado → solo una rueda
		if (turn > 0)
		{
			left  = 0.0f;
			right = fabs(turn);
		}
		else
		{
			left  = fabs(turn);
			right = 0.0f;
		}
    }
    else
    {
        // Avance normal con giro
        left  = speed * (1.0f - turn);
        right = speed * (1.0f + turn);

        // Asegurar rango 0..1
        if (left  < 0) left  = 0;
        if (right < 0) right = 0;
        if (left  > 1) left  = 1;
        if (right > 1) right = 1;
    }

    // Aplicar PWM
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, map_float_to_pwm(left));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, map_float_to_pwm(right));
}

void DriveAuto(void)
{
    float distance;
    float left = 0.0f, right = 0.0f;

    /* Leer distancia protegida */
    osMutexAcquire(controlMutexHandle, osWaitForever);
    distance = Distance_cm;
    osMutexRelease(controlMutexHandle);

    if (distance <= 0.0f)
    {
        left = 0.0f;
        right = 0.0f;
    }
    else if (distance < AUTO_STOP_CM)
    {
        // Obstáculo muy cerca → girar
        left  = AUTO_TURN_SPEED;
        right = 0.0f;
    }
    else if (distance < AUTO_SLOW_CM)
    {
        // Cerca → avanzar lento
        left  = AUTO_SLOW_SPEED;
        right = AUTO_SLOW_SPEED;
    }
    else
    {
        // Libre → avanzar normal
        left  = AUTO_FAST_SPEED;
        right = AUTO_FAST_SPEED;
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, map_float_to_pwm(left));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, map_float_to_pwm(right));
}

int parseXY(const char *buf, uint8_t *x, uint8_t *y)
{
    if (buf[0] != 'X') return 0;

    int i = 1;
    int vx = 0, vy = 0;

    if (buf[i] < '0' || buf[i] > '9') return 0;

    while (buf[i] >= '0' && buf[i] <= '9')
        vx = vx * 10 + (buf[i++] - '0');

    if (buf[i++] != 'Y') return 0;

    if (buf[i] < '0' || buf[i] > '9') return 0;

    while (buf[i] >= '0' && buf[i] <= '9')
        vy = vy * 10 + (buf[i++] - '0');

    if (vx > 100 || vy > 100) return 0;

    *x = vx;
    *y = vy;
    return 1;
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of controlMutex */
  controlMutexHandle = osMutexNew(&controlMutex_attributes);

  /* creation of sensorMutex */
  sensorMutexHandle = osMutexNew(&sensorMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (32, sizeof(char), &myQueue01_attributes);

  /* creation of myQueue02 */
  myQueue02Handle = osMessageQueueNew (4, sizeof(uint32_t), &myQueue02_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TestTask01 */
  TestTask01Handle = osThreadNew(TestTask, NULL, &TestTask01_attributes);

  /* creation of PWM */
  PWMHandle = osThreadNew(PWMTask, NULL, &PWM_attributes);

  /* creation of SensorUTask01 */
  SensorUTask01Handle = osThreadNew(SensorUTask, NULL, &SensorUTask01_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of modeFlags */
  modeFlagsHandle = osEventFlagsNew(&modeFlags_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  osEventFlagsSet(modeFlagsHandle, MODE_MANUAL);
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
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
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		char c = rx_byte;

		// Meter el carácter en la cola SIN BLOQUEO (desde ISR)
		osMessageQueuePut(myQueue01Handle, &c, 0, 0);

		// Reiniciar recepción
		HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM4) return;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{

		static uint32_t ic_rise = 0;
        static uint8_t waiting_rise = 1;  // empezar esperando subida
        uint32_t ic_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

		if (waiting_rise) // pin ECHO, HIGH = subida
		{
			ic_rise = ic_val;
			waiting_rise  = 0;
		}
		else
		{
			uint32_t diff;
			if (ic_val >= ic_rise)
				diff = ic_val - ic_rise;
			else
				diff = (htim->Init.Period - ic_rise) + ic_val;

			osMessageQueuePut(myQueue02Handle, &diff, 0, 0);
			waiting_rise = 1;
		}
	}
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_TestTask */
/**
 * @brief Function implementing the TestTask01 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TestTask */
void TestTask(void *argument)
{
  /* USER CODE BEGIN TestTask */
	/* Infinite loop */
	char c;
	char buffer[20];
	uint8_t idx = 0;

	for (;;) {
		// Espera bloqueante: eficiente
		if (osMessageQueueGet(myQueue01Handle, &c, NULL, osWaitForever)== osOK) {
			// 1) Modo M / A
			if (c == 'C') {
				osEventFlagsClear(modeFlagsHandle, MODE_AUTO);
				osEventFlagsSet(modeFlagsHandle, MODE_MANUAL);
			} else if (c == 'c') {
			    osEventFlagsClear(modeFlagsHandle, MODE_MANUAL);
			    osEventFlagsSet(modeFlagsHandle, MODE_AUTO);
			}

			// 2) Joystick Xnnn,Ynnn
			else {
				if ((c == '\n') || (c == '\r') || (c == '>')) {
					buffer[idx] = '\0';

					uint8_t x, y;
					if (parseXY(buffer, &x, &y)) {
						osMutexAcquire(controlMutexHandle, osWaitForever);
						joy_x = abs((int)x - 100);
						joy_y = abs((int)y - 100);
						osMutexRelease(controlMutexHandle);
					}
					memset(buffer, 0, sizeof(buffer));
					idx = 0;
				} else if (idx < sizeof(buffer) - 1) {
					buffer[idx++] = c;
				} else {
				    memset(buffer, 0, sizeof(buffer));
				    idx = 0;
				}
			}
		}
	}
  /* USER CODE END TestTask */
}

/* USER CODE BEGIN Header_PWMTask */
/**
* @brief Function implementing the PWM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PWMTask */
void PWMTask(void *argument)
{
  /* USER CODE BEGIN PWMTask */
  /* Infinite loop */
    uint8_t x, y;
    uint32_t flags;
    for(;;)
    {
        flags = osEventFlagsWait(
					modeFlagsHandle,
					MODE_MANUAL | MODE_AUTO,
					osFlagsWaitAny | osFlagsNoClear,
					osWaitForever
				);

        // ===== MODO MANUAL =====
		if (flags & MODE_MANUAL)
		{
			for (;;)
			{
				flags = osEventFlagsGet(modeFlagsHandle);
				if (!(flags & MODE_MANUAL))
					break;

				osMutexAcquire(controlMutexHandle, osWaitForever);
				x = joy_x;
				y = joy_y;
				osMutexRelease(controlMutexHandle);

				DriveFromJoystick(x, y);

				osDelay(20);   // 50 Hz
			}
		}
		// ===== MODO AUTOM�?TICO =====
		if (flags & MODE_AUTO)
		{
			for (;;)
			{
				flags = osEventFlagsGet(modeFlagsHandle);
				if (!(flags & MODE_AUTO))
					break;

				DriveAuto();

				osDelay(500);   // control más lento
			}
		}
    }
  /* USER CODE END PWMTask */
}

/* USER CODE BEGIN Header_SensorUTask */
/**
* @brief Function implementing the SensorUTask01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorUTask */
void SensorUTask(void *argument)
{
  /* USER CODE BEGIN SensorUTask */
	uint32_t diff;
    uint32_t flags;
    float distance;

	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

	for(;;)
	{
		flags = osEventFlagsWait(
		                    modeFlagsHandle,
		                    MODE_AUTO,
		                    osFlagsWaitAny | osFlagsNoClear,
		                    osWaitForever
		                );

		if (flags & MODE_AUTO) {
		// 1. Resetear el flag de medición
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			osDelay(1); // Usamos FreeRTOS delay (1ms > 10us)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

			//if (osMessageQueueGet(myQueue02Handle, &diff, NULL, osWaitForever) == osOK)
			if (osMessageQueueGet(myQueue02Handle, &diff, NULL, 100) == osOK)
			{
				distance = diff / 58.0f;
				osMutexAcquire(controlMutexHandle, osWaitForever);
				Distance_cm = distance;
				osMutexRelease(controlMutexHandle);
			}

			osDelay(150); // Medir cada 500ms
		}
	}
  /* USER CODE END SensorUTask */
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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
