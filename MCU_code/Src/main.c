/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "udp.h"
#include "stdio.h"
#include "string.h"
#include "float.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

struct position_cnt {
	uint16_t old_cnt;
	uint16_t new_cnt;
	uint16_t delta_cnt;
	long int position_old;
	long int position_new;
	float deg_position;
	int direction;
	float deg_desired_position;
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NPULSES 4096		// Impulsi per compiere un giro
#define K 3					// Costante controllo proporzionale
#define P_MAX 50000			// Max cunter period nella modalità levetta
#define P_MIN 245			// Min counter period (dovuto a max 3000 rpm motore)
#define CLOCK 100000000		// Frequenza di clock del micro (10^8 Hz)
#define GEARBOX_RATIO 20	// Rapporto di riduzione riduttore


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for Main_Task */
osThreadId_t Main_TaskHandle;
const osThreadAttr_t Main_Task_attributes = {
  .name = "Main_Task",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 512 * 4
};
/* Definitions for UDP_Comm */
osThreadId_t UDP_CommHandle;
const osThreadAttr_t UDP_Comm_attributes = {
  .name = "UDP_Comm",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 4
};
/* Definitions for Second_Task */
osThreadId_t Second_TaskHandle;
const osThreadAttr_t Second_Task_attributes = {
  .name = "Second_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* USER CODE BEGIN PV */

uint8_t light_status=0;
UART_HandleTypeDef *SERIALDEBUG = &huart3;
uint8_t com_is_ok=0;
uint8_t external_button=0;
int watchdog_max=100;
int my_watchdog=-1;
float my_float_1=0;
float my_float_2=0;

uint8_t prova_int[] = {0,0,0,0,0};
float prova_single[] = {0.0,0.0};
uint16_t my_period = 0;
uint8_t control_mode = 0;			// Modalità motore
struct position_cnt p = {0,0,0,0,0,0.0,1,0.0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM4_Init(void);
void MainTask(void *argument);
void UDPCommunication(void *argument);
void SecondTask(void *argument);

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim9);

  my_watchdog=watchdog_max;
  uint8_t numofstartupblinks;
  for (numofstartupblinks=0; numofstartupblinks<3; numofstartupblinks++) {
  	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
  	  HAL_Delay(50);

  	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
  	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
  	  HAL_Delay(100);
  	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
  	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
  	  HAL_Delay(100);

  	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
  	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
  	  HAL_Delay(100);
  	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
  	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
  	  HAL_Delay(100);
  }


  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Main_Task */
  Main_TaskHandle = osThreadNew(MainTask, NULL, &Main_Task_attributes);

  /* creation of UDP_Comm */
  UDP_CommHandle = osThreadNew(UDPCommunication, NULL, &UDP_Comm_attributes);

  /* creation of Second_Task */
  Second_TaskHandle = osThreadNew(SecondTask, NULL, &Second_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim9, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin|Dir_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin Dir_1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin|Dir_1_Pin;
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

}

/* USER CODE BEGIN 4 */

void personal_switch (uint8_t light){

	switch (light%8){
			      	  case 0:
			    		  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,0);
			    	  	  HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,0);
			    	  	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,0);
			    	  	  break;
			      	  case 1:
			    		  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,1);
			    	  	  HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,0);
			    	  	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,0);
			    	  	  break;
			      	  case 2:
			    		  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,0);
			    	  	  HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,0);
			    	  	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,1);
			    	  	  break;
			      	  case 3:
			    		  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,1);
			    	  	  HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,0);
			    	  	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,1);
			    	  	  break;
			      	  case 4:
			    		  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,0);
			    	  	  HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,1);
			    	  	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,0);
			    	  	  break;
			      	  case 5:
			    		  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,1);
			    	  	  HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,1);
			    	  	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,0);
			    	  	  break;
			      	  case 6:
			    		  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,0);
			    	  	  HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,1);
			    	  	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,1);
			    	  	  break;
			      	  case 7:
			    		  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,1);
			    	  	  HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,1);
			    	  	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,1);
			    	  	  break;
			      	  default:
			    		  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,0);
			    	  	  HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,0);
			    	  	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,0);
			    	  	  break;
			      }
}

int motor_step(uint8_t npulse, uint16_t cp, uint8_t direct){

	uint16_t cinit = htim9.Instance->CNT;

	while(1) {

		HAL_GPIO_WritePin(Dir_1_GPIO_Port,Dir_1_Pin,direct%2);
		htim3.Instance->ARR = cp;
		htim4.Instance->ARR = cp;
		htim3.Instance->CCR1 = cp/2;
		htim4.Instance->CCR1 = cp/2;

		if (cinit + npulse > htim9.Instance->ARR) {
			if (htim9.Instance->CNT < npulse) {
				if (htim9.Instance->CNT >= (cinit+npulse)%(htim9.Instance->ARR)) break;
			}
		}
		else{
			if (htim9.Instance->CNT >= (cinit+npulse)%(htim9.Instance->ARR)) break;
		}
	}

	htim3.Instance->CCR1 = 0;
	htim4.Instance->CCR1 = 0;

	if (direct%2 == 1) return 1;
	else return -1;
}

uint16_t omegatoCP(float omega, uint16_t prescaler){

		uint16_t cp;
		float freq = CLOCK / (NPULSES * (1 + prescaler));
		cp = (uint16_t)(360 *freq / omega);

		if (360 *freq / omega > 65535){
			cp = 65535;
		}

		return cp;
}

void recv_fun(void *arg, struct udp_pcb *pcb, struct pbuf *datarecv, const ip_addr_t *addr, uint16_t port){
	/*
	char inc_data[15];
	memcpy(inc_data,datarecv->payload,15);
	com_is_ok=inc_data[0];
	external_button=inc_data[1];
	memcpy(&my_float_1,(float*)(&(inc_data[2])),4);
	memcpy(&my_float_2,(float*)(&(inc_data[6])),4);
	if (com_is_ok==42){
		my_watchdog=watchdog_max;
	}
	pbuf_free(datarecv);
	*/

	char inc_data[13];
	memcpy(inc_data,datarecv->payload,13);
	memcpy(&prova_int[0],(uint8_t*)(&(inc_data[0])),4);
	memcpy(&prova_int[4],(uint8_t*)(&(inc_data[8])),1);
	memcpy(&prova_single[0],(float*)(&(inc_data[4])),4);
	memcpy(&prova_single[1],(float*)(&(inc_data[9])),4);
	my_watchdog = watchdog_max;

	pbuf_free(datarecv);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_MainTask */
/**
  * @brief  Function implementing the Main_Task thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_MainTask */
__weak void MainTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */

  uint8_t motor_start_new = 0;
  uint8_t motor_start_old = 0;
  uint8_t motor_start_new1 = 0;
  uint8_t motor_start_old1 = 0;

  uint8_t control_mode_button_new = 0;
  uint8_t control_mode_button_old = 0;

  uint8_t motor_status = 0;
  uint16_t control_period = 0;

  /*
  uint8_t string_buffer[64];
  int16_t len_string;
  int32_t timeout=25;
  uint8_t print_count = 0;
  */

  /*
  len_string = sprintf(string_buffer, "Starting Main Task\n");
  HAL_UART_Transmit(&huart3,string_buffer,len_string,timeout);
  */

  uint32_t mytick;
  mytick = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  if (my_watchdog<0){
		  htim3.Instance->CCR1 = 0; 	//Pulse
		  htim4.Instance->CCR1 = 0;
	  }
	  else {
		  control_mode_button_new = prova_int[4];

		  if (control_mode_button_new > control_mode_button_old) {
			  control_mode = (control_mode + 1)%4;
		  }

		  control_mode_button_old = control_mode_button_new;

		  switch(control_mode) {

		  	  case 0:			// fermo
		  		htim3.Instance->CCR1 = 0; 	//Pulse
		  	    htim4.Instance->CCR1 = 0;
		  	    break;

		  	  case 1:			// passi

		  		  motor_start_new = prova_int[2];
		  		  motor_start_new1 = prova_int[3];

		  		  if (motor_start_new > motor_start_old){
		  			  p.direction = motor_step(100,1000,1);
		  		  }
		  		  if (motor_start_new1 > motor_start_old1){
		  			  p.direction = motor_step(100,1000,0);
		  		  }

		  		  motor_start_old = motor_start_new;
		  		  motor_start_old1 = motor_start_new1;
		  		  break;

		  	  case 2:			// levetta

		  		  if(abs((int)(100*prova_single[0])) < 10) {
		  			  htim3.Instance->CCR1 = 0; 	//Pulse
		  			  htim4.Instance->CCR1 = 0;
		  		  }
		  		  else{
		  			  if(prova_single[0] > 0) {
		  				  motor_status = 0;
		  				  my_period = (uint16_t)(P_MAX / (P_MIN + (P_MAX - P_MIN) * (prova_single[0]-0.1)/0.9) * P_MIN);
		  				  p.direction = -1;
		  			  }
		  			  else{
		  				  motor_status = 1;
		  				  my_period =  (uint16_t)(P_MAX / (P_MIN + (P_MAX - P_MIN) * (-prova_single[0]-0.1)/0.9) * P_MIN);
		  				  p.direction = 1;
		  			  }
		  			  HAL_GPIO_WritePin(Dir_1_GPIO_Port,Dir_1_Pin,motor_status%2);
		  			  htim3.Instance->ARR = my_period;
		  			  htim4.Instance->ARR = my_period;
		  			  htim3.Instance->CCR1 = my_period/2;
		  			  htim4.Instance->CCR1 = my_period/2;
		  		  }
		  		  break;

		  	  case 3:			// controllo

		  		  p.deg_desired_position = prova_single[1];

		  			  if(p.deg_desired_position > p.deg_position) {
		  				  control_period = omegatoCP(K*GEARBOX_RATIO*(p.deg_desired_position - p.deg_position),htim3.Instance->PSC);
		  				  p.direction = 1;
		  				  motor_status = 1;
		  			  }
		  			  else {
		  				  control_period = omegatoCP(K*GEARBOX_RATIO*(-p.deg_desired_position + p.deg_position),htim3.Instance->PSC);
		  				  p.direction = -1;
		  				  motor_status = 0;
		  			  }

		  			  if (control_period < P_MIN) {
		  				  control_period = P_MIN;		// saturazione velocità
		  			  }

		  			  HAL_GPIO_WritePin(Dir_1_GPIO_Port,Dir_1_Pin,motor_status%2);
		  			  htim3.Instance->ARR = control_period;
		  			  htim4.Instance->ARR = control_period;
		  			  htim3.Instance->CCR1 = control_period/2;
		  			  htim4.Instance->CCR1 = control_period/2;
		  		  /*}
		  		  else{
		  			  htim3.Instance->CCR1 = 0; 	//Pulse
		  			  htim4.Instance->CCR1 = 0;
		  		  }
		  		  */
		  		  break;

		  	  default:
		  		htim3.Instance->CCR1 = 0; 	//Pulse
		  		htim4.Instance->CCR1 = 0;
		  		break;
		  }

		  p.new_cnt = htim9.Instance->CNT;

		  if((p.new_cnt - p.old_cnt) < -32000) {	// overflow
			  p.delta_cnt = p.new_cnt + 65536 - p.old_cnt;
		  }
		  else {
			  p.delta_cnt = p.new_cnt - p.old_cnt;
		  }

		  p.position_new = p.position_old + p.direction * p.delta_cnt;
		  p.deg_position = (float)(p.position_new)*360.0/(NPULSES * GEARBOX_RATIO);

		  p.old_cnt = p.new_cnt;
		  p.position_old = p.position_new;

	  // print_count++;
	  }

    /*if(print_count == 10){
		len_string = sprintf(string_buffer,"motor status: %i, x axis: %3.4f, y axis: %3.4f\n",motor_status,prova_single[0],prova_single[1]);
		HAL_UART_Transmit(&huart3,string_buffer,len_string,timeout);
		print_count = 0;
		}*/

	my_watchdog-=1;
	mytick += 10U;
    osDelayUntil(mytick);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UDPCommunication */
/**
* @brief Function implementing the UDP_Comm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UDPCommunication */
__weak void UDPCommunication(void *argument)
{
  /* USER CODE BEGIN UDPCommunication */
	/* REMOTE ADDRESS DEFINITION */
	struct ip4_addr computer_address;
	IP4_ADDR(&computer_address,192,168,0,100);

	struct ip4_addr local_address;
	IP4_ADDR(&local_address,192,168,0,10);

	/* UDP INITIALIZATION */
	struct udp_pcb *hudp;
	hudp=udp_new();
	udp_bind(hudp,&local_address,9999);
	udp_recv(hudp,recv_fun,NULL);

	uint32_t mytick2;
	mytick2 = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {

	struct pbuf *datasend;
	datasend = pbuf_alloc(PBUF_TRANSPORT,13,PBUF_RAM);
	char dati[13];
	memcpy(&dati[0],(char*)(&prova_int[0]),4);
	memcpy(&dati[4],(char*)(&prova_single[0]),4);
	memcpy(&dati[8],(char*)(&control_mode),1);
	memcpy(&dati[9],(char*)(&p.deg_position),4);
	memcpy(datasend->payload,dati,13);
    udp_sendto(hudp,datasend,&computer_address,11111);
    pbuf_free(datasend);
	mytick2 += 10U;					// 10 ms
	osDelayUntil(mytick2);

  }
  /* USER CODE END UDPCommunication */
}

/* USER CODE BEGIN Header_SecondTask */
/**
* @brief Function implementing the Second_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SecondTask */
__weak void SecondTask(void *argument)
{
  /* USER CODE BEGIN SecondTask */

	uint32_t mytick3;
	mytick3 = osKernelGetTickCount();

	uint8_t button_inc_old=0;
	uint8_t button_inc_new=0;
	uint8_t button_dec_old=0;
	uint8_t button_dec_new=0;

  /* Infinite loop */
  for(;;)
  {

	  if (my_watchdog<0){
	  		  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,0);
	  	  	  HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,0);
	  	  	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,0);
	  	  	  light_status=0;
	  	  }
	  else{
	  		  button_inc_new = prova_int[1];
	  		  button_dec_new = prova_int[0];

	  		  if (button_inc_new > button_inc_old){
	  		      light_status++;
	  		      personal_switch(light_status);
	  		  }

	  		  if (button_dec_new > button_dec_old){
	  			  light_status--;
	  			  personal_switch(light_status);
	  		  }

	  		button_inc_old = button_inc_new;
	  	    button_dec_old = button_dec_new;

	  		my_watchdog-=1;
	  		mytick3 += 25U;
	  		osDelayUntil(mytick3);
	  }
  }
  /* USER CODE END SecondTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
