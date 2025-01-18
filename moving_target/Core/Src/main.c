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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SLOW_PULSE 20
#define FAST_PULSE 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t millis = 0;
uint16_t m_millis = 0;
uint16_t m_pulse_period = SLOW_PULSE;
uint8_t m0_direction = 0;
uint8_t m1_direction = 0;
uint8_t m0_enable = 0;
uint8_t m1_enable = 0;
uint16_t command_expire = 0;

uint16_t test_millis = 0;
uint16_t test_period = 5000;
uint8_t test_mode = 0;
uint8_t uart_buffer[1] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void One_Second_Tick(void);
void Move_Engines(void);
void Set_Engine_Modes(BOT_Direction direction);
BOT_Direction Validate_Direction(BOT_Direction direction);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //USART1->DR = 0x3E; //Start MCU debug signal
  HAL_TIM_Base_Start_IT(&htim1); // start timer
  HAL_UART_Receive_IT (&huart1, uart_buffer, 1);

  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 719;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, M0_Direction_Pin|M0_Pulse_Pin|M1_Direction_Pin|M1_Pulse_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : M0_Direction_Pin M0_Pulse_Pin M1_Direction_Pin M1_Pulse_Pin */
  GPIO_InitStruct.Pin = M0_Direction_Pin|M0_Pulse_Pin|M1_Direction_Pin|M1_Pulse_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STOPPER_LEFT_Pin STOPPER_RIGHT_Pin STOPPER_TOP_Pin STOPPER_BOTTOM_Pin */
  GPIO_InitStruct.Pin = STOPPER_LEFT_Pin|STOPPER_RIGHT_Pin|STOPPER_TOP_Pin|STOPPER_BOTTOM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//Process timer interrupts
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim == &htim1){ //Check if it's TIM1
		//One second timer action
    millis++;
		if (millis == 1000){
			millis = 0;
			One_Second_Tick();
		}

    //Engime moving action
    m_millis++;
    if (m_millis >= m_pulse_period){
      m_millis = 0;
      Move_Engines();
    }

    //Validate command expiry
    if (command_expire > 0){
      command_expire--;
    }
    else{
      Set_Engine_Modes(DIR_STOP);
    }

    //m0_enable = HAL_GPIO_ReadPin(GPIOA, STOPPER_LEFT_Pin);

    //Reset pulse pins
    if (HAL_GPIO_ReadPin(GPIOA, M0_Pulse_Pin)) HAL_GPIO_WritePin(GPIOA, M0_Pulse_Pin, GPIO_PIN_RESET);
    if (HAL_GPIO_ReadPin(GPIOA, M1_Pulse_Pin)) HAL_GPIO_WritePin(GPIOA, M1_Pulse_Pin, GPIO_PIN_RESET);
    
    //Switch test state
    //test_millis++;
    if (test_millis >= test_period){
      test_millis = 0;

      switch (test_mode){
        case 0:
          USART1->DR = 0x30;
          test_mode = 1;
          m0_direction = 0;
          m0_enable = 1;
          m1_direction = 0;
          m1_enable = 0;
          break;

        case 1:
          USART1->DR = 0x31;
          test_mode = 2;
          m0_direction = 0;
          m0_enable = 0;
          m1_direction = 0;
          m1_enable = 1;
          break;

        case 2:
          USART1->DR = 0x32;
          test_mode = 3;
          m0_direction = 1;
          m0_enable = 1;
          m1_direction = 0;
          m1_enable = 0;
          break;


        default:
          USART1->DR = 0x33;
          test_mode = 0;
          m0_direction = 1;
          m0_enable = 0;
          m1_direction = 1;
          m1_enable = 1;
          break;
      }
    }
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    //Handle incoming signal
    command_expire = 150;

    //Set speed
    if (uart_buffer[0] & 0x10){
      m_pulse_period = FAST_PULSE;
    }
    else{
      m_pulse_period = SLOW_PULSE;
    }

    uint16_t direction = uart_buffer[0] & 0x0F;

    //USART1->DR = direction;

    Set_Engine_Modes(Validate_Direction(direction));

    //Set_Engine_Modes(direction);

    HAL_UART_Receive_IT(&huart1, uart_buffer, 1);
}

//Ticks once per second
void One_Second_Tick(void){

  //USART1->DR = 0x2E; //Send char '.' to UART (to test that UART and timer work)

}

void Move_Engines(void){
  if (m0_enable){
    HAL_GPIO_WritePin(GPIOA, M0_Direction_Pin, m0_direction);
    HAL_GPIO_WritePin(GPIOA, M0_Pulse_Pin, GPIO_PIN_SET);
  }
  if (m1_enable){
    HAL_GPIO_WritePin(GPIOA, M1_Direction_Pin, m1_direction);
    HAL_GPIO_WritePin(GPIOA, M1_Pulse_Pin, GPIO_PIN_SET);
  }
}

void Set_Engine_Modes(BOT_Direction direction){
	switch (direction){
    
    case DIR_STOP:
      m0_enable = 0;
      m1_enable = 0;
      break;

    case DIR_TOP:
      m0_enable = 1;
      m0_direction = 1;
      m1_enable = 1;
      m1_direction = 0;
      break;
      
    case DIR_TOP_RIGHT:
      m0_enable = 0;
      m0_direction = 0;
      m1_enable = 1;
      m1_direction = 0;
      break;
      
    case DIR_RIGHT:
      m0_enable = 1;
      m0_direction = 0;
      m1_enable = 1;
      m1_direction = 0;
      break;
      
    case DIR_BOTTOM_RIGHT:
      m0_enable = 1;
      m0_direction = 0;
      m1_enable = 0;
      m1_direction = 0;
      break;
      
    case DIR_BOTTOM:
      m0_enable = 1;
      m0_direction = 0;
      m1_enable = 1;
      m1_direction = 1;
      break;
      
    case DIR_BOTTOM_LEFT:
      m0_enable = 0;
      m0_direction = 0;
      m1_enable = 1;
      m1_direction = 1;
      break;
      
    case DIR_LEFT:
      m0_enable = 1;
      m0_direction = 1;
      m1_enable = 1;
      m1_direction = 1;
      break;
      
    case DIR_TOP_LEFT:
      m0_enable = 1;
      m0_direction = 1;
      m1_enable = 0;
      m1_direction = 0;
      break;
      
	}
}

BOT_Direction Validate_Direction(BOT_Direction direction){
  
  if (HAL_GPIO_ReadPin(GPIOA, STOPPER_LEFT_Pin)){
    switch (direction){
      case DIR_LEFT:
        direction = DIR_STOP;
        break;
      case DIR_BOTTOM_LEFT:
        direction = DIR_BOTTOM;
        break;
      case DIR_TOP_LEFT:
        direction = DIR_TOP;
        break;
      default:
        break;
    }
  }

  if (HAL_GPIO_ReadPin(GPIOA, STOPPER_RIGHT_Pin)){
    switch (direction){
      case DIR_RIGHT:
        direction = DIR_STOP;
        break;
      case DIR_BOTTOM_RIGHT:
        direction = DIR_BOTTOM;
        break;
      case DIR_TOP_RIGHT:
        direction = DIR_TOP;
        break;
      default:
        break;
    }
  }

  if (HAL_GPIO_ReadPin(GPIOA, STOPPER_TOP_Pin)){
    switch (direction){
      case DIR_TOP:
        direction = DIR_STOP;
        break;
      case DIR_TOP_LEFT:
        direction = DIR_LEFT;
        break;
      case DIR_TOP_RIGHT:
        direction = DIR_RIGHT;
        break;
      default:
        break;
    }
  }

  if (HAL_GPIO_ReadPin(GPIOA, STOPPER_BOTTOM_Pin)){
    switch (direction){
      case DIR_BOTTOM:
        direction = DIR_STOP;
        break;
      case DIR_BOTTOM_LEFT:
        direction = DIR_LEFT;
        break;
      case DIR_BOTTOM_RIGHT:
        direction = DIR_RIGHT;
        break;
      default:
        break;
    }
  }

  return direction;

}

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
