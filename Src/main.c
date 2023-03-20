/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "string.h"
#include "stdbool.h"
#include "Module.h"
#include "ATCommandList.h"
#include "Parameter.h"
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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define RESPONSE_BUFFER_SIZE 256
#define RUN_COMMAND_COUNTER_DEFAULT 4
#define RUN_COMMAND_TIMEOUT_MS_DEFAULT 1500

uint16_t oldPos = 0;
uint16_t newPos = 0;
bool passivelyListen;
bool responseReceived;
StatusType commandResponseStatus;

char responseReceiveBuffer[RESPONSE_BUFFER_SIZE];
char responseMainBuffer[RESPONSE_BUFFER_SIZE];

void wakeUpModule(){
	/* Pull Down PWR Key*/
	
	/* Delay 800ms */
	
}

void resetDMAInterrupt(){
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) responseReceiveBuffer, RESPONSE_BUFFER_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

void clearMainBuffer() {
	for (int i = 0; i < RESPONSE_BUFFER_SIZE; i++) {
		responseMainBuffer[i] = 0;
	}
	oldPos = 0;
	newPos = 0;
}

void sendCommand(char* command, uint8_t maxCount, uint32_t timeout){
	uint8_t counter = 0;
	uint32_t timer;
	
	while(counter++ < maxCount){
		clearMainBuffer();
		resetDMAInterrupt();
		timer = HAL_GetTick();
		responseReceived = false;
		commandResponseStatus = STATUS_TIMEOUT;
		HAL_UART_Transmit(&huart2, (uint8_t *) command, (uint16_t) strlen(command), timeout);
		HAL_UART_Transmit(&huart2, (uint8_t *) "\r\n", (uint16_t) strlen(command), timeout);
		while(HAL_GetTick() - timer <= timeout){
			if(responseReceived == true){
				break;
			}
		}
		if(commandResponseStatus == STATUS_SUCCESS){
			break;
		}
		clearMainBuffer();
		HAL_Delay(COMMAND_DELAY_MS);
	}
}

/**
  * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) 
{
  /* Prevent unused argument(s) compilation warning */
  if(huart->Instance == USART2){
		char *success_ptr;
		char *error_ptr;
		
		oldPos = newPos;  // Update the last position before copying new data

		/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
		 * This is to maintain the circular buffer
		 * The old data in the main buffer will be overlapped
		 */
		if (oldPos+Size > RESPONSE_BUFFER_SIZE)  // If the current position + new data size is greater than the main buffer
		{
			clearMainBuffer();
		}

		/* if the current position + new data size is less than the main buffer
		 * we will simply copy the data into the buffer and update the position
		 */
		else
		{
			memcpy ((uint8_t *)responseMainBuffer+oldPos, responseReceiveBuffer, Size);
			newPos = Size+oldPos;
		}
		
	/* start the DMA again */
	resetDMAInterrupt();
		
	/* Checking passively listen */
	if(passivelyListen){
		
	}	
		
	/* Checking completion of command */
	for(int i = 0; i < ERROR_RESPONSE_LENGTH; i++){
		error_ptr = strstr(responseMainBuffer, ERROR_COMMAND_SIGN[i]);
		if(error_ptr != NULL){
			commandResponseStatus = STATUS_ERROR;
			responseReceived = true;
			clearMainBuffer();
			break;
		}
	}
	
	for(int i = 0; i < SUCCESS_RESPONSE_LENGTH; i++){
		success_ptr = strstr(responseMainBuffer, SUCCESS_COMMAND_SIGN[i]);
		if(success_ptr != NULL){
			commandResponseStatus = STATUS_SUCCESS;
			responseReceived = true;
			clearMainBuffer();
			break;
		}
	}
	
	}
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UARTEx_RxEventCallback can be implemented in the user file.
   */
}

StatusType checkModule(){
	StatusType outputStatus = STATUS_UNKNOWN;
	sendCommand("AT", RUN_COMMAND_COUNTER_DEFAULT, RUN_COMMAND_TIMEOUT_MS_DEFAULT);
	outputStatus = commandResponseStatus;
	return outputStatus;
}

StatusType setEchoMode(){
	StatusType outputStatus = STATUS_UNKNOWN;
	return outputStatus;
}

StatusType checkSim(){
	StatusType outputStatus = STATUS_UNKNOWN;
	return outputStatus;
}

StatusType getAndSetOperationalBand(){
	StatusType outputStatus = STATUS_UNKNOWN;
	return outputStatus;
}

StatusType readSignalQualityReport(){
	StatusType outputStatus = STATUS_UNKNOWN;
	return outputStatus;
}

StatusType newMQTT(){
	StatusType outputStatus = STATUS_UNKNOWN;
	return outputStatus;
}

StatusType sendMQTTConnect(){
	StatusType outputStatus = STATUS_UNKNOWN;
	return outputStatus;
}

StatusType sendMQTTSub(){
	StatusType outputStatus = STATUS_UNKNOWN;
	return outputStatus;
}

StatusType MQTTDisconnect(){
	StatusType outputStatus = STATUS_UNKNOWN;
	return outputStatus;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* Set Up Module */
		
		
		
		// HAL_UART_Transmit(&huart2, (uint8_t *) "AT", (uint16_t) sizeof("AT"), 1000);
		StatusType moduleStatus = checkModule();
		HAL_Delay(100);
		//if (checkModule() != STATUS_SUCCESS) {
		//	continue;
		//}
		
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
  huart2.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
