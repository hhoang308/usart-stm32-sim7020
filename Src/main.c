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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
#include "Module.h"
#include "ATCommandList.h"
#include "Parameter.h"
#include "ssd1306.h"
#include "fonts.h"
#include "bitmap.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t oldPos = 0;
uint16_t newPos = 0;
volatile int rssi = -1;
int ber = -1;
uint8_t networkRegistered;
volatile uint8_t programStage = 1;

uint16_t u16_ADCVal = 0;

uint32_t currentMillis = 0;
uint32_t previosMillis = 0;

uint32_t timerSendMessage = 0;

bool passivelyListen = false;
bool responseReceived;
volatile bool mqttParemeterConnected = false;
volatile bool oledButton = false;
volatile bool waterLeak = false;	
volatile bool sendMessage = false; /* true to send message immediately*/
volatile bool resetOLED = false;
volatile bool batteryCalculate = true;

//bool sendMessagePeriod = true;
//volatile bool sendMessageWaterLeak = false;
//volatile bool sendMessageLowBattery = false;
volatile bool OLEDTurnedOff;
	
StatusType commandResponseStatus;

char responseReceiveBuffer[RESPONSE_BUFFER_SIZE];
char responseMainBuffer[RESPONSE_BUFFER_SIZE];
char commandResponse[RESPONSE_BUFFER_SIZE];

char delimiter[4] = ",\r\n";
char message[100];

float flow = 0; /* Lit per minute */
int frequency = 0;
float coefficient;
char bufferflow[20];
float waterFlowPrevious;
float waterFlowNow;
volatile float volume = 0; 
volatile float batteryVoltage = 3.3;
int pulse = 0;

/**/

int counterOLEDDisplay = 0;
int counterWaterLeakage = 0;
int counterResetOLED = 0;
int counterBattery = 0;
int counterSendMessage = 15*60 - 5*60 - 1;
int counterWaterRemainUnleak = 0;

/**/

void turnOnSSD1306(){
	ssd1306_I2C_Write(0x78, 0x00, 0x8D);
	ssd1306_I2C_Write(0x78, 0x00, 0x14);
	ssd1306_I2C_Write(0x78, 0x00, 0xAF);
}

void turnOffSSD1306(){
	ssd1306_I2C_Write(0x78, 0x00, 0x8D);
	ssd1306_I2C_Write(0x78, 0x00, 0x10);
	ssd1306_I2C_Write(0x78, 0x00, 0xAE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
	UNUSED(htim);
	if(htim->Instance == TIM1){ /* 62.5 ms */
		frequency = pulse * 16;
		if(frequency == 16){
			coefficient = 12.0208; // 8.1
		}else if(frequency == 32){
			coefficient = 7.831; // 8.109
		}else if(frequency == 48){
			coefficient = 8.1062;
		}else if(frequency == 64){
			coefficient = 8.0608;
		}else if(frequency == 80){
			coefficient = 8.155;
		}else{
			coefficient = 7.312;
		}
		flow = frequency / (coefficient * 60);
//			if(frequency < 50){
//				if(frequency < 16){
//					coefficient = frequency/2;  /* 0 -> 16 */
////					coefficient = 7.512;
//				}else if(frequency > 32){
//					coefficient = 0.0054 * frequency + 7.947; /* 32.5 -> 49.3 */
//				}else{
//					coefficient = 8.2; /* 16 -> 32.5*/
//				}
//			}else{
//				if(frequency < 82){
//					if(frequency < 66){
//						coefficient = -0.0018 * frequency + 8.307; /* 49.3 -> 65.5 */
//					}else{
//						coefficient = 0.0008 * frequency + 8.138; /* 65.5 -> 82 */
//					}
//				}else{
//					if(frequency < 91){
//						coefficient = -0.0834 * frequency + 15.04; /* 82 -> 90.2 */
//					}else{
//						coefficient = 7.512; /* > 90.2 */
//					}
//				}
//			}
//			if(coefficient == 0){
//				flow = 0;
//			}else{
//				flow = frequency / (coefficient * 60) ; /* pulse * 10 = frequency, 7.5 is a parameter number based on datasheet, lit/minute to lit/second */
//			}
//			counter10ms = 0;
			pulse = 0;
	}
	if(htim->Instance == TIM4){ /* 10ms */
		waterFlowPrevious = waterFlowNow;
		waterFlowNow = flow;
		volume += ((waterFlowNow + waterFlowPrevious)/2) * 0.01; /* Avarage flow rate per 10ms, then multiply with time to have the volume*/
	}
	if(htim->Instance == TIM2){ /* 1s */
		
		/* WatchDog Module */
		
//		if(sendMessage){
//				timerSendMessage++;
//			if(timerSendMessage == 5 * 60){
//				forceShutdownModule();
//			}
//		}
		
		/* Send Message */
		
		if(!sendMessage){
			counterSendMessage++;
			if(counterSendMessage == 15 * 60){
				sendMessage = true;
				programStage = 1;
			}
		}else{
			counterSendMessage = 0;
		}
		
		/* Turn Off OLED*/
		
		if(oledButton){
			counterOLEDDisplay++;
			if(counterOLEDDisplay == 6){
				oledButton = false;
				OLEDTurnedOff = false;
			}
		}else{
			counterOLEDDisplay = 0;
		}
		
		/* Low Battery Alert */
		
		if(!batteryCalculate){
			counterBattery++;
			if(counterBattery == 20 * 60){
				batteryCalculate = true;
			}
		}
		
		/* Reset OLED */
		
		if(!resetOLED){
			counterResetOLED++;
			if(counterResetOLED == 10 * 60){
				resetOLED = true;
			}
		}
		
		/* Leak Alert - Real*/
		
		if(flow > 0){
			counterWaterLeakage++;
			if(counterWaterLeakage == 60 * 60){
				waterLeak = true;
				sendMessage = true;
				counterWaterLeakage = 0;
			}
		}else{
			if(waterLeak){
				counterWaterRemainUnleak++;
				if(counterWaterRemainUnleak == 20 * 60){
					waterLeak = false;
					sendMessage = true;
					counterWaterRemainUnleak = 0;
				}
			}else{
				counterWaterLeakage = 0;
			}
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
	if(hadc->Instance == ADC1){
		u16_ADCVal = HAL_ADC_GetValue(&hadc1);
		batteryVoltage = (((float) u16_ADCVal * 2)/4095)*3.3;
	}
  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ConvCpltCallback must be implemented in the user file.
   */
}

void turnOnModule(){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		/* Delay 800ms (According to Hardware Design Datasheet */
		HAL_Delay(800);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_Delay(200);
}

void wakeUpModule(){
	/* Pull Up/Down PWR key depend on Module*/
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	/* Delay 60ms (According to Hardware Design Datasheet */
	HAL_Delay(60);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_Delay(200);
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

void sendCommand(char command[50], uint8_t maxCount, uint32_t timeout){
	uint8_t counter = 0;
	uint32_t timer;
	char sentCommand[50];
	strcpy(sentCommand, command);
	strcat(sentCommand, "\r\n");
	while(counter++ < maxCount){
		clearMainBuffer();
		if(!strcmp(command,"AT+CPOWD=1")){
			passivelyListen = false;
		}
		resetDMAInterrupt();
		timer = HAL_GetTick();
		responseReceived = false;
		commandResponseStatus = STATUS_TIMEOUT;
//		strcat(command, "\r\n");
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *) sentCommand, (uint16_t) strlen(sentCommand));
//		HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "\r\n", (uint16_t) strlen("\r\n"));
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
		
	/* Checking passively listen */
	if(passivelyListen){
		for(int i = 0; i < PASSIVE_RESPONSE_LENGTH; i++){
			error_ptr = strstr(responseMainBuffer, PASSIVE_RESPONSE_SIGN[0]);
			if(error_ptr != NULL){
				commandResponseStatus = STATUS_SUCCESS;
				clearMainBuffer();
				responseReceived = true;
			}
		}
		passivelyListen = false;
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
			strcpy(commandResponse, responseMainBuffer);
			clearMainBuffer();
			break;
		}
	}
	
	/* Start the DMA again */
	resetDMAInterrupt();
	
	}
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UARTEx_RxEventCallback can be implemented in the user file.
   */
}

StatusType forceShutdownModule(){
	StatusType outputStatus = STATUS_UNKNOWN;
	passivelyListen = true;
	uint32_t timer;
	clearMainBuffer();
	resetDMAInterrupt();
	timer = HAL_GetTick();
	responseReceived = false;
	commandResponseStatus = STATUS_TIMEOUT;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	/* Delay 1000ms (According to Hardware Design Datasheet */
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_Delay(200);
	while(HAL_GetTick() - timer <= 1000){
		if(responseReceived == true){
			break;
		}
	}
	outputStatus = commandResponseStatus;
	return outputStatus;
}

StatusType turnOffModule(){
	StatusType outputStatus = STATUS_UNKNOWN;
	passivelyListen = true;
	sendCommand("AT+CPOWD=1", 1, RUN_COMMAND_TIMEOUT_MS_DEFAULT);
	outputStatus = commandResponseStatus;
	return outputStatus;
}

StatusType checkModule(){
	StatusType outputStatus = STATUS_UNKNOWN;
	sendCommand("AT", RUN_COMMAND_COUNTER_DEFAULT, RUN_COMMAND_TIMEOUT_MS_DEFAULT);
	outputStatus = commandResponseStatus;
	return outputStatus;
}

StatusType setEchoMode(){
	StatusType outputStatus = STATUS_UNKNOWN;
	sendCommand("ATE0", RUN_COMMAND_COUNTER_DEFAULT, RUN_COMMAND_TIMEOUT_MS_DEFAULT);
	outputStatus = commandResponseStatus;
	return outputStatus;
}

StatusType checkSim(){
	StatusType outputStatus = STATUS_UNKNOWN;
	sendCommand("AT+CPIN?", RUN_COMMAND_COUNTER_DEFAULT, RUN_COMMAND_TIMEOUT_MS_DEFAULT);
	outputStatus = commandResponseStatus;
	return outputStatus;
}

StatusType getAndSetOperationalBand(){
	StatusType outputStatus = STATUS_UNKNOWN;
	sendCommand("AT+CBAND=3", RUN_COMMAND_COUNTER_DEFAULT, RUN_COMMAND_TIMEOUT_MS_DEFAULT);
	outputStatus = commandResponseStatus;
	return outputStatus;
}

StatusType readSignalQualityReport(){
	StatusType outputStatus = STATUS_UNKNOWN;
	sendCommand("AT+CSQ", RUN_COMMAND_COUNTER_DEFAULT, RUN_COMMAND_TIMEOUT_MS_DEFAULT);
	outputStatus = commandResponseStatus;
	if(outputStatus == STATUS_SUCCESS){
		/* RSSI and BER */
		
		int numberOfElements = 2;
		char *data_pointer;
		char *token_array[numberOfElements];
		char *token;
		
		uint8_t i = 0;
		
		data_pointer = strstr(commandResponse, ":");
		data_pointer = data_pointer + 2;
		token = strtok(data_pointer, delimiter);
		
		while(token != NULL && i < numberOfElements){
			token_array[i] = token;
			token = strtok(NULL, delimiter);
			i++;
		}
		rssi = atoi(token_array[0]);
		ber = atoi(token_array[1]);
		
	}
	return outputStatus;
}

StatusType readNetworkRegistrationStatus(){
	StatusType outputStatus = STATUS_UNKNOWN;
	sendCommand("AT+CREG?",RUN_COMMAND_COUNTER_DEFAULT, RUN_COMMAND_TIMEOUT_MS_DEFAULT);
	outputStatus = commandResponseStatus;
	if(outputStatus == STATUS_SUCCESS){
		/* Registered */
		
		uint8_t numberOfElements = 2;
		char *data_pointer;
		char *token;
		char *token_array[numberOfElements];
		uint8_t i = 0;
		
		data_pointer = strstr(commandResponse, ":");
		data_pointer = data_pointer + 2;
		
		if(!data_pointer){
			outputStatus = STATUS_FAILURE;
			return outputStatus;
		}
		
		token = strtok(data_pointer, delimiter);
		
		while(token != NULL && i < numberOfElements){
			token_array[i] = token;
			token = strtok(NULL, delimiter);
			i++;
		}
		
		networkRegistered = atoi(token_array[1]); 

	} 
	return outputStatus;
}

StatusType setPhoneFunctionality(uint8_t fun){
	StatusType outputStatus = STATUS_UNKNOWN;
	char command[20];
	sprintf(command, "AT+CFUN=%u", fun);
	sendCommand(command, RUN_COMMAND_COUNTER_DEFAULT, RUN_COMMAND_TIMEOUT_MS_DEFAULT);
	outputStatus = commandResponseStatus;
	return outputStatus;
}

StatusType setDefaultPSDConnection(){
	StatusType outputStatus = STATUS_UNKNOWN;
	sendCommand("AT*MCGDEFCONT=\"ip\",\"nbiot\"", RUN_COMMAND_COUNTER_DEFAULT, RUN_COMMAND_TIMEOUT_MS_DEFAULT);
	outputStatus = commandResponseStatus;
	return outputStatus;
}

StatusType configWakeupIndication(){
	StatusType outputStatus = STATUS_UNKNOWN;
	sendCommand("AT+CPSMSTATUS=1", RUN_COMMAND_COUNTER_DEFAULT, RUN_COMMAND_TIMEOUT_MS_DEFAULT);
	outputStatus = commandResponseStatus;
	return outputStatus;
}

StatusType configSlowClock(){
	StatusType outputStatus = STATUS_UNKNOWN;
	sendCommand("AT+CSCLK=2", RUN_COMMAND_COUNTER_DEFAULT, RUN_COMMAND_TIMEOUT_MS_DEFAULT);
	outputStatus = commandResponseStatus;
	return outputStatus;
}

StatusType configPSM(){
	StatusType outputStatus = STATUS_UNKNOWN;
	sendCommand("AT+CPSMS=1,,,\"01011111\",\"00000100\"", RUN_COMMAND_COUNTER_DEFAULT, RUN_COMMAND_TIMEOUT_MS_DEFAULT);
	outputStatus = commandResponseStatus;
	return outputStatus;
}

StatusType checkMQTTConnection(){
	StatusType outputStatus = STATUS_UNKNOWN;
	sendCommand("AT+CMQNEW?", RUN_COMMAND_COUNTER_DEFAULT + 1, RUN_COMMAND_TIMEOUT_MS_DEFAULT);
	outputStatus = commandResponseStatus;
	if(outputStatus == STATUS_SUCCESS){
		/* Registered */
		
		uint8_t numberOfElements = 3;
		char *data_pointer;
		char *token;
		char *token_array[numberOfElements];
		uint8_t i = 0;
		
		data_pointer = strstr(commandResponse, ":");
		data_pointer = data_pointer + 2;
		
		if(!data_pointer){
			outputStatus = STATUS_FAILURE;
			return outputStatus;
		}
		
		token = strtok(data_pointer, delimiter);
		
		while(token != NULL && i < numberOfElements){
			token_array[i] = token;
			token = strtok(NULL, delimiter);
			i++;
		}
		
		if(!strcmp(token_array[2],"null")){
			mqttParemeterConnected = false;
		}else{
			mqttParemeterConnected = true;
		}

	} 
	return outputStatus;
}

StatusType newMQTT(){
	StatusType outputStatus = STATUS_UNKNOWN;
	char command[50];
	sprintf(command, "AT+CMQNEW=\"%s\",\"%i\",%u,%u", MQTT_SERVER, MQTT_PORT, MQTT_COMMAND_TIMEOUT_MS, MQTT_BUFFER_SIZE);
	sendCommand(command, RUN_COMMAND_COUNTER_DEFAULT + 1, RUN_COMMAND_TIMEOUT_MS_DEFAULT + 2000);
	outputStatus = commandResponseStatus;
	return outputStatus;
}

StatusType sendMQTTConnect(){
	StatusType outputStatus = STATUS_UNKNOWN;
	char command[50];
	sprintf(command, "AT+CMQCON=%u,%u,\"%s\",%u,%u,%u",MQTT_ID, MQTT_VERSION, MQTT_CLIENT_ID, MQTT_KEEP_ALIVE_INTERVAL, MQTT_CONNECT_CLEAN_SESSION, MQTT_CONNECT_WILL_FLAG);
	sendCommand(command, RUN_COMMAND_COUNTER_DEFAULT + 4, RUN_COMMAND_TIMEOUT_MS_DEFAULT + 2000);
	outputStatus = commandResponseStatus;
	return outputStatus;
}

StatusType sendMQTTPub(char topic[], uint8_t qos, bool retained, bool dup, char message[]){
	StatusType outputStatus = STATUS_UNKNOWN;
	uint16_t messageLength = strlen(message);
	char command[200];
	sprintf(command, "AT+CMQPUB=%u,\"%s\",%u,%u,%u,%u,\"%s\"", MQTT_ID, topic, qos, retained, dup, messageLength, message);
	sendCommand(command, RUN_COMMAND_COUNTER_DEFAULT + 1, RUN_COMMAND_TIMEOUT_MS_DEFAULT + 2000);
	outputStatus = commandResponseStatus;
	return outputStatus;
}

StatusType MQTTDisconnect(){
	StatusType outputStatus = STATUS_UNKNOWN;
	char command[20];
	sprintf(command, "AT+CMQDISCON=%u", MQTT_ID);
	sendCommand(command, RUN_COMMAND_COUNTER_DEFAULT, RUN_COMMAND_TIMEOUT_MS_DEFAULT);
	outputStatus = commandResponseStatus;
	return outputStatus;
}

void moduleFlow(){
	
	/* Stage 1 - Set Up Module */	
	
		while(programStage == 1){
			
			if(checkModule() != STATUS_SUCCESS){
				turnOnModule();
				continue;
			}
			
			if(readSignalQualityReport() != STATUS_SUCCESS){
				continue;
			}
			
			if(setEchoMode() != STATUS_SUCCESS){
				continue;
			}
			
			if(checkSim() != STATUS_SUCCESS){
				turnOffModule();
				HAL_Delay(1000);
				turnOnModule();
				continue;
			}
			
//			if(getAndSetOperationalBand() != STATUS_SUCCESS){
//				continue;
//			}
			
			HAL_Delay(1000);
			programStage = 2;	
			
		}
		/* Stage 2 - Check Network */
		
		int8_t tryTimes = 4;
		
		while(programStage == 2){	
			/* Check Registration Status */
			while (tryTimes--){
				/* AT+CREG */
				
				if(readNetworkRegistrationStatus() == STATUS_SUCCESS){	
					programStage = 3;
					break;
				}
				
				if(tryTimes < 2){
					wakeUpModule();
					HAL_Delay(1000);
				}
				HAL_Delay(1000);
			}
			if(tryTimes == -1){ 
				/* Cannot register automatically. Try to register manually 2 times */
				int8_t tryManually = 3;
				while(tryManually--){
					if(setPhoneFunctionality(0) != STATUS_SUCCESS){
						continue;
					}
				
					if(setDefaultPSDConnection() != STATUS_SUCCESS){
						continue;
					}
				
					if(setPhoneFunctionality(1) != STATUS_SUCCESS){
						continue;
					}
					
					if(readNetworkRegistrationStatus() == STATUS_SUCCESS){
						programStage = 3;
						break;
					}		
				}
				if(tryManually == -1){
					turnOffModule();
					programStage = 1;
					HAL_Delay(1000);
					break;
				}
			}
			programStage = 3;
			break;
		}
		
		/* Configure PSM first time only*/
		
		while(programStage == 3){
			programStage = 4;
			break;
	  }
		
//		if(configuredPSM == true){
//			programStage = 4;
//		}
//		
//		/* Stage 3 - Configure PSM Feature*/
//		while(programStage == 3){
//			if(configWakeupIndication() != STATUS_SUCCESS){
//				continue;
//			}
//			if(configPSM() != STATUS_SUCCESS){
//				continue;
//			}
//			configuredPSM = true;
//			programStage = 4;
//			HAL_Delay(2000);
//		}
		
		/* Stage 4 - MQTT */
		
		int8_t tryMQTT;
		while(programStage == 4){
			
			if(checkMQTTConnection() != STATUS_SUCCESS){
				continue;
			}
			
			if(mqttParemeterConnected){
				if(MQTTDisconnect() != STATUS_SUCCESS){
					continue;
				}
			}
			
			tryMQTT = 3;
			
			while(tryMQTT--){
				if(newMQTT() == STATUS_SUCCESS){
					break;
				}
			}
			
			if(tryMQTT == -1){
				turnOffModule();
				HAL_Delay(1000);
				turnOnModule();
			}
			
			if(sendMQTTConnect() != STATUS_SUCCESS){
				continue;
			}
		
			/* Send Message */
			
			int8_t trySend = 4;
			sprintf(message,"bat:%.1f,vol:%.1f,sig:%u,leak:%u", batteryVoltage, volume, rssi, waterLeak);
			while(trySend--){
				if(sendMQTTPub("watermeter1/messages", 0, 0, 0, message) == STATUS_SUCCESS){
					sendMessage = false;
					break;
				}else{
					turnOnModule();
					continue;   
				}
			}
			
			/* Failed to send message*/
			
			if(trySend == -1){
//				turnOffModule();
				programStage = 1;
				break;
			}
		
			if(MQTTDisconnect() != STATUS_SUCCESS){
				continue;
			}
			
			HAL_Delay(1000);
			programStage = 5;
		}
		
		/* What to do when MCU has free time ? */
		passivelyListen = true;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
	currentMillis = HAL_GetTick();
	if(GPIO_Pin == GPIO_PIN_0 && currentMillis - previosMillis > 200){
		oledButton = true;
		previosMillis = currentMillis;
	}
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}

void setUpSSD1306(){
			turnOnSSD1306();
			SSD1306_GotoXY(0,10);
			sprintf(bufferflow, "%2.2f", volume);
			SSD1306_Puts(bufferflow, &Font_16x26, SSD1306_COLOR_WHITE);

			SSD1306_GotoXY(56,37);
			
//			sprintf(bufferflow, "K: %2.21f", coefficient);
			SSD1306_Puts("dm3", &Font_11x18, SSD1306_COLOR_WHITE);	
			/* Display Battery Icon */
			
			if(batteryVoltage > 3.3){
				if(batteryVoltage > 3.4){
					SSD1306_DrawBitmap(112, 0, bat3_icon16x16, 16, 16, 1);
				}else{
					SSD1306_DrawBitmap(112, 0, bat2_icon16x16, 16, 16, 1);
				}
			}else{
				if(batteryVoltage < 3.2){
					SSD1306_DrawBitmap(112, 0, bat0_icon16x16, 16, 16, 1);
				}else{
					SSD1306_DrawBitmap(112, 0, bat1_icon16x16, 16, 16, 1);
				}
			}
			/* Display Signal Icon */
			
			if(rssi > 10){
				if(rssi > 20){
					if(rssi == 99){
						SSD1306_DrawBitmap(112, 16, cancel_icon16x16, 16, 16, 1);
					}else{
						SSD1306_DrawBitmap(112, 16, signal4_icon16x16, 16, 16, 1);
					}
				}else{
					SSD1306_DrawBitmap(112, 16, signal3_icon16x16, 16, 16, 1);
				}
			}else{
				if(rssi < 5){
					if(rssi == 0){
						SSD1306_DrawBitmap(112, 16, cancel_icon16x16, 16, 16, 1);
					}else{
						SSD1306_DrawBitmap(112, 16, signal1_icon16x16, 16, 16, 1);
					}
				}else {
					SSD1306_DrawBitmap(112, 16, signal2_icon16x16, 16, 16, 1);
				}
			}
			
			/*Display Water Tap */
			
			if(waterLeak){
				SSD1306_DrawBitmap(112, 40, water_tap_icon16x16, 16, 16, 1); 
			}
			
			SSD1306_UpdateScreen();
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim4);
	
	SSD1306_Init();
	HAL_ADC_Start_IT(&hadc1);
//	int maxFrequency = frequency;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		if(sendMessagePeriod || sendMessageLowBattery || sendMessageWaterLeak){
//			sendMessage = true;
//			sendMessagePeriod = false;
//			sendMessageLowBattery = false;
//			sendMessageWaterLeak = false;
//		}
//		setUpSSD1306();
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		HAL_Delay(1000);	
		if(sendMessage){
			turnOnModule();
			moduleFlow();		
		}
		if(oledButton){
			setUpSSD1306();
		}else{
			if(!OLEDTurnedOff){
				turnOffSSD1306();
				OLEDTurnedOff = true;
			}
		}
		if(resetOLED){
			SSD1306_Init();
			resetOLED = false;
			counterResetOLED = 0;
		}
		if(batteryCalculate){
			HAL_ADC_Start_IT(&hadc1);
			if(batteryVoltage < 3.2){
				sendMessage = true;
			}
			batteryCalculate = false;
			counterBattery = 0;
		}
		HAL_Delay(1000);		
		
//		sendCommand("AT", RUN_COMMAND_COUNTER_DEFAULT, RUN_COMMAND_TIMEOUT_MS_DEFAULT);
//		HAL_Delay(1000);
//		if(frequency > maxFrequency){
//			maxFrequency = frequency;
//		}
//		SSD1306_GotoXY(0,0);
//		sprintf(bufferflow, "vol: %2.2f", volume);
//		SSD1306_Puts(bufferflow, &Font_11x18, 1);
//		SSD1306_GotoXY(0, 15);
//		sprintf(bufferflow, "K: %2.2f", coefficient);
//		SSD1306_Puts(bufferflow, &Font_11x18, 1);	
//		SSD1306_GotoXY(0, 30);
//		sprintf(bufferflow, "f: %d mf: %d", frequency, maxFrequency);
//		SSD1306_Puts(bufferflow, &Font_11x18, 1);	
//		SSD1306_GotoXY(0, 45);
//		sprintf(bufferflow, "flow: %2.2f", flow);
//		SSD1306_Puts(bufferflow, &Font_11x18, 1);
//		SSD1306_UpdateScreen();
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		HAL_Delay(1000);
//			sprintf(bufferflow, "K: %2.21f", coefficient);
			
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  hi2c1.Init.ClockSpeed = 400000;
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 36000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 125-1;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim4.Init.Prescaler = 36000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20-1;
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
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
