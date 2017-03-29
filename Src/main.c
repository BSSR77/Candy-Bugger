/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

#include "serial.h"
#include "can.h"
#include "errors.h"



/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

WWDG_HandleTypeDef hwwdg;

osThreadId processCanHandle;
osThreadId ProcessUartHandle;
osThreadId housekeepingHandle;
osMessageQId mainCanTxQHandle;
osMessageQId mainCanRxQHandle;
osTimerId WWDGTmrHandle;
osMutexId UartTxMtxHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static uint8_t FWVersionNumber[] = "1.0.0";

static uint32_t CanErr = 0;
static uint16_t CanSent = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_WWDG_Init(void);
void doProcessCan(void const * argument);
void doProcessUart(void const * argument);
void doHousekeeping(void const * argument);
void TmrKickDog(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void logError(LogErrorCode_t e, LogErrorLevel_t l);

void notifySuccess(){
	static uint8_t msg[] = "{\"type\":\"action\",\"success\":true}\n";
	xSemaphoreTake(UartTxMtxHandle, portMAX_DELAY);
	Serial2_writeBuf(msg);
	xSemaphoreGive(UartTxMtxHandle);
}

void notifyFail(){
	static uint8_t msg[] = "{\"type\":\"action\",\"success\":false}\n";
	xSemaphoreTake(UartTxMtxHandle, portMAX_DELAY);
	Serial2_writeBuf(msg);
	xSemaphoreGive(UartTxMtxHandle);
}

void toCaps(uint8_t *str, uint8_t length){
	for(int i=0; i<length; i++){
		if(str[i]>='a' && str[i]<='z'){
			str[i]-=(0x20);
		}
	}
}

uint8_t toHex(uint8_t i){
	return (i<=9 ? '0'+i : 'A'+i-10);
}

uint8_t fromHex(uint8_t i){//0xff = invalid char
	if(i>='0' && i<='9'){
		return i-'0';
	}else if(i>='A' && i<='F'){
		return i-'A'+10;
	}else if(i>='a' && i<='f'){
		return i-'a'+10;
	}
	return 0xff;
}

uint8_t intToDec(uint input, uint8_t *str){ //returns length. Only does positives.
	uint8_t length = 0;
	uint8_t output[10];
	while(input/10){
		length++;
		output[10-length] = toHex(input%10);
		input/=10;
	}
	length++;
	output[10-length] = toHex(input);
	for(int i=0; i<length; i++){
		str[i] = output[10-length+i];
	}
	return length;
}

void intToHex(uint32_t input, uint8_t *str, int length){
	for(int i=0; i<length; i++){
		str[length-1-i]=toHex(input&0x0F);
		input = input>>4;
	}
}

int strToInt(uint8_t *str, uint8_t index, uint8_t length){
	return 0;
}

void cantxcb(){
	CanSent ++;
}

void canercb(uint32_t e){
	CanErr = e;
}

void logError(LogErrorCode_t e, LogErrorLevel_t l){
	static uint8_t errlvlwarn[] = "warn";			//error fixed or accounted for
	static uint8_t errlvlabort[] = "abort";			//error caused function to exit
	static uint8_t errlvlfatal[] = "fatal";			//shit (and soon the watchdog) hit the fan.
	static uint8_t errlvldunno[] = "unknown";		//wrong value or lazy implementation
	static uint8_t errCode[] = "xxxxxxxxxx";	//max is 4 billion for int
	static uint8_t errmsg1[] = "{\"type\":\"error\",\"code\":";
	static uint8_t errmsg2[] = ",\"message\":\"";
	static uint8_t errmsg3[] = "\",\"level\":\"";
	static uint8_t errmsg4[] = "\"}\n";

	xSemaphoreTake(UartTxMtxHandle, portMAX_DELAY);
	uint8_t length = intToDec(e, errCode);
	Serial2_writeBuf(errmsg1);
	Serial2_writeBytes(errCode, length);
	Serial2_writeBuf(errmsg2);
	Serial2_writeBytes(Err_Messages[e].str, Err_Messages[e].length);
	Serial2_writeBuf(errmsg3);
	switch(l){
	case ERR_Fatal:
		Serial2_writeBuf(errlvlfatal); break;
	case ERR_Warn:
		Serial2_writeBuf(errlvlwarn); break;
	case ERR_Abort:
		Serial2_writeBuf(errlvlabort); break;
	default:
		Serial2_writeBuf(errlvldunno); break;
	}
	Serial2_writeBuf(errmsg4);
	xSemaphoreGive(UartTxMtxHandle);
}

static void waitTilAvail(uint length){ //blocks current taks (runs others) until true
	while(Serial2_available() < length){
		osDelay(1);
	}
}

void parseFilter(uint8_t isExt, uint8_t isMasked){
	static uint32_t id;
	static uint32_t mask;
	 int isRemote;

	static uint8_t tempByte;
	/*Get RTR*/
	waitTilAvail(1);
	tempByte = fromHex(Serial2_read());
	if(tempByte > 0x03){
		logError(ERR_invalidRtrBits, ERR_Abort);
		return;
	}else if(tempByte > 0x01){ //mask = must match
		if(tempByte & 0x01){
			isRemote = 1;
		}else{
			isRemote = 0;
		}
	}else{ //mask = don't care
		isRemote = -1;
	}
	/*Get ID*/
	id = 0;
	if(isExt){
		waitTilAvail(8);
		for(int i=0; i<8; i++){
			tempByte = fromHex(Serial2_read());
			if(tempByte > 0x0f){
				logError(ERR_invalidHexChar, ERR_Abort);
				return;
			}
			id |= tempByte << ((7-i)*4);
		}
		if(id > 0x1fffffff){
			logError(ERR_idIsOutOfRange, ERR_Abort);
			return;
		}
	}else{
		waitTilAvail(3);
		for(int i=0; i<3; i++){
			tempByte = fromHex(Serial2_read());
			if(tempByte > 0x0f){
				logError(ERR_invalidHexChar, ERR_Abort);
				return;
			}
			id |= tempByte << ((2-i)*4);
		}
		if(id > 0x7ff){
			logError(ERR_idIsOutOfRange, ERR_Abort);
			return;
		}
	}
	/*Get mask and/or send*/
	if(isMasked){
		if(isExt){
			waitTilAvail(8);
			for(int i=0; i<8; i++){
				tempByte = fromHex(Serial2_read());
				if(tempByte > 0x0f){
					logError(ERR_invalidHexChar, ERR_Abort);
					return;
				}
				mask |= tempByte << ((7-i)*4);
			}
			if(mask > 0x1fffffff){
				logError(ERR_idIsOutOfRange, ERR_Abort);
				return;
			}
			bxCan_addMaskedFilterExt(id, mask, isRemote);
		}else{
			waitTilAvail(3);
			for(int i=0; i<3; i++){
				tempByte = fromHex(Serial2_read());
				if(tempByte > 0x0f){
					logError(ERR_invalidHexChar, ERR_Abort);
					return;
				}
				mask |= tempByte << ((2-i)*4);
			}
			if(mask > 0x7ff){
				logError(ERR_idIsOutOfRange, ERR_Abort);
				return;
			}
			bxCan_addMaskedFilterStd(id, mask, isRemote);
		}
	}else{
		if(isExt){
			bxCan_addFilterExt(id, isRemote);
		}else{
			bxCan_addFilterStd(id, isRemote);
		}
		notifySuccess();
	}
}

void parseFrame(uint8_t isExt, uint8_t isRemote){
	static Can_frame_t newFrame;
	static uint8_t tempByte;

	newFrame.isExt = isExt;
	newFrame.isRemote = isRemote;

	/*Get DLC*/
	waitTilAvail(1);
	newFrame.dlc = fromHex(Serial2_read());
	if(newFrame.dlc > 0x0f){
		logError(ERR_invalidHexChar, ERR_Abort);
		return;
	}

	/*Get ID*/
	newFrame.id = 0;
	if(isExt){
		waitTilAvail(8);
		for(int i=0; i<8; i++){
			tempByte = fromHex(Serial2_read());
			if(tempByte > 0x0f){
				logError(ERR_invalidHexChar, ERR_Abort);
				return;
			}
			newFrame.id |= tempByte << ((7-i)*4);
		}
		if(newFrame.id > 0x1fffffff){
			logError(ERR_idIsOutOfRange, ERR_Abort);
			return;
		}
	}else{
		waitTilAvail(3);
		for(int i=0; i<3; i++){
			tempByte = fromHex(Serial2_read());
			if(tempByte > 0x0f){
				logError(ERR_invalidHexChar, ERR_Abort);
				return;
			}
			newFrame.id |= tempByte << ((2-i)*4);
		}
		if(newFrame.id > 0x7ff){
			logError(ERR_idIsOutOfRange, ERR_Abort);
			return;
		}
	}

	/*Get data*/
	if(!isRemote){
		waitTilAvail(newFrame.dlc*2);
		for(int i=0; i<newFrame.dlc; i++){
			tempByte = fromHex(Serial2_read());
			if(tempByte > 0x0f){
				logError(ERR_invalidHexChar, ERR_Abort);
				return;
			}
			newFrame.Data[i] = tempByte << 4;
			tempByte = fromHex(Serial2_read());
			if(tempByte > 0x0f){
				logError(ERR_invalidHexChar, ERR_Abort);
				return;
			}
			newFrame.Data[i] |= tempByte;
		}
	}
	bxCan_sendFrame(&newFrame);
}

void displayHelp(){
	static uint8_t helpmsg[] = "{\"type\":\"help\",\"msg\":\"\n\nCandy Bugger\n\nSend Frame: '$', [DLC], [ID]x3|8, [Dat]x0~16\n$ designations:\n\t'-' == Std Data Frame\n\t'=' == Ext Data frame\n\t'_' == Std Remote Frame\n\t'+' == Ext Remote Frame\n\nSet Filter: '$', 0b00[RTRm][RTR], [ID]x3|8, [IDm]x3|8\n$ designations:\n\t',' == Std Unmasked Filter\n\t'.' == Ext Unmasked Filter\n\t'<' == Std Masked Filter\n\t'>' == Ext Masked Filter\n\nHelp (for no-ui operation): 'H'|'h'\n\"}\n";
	xSemaphoreTake(UartTxMtxHandle, portMAX_DELAY);
	Serial2_writeBuf(helpmsg);
	xSemaphoreGive(UartTxMtxHandle);
}

void listFilters(){
	static uint8_t truemsg[] = "true";
	static uint8_t falsemsg[] = "flase";
	static uint8_t idmsg[] = "xxxxxxxx";
	static uint8_t filternummsg[] = "xx";

	static uint8_t filtermsg1[] = "{\"type\":\"filterList\",\"filters\":[";
	static uint8_t filtermsg2[]	= "{\"filterNum\":";
	static uint8_t filtermsg3[]	= ",\"isExtended\":";
	static uint8_t filtermsg4[]	= ",\"isMasked\":";
	static uint8_t filtermsg5[]	= ",\"isRemote\":";
	static uint8_t filtermsg6[]	= ",\"maskRemote\":";
	static uint8_t filtermsg7[]	= ",\"id\":\"";
	static uint8_t filtermsg8[]	= "\",\"mask\":\"";
	static uint8_t filtermsg9[]	= "\"}";
	static uint8_t filtermsg10[] = "]}\n";

	static uint8_t isFirstMsg;
	static uint8_t len;

	xSemaphoreTake(UartTxMtxHandle, portMAX_DELAY);
	Serial2_writeBuf(filtermsg1);

	isFirstMsg = 1;
	for(int i=0; i<4*CAN_BANKS; i++){
		static Can_filter_t newFilter;
		if(bxCan_getFilter(&newFilter, i)==0){ /*FIXME get filter is glitched*/
			if(isFirstMsg){
				isFirstMsg = 0;
			}else{
				Serial2_write(',');
			}
			Serial2_writeBuf(filtermsg2);
			len = intToDec(newFilter.filterNum, filternummsg);
			Serial2_writeBytes(filternummsg, len);
			Serial2_writeBuf(filtermsg3);
			newFilter.isExt ? Serial2_writeBuf(truemsg) : Serial2_writeBuf(falsemsg);
			Serial2_writeBuf(filtermsg4);
			newFilter.isMasked ? Serial2_writeBuf(truemsg) : Serial2_writeBuf(falsemsg);
			Serial2_writeBuf(filtermsg5);
			newFilter.isRemote ? Serial2_writeBuf(truemsg) : Serial2_writeBuf(falsemsg);
			Serial2_writeBuf(filtermsg6);
			newFilter.maskRemote ? Serial2_writeBuf(truemsg) : Serial2_writeBuf(falsemsg);
			Serial2_writeBuf(filtermsg7);
			intToHex(newFilter.id, idmsg, (newFilter.isExt?8:3));
			Serial2_writeBytes(idmsg, (newFilter.isExt?8:3));
			if(newFilter.isMasked){
				Serial2_writeBuf(filtermsg8);
				intToHex(newFilter.mask, idmsg, (newFilter.isExt?8:3));
				Serial2_writeBytes(idmsg, (newFilter.isExt?8:3));
			}
			Serial2_writeBuf(filtermsg9);
		}
	}
	Serial2_writeBuf(filtermsg10);
	xSemaphoreGive(UartTxMtxHandle);
	isFirstMsg = 1;
}

void deleteFilter(){
	static uint8_t filterNum;
	static uint8_t tempChar;
	static int success;
	waitTilAvail(1);
	tempChar = Serial2_read();
	tempChar = fromHex(tempChar);
	if(tempChar > 9){
		logError(ERR_invalidDecChar, ERR_Abort);
		return;
	}
	filterNum = 10*tempChar;
	waitTilAvail(1);
	tempChar = Serial2_read();
	tempChar = fromHex(tempChar);
	if(tempChar > 9){
		logError(ERR_invalidDecChar, ERR_Abort);
		return;
	}
	filterNum += tempChar;
	if(filterNum >= CAN_BANKS*4){
		logError(ERR_invalidFilterNum, ERR_Abort);
		return;
	}
	success = bxCan_removeFilter(filterNum);
	(success==-1) ? notifyFail() : notifySuccess();
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_WWDG_Init();

  /* USER CODE BEGIN 2 */
  Serial2_begin();

  /*TODO better boot msg*/
  static uint8_t bootmsg1[] = "{\"type\":\"boot\",\"module\":\"Debugger\",\"version\":\"";
  static uint8_t bootmsg2[] = "\",\"reason\":\"\"}\n";
  Serial2_writeBuf(bootmsg1);
  Serial2_writeBuf(FWVersionNumber);
  Serial2_writeBuf(bootmsg2);

  bxCan_begin(&hcan1, &mainCanRxQHandle, &mainCanTxQHandle);
  bxCan_setTxCallback(cantxcb);
  bxCan_setErrCallback(canercb);
  bxCan_addMaskedFilterStd(0,0,0); //catch all
  bxCan_addMaskedFilterExt(0,0,0);
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of UartTxMtx */
  osMutexDef(UartTxMtx);
  UartTxMtxHandle = osMutexCreate(osMutex(UartTxMtx));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of WWDGTmr */
  osTimerDef(WWDGTmr, TmrKickDog);
  WWDGTmrHandle = osTimerCreate(osTimer(WWDGTmr), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(WWDGTmrHandle, 16);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of processCan */
  osThreadDef(processCan, doProcessCan, osPriorityLow, 0, 512);
  processCanHandle = osThreadCreate(osThread(processCan), NULL);

  /* definition and creation of ProcessUart */
  osThreadDef(ProcessUart, doProcessUart, osPriorityNormal, 0, 512);
  ProcessUartHandle = osThreadCreate(osThread(ProcessUart), NULL);

  /* definition and creation of housekeeping */
  osThreadDef(housekeeping, doHousekeeping, osPriorityBelowNormal, 0, 512);
  housekeepingHandle = osThreadCreate(osThread(housekeeping), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of mainCanTxQ */
  osMessageQDef(mainCanTxQ, 64, Can_frame_t);
  mainCanTxQHandle = osMessageCreate(osMessageQ(mainCanTxQ), NULL);

  /* definition and creation of mainCanRxQ */
  osMessageQDef(mainCanRxQ, 64, Can_frame_t);
  mainCanRxQHandle = osMessageCreate(osMessageQ(mainCanRxQ), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_11TQ;
  hcan1.Init.BS2 = CAN_BS2_4TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
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

/* WWDG init function */
static void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 127;
  hwwdg.Init.Counter = 127;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA3 PA4 
                           PA5 PA6 PA7 PA8 
                           PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4 
                           PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* doProcessCan function */
void doProcessCan(void const * argument)
{

  /* USER CODE BEGIN 5 */
  static Can_frame_t newFrame;

  /*JSON goodness*/
  static uint8_t truemsg[] = "true";
  static uint8_t falsemsg[] = "false";
  static uint8_t stdidmsg[] = "xxx";
  static uint8_t extidmsg[] = "xxxxxxxx";
  static uint8_t datamsg[] = ",\"xx\"";
  static uint8_t framemsg1[] = "{\"type\":\"frame\",\"ide\":"; //bool
  static uint8_t framemsg2[] = ",\"rtr\":";		//bool
  static uint8_t framemsg3[] = ",\"dlc\":";		//number
  static uint8_t framemsg4[] = ",\"id\":\"";		//string (hex)
  static uint8_t framemsg5[] = "\",\"data\":[";	//strings (hex)
  static uint8_t framemsg6[] = "]}\n";			//for data frames
  static uint8_t framemsg5b[] = "\"}\n";			//for remote frames

  /* Infinite loop */
  for(;;)
  {
	  xQueueReceive(mainCanRxQHandle, &newFrame, portMAX_DELAY);
	  xSemaphoreTake(UartTxMtxHandle, portMAX_DELAY);

	  /*bang out the frame in JSON*/
	  Serial2_writeBuf(framemsg1);
	  newFrame.isExt ? Serial2_writeBuf(truemsg) : Serial2_writeBuf(falsemsg);
	  Serial2_writeBuf(framemsg2);
	  newFrame.isRemote ? Serial2_writeBuf(truemsg) : Serial2_writeBuf(falsemsg);
	  Serial2_writeBuf(framemsg3);
	  Serial2_write(toHex(newFrame.dlc));
	  Serial2_writeBuf(framemsg4);
	  if(newFrame.isExt){
		  intToHex(newFrame.id, extidmsg, 8);
		  Serial2_writeBuf(extidmsg);
	  }else{
		  intToHex(newFrame.id, stdidmsg, 3);
		  Serial2_writeBuf(stdidmsg);
	  }
	  if(newFrame.isRemote){
		  Serial2_writeBuf(framemsg5b);
	  }else{
		  Serial2_writeBuf(framemsg5);
		  for(int i=0; i<newFrame.dlc; i++){
			  intToHex(newFrame.Data[i], datamsg+2, 2);
			  if(i==0){
				  Serial2_writeBytes(datamsg+1, sizeof(datamsg)-2);
			  }else{
				  Serial2_writeBuf(datamsg);
			  }
		  }
		  Serial2_writeBuf(framemsg6);
	  }

	  xSemaphoreGive(UartTxMtxHandle);
  }
  /* USER CODE END 5 */ 
}

/* doProcessUart function */
void doProcessUart(void const * argument)
{
  /* USER CODE BEGIN doProcessUart */
	/* Infinite loop */
	for(;;)
	{
		if(Serial2_available()){
			static uint8_t cmd;
			cmd = Serial2_read();
			switch(cmd){
			case '-':
				parseFrame(0,0);
				break;
			case '=':
				parseFrame(1,0);
				break;
			case '_':
				parseFrame(0,1);
				break;
			case '+':
				parseFrame(1,1);
				break;
			case ',':
				parseFilter(0,0);
				break;
			case '.':
				parseFilter(1,0);
				break;
			case '<':
				parseFilter(0,1);
				break;
			case '>':
				parseFilter(1,1);
				break;
			case '/':
				deleteFilter();
				break;
			case '?':
				listFilters();
				break;
			case 'H':
			case 'h':
				displayHelp();
				break;
			default:
				logError(ERR_invalidCommand, ERR_Warn);
			}
		}else{
			osDelay(1);
		}
	}
  /* USER CODE END doProcessUart */
}

/* doHousekeeping function */
void doHousekeeping(void const * argument)
{
  /* USER CODE BEGIN doHousekeeping */

	/*JSON goodness*/
	static uint8_t sentmsg1[] = "{\"type\":\"sent\",\"count\":";
	static uint8_t sentmsg2[] = "}\n";
	static uint8_t sentcount[] = "xxxxx";

	/* Infinite loop */
	for(;;){
		/*First, check for errors*/
		if(CanErr){
			for(int i=0; i<9; i++){
				if(CanErr & (1 << i)){
					logError(ERR_CanErrorEWG + i, ERR_Warn);
				}
			}
			CanErr = 0;
		/*Then, check for frames sent*/
		}else if(CanSent){
			static uint16_t sent; //interrupt proofing
			static uint8_t len;
			sent = CanSent;
			len = intToDec(sent, sentcount);

			xSemaphoreTake(UartTxMtxHandle, portMAX_DELAY);
			Serial2_writeBuf(sentmsg1);
			Serial2_writeBytes(sentcount, len);
			Serial2_writeBuf(sentmsg2);
			xSemaphoreGive(UartTxMtxHandle);

			CanSent -= sent;
		}else{
			osDelay(1);
		}
	}
  /* USER CODE END doHousekeeping */
}

/* TmrKickDog function */
void TmrKickDog(void const * argument)
{
  /* USER CODE BEGIN TmrKickDog */
  taskENTER_CRITICAL();
  HAL_WWDG_Refresh(&hwwdg);
  taskEXIT_CRITICAL();
  /* USER CODE END TmrKickDog */
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
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
