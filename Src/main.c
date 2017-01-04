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


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

WWDG_HandleTypeDef hwwdg;

osThreadId processCanHandle;
osThreadId ProcessUartHandle;
osMessageQId mainCanTxQHandle;
osMessageQId mainCanRxQHandle;
osTimerId WWDGTmrHandle;
osMutexId UartTxMtxHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static uint32_t inputId;
static uint8_t dataBuf[8];
static uint8_t inputMode = 0; //0=id, 1=data
static uint8_t inputLength = 0;

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
void TmrKickDog(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
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

int intToDec(int input, uint8_t *str){ //returns length. Only does positives.
	int length = 0;
	uint8_t output[10];
	while(input/10){
		length++;
		output[10-length] = toHex(input%10);
		input/=10;
	}
	length++;
	output[10-length] = toHex(input%10);
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
	static uint8_t txcpltmsg[] = "\nSENT!\n";
	Serial2_writeBytes(txcpltmsg, sizeof(txcpltmsg)-1);
}

void sayYes(uint8_t *str){
	str[0]='Y';
	str[1]='e';
	str[2]='s';
}

void sayNo(uint8_t *str){
	str[0]='N';
	str[1]='o';
	str[2]=' ';
}

//void stashFrame(){
//	if(Can_available()==CAN_BUFFER_LENGTH && canFullNotified==0){
//		canFullNotified=1;
//		static uint8_t canFullMsg[] = "\nHURRY UP! CAN BUFFER OVERFLOWING!!!\n";
//		Serial2_writeBytes(canFullMsg, sizeof(canFullMsg)-1);
//	}
//}

void SendFrameUI(){
	while(1){
		if(Serial2_available()){
			static uint8_t delims1[] = {',',';'};
			static uint8_t delims2[] = {0x0d,0x0a};
			if(inputMode){//entering data
			  if(Serial2_findAny(delims2, sizeof(delims2))==0){
				  while(Serial2_findAny(delims2, sizeof(delims2))==0) Serial2_read();
				  sendFrameFromInput();
				  inputId = 0;
				  inputMode = 0;
				  inputLength = 0;
				  return;
			  }else{
				  toCaps(Serial2_buffer, 1);
				  uint8_t hex = fromHex(Serial2_read());
					  if(hex <= 0x0f){
					  dataBuf[inputLength/2] = dataBuf[inputLength/2]<<4;
					  dataBuf[inputLength/2] += hex;
					  inputLength++;
					  if(inputLength > 16){
						  static uint8_t idovfmsg[] = "\nDATA TOO LONG! RETRY.\n";
						  Serial2_writeBytes(idovfmsg, sizeof(idovfmsg)-1);
						  inputLength = 0;
						  inputId = 0;
						  inputMode = 0;
						  return;
					  }
				  }
			  }
			}else{//entering id
			  if(Serial2_findAny(delims1, sizeof(delims1))==0){
				  while(Serial2_findAny(delims1, sizeof(delims1))==0) Serial2_read();
				  inputMode = 1;
				  inputLength = 0;
			  }else{
				  toCaps(Serial2_buffer, 1);
				  uint8_t hex = fromHex(Serial2_read());
				  if(hex <= 0x0f){
					  inputId = inputId<<4;
					  inputId += hex;
					  inputLength++;
					  if(inputLength > 8){
						  static uint8_t idovfmsg[] = "\nID TOO LONG! RETRY.\n";
						  Serial2_writeBytes(idovfmsg, sizeof(idovfmsg)-1);
						  inputLength = 0;
						  inputId = 0;
						  return;
					  }
				  }
			  }
			}
		}
	}
}

uint8_t helpmsg[] = "\nUSAGE:\n\
				S: Send Frame\n\
				F: Filter Management\n\
				P: Periodic Send\n\
				H: This Message\n\
				esc: Abort Command\n";

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
  static uint8_t startmsg[] = "boot diag\n";
  Serial2_writeBytes(startmsg, sizeof(startmsg)-1);

  bxCan_begin(&hcan1, &mainCanRxQHandle, &mainCanTxQHandle);
  bxCan_setTxCallback(cantxcb);
//  Can_addFilterStd(0x001, 0);
//  Can_addMaskedFilterStd(0x002, 0x7FF, 0);
//  Can_addFilterExt(0x003, 0);
//  Can_addMaskedFilterExt(0x004, 0x1FFFFFFF, 0);
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
  /*TODO start watchdog kicker*/
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of processCan */
  osThreadDef(processCan, doProcessCan, osPriorityLow, 0, 512);
  processCanHandle = osThreadCreate(osThread(processCan), NULL);

  /* definition and creation of ProcessUart */
  osThreadDef(ProcessUart, doProcessUart, osPriorityNormal, 0, 512);
  ProcessUartHandle = osThreadCreate(osThread(ProcessUart), NULL);

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
//	  if(Serial2_available()){
//		  doCommand();
//		  static uint8_t delims1[] = {',',';'};
//		  static uint8_t delims2[] = {0x0d,0x0a};
//		  if(inputMode){//entering data
//			  if(Serial2_findAny(delims2, sizeof(delims2))==0){
//				  while(Serial2_findAny(delims2, sizeof(delims2))==0) Serial2_read();
//				  sendFrameFromInput();
//				  inputId = 0;
//				  inputMode = 0;
//				  for(int i=0; i<8; i++){dataBuf[i]=0;}
//				  inputLength = 0;
//			  }else{
//				  toCaps(Serial2_buffer, 1);
//				  uint8_t hex = fromHex(Serial2_read());
//					  if(hex <= 0x0f){
//					  dataBuf[inputLength/2] = dataBuf[inputLength/2]<<4;
//					  dataBuf[inputLength/2] += hex;
//					  inputLength++;
//					  if(inputLength > 16){
//						  static uint8_t idovfmsg[] = "\nDATA TOO LONG! RETRY.\n";
//						  Serial2_writeBytes(idovfmsg, sizeof(idovfmsg)-1);
//						  inputLength = 0;
//						  inputId = 0;
//						  inputMode = 0;
//						  for(int i=0; i<8; i++){dataBuf[i]=0;}
//					  }
//				  }
//			  }
//		  }else{//entering id
//			  if(Serial2_findAny(delims1, sizeof(delims1))==0){
//				  while(Serial2_findAny(delims1, sizeof(delims1))==0) Serial2_read();
//				  inputMode = 1;
//				  inputLength = 0;
//			  }else{
//				  toCaps(Serial2_buffer, 1);
//				  uint8_t hex = fromHex(Serial2_read());
//				  if(hex <= 0x0f){
//					  inputId = inputId<<4;
//					  inputId += hex;
//					  inputLength++;
//					  if(inputLength > 8){
//						  static uint8_t idovfmsg[] = "\nID TOO LONG! RETRY.\n";
//						  Serial2_writeBytes(idovfmsg, sizeof(idovfmsg)-1);
//						  inputLength = 0;
//						  inputId = 0;
//					  }
//				  }
//			  }
//		  }
//	  }
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
  hcan1.Init.SJW = CAN_SJW_3TQ;
  hcan1.Init.BS1 = CAN_BS1_12TQ;
  hcan1.Init.BS2 = CAN_BS2_3TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = ENABLE;
  hcan1.Init.AWUM = ENABLE;
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
				//parseFrame(0,0);
				break;
			case '=':
				//parseFrame(1,0);
				break;
			case '_':
				//parseFrame(0,1);
				break;
			case '+':
				//parseFrame(1,1);
				break;
			case ',':
				//parseFilter(0,0);
				break;
			case '.':
				//parseFilter(1,0);
				break;
			case '<':
				//parseFilter(0,1);
				break;
			case '>':
				//parseFilter(1,1);
				break;
			case 'H':
			case 'h':
				//displayHelp();
				break;
			default: ; //do nothing
			}
		}else{
			osDelay(1);
		}
	}
	/* USER CODE END doProcessUart */
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
