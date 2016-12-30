/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "serial.h"
#include "can.h"
#define TIMER_COUNT 16
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

CRC_HandleTypeDef hcrc;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static uint32_t inputId;
static uint8_t dataBuf[8];
static uint8_t inputMode = 0; //0=id, 1=data
static uint8_t inputLength = 0;
static uint8_t canFullNotified = 0;
uint32_t millis = 0;
uint16_t timerDelays[TIMER_COUNT];
uint32_t timerPhases[TIMER_COUNT];
Can_frame_t timerFrames[TIMER_COUNT];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CRC_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);

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

void sendFrameFromInput(){
	if(inputId<=0x7ff){
		Can_sendStd(inputId,0,dataBuf,(inputLength+1)/2);
	}else{
		Can_sendExt(inputId,0,dataBuf,(inputLength+1)/2);
	}
}

void spitFrame(){
	while(Can_available()){
		if(canFullNotified) canFullNotified = 0;
		static Can_frame_t leFrame;
		  Can_read(&leFrame); //16,33,43
		  static uint8_t gotFrameMsg[] = "\nGOT FRAME!\nID: ????????\nRemote: ???\nData: ?? ?? ?? ?? ?? ?? ?? ??\n";
		  if(leFrame.isExt){
			  intToHex(leFrame.core.id, gotFrameMsg+16, 8);
		  }else{
			  intToHex(leFrame.core.id, gotFrameMsg+16, 3);
			  for(int i=0; i<5; i++){gotFrameMsg[19+i]=' ';}
		  }
		  leFrame.isRemote ? sayYes(gotFrameMsg+33) : sayNo(gotFrameMsg+33);
		  for(int i=0; i<8; i++){
			  if(i<leFrame.core.dlc){
				  intToHex(leFrame.core.Data[i], gotFrameMsg+43+(3*i), 2);
			  }else{
				  gotFrameMsg[3*i+43]=' '; gotFrameMsg[3*i+44]=' ';
			  }
		  }
		  Serial2_writeBytes(gotFrameMsg, sizeof(gotFrameMsg)-1);
	}
}

void stashFrame(){
	if(Can_available()==CAN_BUFFER_LENGTH && canFullNotified==0){
		canFullNotified=1;
		static uint8_t canFullMsg[] = "\nHURRY UP! CAN BUFFER OVERFLOWING!!!\n";
		Serial2_writeBytes(canFullMsg, sizeof(canFullMsg)-1);
	}
}

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

void PeriodicFrameUI(){
	static uint8_t tmractivatedmsg[] = "\nTimers activated: ????????????????\n";
	Serial2_writeBuf(tmractivatedmsg);
}

void doCommand(){
	switch(Serial2_read()){
	case 'S':
	case 's':
		Can_setRxCallback(stashFrame);
		static uint8_t sendframemsg[] = "\nSend Frame: type \"[id],[data][enter]\"\n";
		Serial2_writeBytes(sendframemsg, sizeof(sendframemsg)-1);
		SendFrameUI();
		break;
	case 'F':
	case 'f':
		break;
	case 'P':
	case 'p':
		Can_setRxCallback(stashFrame);
		PeriodicFrameUI();
		break;
	case 0x0a:
	case 0x0d:
	case ' ':
	case '\t':
		break;
	case 0x1b: //esc
	case 0x08: ; //bksp
		static uint8_t rootmenumsg[] = "\nAlready at root menu!\n";
		Serial2_writeBytes(rootmenumsg, sizeof(rootmenumsg)-1);
		break;
	default: ; //hacky empty statement to fix the mysterious label error
		static uint8_t helpmsg[] = "\nUSAGE:\n\
				S: Send Frame\n\
				F: Filter Management\n\
				P: Periodic Send\n\
				H: This Message\n\
				esc: Abort Command\n";
		Serial2_writeBuf(helpmsg);
		break;
	}
	spitFrame();
	Can_setRxCallback(spitFrame);
}

void doTimers(){
	for(int i=0; i<TIMER_COUNT; i++){
		if(timerDelays[i]){
			if((millis-timerPhases[i])/timerDelays[i]==0){
				if(timerFrames[i].isExt){
					Can_sendExt(timerFrames[i].core.id, timerFrames[i].isRemote, timerFrames[i].core.Data, timerFrames[i].core.dlc);
				}else{
					Can_sendStd(timerFrames[i].core.id, timerFrames[i].isRemote, timerFrames[i].core.Data, timerFrames[i].core.dlc);
				}
			}
		}
	}
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
  MX_CRC_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  Serial2_begin();
  static uint8_t startmsg[] = "boot diag\n";
  Serial2_writeBytes(startmsg, sizeof(startmsg)-1);

  Can_begin();
  Can_setTxCallback(cantxcb);
  Can_setRxCallback(spitFrame);
//  Can_addFilterStd(0x001, 0);
//  Can_addMaskedFilterStd(0x002, 0x7FF, 0);
//  Can_addFilterExt(0x003, 0);
//  Can_addMaskedFilterExt(0x004, 0x1FFFFFFF, 0);
  Can_addMaskedFilterStd(0,0,0); //catch all
  Can_addMaskedFilterExt(0,0,0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
	  if(Serial2_available()){
		  doCommand();
		  static uint8_t delims1[] = {',',';'};
		  static uint8_t delims2[] = {0x0d,0x0a};
		  if(inputMode){//entering data
			  if(Serial2_findAny(delims2, sizeof(delims2))==0){
				  while(Serial2_findAny(delims2, sizeof(delims2))==0) Serial2_read();
				  sendFrameFromInput();
				  inputId = 0;
				  inputMode = 0;
				  for(int i=0; i<8; i++){dataBuf[i]=0;}
				  inputLength = 0;
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
						  for(int i=0; i<8; i++){dataBuf[i]=0;}
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
					  }
				  }
			  }
		  }
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_3TQ;
  hcan1.Init.BS1 = CAN_BS1_12TQ;
  hcan1.Init.BS2 = CAN_BS2_3TQ;
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

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 7;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_16B;
  hcrc.Init.InitValue = 0;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_NOVEMBER;
  sDate.Date = 0x29;
  sDate.Year = 0x16;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
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

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
