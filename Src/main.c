/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include "arm_math.h"
#include "SHT1X.h"
#include "LED.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart6_rx;

osThreadId TASK_STATE_LEDHandle;
osThreadId TASK_SERIALHandle;
osThreadId TASK_CANHandle;
osThreadId TASK_GPRSHandle;
osThreadId TASK_NEPORTHandle;
osThreadId TASK_SENSOR_LEDHandle;
osSemaphoreId BinarySem_serialHandle;
osSemaphoreId BinarySem_gprsHandle;
osSemaphoreId BinarySem_neportHandle;
osSemaphoreId CountingSem_canHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
TickType_t PreviousWakeTime;                               
uint8_t serial_rxbuf[BUFFER_SIZE]; 
uint8_t neport_rxbuf[BUFFER_SIZE]; 
uint8_t gprs_rxbuf[BUFFER_SIZE]; 
uint8_t can_txbuf[8] = {1,2,3,4,5,6,7,8}; 
uint8_t can_rxbuf[128];
uint8_t serial_rxlen = 0; 
uint8_t neport_rxlen = 0; 
uint8_t gprs_rxlen = 0; 
uint8_t data_to_send[SUB_BOARD][81];
uint8_t main_ID = 0x01;
uint16_t humi_val_i = 0,temp_val_i = 0;
SHT1x sht1x; 

uint8_t channel_count = 0;
uint8_t sensor_online[LEDn];
float sensor_frequency[LEDn];
float sensor_temp[LEDn];
extern GPIO_TypeDef* LED_PORT[LEDn];
extern uint16_t LED_PIN[LEDn];

char gprs_connect_str[] = "AT+CIPSTART=\"TCP\",\"xxx.xxx.xxx.xxx\",\"50000\"\r\n";
char gprs_send_str[] = "AT+CIPSEND=81\r\n";
char gprs_disconnect_str[] = "AT+CIPCLOSE\r\n";
char gprs_shut_str[] = "AT+CIPSHUT\r\n";
char gprs_noecho_str[] = "ATE0\r\n";

enum{
	HANDSHAKE = 0,
	CONNECT,
	TCPACK,
	SHUT,
	WAITFORCMD,
	SEND,
	WAITFORSDC
};
uint8_t GPRSSTATE = 0;
uint8_t can_error_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
void FUNC_STATE_LED(void const * argument);
void FUNC_SERIAL(void const * argument);
void FUNC_CAN(void const * argument);
void FUNC_GPRS(void const * argument);
void FUNC_NEPORT(void const * argument);
void FUNC_SENSOR_LED(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void data_calc(void);
void get_SHT1x(void);
void get_ID(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void data_calc()
{
	int i,s,m,j;
	float count;
	float adc;
	uint16_t frequency, temp;
	float temp_log;
	float A;
	float B;
	float C;
	
	A = 0.0014051f;
  B = 0.0002369f;
  C = 0.0000001019f;
	
	for(i = 0;i < SUB_BOARD;i ++)
	{
		data_to_send[i][0] = 0x53;
		data_to_send[i][1] = 0x01;
		data_to_send[i][2] = 0x02;
		data_to_send[i][3] = 0x03;
		data_to_send[i][4] = 0x04;
		data_to_send[i][5] = 0x05;
		data_to_send[i][6] = 0x06;
		data_to_send[i][7] = main_ID + i;
		
		s=8;
		
		for(m = 0;m < 2;m ++)
		{   
			data_to_send[i][s] = m;s++;
			for(j = 0;j < 8;j ++)
			{
				count = can_rxbuf[j * 4 + m * 32 + i * 64]*256 + can_rxbuf[j * 4 + m * 32 + i * 64 + 1];
				sensor_frequency[channel_count] = 10000000.0f / count;
				
				adc = (can_rxbuf[j * 4 + m * 32 + i * 64 + 2]*256 + can_rxbuf[j * 4 + m * 32 + i * 64 + 3])*4.5185f;
				temp_log = log(adc);
				sensor_temp[channel_count] = 1.0f / (A + B * temp_log + C * temp_log * temp_log * temp_log) - 273.2f; 
				
				frequency = (uint16_t)(sensor_frequency[channel_count] * 10.0f);
				temp = (uint16_t)(sensor_temp[channel_count] * 100.0f);
				
				data_to_send[i][s] = frequency >> 8;s++;
				data_to_send[i][s] = frequency;     s++;
				data_to_send[i][s] = temp >> 8;     s++; 
				data_to_send[i][s] = temp;          s++;	

				channel_count ++;
			}
		}
		
		data_to_send[i][s] = 0x00;s++;
		data_to_send[i][s] = 0x00;s++;

		data_to_send[i][s] = sht1x.Humidity>>8;s++;
		data_to_send[i][s] = sht1x.Humidity;s++;
			
		data_to_send[i][s] = sht1x.Temperature>>8;s++;
		data_to_send[i][s] = sht1x.Temperature;s++;
			
		data_to_send[i][s]=  'E';s++;
	}
}

void get_SHT1x()
{
	uint8_t error = 0, checksum;
	uint16_t T_Value=0,H_Value=0;
	
	error+=SHT1x_Measure( &T_Value,&checksum,0);  //measure temperature 
  error+=SHT1x_Measure( &H_Value,&checksum,1);  //measure humidity
			
  if(error != 0)
		SHT1x_Reset(); //通讯故障,复位传感器
  else
  {
		sht1x.T_Result = T_Value;
    sht1x.H_Result = H_Value;
    SHT1X_Caculation1((float*)&sht1x.T_Result, (float*)&sht1x.H_Result ); 
           
    sht1x.Temperature=sht1x.T_Result;
    sht1x.Humidity=(uint16_t)sht1x.H_Result; 
  }
}

void get_ID()
{
	main_ID = 0;
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) == GPIO_PIN_SET)
		main_ID += 1;
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) == GPIO_PIN_SET)
		main_ID += 2;
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_SET)
		main_ID += 4;
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_SET)
		main_ID += 8;
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_SET)
		main_ID += 16;
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t value = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_CAN1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE); 
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);

	if(!SHT1X_read_statusreg(&value))
  { 
		if(value&0x07)
    {
			for(value=5;value!=0;value--)
      { 
				if(!SHT1X_write_statusreg(0)) break;
        else SHT1X_softreset();
      }
    }
  }
	memset(sensor_online,0,sizeof(sensor_online));
	memset(sensor_frequency,0,sizeof(sensor_frequency));
	memset(sensor_temp,0,sizeof(sensor_temp));
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of BinarySem_serial */
  osSemaphoreDef(BinarySem_serial);
  BinarySem_serialHandle = osSemaphoreCreate(osSemaphore(BinarySem_serial), 1);

  /* definition and creation of BinarySem_gprs */
  osSemaphoreDef(BinarySem_gprs);
  BinarySem_gprsHandle = osSemaphoreCreate(osSemaphore(BinarySem_gprs), 1);

  /* definition and creation of BinarySem_neport */
  osSemaphoreDef(BinarySem_neport);
  BinarySem_neportHandle = osSemaphoreCreate(osSemaphore(BinarySem_neport), 1);

  /* definition and creation of CountingSem_can */
  osSemaphoreDef(CountingSem_can);
  CountingSem_canHandle = osSemaphoreCreate(osSemaphore(CountingSem_can), 3);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	xSemaphoreTake(BinarySem_serialHandle,portMAX_DELAY);
	xSemaphoreTake(BinarySem_gprsHandle,portMAX_DELAY);
	xSemaphoreTake(BinarySem_neportHandle,portMAX_DELAY);
	
	xSemaphoreTake(CountingSem_canHandle,portMAX_DELAY);
	xSemaphoreTake(CountingSem_canHandle,portMAX_DELAY);
	xSemaphoreTake(CountingSem_canHandle,portMAX_DELAY);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of TASK_STATE_LED */
  osThreadDef(TASK_STATE_LED, FUNC_STATE_LED, osPriorityLow, 0, 128);
  TASK_STATE_LEDHandle = osThreadCreate(osThread(TASK_STATE_LED), NULL);

  /* definition and creation of TASK_SERIAL */
  osThreadDef(TASK_SERIAL, FUNC_SERIAL, osPriorityBelowNormal, 0, 128);
  TASK_SERIALHandle = osThreadCreate(osThread(TASK_SERIAL), NULL);

  /* definition and creation of TASK_CAN */
  osThreadDef(TASK_CAN, FUNC_CAN, osPriorityNormal, 0, 128);
  TASK_CANHandle = osThreadCreate(osThread(TASK_CAN), NULL);

  /* definition and creation of TASK_GPRS */
  osThreadDef(TASK_GPRS, FUNC_GPRS, osPriorityBelowNormal, 0, 128);
  TASK_GPRSHandle = osThreadCreate(osThread(TASK_GPRS), NULL);

  /* definition and creation of TASK_NEPORT */
  osThreadDef(TASK_NEPORT, FUNC_NEPORT, osPriorityBelowNormal, 0, 128);
  TASK_NEPORTHandle = osThreadCreate(osThread(TASK_NEPORT), NULL);

  /* definition and creation of TASK_SENSOR_LED */
  osThreadDef(TASK_SENSOR_LED, FUNC_SENSOR_LED, osPriorityLow, 0, 128);
  TASK_SENSOR_LEDHandle = osThreadCreate(osThread(TASK_SENSOR_LED), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_5TQ;
  hcan1.Init.BS2 = CAN_BS2_6TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0 
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE6 PE7 PE8 PE9 
                           PE11 PE12 PE14 PE0 
                           PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_0 
                          |GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0 
                           PC1 PC2 PC3 PC4 
                           PC5 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0 
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5 
                           PA6 PA7 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB12 PB4 
                           PB5 PB6 PB7 PB8 
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD10 PD12 PD13 
                           PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD1 PD2 PD3 
                           PD4 PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* FUNC_STATE_LED function */
void FUNC_STATE_LED(void const * argument)
{

  /* USER CODE BEGIN 5 */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
  /* Infinite loop */
  for(;;)
  {
		if(GPRSSTATE & 0x04)
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
			osDelay(500);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
			osDelay(500);
		}
		else ;
		if(can_error_flag)
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
		}
  }
  /* USER CODE END 5 */ 
}

/* FUNC_SERIAL function */
void FUNC_SERIAL(void const * argument)
{
  /* USER CODE BEGIN FUNC_SERIAL */
	int i;
	BaseType_t pdsem = pdFALSE;
	HAL_UART_Receive_DMA(&huart6,serial_rxbuf,BUFFER_SIZE);
  /* Infinite loop */
  for(;;)
  {
		pdsem = xSemaphoreTake(BinarySem_serialHandle,portMAX_DELAY);
		
		//HAL_UART_Transmit(&huart1,serial_rxbuf,serial_rxlen,1000); //测试GPRS用，之后记得注掉
		
		if(pdsem == pdTRUE)
		{
			if(strstr((char *)serial_rxbuf,"REQUEST FOR DATA!") != NULL)
			{
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET); // 485发送使能
				xSemaphoreGive(CountingSem_canHandle);
				for(i = 0;i < SUB_BOARD;i ++)
				{
					HAL_UART_Transmit(&huart6,&data_to_send[i][0],81,1000);
				}
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET); // 485接收使能
			}
		}
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET); // 485接收使能
		memset(serial_rxbuf,0,sizeof(serial_rxbuf));
		HAL_UART_Receive_DMA(&huart6,serial_rxbuf,BUFFER_SIZE);
  }
  /* USER CODE END FUNC_SERIAL */
}

/* FUNC_CAN function */
void FUNC_CAN(void const * argument)
{
  /* USER CODE BEGIN FUNC_CAN */
	int i,j;
	uint8_t *pcan_rxbuf;
	CanTxMsgTypeDef TxMessage;
	CanRxMsgTypeDef RxMessage;
	CAN_FilterConfTypeDef  sFilterConfig;
	
	hcan1.pTxMsg = &TxMessage;
	hcan1.pRxMsg = &RxMessage;
	sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->DLC = 8;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	memcpy(hcan1.pTxMsg->Data,can_txbuf,8);

  /* Infinite loop */
  for(;;)
  {
    xSemaphoreTake(CountingSem_canHandle,portMAX_DELAY);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
		get_SHT1x();
		get_ID();
		hcan1.pTxMsg->StdId = 0x01;
		HAL_CAN_Transmit(&hcan1, 1000);
		HAL_Delay(2000);
		pcan_rxbuf = &can_rxbuf[0];
		channel_count = 0;
		for(i = 0;i < 2 * SUB_BOARD;i ++)
		{
			hcan1.pTxMsg->StdId = 2 + ((i + 1) << 4);
			HAL_CAN_Transmit(&hcan1, 1000);
			HAL_Delay(10);
			can_error_flag = 0;
			for(j = 0;j < 4;j ++)
			{
				if(HAL_CAN_Receive(&hcan1, CAN_FIFO0,100) == HAL_OK)
				{
					memcpy(pcan_rxbuf + j * 8,hcan1.pRxMsg->Data,8);
				}
				else
				{
					can_error_flag = i + 1;
					break;
				}
			}
			pcan_rxbuf += 32;
		}
		data_calc();
		test_sensor();
		memset(can_rxbuf,0,sizeof(can_rxbuf));
  }
  /* USER CODE END FUNC_CAN */
}

/* FUNC_GPRS function */
void FUNC_GPRS(void const * argument)
{
  /* USER CODE BEGIN FUNC_GPRS */
	int send_count = 0, send_flag = 0;
	BaseType_t pdsem = pdFALSE;
	HAL_UART_Receive_DMA(&huart1,gprs_rxbuf,BUFFER_SIZE);
	osDelay(20000);
	HAL_UART_Transmit(&huart1,(uint8_t *)gprs_noecho_str,strlen(gprs_noecho_str),1000);
  /* Infinite loop */
 for(;;)
  {
    pdsem = xSemaphoreTake(BinarySem_gprsHandle,portMAX_DELAY);
		//HAL_UART_Transmit(&huart6,gprs_rxbuf,gprs_rxlen,1000);
		if(pdsem == pdTRUE)
		{
			if((strstr((char *)gprs_rxbuf,"+PDP: DEACT") != NULL) || (strstr((char *)gprs_rxbuf,"CLOSED") != NULL))
			{
				GPRSSTATE = SHUT;
				HAL_UART_Transmit(&huart1,(uint8_t *)gprs_shut_str,strlen(gprs_shut_str),1000);
			}
			else
			{
				switch(GPRSSTATE)
				{
					case CONNECT:
					{
						if(strstr((char *)gprs_rxbuf,"OK") != NULL)
							GPRSSTATE = TCPACK;
						break;
					}
					case TCPACK:
					{
						if(strstr((char *)gprs_rxbuf,"CONNECT OK") || (strstr((char *)gprs_rxbuf,"ALREADY CONNECT") != NULL))
							GPRSSTATE = WAITFORCMD;
						else if(strstr((char *)gprs_rxbuf,"CONNECT FAIL") || (strstr((char *)gprs_rxbuf,"+CME ERROR") != NULL))
						{
							GPRSSTATE = SHUT;
							HAL_UART_Transmit(&huart1,(uint8_t *)gprs_shut_str,strlen(gprs_shut_str),1000);
						}
						break;
					}
					case HANDSHAKE:
					{
						if(strstr((char *)gprs_rxbuf,"ATE0") != NULL)
						{
							GPRSSTATE = CONNECT;
							HAL_UART_Transmit(&huart1,(uint8_t *)gprs_connect_str,strlen(gprs_connect_str),1000);
						}
						break;
					}
					case SHUT:
					{
						osDelay(5000);
						if(strstr((char *)gprs_rxbuf,"SHUT OK") != NULL)
						{
							GPRSSTATE = CONNECT;
							HAL_UART_Transmit(&huart1,(uint8_t *)gprs_connect_str,strlen(gprs_connect_str),1000);
						}
						break;
					}
					case WAITFORCMD:
					{
						if(gprs_rxlen >= 17)
						{
							//if(strstr((char *)gprs_rxbuf,"S") != NULL)
							if(strstr((char *)gprs_rxbuf,"REQUEST FOR DATA!") != NULL)
							{
								xSemaphoreGive(CountingSem_canHandle);
								GPRSSTATE = SEND;
								HAL_UART_Transmit(&huart1,(uint8_t *)gprs_send_str,strlen(gprs_send_str),1000);
								HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
							}
						}
						else if(send_flag == 1)
						{
								GPRSSTATE = SEND;
								HAL_UART_Transmit(&huart1,(uint8_t *)gprs_send_str,strlen(gprs_send_str),1000);
								HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
						}
						break;
					}
					case SEND:
					{
						if(strstr((char *)gprs_rxbuf,">") != NULL)
						{
							GPRSSTATE = WAITFORSDC;
							HAL_UART_Transmit(&huart1,&data_to_send[send_count][0],81,1000);
							send_count ++;
						}
						break;
					}
					case WAITFORSDC:
					{
						if(strstr((char *)gprs_rxbuf,"SEND OK") != NULL)
						{
							GPRSSTATE = WAITFORCMD;
							if(send_count <  SUB_BOARD)
							{
								osDelay(1000);
								send_flag = 1;
								xSemaphoreGive(BinarySem_gprsHandle);
							}
							else
							{
								send_count = 0;
								send_flag = 0;
							}
						}
						else if((strstr((char *)gprs_rxbuf,"SEND FAIL") != NULL) || (strstr((char *)gprs_rxbuf,"+CME ERROR") != NULL))
						{
							GPRSSTATE = SHUT;
							HAL_UART_Transmit(&huart1,(uint8_t *)gprs_shut_str,strlen(gprs_shut_str),1000);
						}
						break;
					}
					default: break;
				}
			}
		}
		memset(gprs_rxbuf,0,sizeof(gprs_rxbuf));
		HAL_UART_Receive_DMA(&huart1,gprs_rxbuf,BUFFER_SIZE);
  }
  /* USER CODE END FUNC_GPRS */
}

/* FUNC_NEPORT function */
void FUNC_NEPORT(void const * argument)
{
  /* USER CODE BEGIN FUNC_NEPORT */
	int i;
	BaseType_t pdsem = pdFALSE;
	HAL_UART_Receive_DMA(&huart2,neport_rxbuf,BUFFER_SIZE);
  /* Infinite loop */
  for(;;)
  {
    pdsem = xSemaphoreTake(BinarySem_neportHandle,portMAX_DELAY);
		if(pdsem == pdTRUE)
		{
			if(strstr((char *)neport_rxbuf,"REQUEST FOR DATA!") != NULL)
			{
				xSemaphoreGive(CountingSem_canHandle);
				for(i = 0;i < SUB_BOARD;i ++)
				{
					HAL_UART_Transmit(&huart2,&data_to_send[i][0],81,1000);
				}
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
			}
		}
		memset(neport_rxbuf,0,sizeof(neport_rxbuf));
		HAL_UART_Receive_DMA(&huart2,neport_rxbuf,BUFFER_SIZE);
  }
  /* USER CODE END FUNC_NEPORT */
}

/* FUNC_SENSOR_LED function */
void FUNC_SENSOR_LED(void const * argument)
{
  /* USER CODE BEGIN FUNC_SENSOR_LED */
	int i;
  /* Infinite loop */
	for(;;)
  {
		for(i = 0; i < LEDn; i ++)
		{
			if(sensor_online[i] == 1)
			{
				HAL_GPIO_WritePin(LED_PORT[i], LED_PIN[i], GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(LED_PORT[i], LED_PIN[i], GPIO_PIN_SET);
			}
		}
    osDelay(1000);
  }
  /* USER CODE END FUNC_SENSOR_LED */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
	return ;
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
