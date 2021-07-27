/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
	SPI1:
	STM32F205RBTx ---- 300evk:	
	PA4-----------------CS
	PA5-----------------SCK
	PA6-----------------MISO
	PA7-----------------MOSI
	GND(near 5VIN)-----GND
	PA2----------------nRST may exist problem, so not recommended
	PC14----------------DRDY
	PC13----------------1PPS----GPS_PPS
	PB15---Logic analyzer to detect, sync with PPS rising edge
	
	UART1:
	stm32F205rtb6------CH340:
	PB7_UART_RX--------CH340_TX
	PB6_UART_TX--------CH340_RX
	-----SWD shoud disconnected if you want UART keep working. restart power------key point
	
	CAN1:  
	stm32F205rtb6------SN65HVD230 CAN Board:
	PB9 CAN1_TX--------CAN_TX
	PB8 CAN1_RX--------CAN_RX
	CAN_RX0 interrupts---yes
	
	PPS: 
	UBLOX M8N  ------- 300_EVK
	USB power supply
	GNG  ------------ GND_EVK
	天线
	
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//----------------------------------------------------------------------------SPI self function declaration
void HAL_Delay_us(uint8_t num);
void HAL_SPI_T_R(uint8_t* send_data, uint8_t* get_data, uint8_t frequency);
void HAL_SPI_T_R_Burst(uint8_t *send_data, uint8_t* get_data, uint8_t subregister_num, uint8_t frequency);
void HAL_SPI_W_R(uint8_t *send_data, uint8_t *target_register);
void HAL_SPI_Print_Burst(uint8_t* spi_rx_data, uint8_t subregister_num); //statu, gyro*3, acc*3,temp,drdy_length, pps_after.....

uint8_t spi_drdy_on = 0;
uint8_t spi_quiet = 1;
uint8_t spi_single_mode = 0;
uint8_t spi_single_reg = 0x56;
uint8_t spi_burst_mode = 1;
uint8_t spi_burst_reg = 0x3E; //3E is 8 words, 3F is 10 words, pls remember to change other codes on 2 areas
uint8_t spi_burst_subregisters_num = 8;
uint8_t spi_rx_data[11][2] = {0X00};

uint8_t spi_write_mode = 0;	
uint8_t spi_write_save = 0;      //spi write data and write registers, SAVE temp == 0, SAVE permanently == 1
uint8_t spi_write_data = 0X01;	
uint8_t spi_write_register = 0X37;	
uint8_t spi_write_save_data= 0XFF;	
uint8_t spi_write_save_register = 0X76;	

uint8_t print_status=1;
//----------------------------------------------------------------------------SPI self function declaration

//----------------------------------------------------------------------------CAN self function declaration
void vApp_CAN_TxHeader_Init(CAN_TxHeaderTypeDef* pHeader, uint32_t  StdId, uint32_t ExtId, uint32_t IDE, uint32_t RTR, uint32_t DLC);
void vApp_CAN_Filter_Init(CAN_FilterTypeDef * pFilter, uint32_t IdHigh, uint32_t IdLow, uint32_t MaskIdHigh, uint32_t MaskIdLow, uint32_t FIFOAssignment, uint32_t Bank, uint32_t Mode, uint32_t Scale, uint32_t Activation, uint32_t SlaveStartFilterBank);
void vApp_User_CAN_Configuration(void);
void vApp_CAN_Configuration(CAN_TxHeaderTypeDef * pTxHeader, CAN_FilterTypeDef * pFilter, uint32_t StdId, uint32_t ExtId, uint32_t IDE, uint32_t RTR, uint32_t DLC, uint32_t IdHigh, uint32_t IdLow, uint32_t MaskIdHigh, uint32_t MaskIdLow, uint32_t FIFOAssignment, uint32_t Bank, uint32_t Mode, uint32_t Scale, uint32_t Activation, uint32_t SlaveStartFilterBank);
void vApp_User_CAN1_TxMessage(uint8_t aTxData[], uint8_t DLC);
void vApp_CAN_TxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef * pTxHeader, uint8_t aData[], uint8_t DLC);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
CAN_TxHeaderTypeDef hCAN1_TxHeader; //CAN1????
CAN_RxHeaderTypeDef hCAN1_RxHeader; //CAN1????
CAN_FilterTypeDef hCAN1_Filter; //CAN1???

//----------------------------------------------------------------------------CAN END

//-----------------------------------------------------------------------------UART1 Start
uint8_t aRxBuffer;
uint8_t Uart1_RxBuff[256];
uint8_t Uart1_Rx_Cnt = 0;

int value = 0x00;
char *cmd = "";
char uart_rx_data[50];
uint8_t uart_tx_data[2]= "z1";

int fputc(int ch, FILE *f);
void set_value(char *cmd, int *value);
//-----------------------------------------------------------------------------UART1 End

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
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
	//----------------------------------------SPI Variables and preparation BEGIN
	
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  //A4 back to high      1. keep ground together with Master and Slave ---must
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);  //first pull up, used for indicate PPS rising edge detection by pull down(change to reset) later
	//----------------------------------------SPI Variables and preparation END

	//CAN-------------------------------------CAN_1 Variables and preparation BEGIN  0x601
	uint8_t CAN_TxData[3] = {0x00, 0xff, 0x53};
	

	//CAN-------------------------------------CAN_1 Variables and preparation END
	
	//UART1-------------------------------------UART_1 Variables and preparation BEGIN  SWD disconnected---must
	
	

	//UART1-------------------------------------UART_1 Variables and preparation END
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	vApp_User_CAN_Configuration();
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);  //reset power up the IMU330(runing master first, and then start IMU. must for 330)
	HAL_Delay(1000);
	//printf("reg,status,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,temp,drdy_length,pps_after\n");

  while(1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//--------------------------------------------------------------------------------SPI BEGIN
		//interrupt callback func() working at bottom of main.c
		if(spi_single_mode && spi_quiet == 0 && spi_drdy_on == 0)
		{
			HAL_Delay(5);	
			HAL_SPI_T_R(&spi_single_reg, spi_rx_data[1], 200); //single read ID 0x56 or 0x0E z_accel, 0x56, 00, 00, 00		
			if(print_status)
			{
				printf("%02x %02x %02x\n", spi_single_reg, *spi_rx_data[1],*(spi_rx_data[1]+1));	
			}
		}
		
		if(spi_burst_mode && spi_quiet == 0 && spi_drdy_on == 0)
		{
			HAL_Delay(5);	
			HAL_SPI_T_R_Burst(&spi_burst_reg, spi_rx_data[0], spi_burst_subregisters_num, 200);  //only support 330,300
			
			if(print_status)
			{
				printf("%02x",  spi_burst_reg);
				HAL_SPI_Print_Burst(spi_rx_data[0], spi_burst_subregisters_num); 
			}		
		}
		
		if(spi_write_mode) //Write mode: write data to target register, such as 0xF010, target address is 0x70, and data input is 0x10
		{
			HAL_Delay(5);						
			HAL_SPI_W_R(&spi_write_data, &spi_write_register); 
			printf("write data:0x%02x to register:0x%02x finished!\n", spi_write_data, spi_write_register);				
			if(spi_write_save == 1)
			{
				HAL_Delay(1000); //parpare to write register0x76 with 0xFF data to save configuration
				HAL_SPI_W_R(&spi_write_save_data, &spi_write_save_register);
				HAL_Delay(1000); //parpare to write register0x76 with 0xFF data to save configuration
				printf("saved configurations!\n");
			}
			spi_write_mode = 0;
		}
		//--------------------------------------------------------------------------------SPI END
		
		//--------------------------------------------------------------------------------CAN BEGIN				
		//vApp_User_CAN1_TxMessage(CAN_TxData, 3);  //send CAN message, data(CAN_TxData) and ID(vApp_User_CAN_Configuration)
		//receive realized in interrupt function: HAL_CAN_RxFifo0MsgPendingCallback
		//--------------------------------------------------------------------------------CAN END

		//USART1--------------------------------------------------------------------------USART1 BEGIN		
		//memset(uart_rx_data, 0, 50);
		//HAL_UART_Receive(&huart1,(uint8_t *)uart_rx_data,50,0); //1000ms receive one data
		
		//if(strlen(uart_rx_data) != 0)
		//{
			//printf("%s %d\n", uart_rx_data, strlen(uart_rx_data));
		//}
		
		
		
		//sscanf(uart_rx_data, "%s = %u", cmd, &value);
		//set_value(cmd, &value);		
		
		//USART1--------------------------------------------------------------------------USART1 END		
		
		// 1 pps signal simulator---------------------------------------------------------PPS START
	  //temp += 5;
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		//if(temp%335 == 0)
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		// 1 pps signal simulator---------------------------------------------------------PPS START
				
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15); // only reverse the voltage level
		
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : pps_PC13_Pin */
  GPIO_InitStruct.Pin = pps_PC13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(pps_PC13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : drdy_ready_PC14_Pin */
  GPIO_InitStruct.Pin = drdy_ready_PC14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(drdy_ready_PC14_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief delya us by for loop times
  * @param None
  * @retval None
  */
void HAL_Delay_us(uint8_t num)
{
 for(uint8_t i=0; i<num; i++) 
  { 
   for(uint8_t j=0; j<20; j++)
   {
   }
 }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  UNUSED(huart);
 
	if(Uart1_Rx_Cnt >= 255)  //????
	{
		Uart1_Rx_Cnt = 0;
		memset(Uart1_RxBuff,0x00,sizeof(Uart1_RxBuff));
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_tx_data, sizeof(uart_tx_data),0xFFFF);	
	}
	else
	{
		Uart1_RxBuff[Uart1_Rx_Cnt++] = aRxBuffer;		
	
		if((Uart1_RxBuff[Uart1_Rx_Cnt-1] == 0x0A)&&(Uart1_RxBuff[Uart1_Rx_Cnt-2] == 0x0D)) 
		{
			HAL_UART_Transmit(&huart1, (uint8_t *)&Uart1_RxBuff, Uart1_Rx_Cnt,0xFFFF); 
			
			char *next_token = NULL;
			cmd = strtok_r(Uart1_RxBuff, "=", &next_token);
			value = strtoimax(strtok_r(NULL, "=", &next_token),next_token,0);			
			set_value(cmd, &value);	
			
			Uart1_Rx_Cnt = 0;
			memset(Uart1_RxBuff,0x00,sizeof(Uart1_RxBuff)); 
		}
	}
	
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);   
}

void set_value(char *cmd, int *value)
	{
		printf("cmd:%s value:%d\n",cmd, *value);
		if(strcmp(cmd, "spi_drdy_on")==0)
		{
			spi_drdy_on = *value;
			printf("get request success!");
		}
		else if(strcmp(cmd, "spi_quiet")==0)
		{
			spi_quiet = *value;
			printf("get request success!");
		}
		else if(strcmp(cmd, "spi_single_mode")==0)
		{
			spi_single_mode = *value;
			printf("get request success!");
		}
		else if(strcmp(cmd, "spi_single_reg")==0)
		{
			spi_single_reg = *value;
			printf("get request success!");
		}
		else if(strcmp(cmd, "spi_burst_mode")==0)
		{
			spi_burst_mode = *value;
			printf("get request success!");
		}
		else if(strcmp(cmd, "spi_burst_reg")==0)
		{
			spi_burst_reg = *value;
			printf("get request success!");
		}
		else if(strcmp(cmd, "spi_burst_subregisters_num")==0)
		{
			spi_burst_subregisters_num = *value;
			printf("get request success!");
		}
		else if(strcmp(cmd, "spi_write_mode")==0)
		{
			spi_write_mode = *value;
			printf("get request success!");
		}
		else if(strcmp(cmd, "spi_write_save")==0)
		{
			spi_write_save = *value;
			printf("get request success!");
		}
		else if(strcmp(cmd, "spi_write_data")==0)
		{
			spi_write_data = *value;
			printf("get request success!");
		}
		else if(strcmp(cmd, "spi_write_register")==0)
		{
			spi_write_register = *value;
			printf("get request success!");
		}
		else if(strcmp(cmd, "print_status")==0)
		{
			print_status = *value;
			printf("get request success!");
		}
		else if(strcmp(cmd, "current_value")==0)
		{
			 printf("spi_drdy_on=0x%x\nspi_quiet=0x%x\nspi_single_mode=0x%x\nspi_single_reg=0x%x\nspi_burst_mode=0x%x\nspi_burst_reg=0x%x\nspi_burst_subregisters_num=0x%x\nspi_write_mode=0x%x\nspi_write_save=0x%x\nspi_write_data=0x%x\nspi_write_register=0x%x\nprint_status=0x%x\n",\
								spi_drdy_on, spi_quiet, spi_single_mode, spi_single_reg, spi_burst_mode, spi_burst_reg, spi_burst_subregisters_num,spi_write_mode, spi_write_save, spi_write_data, spi_write_register, print_status);
		}
	}	


/**
  * @brief send and receive the data, single read（8bits mode, 1Mhz）
  * @param None
  * @retval None
  */
void HAL_SPI_T_R(uint8_t *send_data, uint8_t *get_data, uint8_t frequency)
{  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //SET A4 low 
  //HAL_SPI_TransmitReceive(&hspi1,data,data,2,1000); // time interval between 2 word(16-bits) cannot be adjusted, so not used the sentence
  HAL_SPI_Transmit(&hspi1,send_data,2,5);  //request to register
  HAL_Delay_us(20); //give time interval 16 us between 2 16bits, met t-interval > 15 micro-seconds
  HAL_SPI_Receive(&hspi1,get_data,2,5);   //receive IMU register back data
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  //A4 back to high
}

/**
  * @brief send data to target register for writing
  * @param None
  * @retval None
  */
void HAL_SPI_W_R(uint8_t *send_data, uint8_t *target_register)
{  
	uint8_t write_data[2] = {target_register[0] | 0x80, send_data[0]};
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //SET A4 low 
  HAL_SPI_Transmit(&hspi1,write_data,2,5);  //request to register
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  //A4 back to high	
}

/**
  * @brief send and receive the data, burst read（8bits mode, 1Mhz）
  * @param None
  * @retval None
  */
void HAL_SPI_T_R_Burst(uint8_t *send_data, uint8_t* get_data, uint8_t subregister_num, uint8_t frequency)
{		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	//SET A4 low	
		//HAL_SPI_TransmitReceive(&hspi1,data,data,2,1000);	// time interval between 2 word(16-bits) cannot be adjusted, so not used the sentence
		HAL_SPI_Transmit(&hspi1,send_data,2,5);	

		for(int i=0; i<subregister_num; i++)
		{
			HAL_Delay_us(20);
			HAL_SPI_Receive(&hspi1, &(get_data[2*i]),2,5);			
		}						
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  //A4 back to high    
}

void HAL_SPI_Print_Burst(uint8_t* spi_rx_data, uint8_t subregister_num) //statu, gyro*3, acc*3,temp,drdy_length, pps_after.....
{
	for(int i=0; i < subregister_num; i++)  //spend 10 ms, i++ ~ 1ms
		{
			int temp = 0.0; 
			temp = (spi_rx_data[2*i]*256 + spi_rx_data[2*i+1]);
			if(temp > 32767)
				temp -= 65536;
			if(i>0 && i<4)
			{				
				printf(",%f", temp/64.0);
			}
			else if(i>3 && i<7)
			{
				printf(",%f", temp/4000.0);
			}
			else if(i == 7)
			{
				printf(",%f", temp*0.073111172849435+31.0);  //以秒为单位输出drdy_length;
			}
			else if(i == 8)
			{
				printf(",%f", temp*0.001);  //以秒为单位输出drdy_length;
			}
			else if(i == 9)
			{
				printf(",%f", temp*0.0001);  //以秒为单位输出pps_after;
			}
			else
			{
				printf(",0x%02x%02x", spi_rx_data[2*i], spi_rx_data[2*i+1]);
			}	
		}
		printf("\n");
}

void vApp_CAN_TxHeader_Init(CAN_TxHeaderTypeDef    * pHeader,
                                                        uint32_t                             StdId, 
                                                        uint32_t                             ExtId, 
                                                        uint32_t                             IDE, 
                                                        uint32_t                             RTR, 
                                                        uint32_t                             DLC)
{
    pHeader->StdId    = StdId;    //11?     ?????
    pHeader->ExtId    = ExtId;    //29?     ?????
    pHeader->IDE        = IDE;        //1?        0:??? 1:???
    pHeader->RTR        = RTR;      //1?   0:??? 1:???
    pHeader->DLC        = DLC;        //4?   ????????
    pHeader->TransmitGlobalTime    =    ENABLE;
} 

void vApp_CAN_Filter_Init(CAN_FilterTypeDef * pFilter,
                                                    uint32_t IdHigh,
                                                    uint32_t IdLow,
                                                    uint32_t MaskIdHigh,
                                                    uint32_t MaskIdLow,
                                                    uint32_t FIFOAssignment,
                                                    uint32_t Bank,
                                                    uint32_t Mode,
                                                    uint32_t Scale,
                                                    uint32_t Activation,
                                                    uint32_t SlaveStartFilterBank)
{
    pFilter->FilterIdHigh                 = 0;
    pFilter->FilterIdLow                  = 0;
    pFilter->FilterMaskIdHigh         =    0;
    pFilter->FilterMaskIdLow             =    0;
    pFilter->FilterFIFOAssignment = CAN_FILTER_FIFO0;
    pFilter->FilterBank                     = 0;
    pFilter->FilterMode                     = CAN_FILTERMODE_IDMASK;
    pFilter->FilterScale                     = CAN_FILTERSCALE_32BIT;
    pFilter->FilterActivation         = ENABLE;
    pFilter->SlaveStartFilterBank = 14;
}

void vApp_User_CAN_Configuration(void)
{
	/*----------------- CAN????? --------------------------*/
	vApp_CAN_Configuration(&hCAN1_TxHeader, &hCAN1_Filter,0x12, 0x18EAFF03, CAN_ID_EXT, CAN_RTR_DATA, 3,	0, 0, 0, 0, CAN_FILTER_FIFO0, 0, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT, ENABLE, 0);
}

void vApp_CAN_Configuration(CAN_TxHeaderTypeDef    * pTxHeader,
                                                        CAN_FilterTypeDef     * pFilter,
                                                        uint32_t                             StdId, 
                                                        uint32_t                             ExtId, 
                                                        uint32_t                             IDE, 
                                                        uint32_t                             RTR, 
                                                        uint32_t                             DLC,
                                                        uint32_t                             IdHigh,
                                                        uint32_t                             IdLow,
                                                        uint32_t                             MaskIdHigh,
                                                        uint32_t                             MaskIdLow,
                                                        uint32_t                             FIFOAssignment,
                                                        uint32_t                             Bank,
                                                        uint32_t                             Mode,
                                                        uint32_t                             Scale,
                                                        uint32_t                             Activation,
                                                        uint32_t                             SlaveStartFilterBank)
{
    /*-1- ???TxHeader?? ----------------------------------------*/
    vApp_CAN_TxHeader_Init(pTxHeader, StdId, ExtId, IDE, RTR, DLC);
    
    /*-2- ???????? ------------------------------------------*/
    vApp_CAN_Filter_Init(pFilter, IdHigh, IdLow, MaskIdHigh, MaskIdLow, FIFOAssignment, Bank, Mode, Scale, Activation, SlaveStartFilterBank);
    HAL_CAN_ConfigFilter(&hcan1, pFilter);
    
    /*-3- ??CAN ---------------------------------------------------*/
    while(HAL_CAN_Start(&hcan1) != HAL_OK )
    {        
        HAL_Delay(100);
    }    
    /*-4- ?????? ----------------------------------------------*/
		
		/* Activate CAN RX notification */
		if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		{
			/* Notification Error */
			Error_Handler();
		}
}

void vApp_User_CAN1_TxMessage(uint8_t aTxData[], uint8_t DLC)
{
	vApp_CAN_TxMessage(&hcan1, &hCAN1_TxHeader, aTxData, DLC);
}

void vApp_CAN_TxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef * pTxHeader, uint8_t aData[], uint8_t DLC)
{
    uint32_t Tx_MailBox;
		uint32_t temp = 0x18;  
	  uint8_t aRxData[8];
    /*-1- ??????? ----------------------------------------*/
    pTxHeader->DLC    =    DLC;
    /*-2- ??aData ---------------------------------------------*/
		temp = HAL_CAN_AddTxMessage(hcan, pTxHeader, aData, &Tx_MailBox);
	printf("REQUEST: Time:%d ID:0x%x LEN:%d TXDATA[0]:%02x %02x %02x\n", hCAN1_TxHeader.TransmitGlobalTime, hCAN1_TxHeader.ExtId, hCAN1_TxHeader.DLC, 
							aData[0], aData[1], aData[2]);
    while(temp != HAL_OK)
    {
        HAL_Delay(100);
    }		
		HAL_Delay(1000);	
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t aRxData[8], i;		
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hCAN1_RxHeader, aRxData) == HAL_OK)
    {      
			if(hCAN1_RxHeader.ExtId == 0x18FF5380)
			{
				printf("Receive: Time:%d ID:0x%x LEN:%d RxData[0]:%x %x %x %x %x %x %x %x\n", hCAN1_RxHeader.Timestamp, hCAN1_RxHeader.ExtId, hCAN1_RxHeader.DLC, 
							aRxData[0], aRxData[1], aRxData[2], aRxData[3], aRxData[4], aRxData[5], aRxData[6], aRxData[7]);
			}
						
    }
}

int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xffff);
	return ch;
}
int drdy_index = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{		
		if(GPIO_Pin == drdy_ready_PC14_Pin && spi_drdy_on)
		{
			if(spi_single_mode)
			{
				HAL_SPI_T_R(&spi_single_reg, spi_rx_data[1], 200); //single read ID 0x56 or 0x0E z_accel, 0x56, 00, 00, 00		
				if(print_status)
				{
					printf("%02x %02x %02x\n", spi_single_reg, *spi_rx_data[1],*(spi_rx_data[1]+1));	
				}			
			}
			
			if(spi_burst_mode)
			{
				HAL_SPI_T_R_Burst(&spi_burst_reg, spi_rx_data[0], spi_burst_subregisters_num, 200);  //only support 330,300				
				if(print_status)
				{
					printf("%02x",  spi_burst_reg);
					HAL_SPI_Print_Burst(spi_rx_data[0], spi_burst_subregisters_num); 
				}
			}
			//HAL_UART_Transmit(&huart1,UART_Tx_n,2,0xffff);//send TxData by huart1,length is 10,timeout is 0xffff		
			//HAL_UART_Transmit(&huart1,UART_Tx_t,2,0xffff);//send TxData by huart1,length is 10,timeout is 0xffff
			

		}
		
		
		
		if(GPIO_Pin == pps_PC13_Pin && spi_drdy_on)  //another interrupt pin to detect PPS rising edge
		{				 
//			  if(drdy_index != 200)
//				{
//					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);// detect pps rising edge, pull down
//			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
//					printf("drdy:%d\n", drdy_index);
//				}
//				drdy_index = 0;
//			  printf("pps");
				//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15); // only reverse the voltage level
		}
		
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
