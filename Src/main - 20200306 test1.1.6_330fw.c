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
	
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

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
int fputc(int ch, FILE *f);
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
	uint8_t SPI_TXdata[2] = {0X37, 0X00};	
	uint8_t SPI_TXdata2[2] = {0X7E, 0X00};	
	uint8_t SPI_Burstdata[2] = {0X3E, 0X00};	 //3E is 8 words, 3F is 10 words, pls remember to change other codes 
	uint8_t SPI_RXdata[11][2] = {0X00};
	
	uint8_t SPI_W_SAVE = 1;      //spi write data and write registers, SAVE temp == 0, SAVE permanently == 1
	uint8_t SPI_WRdata[1] = {0X01};	
	uint8_t SPI_WRregister[1] = {0X37};	
	uint8_t SPI_WRdata_SAVE = 0XFF;	
	uint8_t SPI_WRreg_SAVE = 0X76;		
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  //A4 back to high      1. keep ground together with Master and Slave ---must
	//----------------------------------------SPI Variables and preparation END

	//CAN-------------------------------------CAN_1 Variables and preparation BEGIN  0x601
	uint8_t CAN_TxData[3] = {0x00, 0xff, 0x53};
	

	//CAN-------------------------------------CAN_1 Variables and preparation END
	
	//UART1-------------------------------------UART_1 Variables and preparation BEGIN  SWD disconnected---must
	uint8_t UART_TxData[2]= "z1";
	uint8_t UART_RxData='F';
	//UART1-------------------------------------UART_1 Variables and preparation END
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	vApp_User_CAN_Configuration();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);  //reset power up the IMU330(runing master first, and then start IMU. must for 330)
	HAL_Delay(1000);
	printf("restart IMU finished!\t");	
	printf("start while loop function:\n");
	int temp = 0;
	int i = 0;
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
		//--------------------------------------------------------------------------------SPI BEGIN
		//single mode
		HAL_Delay(5);
		HAL_SPI_T_R(SPI_TXdata, SPI_RXdata[0], 200); //single read ID 0x56 or 0x0E z_accel, 0x56, 00, 00, 00
		printf("reg: 0x%02x DATA single: 0x%02x-%02x\n", *SPI_TXdata,*SPI_RXdata[0],*(SPI_RXdata[0]+1));
		//HAL_Delay(5);	
		//HAL_SPI_T_R(SPI_TXdata2, SPI_RXdata[1], 200); //single read ID 0x56 or 0x0E z_accel, 0x56, 00, 00, 00		
		//printf("reg: 0x%02x DATA single: 0x%02x-%02x\n", *SPI_TXdata2,*SPI_RXdata[1],*(SPI_RXdata[1]+1));		
		
		//Write mode: write data to target register, such as 0xF010, target address is 0x70, and data input is 0x10
		for(; i<10; i++)
		{
			HAL_Delay(5);	
			printf("write register:0x%02x  write data:0x%02x\n", *SPI_WRregister, *SPI_WRdata);			
			HAL_SPI_W_R(SPI_WRdata, SPI_WRregister);
		}  
		if(SPI_W_SAVE == 1 && i < 11)
		{
			HAL_Delay(1000); //parpare to write register0x76 with 0xFF data to save configuration
		  HAL_SPI_W_R(&SPI_WRdata_SAVE, &SPI_WRreg_SAVE);
			i++;
			HAL_Delay(1000); //parpare to write register0x76 with 0xFF data to save configuration
			printf("write register SAVED \n");
		}
		
		//burst mode
		HAL_Delay(5);	
		HAL_SPI_T_R_Burst(SPI_Burstdata, SPI_RXdata[0], 8, 200);  //only support 330,300
		printf("request:0x%02x burst:", *SPI_Burstdata);
		for(int i=0; i < 8; i++)
		{
			printf("0x%02x%02x ", SPI_RXdata[i][0], SPI_RXdata[i][1]);
		}	
		printf("\n");
		
		//--------------------------------------------------------------------------------SPI END
		
		//--------------------------------------------------------------------------------CAN BEGIN				
		//vApp_User_CAN1_TxMessage(CAN_TxData, 3);  //send CAN message, data(CAN_TxData) and ID(vApp_User_CAN_Configuration)
		//receive realized in interrupt function: HAL_CAN_RxFifo0MsgPendingCallback
		//--------------------------------------------------------------------------------CAN END

		//USART1--------------------------------------------------------------------------USART1 BEGIN	
		//printf("--Start: send by UART area--");		
		//HAL_UART_Transmit(&huart1,UART_TxData,2,0xffff);//send TxData by huart1,length is 10,timeout is 0xffff		
		//HAL_UART_Receive(&huart1,(uint8_t *)&UART_RxData,1,1000); //1000ms receive one data
		//if (UART_RxData != 'F')
		//{
			//UART_RxData = 0;
		//}		
		//printf("--End: UART area--");	
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

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

/**
  * @brief send and receive the data, single read£¨8bits mode, 1Mhz£©
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
  * @brief send and receive the data, burst read£¨8bits mode, 1Mhz£©
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
