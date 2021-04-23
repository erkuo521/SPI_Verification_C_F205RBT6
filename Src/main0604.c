/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
	* @author         : Silva.Lee 20200603
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

#include "sensor.h"

#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

#define REG_ACC_X                           (0x0A)
#define REG_ACC_Y                           (0x0C)
#define REG_ACC_Z                           (0x0E)
#define REG_GYRO_X                          (0x04)
#define REG_GYRO_Y                          (0x06)
#define REG_GYRO_Z                          (0x08)

#define PRODUCT_ID               (0x56)
#define REG_BOARD_TEMP           (0x18)
#define REG_DRDY_RATE            (0x37)
#define REG_ACCEL_LPF            (0x38)
#define REG_RATE_LPF              (0x78)

#define REG_ACCEL_SCALE_FACTOR    (0x46)
#define REG_RATE_SCALE_FACTOR      (0x47)
#define REG_MASTER_STATUS          (0x5A)
#define REG_HW_STATUS              (0x5C)
#define REG_SW_STATUS               (0x5E)
#define REG_ACCEL_RANGE             (0x70)
#define REG_RATE_RANGE              (0x71)
#define REG_ORIENTATION_MSB          (0x74)
#define REG_ORIENTATION_LSB          (0x75)


#define REG_SAVE_CONFIG            (0x76)

#define MEM_SIZE   (0xF0) //memory size of the development board, n sizeof(imu_size_t)


/* USER CODE BEGIN PD */
//----------------------------------------------------------------------------SPI self function declaration
void HAL_Delay_us(uint8_t num);
void HAL_SPI_T_R(uint8_t* send_data, uint8_t* get_data, uint8_t frequency);
void HAL_SPI_T_R_adapt_330_write(uint8_t* send_data, uint8_t* get_data, uint8_t frequency);
void HAL_SPI_T_R_Burst(uint8_t *send_data, uint8_t *get_data, uint8_t subregister_num, uint8_t frequency);
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

void test_timer(void)
{
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12); // only reverse the voltage level
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);	//SET A4 low	
	HAL_Delay_us(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);	//SET A4 high
	HAL_Delay_us(10);
}


// dat_out_p input as: uint8_t data_in[2] = {0,0}
void single_spi_read(uint8_t reg,uint8_t *dat_out_p)
{
    uint8_t SPI_TXdata[2] = {0x00, 0x00};	
		uint8_t SPI_RXdata[11][2] = {0X00}; //prepare spi buffer
		
		SPI_TXdata[0] = reg;
    HAL_SPI_T_R(SPI_TXdata, SPI_RXdata[0], 200);
		
		*dat_out_p++ =  *SPI_RXdata[0];
		*dat_out_p = *(SPI_RXdata[0]+1);
}

//aceinna imu: write command(eg:0x35|0x80) by appending the write-bit/address combination with the value
//to be written to the direct register 
void single_spi_write(uint8_t reg_addr,uint8_t data)
{
		uint8_t SPI_TXdata[2] = {0x00, 0x00};	
		uint8_t SPI_RXdata[11][2] = {0x00}; //prepare spi buffer
		
		SPI_TXdata[0] = reg_addr | 0x80;
		SPI_TXdata[1] = data;
		
    HAL_SPI_T_R_adapt_330_write(SPI_TXdata, SPI_RXdata[0], 200);
}


void readout_all_register(void)
{
		uint8_t buf_reg[2]={0};
		uint8_t reg_table[10] = {0};
		
	  printf("read all register start! \n");
		printf("read all register start! \n");
		
    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(PRODUCT_ID,buf_reg);
		HAL_Delay_us(100);
		printf("read reg---%s MSB: 0x%x, LSB 0x%x\n", "PRODUCT_ID 0x56", buf_reg[0], buf_reg[1]);

		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(REG_BOARD_TEMP,buf_reg);
		HAL_Delay_us(100);
		printf("read reg---%s MSB: 0x%x, LSB 0x%x\n", "REG_BOARD_TEMP 0x18", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(REG_DRDY_RATE,buf_reg);
		HAL_Delay_us(100);
		printf("read reg---%s MSB: 0x%x, LSB 0x%x\n", "REG_DRDY_RATE 0x37", buf_reg[0], buf_reg[1]);
		
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(REG_RATE_LPF,buf_reg);
		HAL_Delay_us(100);
		printf("read reg---%s MSB: 0x%x, LSB 0x%x\n", "REG_RATE_LPF 0x78", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(REG_ACCEL_SCALE_FACTOR,buf_reg);
		HAL_Delay_us(100);
		printf("read reg---%s MSB: 0x%x, LSB 0x%x\n", "REG_ACCEL_SCALE_FACTOR 0x46", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(REG_MASTER_STATUS,buf_reg);
		HAL_Delay_us(100);
		printf("read reg---%s MSB: 0x%x, LSB 0x%x\n", "REG_MASTER_STATUS 0x5A", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(REG_HW_STATUS,buf_reg);
		HAL_Delay_us(100);
		printf("read reg---%s MSB: 0x%x, LSB 0x%x\n", "REG_HW_STATUS 0x5C", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(REG_SW_STATUS,buf_reg);
		HAL_Delay_us(100);
		printf("read reg---%s MSB: 0x%x, LSB 0x%x\n", "REG_SW_STATUS 0x5E", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(REG_ACCEL_RANGE,buf_reg);
		HAL_Delay_us(100);
		printf("read reg---%s MSB: 0x%x, LSB 0x%x\n", "REG_ACCEL_RANGE 0x70", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(REG_ORIENTATION_MSB,buf_reg);
		HAL_Delay_us(100);
		printf("read reg---%s MSB: 0x%x, LSB 0x%x\n", "REG_ORIENTATION_MSB 0x74", buf_reg[0], buf_reg[1]);


		printf("read all register end! \n");
		printf("read all register end! \n");
		
		printf("\n");
		printf("\n");
		printf("\n");
}


void unit_test(void)
{



}

void acc_readxyz_raw(acc_raw_t *rawdata_out_p)
{
    uint8_t buf_x[2] = {0};
    uint8_t buf_y[2] = {0};
    uint8_t buf_z[2] = {0};
		
    buf_x[0] = REG_ACC_X;
    buf_x[1] = 0x00;		
		single_spi_read(REG_ACC_X,buf_x);
		HAL_Delay_us(100);

    buf_y[0] = REG_ACC_Y;
    buf_y[1] = 0x00;
    single_spi_read(REG_ACC_Y,buf_y);
		HAL_Delay_us(100);
				
    buf_z[0] = REG_ACC_Z;
    buf_z[1] = 0x00;
		single_spi_read(REG_ACC_Z,buf_z);
    HAL_Delay_us(100);
		
		/* output acc: mg*/
    rawdata_out_p->x = (int16_t)((buf_x[0] << 8) + buf_x[1])/4;
    rawdata_out_p->y = (int16_t)((buf_y[0] << 8) + buf_y[1])/4;
    rawdata_out_p->z = (int16_t)((buf_z[0] << 8) + buf_z[1])/4;
}



void gyro_readxyz_raw(gyro_raw_t *rawdata_out_p)
{
    uint8_t buf_x[2] = {0};
    uint8_t buf_y[2] = {0};
    uint8_t buf_z[2] = {0};

    buf_x[0] = REG_GYRO_X;
    buf_x[1] = 0x00;		
		single_spi_read(REG_GYRO_X,buf_x);
		HAL_Delay_us(100); //mandatory delay for single read

    buf_y[0] = REG_GYRO_Y;
    buf_y[1] = 0x00;
    single_spi_read(REG_GYRO_Y,buf_y);
		HAL_Delay_us(100);
		
		
    buf_z[0] = REG_GYRO_Z;
    buf_z[1] = 0x00;
		single_spi_read(REG_GYRO_Z,buf_z);
		HAL_Delay_us(100);

		/* output gyro: dps*/
    rawdata_out_p->x = (int16_t)((buf_x[0] << 8) + buf_x[1])/64;
    rawdata_out_p->y = (int16_t)((buf_y[0] << 8) + buf_y[1])/64;
    rawdata_out_p->z = (int16_t)((buf_z[0] << 8) + buf_z[1])/64;
}

void register_init(void)
{
  	uint8_t buf_test[2] = {0};//init value
	
	  //init the value start
    buf_test[0] = 0x00;
    buf_test[1] = 0x00;		
		single_spi_read(0x37,buf_test);
		HAL_Delay_us(100); //mandatory delay for single read
		single_spi_write(0x37,0x02); //set 100Hz odr
		HAL_Delay_us(100);
		buf_test[0] = 0x00;
    buf_test[1] = 0x00;	
		single_spi_read(0x37,buf_test); //close acc LPF
		HAL_Delay_us(100);
		
		buf_test[0] = 0x00;
    buf_test[1] = 0x00;		
		single_spi_read(0x37,buf_test); //check 0x37,0x38
		HAL_Delay_us(100); 
	
	  buf_test[0] = 0x00;
    buf_test[1] = 0x00;		
		single_spi_read(0x78,buf_test);// read gyro LPF
		HAL_Delay_us(100); //mandatory delay for single read
		single_spi_write(0x78,0x05); //close gyro LPF
		HAL_Delay_us(100);
		
		buf_test[0] = 0x00;
    buf_test[1] = 0x00;		
		single_spi_read(0x78,buf_test); //check gyro LPF
		HAL_Delay_us(100); 
	  //init the value end
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA4 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_12;
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
  * @brief delya us by for loop times, clc by Yifan
  * @param None
  * @retval None
  */
void HAL_Delay_us(uint8_t num)
{
	for(uint8_t i=0; i<num; i++) 
  { 
			for(uint8_t j=0; j<24; j++)
			{
			}
	}
}

/**
  * @brief send and receive the data, single read£¨8bits mode, 1Mhz£©
  * @param None
  * @retval None;
  */
void HAL_SPI_T_R(uint8_t *send_data, uint8_t *get_data, uint8_t frequency)
{		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	//SET A4 low	
    HAL_Delay_us(4);
		//HAL_SPI_TransmitReceive(&hspi1,data,data,2,1000);	// time interval between 2 word(16-bits) cannot be adjusted, so not used the sentence
		HAL_SPI_Transmit(&hspi1,send_data,2,5);		//request to register
		HAL_Delay_us(20); //give time interval 16 us between 2 16bits, met t-interval > 15 micro-seconds
		HAL_SPI_Receive(&hspi1,get_data,2,5);			//receive IMU register back data
		HAL_Delay_us(4);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  //A4 back to high
    HAL_Delay_us(2);
}


void HAL_SPI_T_R_adapt_330_write(uint8_t *send_data, uint8_t *get_data, uint8_t frequency)
{		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	//SET A4 low	
    HAL_Delay_us(5);
		//HAL_SPI_TransmitReceive(&hspi1,data,data,2,1000);	// time interval between 2 word(16-bits) cannot be adjusted, so not used the sentence
		HAL_SPI_Transmit(&hspi1,send_data,2,10);		//request to register
		//HAL_Delay_us(20); //give time interval 16 us between 2 16bits, met t-interval > 15 micro-seconds
		//HAL_SPI_Receive(&hspi1,get_data,1,5);			//receive IMU register back data
		HAL_Delay_us(5);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  //A4 back to high
    HAL_Delay_us(2);
}




/**
  * @brief send and receive the data, burst read£¨8bits mode, 1Mhz£©
  * @param None
  * @retval None
  */
#if 0
void HAL_SPI_T_R_Burst(uint8_t *send_data, uint8_t *get_data, uint8_t subregister_num, uint8_t frequency)
{		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	//SET A4 low	
		//HAL_SPI_TransmitReceive(&hspi1,data,data,2,1000);	// time interval between 2 word(16-bits) cannot be adjusted, so not used the sentence
		HAL_SPI_Transmit(&hspi1,send_data,2,5);		
		HAL_Delay_us(8); //give time interval 16 us between 2 16bits, met t-interval > 15 micro-seconds
		for(int i=0; i<subregister_num; i++)
		{
			HAL_SPI_Receive(&hspi1,get_data++,2,5);
			HAL_Delay_us(8);
		}						
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  //A4 back to high			
    HAL_Delay(1000/frequency);
}
#endif
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
	vApp_CAN_Configuration(&hCAN1_TxHeader, &hCAN1_Filter,0x12, 0x18f02700, CAN_ID_EXT, CAN_RTR_DATA, 8,	0, 0, 0, 0, CAN_FILTER_FIFO0, 0, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT, ENABLE, 0);
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
    /*-1- ??????? ----------------------------------------*/
    pTxHeader->DLC    =    DLC;
    /*-2- ??aData ---------------------------------------------*/
		temp = HAL_CAN_AddTxMessage(hcan, pTxHeader, aData, &Tx_MailBox);
    while(temp != HAL_OK)
    {
        HAL_Delay(100);
    }		
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t aRxData[8], i;		
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hCAN1_RxHeader, aRxData) == HAL_OK)
    {      
			printf("%d 0x%x %d %x %x %x %x %x %x %x %x\n", hCAN1_RxHeader.Timestamp, hCAN1_RxHeader.ExtId, hCAN1_RxHeader.DLC, 
							aRxData[0], aRxData[1], aRxData[2], aRxData[3], aRxData[4], aRxData[5], aRxData[6], aRxData[7]);
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


uint16_t g_count_mlbuffer = 0;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	gyro_raw_t rawdata_gyro;
	acc_raw_t rawdata_acc;
	
	gyro_raw_t *rawdata_out_gyro_p = &rawdata_gyro;
  acc_raw_t *rawdata_out_acc_p = &rawdata_acc;
	
	
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
	
	uint8_t SPI_TXdata[2] = {0X08, 0X00};	
	uint8_t SPI_TXdata_gyrox[2] = {0X04, 0X00};	
	uint8_t SPI_TXdata_gyroy[2] = {0X06, 0X00};	
	uint8_t SPI_TXdata_gyroz[2] = {0X08, 0X00};	
	uint8_t SPI_TXdata_accx[2] = {0X0A, 0X00};	
	uint8_t SPI_TXdata_accy[2] = {0X0C, 0X00};	
	uint8_t SPI_TXdata_accz[2] = {0X0E, 0X00};	
	uint8_t SPI_RXdata[11][2] = {0X00}; //prepare spi buffer
	uint8_t SPI_Burstdata[2] = {0X3E, 0X00};	
	uint8_t UART_TxData[2]= {0x5a,0x5b};
	uint8_t UART_TxData4[4]= {0,0,0,0};
	uint8_t SPI_Burst_data[2] = {0X3F, 0X00};	 //3E is 8 words, 3F is 10 words, pls remember to change other codes on 2 areas
	uint8_t SPI_RXdata_burst[11][2] = {0X00};

	uint8_t check_data = 0;
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  //A4 back to high      1. keep ground together with Master and Slave ---must

	//----------------------------------------SPI Variables and preparation END

	//CAN-------------------------------------CAN_1 Variables and preparation BEGIN  0x601
	uint8_t CAN_TxData[8] = {0x23, 0x81, 0x60, 0x00, 0x55, 0x55, 0x08, 0x00};
	

	//CAN-------------------------------------CAN_1 Variables and preparation END
	
	//UART1-------------------------------------UART_1 Variables and preparation BEGIN  SWD disconnected---must
	
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

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	//HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);  //reset power up the IMU330(runing master first, and then start IMU. must for 330)
	HAL_Delay(500);
	
	//printf("restart IMU finished!\n");	
	
	gyro_raw_t data_g;
	acc_raw_t data_a;
	uint8_t buf_test[2] = {0};
	
	//init the memory buffer to store the SPI data
	imu_size_t *p_imu_buffer= NULL;
	imu_size_t *p_imu_head= NULL;
	p_imu_buffer=((imu_size_t*)malloc(MEM_SIZE*sizeof(imu_size_t)));//buffer head address
	p_imu_head=p_imu_buffer;
	g_count_mlbuffer = 0;
  
	
//	while(1)
//	{
//	  check_data = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1);
//	
//	  check_data = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1);
//	
//	  check_data++;
//	}
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_Delay(10);
		
#if defined(SIGNAL_READ) // signal read
		gyro_readxyz_raw(&data_g);
		acc_readxyz_raw(&data_a);
		printf("%d, %d\n", data_g.z,data_a.z);
#endif

		//register_init();

    //signal register function test		
#if 0
		readout_all_register();
		HAL_Delay(10);
		
		buf_test[0] = 0x00;
    buf_test[1] = 0x00;		
		single_spi_read(0x36,buf_test);
		HAL_Delay_us(100); //mandatory delay for single read
		single_spi_write(0x37,0x02);
		HAL_Delay_us(100);
		buf_test[0] = 0x00;
    buf_test[1] = 0x00;		
		single_spi_read(0x36,buf_test);
		HAL_Delay_us(100);
#endif

    /* USER CODE BEGIN 3 */
#if 0
    buf_test[0] = 0x00;
    buf_test[1] = 0x00;		
		single_spi_read(0x70,buf_test);
		HAL_Delay_us(100); //mandatory delay for single read
		single_spi_write(0x70,0x10);
		HAL_Delay_us(100);
		
		buf_test[0] = 0x00;
    buf_test[1] = 0x00;		
		single_spi_read(0x70,buf_test);
		HAL_Delay_us(100); //mandatory delay for single read
		
		buf_test[0] = 0x00;
    buf_test[1] = 0x00;		
		single_spi_read(0x46,buf_test);
		HAL_Delay_us(100); 
		
		
		//second try
		buf_test[0] = 0x00;
    buf_test[1] = 0x00;		
		single_spi_read(0x71,buf_test);
		HAL_Delay_us(100); //mandatory delay for single read
		single_spi_write(0x71,0x20);
		HAL_Delay_us(100);
		buf_test[0] = 0x00;
    buf_test[1] = 0x00;		
		single_spi_read(0x71,buf_test);
		HAL_Delay_us(100); //mandatory delay for single read
		
		
		buf_test[0] = 0x00;
    buf_test[1] = 0x00;		
		single_spi_read(0x47,buf_test);
		HAL_Delay_us(100);
		
#endif

#if 0
    //store the SPI data in buffer
		if(g_count_mlbuffer <= MEM_SIZE)
		{
			  g_count_mlbuffer++;
		    HAL_SPI_T_R_Burst(SPI_Burst_data, SPI_RXdata_burst[0], 10, 200);  //only support 330,300
		    data_g.z = (int16_t)((( SPI_RXdata_burst[3][0] ) << 8) + SPI_RXdata_burst[3][1])/6.4; //0.1 dps
		    data_a.z = (int16_t)((( SPI_RXdata_burst[6][0] ) << 8) + SPI_RXdata_burst[6][1])/4; //mg
			  p_imu_head->imu_accdata.z=data_a.z;
			  p_imu_head->imu_gyrodata.z=data_g.z;
			  p_imu_head++;
			  if(MEM_SIZE == g_count_mlbuffer)
				{
				    p_imu_head=p_imu_buffer;
				}

		}//output the SPI data via uart interface 
		else
		{
			 //uart output all the SPI data in buffer
			 while(g_count_mlbuffer != 0)
			 {
			     g_count_mlbuffer--;
				   data_g.z=p_imu_head->imu_gyrodata.z;
				   data_a.z=p_imu_head->imu_accdata.z;
				   p_imu_head++;
				   printf("IMU %d, %d\n", data_g.z,data_a.z);
			 }
       
			 p_imu_head=p_imu_buffer; //init the buffer point
			 memset(p_imu_head,0,sizeof(MEM_SIZE*sizeof(imu_size_t)));
		
		}
#endif



#if 1 //burst read
		HAL_SPI_T_R_Burst(SPI_Burst_data, SPI_RXdata_burst[0], 10, 200);  //only support 330,300
		
		data_g.z = (int16_t)((( SPI_RXdata_burst[3][0] ) << 8) + SPI_RXdata_burst[3][1])/6.4; //0.1 dps
		data_a.z = (int16_t)((( SPI_RXdata_burst[6][0] ) << 8) + SPI_RXdata_burst[6][1])/4; //mg
		printf("IMU %d, %d\n", data_g.z,data_a.z);
		//HAL_SPI_T_R_Burst(SPI_Burst_data, SPI_RXdata_burst[0], 10, 200);  //only support 330,300
#endif		
  }
  /* USER CODE END 3 */
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
