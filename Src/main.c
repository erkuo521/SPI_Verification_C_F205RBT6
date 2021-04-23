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
	PA2----------------nRST
	PC3----------------DRDY
	 
	UART1:  921600 bps
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


//#define IMU828_M
#define OPENIMU330_M  //330, CS always low style as IMU828
//#define IMU383_M  //383, CS trigger style as 381/380
//#define IMU331_M


#if defined(OPENIMU330_M)
  #define GYRO_SCALE       (64.0)  // dps
  #define ACC_SCALE        (4000.0) //g
#elif defined(IMU828_M)
  #define GYRO_SCALE       (16.0)  // dps 2000dps based on 500dps 
  #define ACC_SCALE        (1024.0) //g  32G based on 8g 
#elif defined(IMU383_M)
  #define GYRO_SCALE       (200.0)  // dps
  #define ACC_SCALE        (4000.0) //g
#endif


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


//IMU828 define special
#define LPF_USER_GRYO   (0x38)
#define LPF_USER_ACC    (0x3B)
#define LPF_HW_GYRO     (0x77)
#define LPF_HW_ACC      (0x78)


/* USER CODE BEGIN PD */
//----------------------------------------------------------------------------SPI self function declaration
void HAL_Delay_us(uint16_t num);
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

int fputc(int ch, FILE *f);

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);


/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;

uint8_t test2 = 0;
uint8_t g_trg = 0;
uint32_t g_count = 0;

void test_timer(void)
{
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12); // only reverse the voltage level
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);	//SET A4 low	
	HAL_Delay_us(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);	//SET A4 high
	HAL_Delay_us(10);
}



/* USER CODE BEGIN 4 */
/**
  * @brief delya us by for loop times, clc by Yifan
  * @param None
  * @retval None
  */
void HAL_Delay_us(uint16_t num)
{
	for(uint16_t i=0; i<num; i++) 
  { 
			for(uint16_t j=0; j<24; j++)
			{
			}
	}
}

//不准！！
void HAL_Delay_ms(uint16_t num)
{
	for(uint16_t i=0; i<num; i++) 
  { 
		/*1 ms */
		for(uint16_t i=0; i<1000; i++) 
		{ 	
			HAL_Delay_us(1);	
		}	
	}
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





void read_all_register_default_value_IMU330BI(void)
{
		uint8_t buf_reg[2]={0};
		uint8_t reg_table[10] = {0};
		
	  printf("read all 330 register start! \n");
		
    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x37,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "DRDY, reg:0x37", buf_reg[0], buf_reg[1]);

		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Accel LPF, reg:0x38", buf_reg[0], buf_reg[1]);

		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x46,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Acc Scale, reg:0x46", buf_reg[0], buf_reg[1]);

		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x47,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Rate Scale, reg:0x47", buf_reg[0], buf_reg[1]);

		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x70,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Acc Range, reg:0x70", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x71,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Rate Range, reg:0x71", buf_reg[0], buf_reg[1]);

		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x52,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "SN1, reg:0x52", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x54,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "SN2, reg:0x54", buf_reg[0], buf_reg[1]);
		
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x56,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "PRD_ID, reg:0x56", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x58,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "SN4, reg:0x58", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x5A,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Master Status, reg:0x5A", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x5C,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Hw Status, reg:0x5C", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x5E,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "SW Status, reg:0x5E", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x74,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Orientation, reg:0x74", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Rate LPF, reg:0x40", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x7F,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "SW version, reg:0x7F", buf_reg[0], buf_reg[1]);
		
		printf("read all 330 register end! \n");
		
		printf("\n");
		printf("\n");
		printf("\n");
}


void change_ODR_Test_1PPS_IMU330(void)
{
		uint8_t buf_reg[2]={0};
		uint8_t reg_table[10] = {0};
		
		printf("Change ODR!!! and start test pps \n");
		
    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x37,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "read ODR default, reg:0x37", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x37,0x02); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x37,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x02 return, reg:0x37", buf_reg[0], buf_reg[1]);
	}


void lpf_set_check_IMU330(void)
{
		uint8_t buf_reg[2]={0};
		uint8_t reg_table[10] = {0};
		
	  printf("330 user lpf setting check started!!!!!!!!!!! \n");
	
		/*      user gyro lpf       */
		printf("330 user gyro lpf setting check started!!! \n");
		
    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "default, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x00); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x03); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x03 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x04); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x04 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x38,0x05); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x05 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x06); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x06 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x30); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x30 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x40); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x40 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x50); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x50 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x60); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x60 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x90); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x90 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x38,0x80); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x80 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x40); //
		HAL_Delay_us(100);
		
		printf("330 user gyro lpf setting check end and restore default!!! \n");
		
		
		/****************************************************/
		/****************************************************/
		/****************************************************/
		/****************************************************/
		
		/*      user acc lpf       */
		printf("330 user acc lpf setting check started!!! \n");
		
    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "default, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x00); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x03); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x03 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x04); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x04 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x05); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x05 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x06); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x06 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x30); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x30 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x40); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x40 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x50); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x50 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x60); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x60 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x90); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x90 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x80); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x80 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x40); //
		HAL_Delay_us(100);
		
		printf("330 user acc lpf setting check end and restore default!!! \n");		
}



void read_output_rate_value_IMU331(void)
{
		uint8_t buf_reg[2]={0};
		uint8_t reg_table[10] = {0};
		
	  printf("read all IMU331 register start! \n");
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x37,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "OUTPUT_DATA_RATE, reg:0x37", buf_reg[0], buf_reg[1]);	
}


void read_all_register_default_value_IMU331(void)
{
		uint8_t buf_reg[2]={0};
		uint8_t reg_table[10] = {0};
		
	  printf("read all IMU331 register start! \n");
	
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x37,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "OUTPUT_DATA_RATE, reg:0x37", buf_reg[0], buf_reg[1]);

		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "RATE_SENSOR_LOW_PASS_FILTER, reg:0x38", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "ACCELEROMETER_LOW_PASS_FILTER, reg:0x3B", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3C,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "DIAGNOSTIC_STATUS, reg:0x3C", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x56,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "PRODUCT_ID(High), reg:0x56", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x57,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "PRODUCT_ID(Low), reg:0x57", buf_reg[0], buf_reg[1]);

    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x60,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "SERIAL_NUMBER1, reg:0x60", buf_reg[0], buf_reg[1]);

    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x61,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "SERIAL_NUMBER2, reg:0x61", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x62,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "SERIAL_NUMBER3, reg:0x62", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x63,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "SERIAL_NUMBER4, reg:0x63", buf_reg[0], buf_reg[1]); //the first of SN number

    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x64,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "FW_VERSION1, reg:0x64", buf_reg[0], buf_reg[1]);

    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x65,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "FW_VERSION2, reg:0x65", buf_reg[0], buf_reg[1]);

    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x66,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "FW_VERSION3, reg:0x66", buf_reg[0], buf_reg[1]); //the first of FW number


		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x70,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "AVERAGE_CONTROL, reg:0x70", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x74,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "ORIENTATION_MSB, reg:0x74", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x7E,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Hardware_version, reg:0x7E", buf_reg[0], buf_reg[1]);
		
		printf("read all IMU331 register end! \n");
		
		printf("\n");
		printf("\n");
		printf("\n");
}


void lpf_set_check_IMU331(void)
{
		uint8_t buf_reg[2]={0};
		uint8_t reg_table[10] = {0};
		
	  printf("331 user lpf setting check started!!!!!!!!!!! \n");
	
		/*      user gyro lpf       */
		printf("331 user gyro lpf setting check started!!! \n");
		
    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "default, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x00); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x10); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x10 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x38,0x60); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x60 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x50); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x50 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x38,0x40); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x40 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x38,0x80); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x80 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x38,0x90); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x90 return, reg:0x38", buf_reg[0], buf_reg[1]);

    single_spi_write(0x38,0x30); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x30 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x38,0x20); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x20 return, reg:0x38", buf_reg[0], buf_reg[1]);


    single_spi_write(0x38,0xA0); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0xA0 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x38,0xB0); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0xB0 return, reg:0x38", buf_reg[0], buf_reg[1]);

		
		single_spi_write(0x38,0x80); // restore default value
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x80 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		printf("331 user gyro lpf setting check end and restore default!!! \n");
		
		
		/****************************************************/
		/****************************************************/
		/****************************************************/
		/****************************************************/
		
				/*      user acc lpf       */
		printf("331 user acc lpf setting check started!!! \n");
		
    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "default, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x3B,0x00); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x3B,0x10); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x10 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x3B,0x60); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x60 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x3B,0x50); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x50 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x3B,0x40); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x40 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x3B,0x80); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x80 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x3B,0x90); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x90 return, reg:0x38", buf_reg[0], buf_reg[1]);

    single_spi_write(0x3B,0x30); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x30 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x3B,0x20); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x20 return, reg:0x38", buf_reg[0], buf_reg[1]);


    single_spi_write(0x3B,0xA0); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0xA0 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x3B,0xB0); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0xB0 return, reg:0x38", buf_reg[0], buf_reg[1]);


		single_spi_write(0x3B,0x80); // restore default value
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x80 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
	  
		printf("331 user acc lpf setting check end and restore default!!! \n");		
}


void change_output_rate_imu331(void)
{
	  uint8_t buf_reg[2]={0};
		uint8_t reg_table[10] = {0};

    single_spi_write(0x37,0x0A); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x37,buf_reg); //
		HAL_Delay_us(100);
		//printf("%s MSB:0x%x, LSB:0x%x\n", "output data rate test reg:0x37", buf_reg[0], buf_reg[1]);

		
		single_spi_write(0x76,0x37); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x37,buf_reg); //
		HAL_Delay_us(100);
		//printf("%s MSB:0x%x, LSB:0x%x\n", "output data rate test reg:0x37", buf_reg[0], buf_reg[1]);

}


void read_all_register_default_value_IMU828(void)
{
		uint8_t buf_reg[2]={0};
		uint8_t reg_table[10] = {0};
		
	  printf("read all 828 register start! \n");

    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x01,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Fault_detection_enable, reg:0x01", buf_reg[0], buf_reg[1]);

		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x02,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Fault_detection_disable, reg:0x02", buf_reg[0], buf_reg[1]);

		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x03,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Fault_detect_key, reg:0x03", buf_reg[0], buf_reg[1]);

		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x1A,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Sensor_control_chip1, reg:0x1A", buf_reg[0], buf_reg[1]);

		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x1B,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Sensor_control_chip2, reg:0x1B", buf_reg[0], buf_reg[1]);
			
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x1D,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Sensors_status_chip1, reg:0x1D", buf_reg[0], buf_reg[1]);
	
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x1E,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Sensors_status_chip2, reg:0x1E", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x35,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "self-test, reg:0x35", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x36,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "reserved!!, reg:0x36", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x37,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "OUTPUT_DATA_RATE, reg:0x37", buf_reg[0], buf_reg[1]);

		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "RATE_SENSOR_LOW_PASS_FILTER, reg:0x38", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "ACCELEROMETER_LOW_PASS_FILTER, reg:0x3B", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3C,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "DIAGNOSTIC_STATUS, reg:0x3C", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x56,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "PRODUCT_ID, reg:0x56", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x57,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "PRODUCT_ID, reg:0x57", buf_reg[0], buf_reg[1]);

    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x60,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "SERIAL_NUMBER1, reg:0x60", buf_reg[0], buf_reg[1]);

    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x61,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "SERIAL_NUMBER2, reg:0x61", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x62,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "SERIAL_NUMBER3, reg:0x62", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x63,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "SERIAL_NUMBER4, reg:0x63", buf_reg[0], buf_reg[1]);

    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x64,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "FW_VERSION1, reg:0x64", buf_reg[0], buf_reg[1]);

    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x65,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "FW_VERSION2, reg:0x65", buf_reg[0], buf_reg[1]);

    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x66,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "FW_VERSION3, reg:0x66", buf_reg[0], buf_reg[1]);


		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x70,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "AVERAGE_CONTROL, reg:0x70", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x74,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "ORIENTATION_MSB, reg:0x74", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x77,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro HW filter cutoff frequency, reg:0x77", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc HW filter cutoff frequency, reg:0x78", buf_reg[0], buf_reg[1]);
		
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x7E,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Hardware_version, reg:0x7E", buf_reg[0], buf_reg[1]);
		
		printf("read all register end! \n");
		
		printf("\n");
		printf("\n");
		printf("\n");
}



void self_test_IMU828(void)
{
		uint8_t buf_reg[2]={0};
		uint8_t reg_table[10] = {0};
		
	  printf("828 self-test start! \n");
	
    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x35,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "self-test, reg:0x35", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x35,0x04); //sensor 0 self test
		HAL_Delay_ms(2000); //waiting for the test finished
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3C,buf_reg); //check the sensor test result
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Diagnostic sensor 0, reg:0x3C", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x35,0x05); //sensor 1 self test
		HAL_Delay_ms(2000); //waiting for the test finished
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3C,buf_reg); //check the sensor test result
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Diagnostic sensor 1, reg:0x3C", buf_reg[0], buf_reg[1]);
		
		printf("828 self-test finished! \n");	
}


void lpf_set_check_IMU828(void)
{
		uint8_t buf_reg[2]={0};
		uint8_t reg_table[10] = {0};
		
	  printf("828 user lpf setting check started!!!!!!!!!!! \n");
	
		/*      user gyro lpf       */
		printf("828 user gyro lpf setting check started!!! \n");
		
    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "default, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x00); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x01); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 1 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x02); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 2 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x38,0x03); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 3 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x04); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 4 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x05); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 5 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x06); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 6 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x07); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 7 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x08); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 8 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x09); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 9 return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x38,0x0A); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x38,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro lpf write 0x0A return, reg:0x38", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x38,0x06); //
		HAL_Delay_us(100);
		
		printf("828 user gyro lpf setting check end and restore default!!! \n");
		
		
		/*      user acc lpf       */
		printf("828 user acc lpf setting check started!!! \n");
		
    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "default, reg:0x3B", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x3B,0x00); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0 return, reg:0x3B", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x3B,0x01); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 1 return, reg:0x3B", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x3B,0x02); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 2 return, reg:0x3B", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x3B,0x03); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 3 return, reg:0x3B", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x3B,0x04); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 4 return, reg:0x3B", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x3B,0x05); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 5 return, reg:0x3B", buf_reg[0], buf_reg[1]);
		
		
		single_spi_write(0x3B,0x06); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 6 return, reg:0x3B", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x3B,0x07); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 7 return, reg:0x3B", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x3B,0x08); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 8 return, reg:0x3B", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x3B,0x09); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 9 return, reg:0x3B", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x3B,0x0A); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x3B,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc lpf write 0x0A return, reg:0x3B", buf_reg[0], buf_reg[1]);
		
		
		
		single_spi_write(0x3B,0x04); //
		HAL_Delay_us(100);
		
		printf("828 user acc lpf setting check end and restore default!!! \n");
		
		
		/*      HW gyro lpf       */
		printf("828 HW gyro lpf setting check started!!! \n");
		
    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x77,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "default, reg:0x77", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x77,0x00); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x77,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro HW lpf write 0 return, reg:0x77", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x77,0x01); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x77,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro HW lpf write 1 return, reg:0x77", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x77,0x02); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x77,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro HW lpf write 2 return, reg:0x77", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x77,0x03); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x77,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro HW lpf write 3 return, reg:0x77", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x77,0x04); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x77,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro HW lpf write 4 return, reg:0x77", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x77,0x05); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x77,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro HW lpf write 5 return, reg:0x77", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x77,0x06); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x77,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro HW lpf write 6 return, reg:0x77", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x77,0x07); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x77,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro HW lpf write 7 return, reg:0x77", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x77,0x08); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x77,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "gyro HW lpf write 8 return, reg:0x77", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x77,0x03); //
		HAL_Delay_us(100);
		
		printf("828 HW gyro lpf setting check end and restore default!!! \n");
		
		/*      HW acc lpf       */
		printf("828 HW acc lpf setting check started!!! \n");
		
    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "default, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x00); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc HW lpf write 0 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x01); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc HW lpf write 1 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x02); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc HW lpf write 2 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x03); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc HW lpf write 3 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x04); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc HW lpf write 4 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x05); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc HW lpf write 5 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x06); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x78,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "acc HW lpf write 6 return, reg:0x78", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x78,0x04); //
		HAL_Delay_us(100);
		
		printf("828 HW acc lpf setting check end and restore default!!! \n");
		
		printf("828 self-test finished!!!!!!!!!!!!!!!!!!!!!!! \n");	
}


void unit_write_read_register_test(void)
{
		uint8_t buf_reg[2]={0};
		uint8_t reg_table[10] = {0};
		
	  printf("read all register start! \n");
		printf("read all register start! \n");
		
    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x01,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "Fault detection enable, reg:0x01", buf_reg[0], buf_reg[1]);

}


/* single read, polling mode */
void acc_readxyz_raw(acc_raw_t *rawdata_out_p)
{
    uint8_t buf_x[2] = {0};
    uint8_t buf_y[2] = {0};
    uint8_t buf_z[2] = {0};
		
    buf_x[0] = REG_ACC_X;
    buf_x[1] = 0x00;		
		single_spi_read(REG_ACC_X,buf_x);
		HAL_Delay_us(15);

    buf_y[0] = REG_ACC_Y;
    buf_y[1] = 0x00;
    single_spi_read(REG_ACC_Y,buf_y);
		HAL_Delay_us(15);
				
    buf_z[0] = REG_ACC_Z;
    buf_z[1] = 0x00;
		single_spi_read(REG_ACC_Z,buf_z);
    HAL_Delay_us(15);
		
		/* output acc: mg*/
    rawdata_out_p->x = (int16_t)((buf_x[0] << 8) + buf_x[1])/1.024;
    rawdata_out_p->y = (int16_t)((buf_y[0] << 8) + buf_y[1])/1.024;
    rawdata_out_p->z = (int16_t)((buf_z[0] << 8) + buf_z[1])/1.024;
}


/* single read, polling mode */
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
    rawdata_out_p->x = (int16_t)((buf_x[0] << 8) + buf_x[1])/1.6;
    rawdata_out_p->y = (int16_t)((buf_y[0] << 8) + buf_y[1])/1.6;
    rawdata_out_p->z = (int16_t)((buf_z[0] << 8) + buf_z[1])/1.6;
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
  huart1.Init.BaudRate = 921600; //460800;
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

  /*Configure GPIO pin : openimu_irq_Pin(PC3) */
  GPIO_InitStruct.Pin = openimu_irq_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(openimu_irq_GPIO_Port, &GPIO_InitStruct);

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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
  * @brief send and receive the data, burst read£¨8bits mode
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
			//HAL_Delay_us(20); //should use 20us delay!!
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
 
    vApp_CAN_TxHeader_Init(pTxHeader, StdId, ExtId, IDE, RTR, DLC);
    
    vApp_CAN_Filter_Init(pFilter, IdHigh, IdLow, MaskIdHigh, MaskIdLow, FIFOAssignment, Bank, Mode, Scale, Activation, SlaveStartFilterBank);
    HAL_CAN_ConfigFilter(&hcan1, pFilter);
    
    while(HAL_CAN_Start(&hcan1) != HAL_OK )
    {        
        HAL_Delay(100);
    }    
		
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
    
    pTxHeader->DLC    =    DLC;
    
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


/* External Interrupt Mode with Falling edge trigger detection, default Pull-UP status. UI setting */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#if defined(IMU828_M)
    g_count++;
	  g_trg = 1;
	
#if 0	
	  if(10 == g_count)
		{
		  g_trg = 1;
      g_count = 0;			
		}
#endif
	
#endif
	
#if defined(OPENIMU330_M)	
		g_trg = 1;
//	  g_count++;
//	  if(2 == g_count)
//		{
//		  g_trg = 1;
//      g_count = 0;			
//		}
#endif
		
		
}


//IMU828 read data signal reg
void IMU828_signal_read(void)
{
   acc_raw_t acc_struct_data;
	 acc_struct_data.x = 0;
	 acc_struct_data.y = 0;
	 acc_struct_data.z = 0;
	 gyro_raw_t gyro_struct_data;
	 gyro_struct_data.x = 0;
	 gyro_struct_data.y = 0;
	 gyro_struct_data.z = 0;
	
	 acc_readxyz_raw(&acc_struct_data);
	 gyro_readxyz_raw(&gyro_struct_data);
	
   printf("%f,%f,%f\n", (float)acc_struct_data.x,(float)acc_struct_data.y,(float)acc_struct_data.z);
	 //printf("%f,%f,%f\n", (float)gyro_struct_data.x,(float)gyro_struct_data.y,(float)gyro_struct_data.z);

}


void IMU331_change_ODR(void)
{
		uint8_t buf_reg[2]={0};
		uint8_t reg_table[10] = {0};
		
		/*  user gyro lpf */
		//printf("828 user gyro lpf setting check started!!! \n");
		
    buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x36,buf_reg);
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "reg read:0x36", buf_reg[0], buf_reg[1]);
		
		single_spi_write(0x37,0x02); //
		HAL_Delay_us(100);
		buf_reg[0] = 0;
    buf_reg[1] = 0;		
		single_spi_read(0x36,buf_reg); //
		HAL_Delay_us(100);
		printf("%s MSB:0x%x, LSB:0x%x\n", "reg:0x36 read after write", buf_reg[0], buf_reg[1]);
}


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
	uint8_t SPI_Burst_data[2] = {0x3E, 0x00};	 //3E is 8 words, 3F is 10 words, pls remember to change other codes on 2 areas
	uint8_t SPI_RXdata_burst[11][2] = {0X00};
	
	uint8_t buf_reg_test[2]={0};
	uint8_t reg_table_test[10] = {0};

	//CAN-------------------------------------CAN_1 Variables and preparation BEGIN  0x601
	uint8_t CAN_TxData[8] = {0x23, 0x81, 0x60, 0x00, 0x55, 0x55, 0x08, 0x00};
	//CAN-------------------------------------CAN_1 Variables and preparation END
	
	uint8_t UART_RxData='F';
	uint8_t check_data = 0;
	gyro_raw_t data_g;
	acc_raw_t data_a;
	uint8_t buf_test[2] = {0};
	
	//init the memory buffer to store the SPI data
	imu_size_t *p_imu_buffer= NULL;
	imu_size_t *p_imu_head= NULL;
	p_imu_buffer=((imu_size_t*)malloc(MEM_SIZE*sizeof(imu_size_t)));//buffer head address
	p_imu_head=p_imu_buffer;
	g_count_mlbuffer = 0;
	uint8_t test1 = 0;
	
	float a_x = 0.0;
	float a_y = 0.0;
	float a_z = 0.0;
	
	float g_x = 0.0;
	float g_y = 0.0;
	float g_z = 0.0;
	
	uint8_t buf_x[2] = {0};
  uint8_t buf_y[2] = {0};
  uint8_t buf_z[2] = {0};
	
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	//----------------------------------------SPI Variables and preparation BEGIN
	
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
	vApp_User_CAN_Configuration();
 
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  //A4 back to high      1. keep ground together with Master and Slave ---must
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); //rest Pin high/low/high
	HAL_Delay(900);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);  //reset power up the IMU330(runing master first, and then start IMU. must for 330)
	HAL_Delay(500);
	
//	while(1)
//	{
//	    //read_all_register_default_value_IMU331();
//		  HAL_Delay(500);
//		  //lpf_set_check_IMU331();
//		  //change_output_rate_imu331();
//		  HAL_Delay(100);
//		  read_output_rate_value_IMU331();
//	}

//   while(1)
//	 {
//	     IMU828_signal_read();
//		   HAL_Delay(10);
//	 
//	 }
//  
	
#if 0
	while(1)
	{
		#if defined(OPENIMU330_M) 
		  printf("330 SPI testing! \n");
		  read_all_register_default_value_IMU330BI();
		  //lpf_set_check_IMU330();
		#elif defined(IMU828_M)
		  printf("\n");
		  printf("\n");
		  printf("\n");
		  printf("\n");
		  printf("\n");
		  printf("\n");
		  printf("\n");
		  printf("IMU828 big beta test start! \n");
		  printf("\n");
			
	
	  read_all_register_default_value_IMU828();
		  self_test_IMU828();
		  lpf_set_check_IMU828();
	
		  printf("\n");
			printf("IMU828 big beta test end! \n");
			printf("\n");
		  printf("\n");
		  printf("\n");
		  printf("\n");
		  printf("\n");
		  printf("\n");
		  printf("\n");
		
		#elif defined(IMU383_M)
		  read_all_register_default_value_IMU383();
		#endif
		
	}
#endif


	
#if defined(IMU383_M)
	while(1)
	{
	  uint8_t buf_x[2] = {0};
    uint8_t buf_y[2] = {0};
    uint8_t buf_z[2] = {0};
		int16_t data_x = 0;
		int16_t data_y = 0;
		int16_t data_z = 0;
		
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
    data_x = (int16_t)((buf_x[0] << 8) + buf_x[1])/4000;
    data_y = (int16_t)((buf_y[0] << 8) + buf_y[1])/4000;
    data_z = (int16_t)((buf_z[0] << 8) + buf_z[1])/4000;
		HAL_Delay_ms(10);
		
		printf("%d,%d,%d\n", data_x,data_y,data_z);
	}
#endif
	
	//configuration
	//change_ODR_Test_1PPS_IMU330();
	//change_output_rate_imu331();
	

//  while(1)
//	{
//		
//		uint8_t buf_x[2] = {0};
//    uint8_t buf_y[2] = {0};
//    uint8_t buf_z[2] = {0};
//		int16_t data_x = 0;
//		int16_t data_y = 0;
//		int16_t data_z = 0;
//		
//    buf_x[0] = REG_ACC_X;
//    buf_x[1] = 0x00;		
//		single_spi_read(REG_ACC_X,buf_x);
//		HAL_Delay_us(100);
//	  
//	  
//		printf("%f\n",a_z);
//		
//	
//	}


  while(1)
	{
		a_x = 0.0;
		a_y = 0.0;
		a_z = 0.0;
		g_x = 0.0;
		g_y = 0.0;
		g_z = 0.0;
		
		//IMU331_change_ODR();
		
		//change_output_rate_imu331();
		//HAL_Delay(10);
		//lpf_set_check_IMU828();
		//read_all_register_default_value_IMU828();
		//HAL_Delay_us(500);
		if(1 == g_trg)
		{
#ifdef IMU331_M			
			HAL_Delay_us(80); // IMU331 need this delay when implement the 400Hz ODR in drdy mode.
#endif
			g_trg = 0;	
			HAL_SPI_T_R_Burst(SPI_Burst_data, SPI_RXdata_burst[0], 10, 200);  
			
			a_x = ((int16_t)((( SPI_RXdata_burst[4][0] ) << 8) + SPI_RXdata_burst[4][1]))/ACC_SCALE; //g
		  a_y = ((int16_t)((( SPI_RXdata_burst[5][0] ) << 8) + SPI_RXdata_burst[5][1]))/ACC_SCALE; //g
		  a_z = ((int16_t)((( SPI_RXdata_burst[6][0] ) << 8) + SPI_RXdata_burst[6][1]))/ACC_SCALE; //g
		  g_x = ((int16_t)((( SPI_RXdata_burst[1][0] ) << 8) + SPI_RXdata_burst[1][1]))/GYRO_SCALE; //dps
		  g_y = ((int16_t)((( SPI_RXdata_burst[2][0] ) << 8) + SPI_RXdata_burst[2][1]))/GYRO_SCALE; //dps
		  g_z = ((int16_t)((( SPI_RXdata_burst[3][0] ) << 8) + SPI_RXdata_burst[3][1]))/GYRO_SCALE; //dps

		  //printf("%f,%f,%f,%f,%f,%f\n", a_x,a_y,a_z,g_x,g_y,g_z);
			printf("%f,%f,%f\n", a_x,a_y,a_z);
			//printf("%f,%f,%f\n", g_x,g_y,g_z);
			//printf("%f\n",a_z);
		}	
	}
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
