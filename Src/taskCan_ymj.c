
#include "stm32f4xx_hal.h"
#include "osapi.h"
#include "osresources.h"
#include "taskCan.h"
#include "stdio.h"

extern UART_HandleTypeDef huart1;

CAN_HandleTypeDef CanHandle;

CAN_TxHeaderTypeDef CanTxHeader;
CAN_RxHeaderTypeDef CanRxHeader;

uint8_t CanTxData[8];
uint8_t CanRxData[8];
uint32_t TxMailbox;

/*
 * CAN IO init
 * CAN_120R_CTL   <==>    PC8
 * CAN_AB         <==>    PC9
 * CAN_RX         <==>    PA11
 * CAN_TX         <==>    PA12
 */
void can_config(int baudrate)
{
     /*##-0- CAN IO init    #######################################*/
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_Initure;
    GPIO_Initure.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_Initure);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

    /*##-1- Configure the CAN peripheral #######################################*/
    int prescaler = HAL_RCC_GetPCLK1Freq()/9/baudrate;

    CanHandle.Instance = CAN1;
    CanHandle.Init.TimeTriggeredMode = DISABLE;
    CanHandle.Init.AutoBusOff = DISABLE;
    CanHandle.Init.AutoWakeUp = DISABLE;
    CanHandle.Init.AutoRetransmission = ENABLE;
    CanHandle.Init.ReceiveFifoLocked = DISABLE;
    CanHandle.Init.TransmitFifoPriority = DISABLE;
    CanHandle.Init.Mode = CAN_MODE_NORMAL; //   CAN_MODE_LOOPBACK
    CanHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
    CanHandle.Init.TimeSeg1 = CAN_BS1_6TQ;
    CanHandle.Init.TimeSeg2 = CAN_BS2_2TQ;
    CanHandle.Init.Prescaler = 5;   // prescaler

    if (HAL_CAN_Init(&CanHandle) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /*##-2- Configure the CAN Filter ###########################################*/
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /*##-3- Start the CAN peripheral ###########################################*/
    if (HAL_CAN_Start(&CanHandle) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /*##-4- Activate CAN RX notification #######################################*/
    if (HAL_CAN_ActivateNotification(&CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
         _Error_Handler(__FILE__, __LINE__);
    }
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);   
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);         
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
    if (hcan->Instance == CAN1)
    {
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_Initure;
        GPIO_Initure.Pin = GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;
        GPIO_Initure.Pull = GPIO_PULLUP;
        GPIO_Initure.Speed = GPIO_SPEED_FAST;
        GPIO_Initure.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOA, &GPIO_Initure);
    }
}

uint8_t CAN_Send_Msg(void)
{
    CanTxHeader.StdId = 0x000000B4;
    //CanTxHeader.ExtId = 0x01;
    CanTxHeader.RTR = CAN_RTR_DATA;
    CanTxHeader.IDE = CAN_ID_STD;
    CanTxHeader.DLC = 8;
    CanTxHeader.TransmitGlobalTime = DISABLE;

    CanTxData[0] = 0x00;
    CanTxData[1] = 0x00;
    CanTxData[2] = 0x00;
    CanTxData[3] = 0x00;
    CanTxData[4] = 0x10;
    CanTxData[5] = 0x00;
    CanTxData[6] = 0x00;
    CanTxData[7] = 0xCC;

    /* Request transmission */
    if (HAL_CAN_AddTxMessage(&CanHandle, &CanTxHeader, CanTxData, &TxMailbox) != HAL_OK)
    {
        /* Transmission request Error */
        return 0;
    }

    return 1;
}

void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&CanHandle);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxHeader, CanRxData) != HAL_OK)
  {
    /* Reception Error */
  }

  if ((CanRxHeader.StdId == 0x11) && (CanRxHeader.IDE == CAN_ID_STD) && (CanRxHeader.DLC == 2))
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)CanRxData, 2, 5000);
  }
}

uint8_t CAN_Receive_Msg(void)
{
    uint32_t i;

    if (HAL_CAN_GetRxMessage(&CanHandle, CAN_RX_FIFO0, &CanRxHeader, CanRxData) != HAL_OK)
    {
        /* Reception Error */
        return 0;
    }

    if ((CanRxHeader.StdId != 0x11) ||
        (CanRxHeader.RTR != CAN_RTR_DATA) ||
        (CanRxHeader.IDE != CAN_ID_STD) ||
        (CanRxHeader.DLC != 2) ||
        ((CanRxData[0] << 8 | CanRxData[1]) != 0xCAFE))
    {
        /* Rx message Error */
        return 0;
    }

    //HAL_UART_Transmit(&huart1, (uint8_t *)&canTag, 1, 5000);

    return 1;
}

void CanTask(void const *argument)
{
    int baudRate;
    static int num = 0;

    //baudRate = configGetCANBaudRate(); 
    can_config(0);

    while (1)
    {
        num++;
        if (num == 50)
        {
            // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
        }
        else if (num == 100)
        {
            num = 0;
            // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // GPIO_PIN_8

            CAN_Send_Msg();
        }

        CAN_Receive_Msg();

        OS_Delay(10);
    }
}
