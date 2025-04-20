/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include "usart.h"
#include <string.h>
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
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
	CAN_Filter_Config();
	HAL_CAN_Start(&hcan1);
	
  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

//过滤器函数
void CAN_Filter_Config(){
	CAN_FilterTypeDef sFilterConfig;
	
	sFilterConfig.FilterBank = 0;						//筛选器编号, CAN1是0-13, CAN2是14-27
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;	//采用掩码模式
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;	//设置筛选器的尺度, 采用32位
	sFilterConfig.FilterIdHigh = 0X0000;				//过滤器ID高16位,即CAN_FxR1寄存器的高16位
	sFilterConfig.FilterIdLow = 0X0000;					//过滤器ID低16位,即CAN_FxR1寄存器的低16位
	sFilterConfig.FilterMaskIdHigh = 0X0000;			//过滤器掩码高16位,即CAN_FxR2寄存器的高16位
	sFilterConfig.FilterMaskIdLow = 0X0000;				//过滤器掩码低16位,即CAN_FxR2寄存器的低16位
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;	//设置经过筛选后数据存储到哪个接收FIFO
	sFilterConfig.FilterActivation = ENABLE;			//是否使能本筛选器
	sFilterConfig.SlaveStartFilterBank = 14;			//指定为CAN1分配多少个滤波器组
	
	if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();	
	}
}

//发送函数
uint8_t CAN1_Send_Msg(uint8_t *msg, uint8_t len)
{
	uint16_t i = 0;
	uint32_t txMailBox;
	uint8_t send_buf[8];
	
	CAN_TxHeaderTypeDef	txHeader;
	
	txHeader.StdId = 0x7DF;
	txHeader.ExtId = 0x00;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = len;
	
	for(i = 0; i < len; i++)
		send_buf[i] = msg[i];
	
	if(HAL_CAN_AddTxMessage(&hcan1, &txHeader, send_buf, &txMailBox) != HAL_OK)
	{
		printf("CAN Send failed!\r\n");
		return 1;
	}
	printf("CAN Send success!\r\n");		
	return 0;
}



//单帧接收函数
uint8_t CAN1_Recv_Msg(uint8_t *buf)
{
	uint16_t i = 0;
	CAN_RxHeaderTypeDef	rxHeader;	
	
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, buf);	
	
	if(rxHeader.IDE == CAN_ID_STD)
		printf("StdId ID: %02x\r\n", rxHeader.StdId);
	else
		printf("ExtId ID: %02x\r\n", rxHeader.ExtId);
	
	printf("CAN IDE: %d\r\n", rxHeader.IDE);
	printf("CAN RTR: %d\r\n", rxHeader.RTR);
	printf("CAN DLC: %d\r\n", rxHeader.DLC);
	printf("Recv Data: ");
	
	for(i = 0; i < rxHeader.DLC; i++)
		printf("%02x  ",buf[i]);
	
	printf("\r\n");
	return rxHeader.DLC;
}


//故障码接收函数
uint8_t CAN1_Recv_Msg_2(uint8_t *buf)
{
		uint16_t i = 0;	
		uint8_t rece_buffer[8];
		uint8_t rece_length;
	
		CAN_RxHeaderTypeDef	rxHeader;
		
		//先接收第一帧进行分析
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rece_buffer);
	
		if(rece_buffer[0]==0x10)
		{
			printf("检测到多帧数据！！！\r\n");
			rece_length=rece_buffer[1];
		}	
		
		if(rxHeader.IDE == CAN_ID_STD)
			printf("StdId ID: %02x\r\n", rxHeader.StdId);
		else
			printf("ExtId ID: %02x\r\n", rxHeader.ExtId);
		
		printf("CAN IDE: %d\r\n", rxHeader.IDE);
		printf("CAN RTR: %d\r\n", rxHeader.RTR);
		printf("CAN DLC: %d\r\n", rxHeader.DLC);
		printf("Recv Data: ");
		
		for(i = 0; i < rxHeader.DLC; i++)
			printf("%02x ",buf[i]);
		
		printf("\r\n\r\n");
		return rxHeader.DLC;
}


//中断接收函数
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//		uint16_t i = 0;
//		uint8_t buf[8] = {0};
//		CAN_RxHeaderTypeDef	rxHeader;
//		
//		if(hcan->Instance == CAN1)
//		{
//				printf("*******************************\r\n");
//				printf("Recv via STM32F429 Interrupt\r\n");
//			
//				HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, buf);
//			
//				if(rxHeader.IDE == CAN_ID_STD)
//					printf("StdId ID: %d\n", rxHeader.StdId);
//				else
//					printf("ExtId ID: %d\n", rxHeader.ExtId);
//				printf("\r\n");
//				printf("CAN IDE: %d\n", rxHeader.IDE);printf("\r\n");
//				printf("CAN RTR: %d\n", rxHeader.RTR);printf("\r\n");
//				printf("CAN DLC: %d\n", rxHeader.DLC);printf("\r\n");
//				printf("Recv Data: ");
//			
//				for(i = 0; i < rxHeader.DLC; i++)
//					printf("%c ",buf[i]);
//			
//				printf("\r\n");
//				printf("*******************************\r\n");
//		}
//}





/* USER CODE END 1 */
