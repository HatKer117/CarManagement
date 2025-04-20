/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "base.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//发送数组
uint8_t txdata_CT[8]	= {0x02,0x01,0x05,0x00,0x00,0x00,0x00,0x00};//41	冷却液温度  				℃
uint8_t txdata_MAP[8] = {0x02,0x01,0x0B,0x00,0x00,0x00,0x00,0x00};//47	进气歧管绝对压力		kPa
uint8_t txdata_RPM[8] = {0x02,0x01,0x0C,0x00,0x00,0x00,0x00,0x00};//48	发动机转速	  			Rpm 	
uint8_t txdata_VS[8] 	= {0x02,0x01,0x0D,0x00,0x00,0x00,0x00,0x00};//49	车速  						km/h
uint8_t txdata_IAT[8] = {0x02,0x01,0x0F,0x00,0x00,0x00,0x00,0x00};//51	进气温度						℃
uint8_t txdata_OSD[8] = {0x02,0x01,0x14,0x00,0x00,0x00,0x00,0x00};//80	氧传感器1电压  		V
uint8_t txdata_FL[8] 	= {0x02,0x01,0x2F,0x00,0x00,0x00,0x00,0x00};//98 	燃油液位	 					%
uint8_t txdata_FH[8] 	= {0x02,0x01,0x04,0x00,0x00,0x00,0x00,0x00};//40	负荷计算值	 				%
uint8_t txdata_FRT[8] = {0x02,0x01,0x1F,0x00,0x00,0x00,0x00,0x00};//75 	发动机累计运行时间 s
uint8_t txdata_DTC[8] = {0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00};//无  故障码

//接收数组
uint8_t rxdata[8];
uint8_t rxdata_2[256];
float carInfo[9];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//串口重定向
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
 
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}

void getInfo()
{
	//获取冷却液温度
	CAN1_Send_Msg(txdata_CT, 8);
	CAN1_Recv_Msg(rxdata);
	carInfo[0]=rxdata[3]-40.0;
	
	//获取进气歧管绝对压力
	CAN1_Send_Msg(txdata_MAP, 8);
	CAN1_Recv_Msg(rxdata);
	carInfo[1]=rxdata[3];
	
	//获取发动机转速
	CAN1_Send_Msg(txdata_RPM, 8);
	CAN1_Recv_Msg(rxdata);	
	carInfo[2]=(rxdata[3]*256+rxdata[4])/4.0;
	
	//获取车速
	CAN1_Send_Msg(txdata_VS, 8);
	CAN1_Recv_Msg(rxdata);
	carInfo[3]=rxdata[3];
	
	//获取进气温度
	CAN1_Send_Msg(txdata_IAT, 8);
	CAN1_Recv_Msg(rxdata);	
	carInfo[4]=rxdata[3]-40.0;
	
	//获取氧传感器1电压
	CAN1_Send_Msg(txdata_OSD, 8);
	CAN1_Recv_Msg(rxdata);
	carInfo[5]=rxdata[3]*0.005;

	//获取燃油液位
	CAN1_Send_Msg(txdata_FL, 8);
	CAN1_Recv_Msg(rxdata);
	carInfo[6]=rxdata[3]*100.0/255.0;

	//获取负荷计算值
	CAN1_Send_Msg(txdata_FH, 8);
	CAN1_Recv_Msg(rxdata);
	carInfo[7]=rxdata[3]*100.0/255.0;

	//获取发动机累计运行时间
	CAN1_Send_Msg(txdata_FRT, 8);
	CAN1_Recv_Msg(rxdata);	
	carInfo[8]=rxdata[3]*256.0+rxdata[4];
	
//	//获取故障码
//	CAN1_Send_Msg(txdata_DTC, 8);
//	CAN1_Recv_Msg_2(rxdata_2);

}

void showInfo()
{
	printf("冷却液温度为%.0f度\r\n",carInfo[0]);
	printf("进气歧管绝对压力为%.0fkPa\r\n",carInfo[1]);
	printf("发动机转速为%.0fRpm\r\n",carInfo[2]);
	printf("车速为%.0fkm/h\r\n",carInfo[3]);
	printf("进气温度为%.0f度\r\n",carInfo[4]);
	printf("氧传感器1电压为%.2fV\r\n",carInfo[5]);
	printf("燃油液位为%.2f%%\r\n",carInfo[6]);
	printf("负荷计算值为%.2f%%\r\n",carInfo[7]);
	printf("发动机累计运行时间为%.0fs\r\n",carInfo[8]);	
}

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	delay_init(72);
	
	HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
	
	getInfo();
	showInfo();
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
