/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static FLASH_EraseInitTypeDef Erase_struct;
static FLASH_EraseInitTypeDef  Erase_struct_flag;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_START_ADDRESS  0x08020000  //ADDR of main programm memory to jump
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
uint8_t count = 0;
uint8_t buf_tx[8] = {'R','E','A','D','Y','F','F','F'};
uint8_t buf_rx[8];
uint8_t RXmsgData[8];
uint32_t mailBoxNum_1 = CAN_TX_MAILBOX0;
uint32_t program_data_1 = 0;
uint32_t program_data_2 = 0;
uint32_t flag_addr = 0x0807FFFC;
uint32_t flag_data = 0x12345678;
uint32_t erase_error = 0; //flag for recording the error (0xFFFFFFFFU means that all the sectors have been correctly erased)
uint32_t erase_error_flag = 0; //flag for recording the error (0xFFFFFFFFU means that all the sectors have been correctly erased)
uint32_t FLAG_DOWNLOAD_OVER = 0;
uint32_t FLAG_ERASE_OVER = 0;
uint64_t address_data = 0x08020000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
  
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Ascii_To_Hex( uint8_t* buff, uint8_t count)
{
	uint8_t i;
	
	for(i=0; i<count;i++)
	{
		if(buff[i] <= '9' && buff[i] >= '0' )
		{
			buff[i] -= 0x30;
		}
		else
		{
			buff[i] = buff[i] - 0x41 + 10;
		}	
	}	
}
void GoTo() //jump to the memory area specified in #define APP_START_ADDRESS
{
	//sprintf(buf_tx, "12345678\r\n");
	uint32_t appJumpAdress;
	
	appJumpAdress = *((volatile uint32_t*)(APP_START_ADDRESS+4));
	
	HAL_RCC_DeInit();
	//HAL_CAN_DeInit(&hcan1);
	HAL_DeInit();
	
	void(*GoToApp)(void);
	GoToApp = (void (*)(void)) appJumpAdress;
	
	//__disable_irq();
	__set_MSP(*((volatile uint32_t*)APP_START_ADDRESS));
	GoToApp();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
		HAL_FLASH_Unlock(); // unlock FLASH REMEMBER LOCK FLASH AFTER FINISHING DOWNLOAD!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		if (*(__IO  uint32_t*)flag_addr == flag_data)	{ //считали данные из последней ячейки памяти, если флаг выставлкн, то
			Erase_struct.TypeErase = FLASH_TYPEERASE_SECTORS; //type of erasable memory - sectors
			Erase_struct.Sector = FLASH_SECTOR_5; //sector number to start erasing
			Erase_struct.NbSectors = 2; //number of erased sectars starting from
			HAL_FLASHEx_Erase(&Erase_struct, &erase_error); //erase memory
			Erase_struct_flag.TypeErase = FLASH_TYPEERASE_SECTORS; //type of erasable memory - sectors
			Erase_struct_flag.Sector = FLASH_SECTOR_7; //sector number to start erasing
			Erase_struct_flag.NbSectors = 1; //number of erased sectars starting from
			HAL_FLASHEx_Erase(&Erase_struct_flag, &erase_error_flag); //erase memory
			FLAG_ERASE_OVER = 1; //выставляем флаг окончания чиски секторов памяти
		}
		else { // если последяя ячейка памяти пустая, то
			HAL_FLASH_Lock(); //memory write lock
			GoTo(); //прыгаем в основную программу
		}
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
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	TxHeader.StdId = 0x7f0;
  TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
		if (FLAG_DOWNLOAD_OVER == 1) { //если отправка прошивки по кану завершена, то
			HAL_FLASH_Lock(); //memory write lock
			//HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
			//HAL_Delay(500);
			NVIC_SystemReset(); //перезагрузка
		}
		if (FLAG_ERASE_OVER == 1) { //если чиска секторов памяти завершена, то
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, buf_tx, &mailBoxNum_1); //отправляем сообщение utc о готовности
			FLAG_ERASE_OVER = 0; //сбоасываем флаг чистки секторов, чтоб не отправиять сообщение о готовности приема прошивки брлее одного раза
			//HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
			//HAL_Delay(500);
		}
		//HAL_Delay(500);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
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
	CAN_FilterTypeDef canFilterConfig;
  canFilterConfig.FilterBank = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x07F5<<5;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  canFilterConfig.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);
  /* USER CODE END CAN1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{ 

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RXmsgData);
	if (buf_rx[0] == 'S' && buf_rx[1] == 'T' && buf_rx[2] == 'A' && buf_rx[3] == 'R' && buf_rx[4] == 'T') { // //checking starting of the firmware transfer

	//NVIC_SystemReset();

	}
	else { //пропускаем все слова, кроме Start

		if (RXmsgData[0] == 'E' && RXmsgData[1] == 'N' && RXmsgData[2] == 'D') { //utc прислало end, то
			HAL_FLASH_Lock(); //блокируем память
			FLAG_DOWNLOAD_OVER = 1; //выставляем флаг окончания загрузки в 1
		}

		program_data_1 = (RXmsgData[0] << 24) + (RXmsgData[1] << 16) + (RXmsgData[2] << 8) + RXmsgData[3]; //формируем первое слово
		program_data_2 = (RXmsgData[4] << 24) + (RXmsgData[5] << 16) + (RXmsgData[6] << 8) + RXmsgData[7]; //формируем второе слово
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address_data, program_data_1); //записываем в память основной программы первое сформированное сьово
		address_data = address_data + 4; // увеличиваем алрес ячейки памяти для записи основной программы на 4, то есть в следующий раз будем писать уже в следующую ячейку
		program_data_1 = 0; //чистим первое слово
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address_data, program_data_2); //пишем второе слово
		address_data = address_data + 4;
		program_data_2 = 0;
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, buf_rx, &mailBoxNum_1); //отправляем сообщение в utc о готовности принять следующие 2 слова
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
