/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "uart.h"
#include "spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEFAULT_TIMEOUT 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

uint8_t newline[] = "\n\r";
uint8_t endLine[] = "\r";

uint8_t rxBuffer[3];
bool dataReceived = false; // data received flag
bool dataTransmitted = true; // data transmitted flag

uint8_t commandBuffer[64];
uint8_t charCount; // for commandBuffer

enum Command {NoCommand = 0, WriteFPGA, WriteFlash, ReadFlash, ReadI2C, WriteI2C, ReadSPI, WriteSPI,
	          Rubidium, PrintOffset, NmeaPrint, Info, Tie} command = NoCommand;

int32_t fpgaDataSize = 0;
uint32_t flashDataSize = 0;
uint32_t flashCurAddr = 0;
uint8_t regAddr = 0;
uint8_t regVal = 0;
uint8_t regReadCount = 1;
bool initDone = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void processCommand(char *buf, uint32_t len)
{
	static const char updateCmd[] = "update";
	static const char readCmd[] = "read";
	static const char fpgaCmd[] = "fpga";
	static const char readI2C[] = "i2cget";
	static const char writeI2C[] = "i2cset";
	static const char readSPI[] = "spiget";
	static const char writeSPI[] = "spiset";
	static const char nmeaCmd[] = "nmea";
	static const char infoCmd[] = "info";
	static const char stopCmd[] = "stop";
	static const char resetCmd[] = "reset";

	char *s = strtok(buf, " ");
	if (s == 0)
		return;
	// update FPGA data command
	if (strcmp(s, updateCmd) == 0)
	{
//		s = strtok(0, " ");
//		if (s)
//		{
//			// save update size to flashDataSize
//			flashDataSize = atoi(s);
//			if (flashDataSize > 0)
//			{
//				command = WriteFlash;
//				switchLeds(flashDataSize % 255);
//				CDC_Transmit_FS((uint8_t*)updateCmd, sizeof(updateCmd));
//				// prepare for data write
//				flashWriteStatus(0x0);
//				flashChipErase();
//				// write data size to flash
//				uint8_t data[3] = {(flashDataSize) & 0xFF, (flashDataSize >> 8) & 0xFF, (flashDataSize >> 16) & 0xFF};
//				flashWriteBlock(data, 0, 3);
//				// skip first 3 bytes containing data size
//				flashCurAddr = 3;
//				flashDataSize += 3;
//			}
//		}
	}
	// read FPGA data command
	else if (strcmp(s, readCmd) == 0)
	{
		command = ReadFlash;
	}
	// write FPGA from PC command
	else if (strcmp(s, fpgaCmd) == 0)
	{
//		s = strtok(0, " ");
//		if (s)
//		{
//			fpgaDataSize = atoi(s);
//			if (fpgaDataSize > 0)
//			{
//				s = strtok(0, " ");
//				if (s)
//				{
//					int mode = atoi(s);
//					switch (mode)
//					{
//					case 1:
//						initDSPLL8_192();
//						fMode = f2048;
//						break;
//					default:
//						initDSPLL();
//						fMode = f10;
//						break;
//					}
//				}
//				switchLeds(0x0);
//				initDone = false;
//				command = WriteFPGA;
//				fpgaWriteStart();
//			}
//		}
	}
	// read I2C register command
	else if (strcmp(s, readI2C) == 0)
	{
		s = strtok(0, " ");
		if (s)
		{
			regAddr = atoi(s);
			command = ReadI2C;
		}
	}
	// write I2C register command
	else if (strcmp(s, writeI2C) == 0)
	{
		s = strtok(0, " ");
		if (s)
		{
			regAddr = atoi(s);
			command = WriteI2C;
			s = strtok(0, " ");
			if (s)
			{
				regVal = atoi(s);
			}
		}
	}
	// read SPI register command
	else if (strcmp(s, readSPI) == 0)
	{
		s = strtok(0, " ");
		if (s)
		{
			regAddr = atoi(s);
			command = ReadSPI;
			s = strtok(0, " ");
			if (s)
				regReadCount = atoi(s);
			else
				regReadCount = 1;
		}
	}
	// write SPI register command
	else if (strcmp(s, writeSPI) == 0)
	{
		s = strtok(0, " ");
		if (s)
		{
			regAddr = atoi(s);
			s = strtok(0, " ");
			if (s)
			{
				regVal = atoi(s);
				command = WriteSPI;
			}
		}
	}
	// read nmea command
	else if (strcmp(s, nmeaCmd) == 0)
	{
//		s = strtok(0, " ");
//		if (s)
//		{
//			uartSendIt(uart3, "$POVER*5E\r\n");
//		}
//		else
//		{
//			command = NmeaPrint;
//		}
	}
	// some extra info command
	else if (strcmp(s, infoCmd) == 0)
	{
		command = Info;
	}
	else if (strcmp(s, stopCmd) == 0)
	{
		command = NoCommand;
	}
	else if (strcmp(s, resetCmd) == 0)
	{
		HAL_NVIC_SystemReset();
	}
}

uint8_t readI2C1Reg(uint8_t addr)
{
	uint8_t data = 0;

	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, &addr, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, 0xD0, &data, 1, HAL_MAX_DELAY);

	return data;
}

void writeI2C1Reg(uint8_t addr, uint8_t byte)
{
	uint8_t data[] = {addr, byte};
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, data, sizeof(data), HAL_MAX_DELAY);
}

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
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_UART_Transmit_IT(&hlpuart1, (uint8_t *)newline, sizeof(newline)-1);
  HAL_UART_Receive_IT(&hlpuart1, rxBuffer, 1);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(500);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x30A0A7FB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

  if(huart == &hlpuart1) {

    dataReceived = true;
    while (huart->RxState != HAL_UART_STATE_READY) { }
    HAL_UART_Receive_IT(huart, rxBuffer, 1);
    commandBuffer[charCount] = rxBuffer[0];
    charCount += 1;
    if (rxBuffer[0] == endLine[0]) {
    	processCommand((char*)commandBuffer, 100);

    	while (huart->gState != HAL_UART_STATE_READY) { }
    	HAL_UART_Transmit(huart, (uint8_t *)newline, sizeof(newline)-1, DEFAULT_TIMEOUT);
    	strncpy((char*)commandBuffer, "", sizeof(commandBuffer));
    	charCount = 0;
    	while (huart->gState != HAL_UART_STATE_READY) { }
    	HAL_UART_Transmit(huart, (uint8_t *)newline, sizeof(newline)-1, DEFAULT_TIMEOUT);
    }
    if (dataTransmitted) {
    	while (huart->gState != HAL_UART_STATE_READY) { }
    	HAL_UART_Transmit_IT(huart, rxBuffer, 1);
    	dataReceived = false;
    	dataTransmitted = false;
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

  if(huart == &hlpuart1) {

    dataTransmitted = true;

    if (dataReceived) {
      while (huart->gState != HAL_UART_STATE_READY) { }
      HAL_UART_Transmit_IT(huart, rxBuffer, 1);
      dataReceived = false;
      dataTransmitted = false;
    }
  }
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{

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
