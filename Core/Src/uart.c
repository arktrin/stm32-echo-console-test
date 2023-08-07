/*
 * uart.c
 *
 *  Created on: Sep 20, 2021
 *      Author: sparrow
 */

#include "main.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
UART_HandleTypeDef *huarts[2] = {&huart1, &huart3};

typedef struct Fifo
{
	size_t  front;
	size_t  size;
	uint8_t data[BUF_SIZE];
} Fifo;

uint8_t uart1Buf[HALF_SIZE] = {0}; // buffer for UART1
size_t uart1DataSize = 0; // UART1 buffer size
Fifo fifo = {0, 0, {0}}; // FIFO for UART3
char uartOutBuf[2][64] = {0}; // transmit buffers for UART1 and UART3

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

size_t uart1ReadAll(uint8_t *buf)
{
	size_t res = uart1DataSize;
	if (uart1DataSize > 0)
	{
		memcpy(buf, uart1Buf, uart1DataSize);
		buf[uart1DataSize] = '\0';
		uart1DataSize = 0;
		++res;
	}
	return res;
}

size_t uart3ReadAll(uint8_t *buf)
{
	size_t res = fifo.size;
	if (fifo.size > 0)
	{
		int i = 0;
		for(; (fifo.size > 0) && (i < BUF_SIZE - 1); ++i, fifo.size--)
		{
			buf[i] = fifo.data[fifo.front];
			fifo.front = (fifo.front + 1) % BUF_SIZE;
		}
		buf[i] = '\0';
		++res;
	}
	return res;
}

void uartRequestData(Uart uart)
{
	if (uart == uart1)
	{
		uart1DataSize = 0;
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, uart1Buf, HALF_SIZE);
	}
	else
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, fifo.data, BUF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	}
}

void setUartCommand(Uart uart, const char *str)
{
	strcpy(uartOutBuf[uart], str);
}

void uartSendCommand(Uart uart)
{
	UART_HandleTypeDef *huart = huarts[uart];
	char *buf = uartOutBuf[uart];
	HAL_UART_Transmit(huart, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}

void uartSendIt(Uart uart, const char *str)
{
	UART_HandleTypeDef *huart = huarts[uart];
	char *buf = uartOutBuf[uart];
	strcpy(buf, str);
	HAL_UART_Transmit_IT(huart, (uint8_t*)buf, strlen(buf));
}

//void initRubidium()
//{
//	HAL_UART_Transmit(&huart1, (uint8_t*)"a", 1, HAL_MAX_DELAY);
//}
//
//void rubidiumDiscipline(int val)
//{
//	char *buf = uartOutBuf[uart1];
//	val = round(val / 1.6384) - 20000;
//	int n = sprintf(buf, "<FD%+05d>\r\n", val);
//	if (n > 0)
//	{
//		HAL_UART_Transmit_IT(&huart1, (uint8_t*)buf, n);
//	}
//}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	if (huart == &huart1)
//	{
//		uart1DataSize += Size;
//	}
//	else
//	{
//		fifo.size += Size;
//		if (fifo.size >= BUF_SIZE)
//		{
//			fifo.front = (fifo.front + HALF_SIZE) % BUF_SIZE;
//			fifo.size -= Size;
//		}
//	}
//}
//
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if (huart == &huart1)
//		uartRequestData(uart1);
//	else if (huart == &huart3)
//		uartRequestData(uart3);
//}

//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//	uint8_t err = HAL_UART_GetError(huart);
//	if (err == HAL_UART_ERROR_ORE)
//	{
//		__HAL_UART_CLEAR_OREFLAG(huart);
//		HAL_UART_Abort(huart);
//		if (huart == &huart1)
//		{
//			uartRequestData(uart1);
//			//CDC_Transmit_FS((uint8_t*)"\n\n! uart1 error !\n\n", 19);
//		}
//		else
//		{
//			uartRequestData(uart3);
//			CDC_Transmit_FS((uint8_t*)"\n\n! uart3 error !\n\n", 19);
//		}
//	}
//}
