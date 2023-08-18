/*
 * spi.c
 *
 *  Created on: Sep 22, 2021
 *      Author: sparrow
 */

#include "main.h"
#include "spi.h"
#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef hspi2;

uint8_t readSPI8(uint8_t addr)
{
	uint8_t buf[3] = {0x01, addr, 0};
//	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
//	HAL_SPI_Receive(&hspi1, buf, sizeof(buf), HAL_MAX_DELAY);
//	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
	return buf[2];
}

uint16_t readSPI16(uint8_t addr)
{
	uint8_t buf[4] = {0x02, addr, 0, 0};
	uint16_t res;
//	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
//	HAL_SPI_Receive(&hspi1, buf, sizeof(buf), HAL_MAX_DELAY);
//	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
	uint16_t b1 = buf[2];
	uint16_t b2 = buf[3];
	res = b1 | (b2 << 8);
	return res;
}
