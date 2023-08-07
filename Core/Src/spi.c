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
//
//uint32_t readSPI24(uint8_t addr)
//{
////	uint8_t buf[5] = {0x03, addr, 0, 0, 0};
////	uint32_t res;
////	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
////	HAL_SPI_Receive(&hspi1, buf, sizeof(buf), HAL_MAX_DELAY);
////	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
////	uint32_t b1 = buf[2];
////	uint32_t b2 = buf[3];
////	uint32_t b3 = buf[4];
////	res = b1 | (b2 << 8) | (b3 << 16);
////	return res;
//}
//
//uint32_t readSPI32(uint8_t addr)
//{
////	uint8_t buf[6] = {0x04, addr, 0, 0, 0, 0};
////	uint32_t res;
////	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
////	HAL_SPI_Receive(&hspi1, buf, sizeof(buf), HAL_MAX_DELAY);
////	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
////	uint32_t b1 = buf[2];
////	uint32_t b2 = buf[3];
////	uint32_t b3 = buf[4];
////	uint32_t b4 = buf[5];
////	res = b1 | (b2 << 8) | (b3 << 16) | (b4 << 24);
////	return res;
//}
//
//uint64_t readSPI40(uint8_t addr)
//{
////	uint8_t buf[7] = {0x05, addr, 0, 0, 0, 0, 0};
////	uint64_t res;
////	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
////	HAL_SPI_Receive(&hspi1, buf, sizeof(buf), HAL_MAX_DELAY);
////	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
////	uint64_t b1 = buf[2];
////	uint64_t b2 = buf[3];
////	uint64_t b3 = buf[4];
////	uint64_t b4 = buf[5];
////	uint64_t b5 = buf[6];
////	res = b1 | (b2 << 8) | (b3 << 16) | (b4 << 24) | (b5 << 24);
//	return res;
//}
//
//uint64_t readSPI48(uint8_t addr)
//{
//	uint8_t buf[8] = {0x06, addr, 0, 0, 0, 0, 0, 0};
////	uint64_t res;
////	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
////	HAL_SPI_Receive(&hspi1, buf, sizeof(buf), HAL_MAX_DELAY);
////	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
////	uint64_t b1 = buf[2];
////	uint64_t b2 = buf[3];
////	uint64_t b3 = buf[4];
////	uint64_t b4 = buf[5];
////	uint64_t b5 = buf[6];
////	uint64_t b6 = buf[7];
//	res = b1 | (b2 << 8) | (b3 << 16) | (b4 << 24) | (b5 << 32) | (b6 << 40);
//	return res;
//}
//
//void writeSPI8(uint8_t addr, uint8_t byte)
//{
////	uint8_t buf[3] = {0x81, addr, byte};
////	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
////	HAL_SPI_Transmit(&hspi1, buf, sizeof(buf), HAL_MAX_DELAY);
////	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
//}
//
//void writeSPI40(uint8_t addr, uint64_t val)
//{
////	uint8_t c1 = val & 0xFF;
////	uint8_t c2 = (val & 0xFFFF) >> 8;
////	uint8_t c3 = (val & 0xFFFFFF) >> 16;
////	uint8_t c4 = (val & 0xFFFFFFFF) >> 24;
////	uint8_t c5 = (val & 0xFFFFFFFFFF) >> 32;
////	uint8_t buf[] = {0x85, addr, c1, c2, c3, c4, c5};
////	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
////	HAL_SPI_Transmit(&hspi1, buf, sizeof(buf), HAL_MAX_DELAY);
////	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
//}
//
//void writeSPI48(uint8_t addr, uint64_t val)
//{
////	uint8_t c1 = val & 0xFF;
////	uint8_t c2 = (val & 0xFFFF) >> 8;
////	uint8_t c3 = (val & 0xFFFFFF) >> 16;
////	uint8_t c4 = (val & 0xFFFFFFFF) >> 24;
////	uint8_t c5 = (val & 0xFFFFFFFFFF) >> 32;
////	uint8_t c6 = (val & 0xFFFFFFFFFFFF) >> 40;
////	uint8_t buf[] = {0x86, addr, c1, c2, c3, c4, c5, c6};
////	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
////	HAL_SPI_Transmit(&hspi1, buf, sizeof(buf), HAL_MAX_DELAY);
////	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
//}
