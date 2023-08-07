/*
 * spi.h
 *
 *  Created on: Sep 22, 2021
 *      Author: sparrow
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

// read single byte from register
uint8_t readSPI8(uint8_t addr);
// read 2 bytes from register
uint16_t readSPI16(uint8_t addr);

/*SPI registers definitions*/
#define FW_VER      0x0 // size = 2
#define HW_VER      0x1 // size = 2
#define COMMAND     0x2 // size = 1
#define CONTROL     0x3 // size = 1
#define ANT_DEL     0x4 // size = 2
#define OFFSET 		0x8 // size = 4
#define TIE_UNFILT  0x9 // size = 128
#define SAMPLES_COUNT   0xA // size = 1
#define FREQ_SUM	0xB // size = 6
#define PPS_PULSE_SUM   0xC // size = 3

/*CONTROL register bits masks*/
#define SYNC_EN    0x1
#define PPS_SRC    0x2
#define CLK_SRC    0x4
#define WANDER_ENA 0x8
#define FREQ_ENA   0x10
#define RST_PPS	   0x1
#define RST_FREQ   0x2

/*TIE data size*/
#define TIE_DATA_SIZE 256

#endif /* INC_SPI_H_ */
