/*
 * uart.h
 *
 *  Created on: Sep 20, 2021
 *      Author: sparrow
 */

#ifndef INC_UART_H_
#define INC_UART_H_

typedef enum Uart {uart1 = 0, uart3 = 1} Uart;

size_t uart1ReadAll(uint8_t *buf);
size_t uart3ReadAll(uint8_t *buf);
void uartRequestData(Uart uart);
void setUartCommand(Uart uart, const char *str);
void uartSendCommand(Uart uart);
void uartSendIt(Uart uart, const char *str);
void rubidiumDiscipline(int val);
void initRubidium();

#define BUF_SIZE 4096
#define HALF_SIZE  BUF_SIZE / 2

#endif /* INC_UART_H_ */
