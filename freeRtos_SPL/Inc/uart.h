/*
 * uart.h
 *
 *  Created on: 26 июл. 2016 г.
 *      Author: yurock
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "stm32f4xx_usart.h"


#define UART_INBUFSZ	64
#define UART_OUTBUFSZ	64

/* \~russian
 * Структура данных для драйвера UART.
 */
typedef struct _uart_t {
	xSemaphoreHandle xTransniter;
	xSemaphoreHandle xReciever;
	xSemaphoreHandle xMutTx;
	USART_InitTypeDef config;
	USART_TypeDef *port;        /*!< UART registers base address  */
	unsigned char outBuf [UART_OUTBUFSZ];
	unsigned char *outHead, *outTail;
	unsigned char inBuf [UART_INBUFSZ];
	unsigned char *inHead, *inTail;
	BaseType_t whaitTransmit;
} uart_t;




BaseType_t uart_init (uart_t *u, USART_TypeDef *port, uint32_t baud);
void uart_interrupt(void *arg);
char GetCharUart(uart_t *u);
void PutCharUart(uart_t *u, char c);
void PutStringUart(uart_t *u, const char *str);

#endif /* INC_UART_H_ */
