/*
 * uart.h
 *
 *  Created on: 26 июл. 2016 г.
 *      Author: yurock
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "portmacro.h"
#include "main.h"
#include "queue.h"


#define UART_INBUFSZ	64
#define UART_OUTBUFSZ	64

/* \~russian
 * Структура данных для драйвера UART.
 */
typedef struct _uart_t {
	osSemaphoreId xSemaHandl;
	UART_HandleTypeDef *port;
	unsigned char outBuf [UART_OUTBUFSZ];
	unsigned char *outHead, *outTail;
	unsigned char inBuf [UART_INBUFSZ];
	unsigned char *inHead, *inTail;
} uart_t;






BaseType_t uart_init (uart_t *u, UART_HandleTypeDef *port);
void uart_interrupt(void *arg);
char GetCharUart(uart_t *u);
void PutCharUart(uart_t *u, char c);
void PutStringUart(uart_t *u, const char *str);



BaseType_t initUartTask(uart_t *u);

//void MX_FREERTOS_UART_Init();

//void vTaskUartRx(void *pvParameters);
//void vTaskUartTx(void *pvParameters);

#endif /* INC_UART_H_ */
