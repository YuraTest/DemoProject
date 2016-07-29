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


#define UART_STACKSZ	0x200
#define UART_INBUFSZ	32
#define UART_OUTBUFSZ	32
#define LEN_BUF 256

typedef struct {
	char *head;
	char *tail;
	//uint16_t count;
	char buf[LEN_BUF];
} UART_messages_t;

/* \~russian
 * Структура данных для драйвера UART.
 */
typedef struct _uart_t {
	//stream_interface_t *interface;
	//xSemaphoreHandle transmitter;
	//xSemaphoreHandle receiver;
	osSemaphoreId xSemaHandl;
	//xQueueHandle xQueueReceiver;
	//xQueueHandle xQueueTransmiter;
	UART_HandleTypeDef *port;
	//UART_messages_t RxData;
	//UART_messages_t TxData;
	unsigned char outBuf [UART_OUTBUFSZ];
	unsigned char *outHead, *outTail;
	unsigned char inBuf [UART_INBUFSZ];
	unsigned char *inHead, *inTail;
	//unsigned portSHORT rstack;
	//ARRAY (rstack, UART_STACKSZ);		/* task receive stack */
} uart_t;






BaseType_t uart_init (uart_t *u, UART_HandleTypeDef *port);
void uart_interrupt(void *arg);
char GetChar(uart_t *u);
void PutChar(uart_t *u, char c);

BaseType_t initUartTask(uart_t *u);

void MX_FREERTOS_UART_Init();

//void vTaskUartRx(void *pvParameters);
//void vTaskUartTx(void *pvParameters);

#endif /* INC_UART_H_ */
