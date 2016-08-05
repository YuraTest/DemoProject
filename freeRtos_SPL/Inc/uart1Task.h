/*
 * uart1Task.h
 *
 *  Created on: 1 авг. 2016 г.
 *      Author: yurock
 */

#ifndef INC_UART1TASK_H_
#define INC_UART1TASK_H_

#include "main.h"
#include "uart.h"



/* Структура для приёма / передачи сообщения по RS232 */
typedef struct {
	uint8_t data[SIZE_DATA_BUF];
	uint32_t size;
} uart_msg_t;

BaseType_t initUartTask(uart_t *u);

#endif /* INC_UART1TASK_H_ */
