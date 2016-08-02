/*
 * uart1Task.c
 *
 *  Created on: 1 авг. 2016 г.
 *      Author: yurock
 */
#include "uart1Task.h"
#include "ff.h"

/* ---------------------------------------------------------- */

static void Usart1Rx(void * argument);

/* ---------------------------------------------------------- */

BaseType_t initUartTask(uart_t *u) {
	BaseType_t result = pdPASS;
	result = xTaskCreate(Usart1Rx, "Uart1_Rx", 1000, (void * )u, 2, NULL);

	return result;
}

//const short * helloStr = "Тест УАРТ\n";
/* Usart1Rx function */
char TxData[32];
void Usart1Rx(void * argument) {
	/* USER CODE BEGIN Usart1Rx */
	uart_t *u = (uart_t *) argument;
	uint8_t data;
	//sprintf(TxData, helloStr, 0);
	//PutStringUart(u, TxData);

	sprintf(TxData, "Start Task for uart1\n");
	PutStringUart(u, TxData);

	for (;;) {

		data = GetCharUart(u);
		PutCharUart(u, data);
	}
	/* USER CODE END Usart1Rx */
}
