/*
 * uart1Task.c
 *
 *  Created on: 1 авг. 2016 г.
 *      Author: yurock
 */
#include "uart1Task.h"
//#include "fatfs.h"


/* ---------------------------------------------------------- */

//osThreadId U1RXHandle;
//osMessageQId RecQHandle;
static void Usart1Rx(void * argument);

/* ---------------------------------------------------------- */

BaseType_t initUartTask(uart_t *u) {
	BaseType_t result = pdPASS;
	//osThreadDef(U1RX, Usart1Rx, osPriorityBelowNormal + 2, 0, 256);
	//U1RXHandle = osThreadCreate(osThread(U1RX), (void *) u);
	result = xTaskCreate(Usart1Rx, "Uart1_Rx", 1000, (void * )u, 2, NULL);

	//if (U1RXHandle == NULL) {
	//	result = pdFAIL;
	//	HAL_GPIO_WritePin(GPIOD, GreenLed_Pin, 1);
	//}
	return result;
}

//FATFS fs;
//FRESULT res;
//FILINFO fno;
//DIR dir;
const char * helloStr = "Test uart. prio = %d\n";
/* Usart1Rx function */
char TxData[32];
void Usart1Rx(void * argument) {
	/* USER CODE BEGIN Usart1Rx */
	uart_t *u = (uart_t *) argument;
	uint8_t data;
	/* Infinite loop */
	/*static uint32_t sizeHal = sizeof(UART_HandleTypeDef);
	 static uint32_t sizeRtos = sizeof(uart_t);
	 if (sizeHal > 100) {
	 HAL_GPIO_WritePin(GPIOD, BlueLed_Pin, 1);
	 }
	 if (sizeRtos > 200) {
	 HAL_GPIO_WritePin(GPIOD, BlueLed_Pin, 1);
	 }*/
	//sprintf(TxData, helloStr, 0);
	//PutStringUart(u, TxData);

	sprintf(TxData, "Start Task for uart1\n");
	PutStringUart(u, TxData);

	//монтируем диск без проверки
	/*	if (f_mount(&fs, "0", 1) == FR_OK) {
	 //монтируем диск без проверки

	 //открываем директорию
	 if (f_opendir(&dir, "\\") == FR_OK) {

	 //чи таем содержимое директории
	 for (;;) {
	 res = f_readdir(&dir, &fno);
	 if ((res != FR_OK) || (fno.fname[0] == 0)) {
	 break;
	 }
	 PutStringUart(u, fno.fname);
	 PutStringUart(u, " \r");
	 }
	 }
	 }
	 f_mount(&fs, "0", 0);*/
	for (;;) {

		data = GetCharUart(u);
		PutCharUart(u, data);
		/**TxData1.head++ = data;
		 if (TxData1.head >= TxData1.buf + LEN_BUF)
		 TxData1.head = TxData1.buf;
		 if ((huart1.Instance->SR & USART_SR_TXE) && *(huart1.Instance->SR & )) {
		 if (TxData1.head != TxData1.tail)
		 UsartTx(&huart1);
		 }*/
		/*if (HAL_UART_Transmit(&huart1, &data, 1, HAL_MAX_DELAY) == HAL_OK) {
		 HAL_GPIO_WritePin(GPIOD, BlueLed_Pin, 1);
		 } else
		 HAL_GPIO_WritePin(GPIOD, BlueLed_Pin, 0);*/

	}
	/* USER CODE END Usart1Rx */
}
