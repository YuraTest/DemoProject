/*
 * uart.c
 *
 *  Created on: 26 июл. 2016 г.
 *      Author: yurock
 */
#include "stm32f4xx_hal.h"
#include "uart.h"
#include "fatfs.h"

static void UsartTx(uart_t *u);

/* ---------------------------------------------------------- */

static void UsartTx(uart_t *u) {
	char tmp;
	if (u->outHead != u->outTail) {
		tmp = *u->outTail;
		//u->port->Instance->DR = *u->outTail;
		if (++u->outTail >= u->outBuf + UART_OUTBUFSZ)
			u->outTail = u->outBuf;
		u->port->Instance->DR = (uint16_t) tmp;
	}

	/*unsigned char *newlast = u->outHead + 1;// временный указатель инкрементируем относительно старой головы
	 if (newlast >= u->outBuf + UART_INBUFSZ)// Если временный указатель больше или равен очереди,
	 newlast = u->outBuf;			// то приравниваем его началу буфера
	 Если нет места в буфере - теряем данные.
	 if (u->outTail != newlast) {// Если новый указатель не равен хвосту, от куда читаются данные
	 *u->outHead = c;			// Добавляем символ
	 u->outHead = newlast;	// Устанавливаем новый указатель головы
	 // Посылаем
	 }*/

}

/* ---------------------------------------------------------- */
/* Обработчик прерывания для UART
 * Параметры: указатель на структуру uart_t */
void uart_interrupt(void *arg) {
	uart_t *u = arg;
	/* Приём. */
	if ((READ_REG(u->port->Instance->SR) & USART_SR_RXNE)) {
		/* В буфере FIFO приемника есть данные. */
		unsigned c = u->port->Instance->DR;

		unsigned char *newlast = u->inHead + 1;	// временный указатель инкрементируем относительно старой головы
		if (newlast >= u->inBuf + UART_INBUFSZ)	// Если временный указатель больше или равен очереди,
			newlast = u->inBuf;				// то приравниваем его началу буфера

		/* Если нет места в буфере - теряем данные. */
		if (u->inTail != newlast) {	// Если новый указатель не равен хвосту, от куда читаются данные
			*u->inHead = c;			// Добавляем символ
			u->inHead = newlast;	// Устанавливаем новый указатель головы
			// Посылаем семофор
			osSemaphoreRelease(u->xSemaHandl);
		}
		//passive = 0;
	}

	/* Передача. */
	if (READ_REG(u->port->Instance->SR) & USART_SR_TC) {

		if (u->outHead != u->outTail) {
			/* Шлём очередной байт. */
			u->port->Instance->DR = *u->outTail;
			if (++u->outTail >= u->outBuf + UART_OUTBUFSZ)
				u->outTail = u->outBuf;
		} else {
			/* Нет данных для передачи - сброс прерывания. */
			CLEAR_BIT(u->port->Instance->SR, USART_SR_TC);
			//u->port->Instance->SR = ARM_UART_RIS_TX;
			//passive = 0;
		}
	}
}

/* Инициализация драйвера UART
 * Параметры: указатель на структуру uart, указатель на структуру UART_HAL
 * возвращаемое значение: В случае успешной инициализации pdPASS, иначк pdFAIL */
BaseType_t uart_init(uart_t *u, UART_HandleTypeDef *port) {
	BaseType_t result = pdPASS;
	u->port = port;
	u->inHead = u->inBuf;
	u->inTail = u->inBuf;
	u->outHead = u->outBuf;
	u->outTail = u->outBuf;

	//vSemaphoreCreateBinary(u->transmitter);
	//vSemaphoreCreateBinary(u->receiver);
	//if ((u->transmitter == NULL) || (u->receiver == NULL))
	osSemaphoreDef(myCountSem);
	u->xSemaHandl = osSemaphoreCreate(osSemaphore(myCountSem), 32);
	if (u->xSemaHandl == NULL) {
		result = pdFAIL;
		HAL_GPIO_WritePin(GPIOD, RedLed_Pin, 1);
	}
	SET_BIT(u->port->Instance->CR1, USART_CR1_RXNEIE | USART_CR1_TCIE);
	//CLEAR_BIT(u->port->Instance->SR, USART_SR_TC);
	return result;
}

/* Чтение байта из буфера UART
 * Параметр: указатель на структуру uart_t
 * Возвращаемое значение: прочитанный байт */
char GetCharUart(uart_t *u) {
	osSemaphoreWait(u->xSemaHandl, osWaitForever);// ожидаем появление байта в буфере
	char data;
	data = *u->inTail;
	unsigned char *newlast = u->inTail + 1;
	if (newlast >= u->inBuf + UART_INBUFSZ)	// Если временный указатель больше или равен очереди,
		newlast = u->inBuf;				// то приравниваем его началу буфера
	u->inTail = newlast;
	return data;
}

/* Запись байта в буфер UART
 * Параметр: указатель на структуру uart_t, записываемый байт
 * Возвращаемое значение: - */
void PutCharUart(uart_t *u, char c) {
	unsigned char *newlast = u->outHead + 1;
	if (newlast >= u->outBuf + UART_INBUFSZ)// Если временный указатель больше или равен очереди,
		newlast = u->outBuf;				// то приравниваем его началу буфера
	/* Если нет места в буфере - теряем данные. */
	if (u->outTail != newlast) {// Если новый указатель не равен хвосту, от куда читаются данные
		*u->outHead = c;			// Добавляем символ
		u->outHead = newlast;	// Устанавливаем новый указатель головы
		/* Если передатчик свободный, то посылаем байт */
		if (u->port->Instance->SR & USART_SR_TXE) {
			UsartTx(u);
		}
	}
}

/* Запись строки, заканчивающейся нулём в буфера UART
 * Параметр: указатель на структуру uart_t, строка, заканчивающаяся нулевым байтом
 * Возвращаемое значение: - */
void PutStringUart(uart_t *u, const char *str) {
	char *buf = (char *) str;
	while (*buf != 0x00)
		PutCharUart(u, *buf++);
}

/* ---------------------------------------------------------- */

osThreadId U1RXHandle;
osMessageQId RecQHandle;
void Usart1Rx(void const * argument);

/* ---------------------------------------------------------- */

BaseType_t initUartTask(uart_t *u) {
	BaseType_t result = pdPASS;
	osThreadDef(U1RX, Usart1Rx, osPriorityBelowNormal + 2, 0, 256);
	U1RXHandle = osThreadCreate(osThread(U1RX), (void *) u);

	if (U1RXHandle == NULL) {
		result = pdFAIL;
		HAL_GPIO_WritePin(GPIOD, GreenLed_Pin, 1);
	}
	return result;
}

FATFS fs;
FRESULT res;
FILINFO fno;
DIR dir;
const char * helloStr = "Test uart\n";
/* Usart1Rx function */
void Usart1Rx(void const * argument) {
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
	PutStringUart(u, helloStr);

	//монтируем диск без проверки
	if (f_mount(&fs, "0", 1) == FR_OK) {
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
	f_mount(&fs, "0", 0);
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

#ifdef OLD_A
osThreadId U1RXHandle;
osMessageQId RecQHandle;

extern UART_HandleTypeDef huart1;

void Usart1Rx(void const * argument);

static void UsartTx(UART_HandleTypeDef *UartHandle);

//uint8_t b1;
//uint8_t b2;

//typedef struct {
//	uint8_t usart;
//	uint8_t byte;
//} myMes;

//portCHAR uartBuf[32];
UART_messages_t RxData1;
UART_messages_t TxData1;

static void UsartTx(UART_HandleTypeDef *UartHandle) {
	char tmp;
	switch ((uint32_t) UartHandle->Instance) {
		case (uint32_t) USART1:
		tmp = *TxData1.tail++;
		if (TxData1.tail >= TxData1.buf + LEN_BUF) // Если достигли конца буфера, то переходим к началу
		TxData1.tail = TxData1.buf;
		//TxCountByte--; // Уменьшаем счётчик данных для отправки
		//USART_SendData(USART1, tmp);
		huart1.Instance->DR = (uint16_t) tmp;
		break;
		default:
		break;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	uint8_t data;
	//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);
	switch ((uint32_t) UartHandle->Instance) {
		case (uint32_t) USART1:
		//HAL_UART_Receive(&huart1, &data, 1, portMAX_DELAY);
		data = huart1.Instance->DR & 0xFF;

		//if(++(*RxData1.head) > RxData1.buf + LEN_BUF)
		//	RxData1.head = RxData1.buf;
		//xQueueSend(RecQHandle, &m, 1);
		xQueueSendFromISR(RecQHandle, &data, &xHigherPriorityTaskWoken);
		// Do this need?
		//HAL_UART_Receive_IT(&huart1, &b1, 1);
		//portYIELD_FROM_ISR(xHigherPriorityTaskWoken)
		//;
		break;
		/*	case (uint32_t) USART2:
		 HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
		 m.usart = 2;
		 m.byte = b2;
		 xQueueSend(RecQHandle, &m, portMAX_DELAY);
		 //HAL_UART_Receive_IT(&huart2, &b2, 1);
		 break;*/
		default:
		break;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (TxData1.head != TxData1.tail)
	UsartTx(&huart1);

}

void MX_FREERTOS_UART_Init() {
	osThreadDef(U1RX, Usart1Rx, osPriorityBelowNormal + 2, 0, 256);
	U1RXHandle = osThreadCreate(osThread(U1RX), NULL);

	osMessageQDef(RecQ, 32, portCHAR);
	RecQHandle = osMessageCreate(osMessageQ(RecQ), NULL);

	//HAL_UART_MspInit(&huart1);
	//HAL_UART_Receive_IT(&huart1, (uint8_t *)uartBuf, 32);
	//HAL_UART_Receive_IT(&huart1, &b1, 1);
	/* Enable the UART Parity Error and Data Register not empty Interrupts */
	SET_BIT(huart1.Instance->CR1, USART_CR1_RXNEIE | USART_CR1_TCIE);
	RxData1.tail = RxData1.buf;
	RxData1.head = RxData1.buf;
	TxData1.tail = TxData1.buf;
	TxData1.head = TxData1.buf;
	//RxData1.count = 0;

}

/* Usart1Rx function */
void Usart1Rx(void const * argument) {
	/* USER CODE BEGIN Usart1Rx */
	uint8_t data;
	/* Infinite loop */
	for (;;) {
		xQueueReceive(RecQHandle, &data, portMAX_DELAY);
		*TxData1.head++ = data;
		if (TxData1.head >= TxData1.buf + LEN_BUF)
		TxData1.head = TxData1.buf;
		if ((huart1.Instance->SR & USART_SR_TXE)/* && *(huart1.Instance->SR & )*/) {
			if (TxData1.head != TxData1.tail)
			UsartTx(&huart1);
		}
		/*if (HAL_UART_Transmit(&huart1, &data, 1, HAL_MAX_DELAY) == HAL_OK) {
		 HAL_GPIO_WritePin(GPIOD, BlueLed_Pin, 1);
		 } else
		 HAL_GPIO_WritePin(GPIOD, BlueLed_Pin, 0);*/

	}
	/* USER CODE END Usart1Rx */
}
#endif

