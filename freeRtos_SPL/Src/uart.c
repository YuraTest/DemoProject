/*
 * uart.c
 *
 *  Created on: 26 июл. 2016 г.
 *      Author: yurock
 */

#include "uart.h"


static void UsartTx(uart_t *u);
static uart_t *uarts[10];

/* ---------------------------------------------------------- */

void USART1_IRQHandler() {
	if (uarts[0] != NULL)
		uart_interrupt(uarts[0]);
}

void USART2_IRQHandler() {
	if (uarts[1] != NULL)
		uart_interrupt(uarts[1]);
}

void USART3_IRQHandler() {
	if (uarts[2] != NULL)
		uart_interrupt(uarts[2]);
}

void UART4_IRQHandler() {
	if (uarts[3] != NULL)
		uart_interrupt(uarts[3]);
}

void UART5_IRQHandler() {
	if (uarts[4] != NULL)
		uart_interrupt(uarts[4]);
}

void USART6_IRQHandler() {
	if (uarts[5] != NULL)
		uart_interrupt(uarts[5]);
}

/* ---------------------------------------------------------- */

static void UsartTx(uart_t *u) {
	char tmp;
	if (u->outHead != u->outTail) {
		tmp = *u->outTail;
		//u->port->Instance->DR = *u->outTail;
		if (++u->outTail >= u->outBuf + UART_OUTBUFSZ)
			u->outTail = u->outBuf;
		u->port->DR = (uint16_t) tmp;
	}
}

/* ---------------------------------------------------------- */
/* Обработчик прерывания для UART
 * Параметры: указатель на структуру uart_t */
void uart_interrupt(void *arg) {
	static portBASE_TYPE xPrio;
	uart_t *u = arg;
	/* Приём. */
	if ((READ_REG(u->port->SR) & USART_SR_RXNE)) {
		/* В буфере FIFO приемника есть данные. */
		unsigned c = u->port->DR;

		unsigned char *newlast = u->inHead + 1;	// временный указатель инкрементируем относительно старой головы
		if (newlast >= u->inBuf + UART_INBUFSZ)	// Если временный указатель больше или равен очереди,
			newlast = u->inBuf;				// то приравниваем его началу буфера

		/* Если нет места в буфере - теряем данные. */
		if (u->inTail != newlast) {	// Если новый указатель не равен хвосту, от куда читаются данные
			*u->inHead = c;			// Добавляем символ
			u->inHead = newlast;	// Устанавливаем новый указатель головы
			// Посылаем семофор
			xPrio = pdFALSE;
			xPrio = xSemaphoreGiveFromISR(u->xSemaHandl, &xPrio);
			if (xPrio == pdTRUE)
				portEND_SWITCHING_ISR(&xPrio);
		}
	}
	/* Передача. */
	if (READ_REG(u->port->SR) & USART_SR_TC) {

		if (u->outHead != u->outTail) {
			/* Шлём очередной байт. */
			u->port->DR = *u->outTail;
			if (++u->outTail >= u->outBuf + UART_OUTBUFSZ)
				u->outTail = u->outBuf;
		} else {
			/* Нет данных для передачи - сброс прерывания. */
			CLEAR_BIT(u->port->SR, USART_SR_TC);
			//passive = 0;
		}
	}
}

/* Инициализация драйвера UART
 * Параметры: указатель на структуру uart, указатель на структуру UART_HAL
 * возвращаемое значение: В случае успешной инициализации pdPASS, иначк pdFAIL */

BaseType_t uart_init(uart_t *u, USART_TypeDef *port, uint32_t baud) {
	BaseType_t result = pdPASS;
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_ClockInitTypeDef USART_ClockInitStructure;

	u->port = port;
	u->inHead = u->inBuf;
	u->inTail = u->inBuf;
	u->outHead = u->outBuf;
	u->outTail = u->outBuf;

	u->config.USART_BaudRate = baud;
	u->config.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	u->config.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	u->config.USART_Parity = USART_Parity_No;
	u->config.USART_StopBits = USART_StopBits_1;
	u->config.USART_WordLength = USART_WordLength_8b;
	uint32_t irq;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	if (u->port == USART1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

		//USART_OverSampling8Cmd(u->port, ENABLE);
		GPIO_InitStruct.GPIO_Pin = (1 << UART1_RxPin);
		GPIO_Init(UART1_RxPort, &GPIO_InitStruct);
		GPIO_InitStruct.GPIO_Pin = (1 << UART1_TxPin);
		GPIO_Init(UART1_TxPort, &GPIO_InitStruct);
		GPIO_PinAFConfig(UART1_TxPort, UART1_TxPin,
		GPIO_AF_USART1);
		GPIO_PinAFConfig(UART1_RxPort, UART1_RxPin,
		GPIO_AF_USART1);

		irq = USART1_IRQn;
		uarts[0] = u;
	}
	if (u->port == USART2) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

		//USART_OverSampling8Cmd(u->port, ENABLE);
		GPIO_InitStruct.GPIO_Pin = (1 << UART2_RxPin);
		GPIO_Init(UART2_RxPort, &GPIO_InitStruct);
		GPIO_InitStruct.GPIO_Pin = (1 << UART2_TxPin);
		GPIO_Init(UART2_TxPort, &GPIO_InitStruct);
		GPIO_PinAFConfig(UART2_TxPort, UART2_TxPin,
		GPIO_AF_USART2);
		GPIO_PinAFConfig(UART2_RxPort, UART2_RxPin,
		GPIO_AF_USART2);

		irq = USART2_IRQn;
		uarts[1] = u;
	}
	if (u->port == USART3) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

		//USART_OverSampling8Cmd(u->port, ENABLE);
		GPIO_InitStruct.GPIO_Pin = (1 << UART3_RxPin);
		GPIO_Init(UART3_RxPort, &GPIO_InitStruct);
		GPIO_InitStruct.GPIO_Pin = (1 << UART3_TxPin);
		GPIO_Init(UART3_TxPort, &GPIO_InitStruct);
		GPIO_PinAFConfig(UART3_TxPort, UART3_TxPin,
		GPIO_AF_USART3);
		GPIO_PinAFConfig(UART3_RxPort, UART3_RxPin,
		GPIO_AF_USART3);

		irq = USART3_IRQn;
		uarts[2] = u;
	}
	if (u->port == USART6) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

		//USART_OverSampling8Cmd(u->port, ENABLE);
		GPIO_InitStruct.GPIO_Pin = (1 << UART6_RxPin);
		GPIO_Init(UART6_RxPort, &GPIO_InitStruct);
		GPIO_InitStruct.GPIO_Pin = (1 << UART6_TxPin);
		GPIO_Init(UART6_TxPort, &GPIO_InitStruct);
		GPIO_PinAFConfig(UART6_TxPort, UART6_TxPin,
		GPIO_AF_USART6);
		GPIO_PinAFConfig(UART6_RxPort, UART6_RxPin,
		GPIO_AF_USART6);

		irq = USART6_IRQn;
		uarts[5] = u;
	}
	if (u->port == UART4) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

		//USART_OverSampling8Cmd(u->port, ENABLE);
		GPIO_InitStruct.GPIO_Pin = (1 << UART4_RxPin);
		GPIO_Init(UART4_RxPort, &GPIO_InitStruct);
		GPIO_InitStruct.GPIO_Pin = (1 << UART4_TxPin);
		GPIO_Init(UART4_TxPort, &GPIO_InitStruct);
		GPIO_PinAFConfig(UART4_TxPort, UART4_TxPin,
		GPIO_AF_UART4);
		GPIO_PinAFConfig(UART4_RxPort, UART4_RxPin,
		GPIO_AF_UART4);

		irq = UART4_IRQn;
		uarts[3] = u;
	}
	if (u->port == UART5) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

		//USART_OverSampling8Cmd(u->port, ENABLE);
		GPIO_InitStruct.GPIO_Pin = (1 << UART5_RxPin);
		GPIO_Init(UART5_RxPort, &GPIO_InitStruct);
		GPIO_InitStruct.GPIO_Pin = (1 << UART5_TxPin);
		GPIO_Init(UART5_TxPort, &GPIO_InitStruct);
		GPIO_PinAFConfig(UART5_TxPort, UART5_TxPin,
		GPIO_AF_UART5);
		GPIO_PinAFConfig(UART5_RxPort, UART5_RxPin,
		GPIO_AF_UART5);

		irq = UART5_IRQn;
		uarts[4] = u;
	}

	USART_DeInit(u->port);
	USART_ClockStructInit(&USART_ClockInitStructure);
	USART_ClockInit(u->port, &USART_ClockInitStructure);
	USART_Cmd(u->port, ENABLE);
	USART_Init(u->port, &u->config);
	USART_ITConfig(u->port, USART_IT_TC, ENABLE);
	USART_ITConfig(u->port, USART_IT_RXNE, ENABLE);
	NVIC_SetPriority(irq, 5);
	NVIC_EnableIRQ(irq);
	//osSemaphoreDef(myCountSem);
	u->xSemaHandl = xSemaphoreCreateCounting(32, 0);
	//osSemaphoreCreate(osSemaphore(myCountSem), 32);

	if (u->xSemaHandl == NULL) {
		result = pdFAIL;
	}
	return result;
}

/* Чтение байта из буфера UART
 * Параметр: указатель на структуру uart_t
 * Возвращаемое значение: прочитанный байт */
char GetCharUart(uart_t *u) {
	xSemaphoreTake(u->xSemaHandl, portMAX_DELAY);// ожидаем появление байта в буфере
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
		newlast = u->outBuf;			// то приравниваем его началу буфера
	/* Если нет места в буфере - теряем данные. */
	if (u->outTail != newlast) {// Если новый указатель не равен хвосту, от куда читаются данные
		*u->outHead = c;			// Добавляем символ
		u->outHead = newlast;	// Устанавливаем новый указатель головы
		/* Если передатчик свободный, то посылаем байт */
		if (u->port->SR & USART_SR_TXE) {
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

