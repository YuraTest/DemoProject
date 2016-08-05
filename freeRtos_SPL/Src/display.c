/*
 * display.c
 *
 *  Created on: 4 авг. 2016 г.
 *      Author: yurock
 */

#include "display.h"

static DisplayData_t displayData;
static display_t *displays[6];

static void DMA_Hendler(display_t* d);
static void taskDisplayRx(void *arg);

BaseType_t display_init(display_t *d, uart_t *u, USART_TypeDef *port) {
	BaseType_t xRes = pdPASS;
	uint32_t irq;
	d->uart = u;
	d->data = &displayData;

	if (uart_init(u, port, 115200) != pdPASS)
		xRes = pdFAIL;
	USART_Cmd(u->port, DISABLE);
	USART_ITConfig(u->port, USART_IT_TC, DISABLE);

	if (u->port == USART1) {
		d->stream = DMA2_Stream7;
		d->chanlDMA = DMA_Channel_4;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		irq = DMA2_Stream7_IRQn;
		displays[0] = d;
	}
	if (u->port == USART2) {
		d->stream = DMA1_Stream6;
		d->chanlDMA = DMA_Channel_4;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		irq = DMA1_Stream6_IRQn;
		displays[1] = d;
	}
	if (u->port == USART3) {
		d->stream = DMA1_Stream3;
		d->chanlDMA = DMA_Channel_4;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		irq = DMA1_Stream3_IRQn;
		displays[2] = d;
	}
	if (u->port == USART6) {
		d->stream = DMA2_Stream7;
		d->chanlDMA = DMA_Channel_5;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		irq = DMA2_Stream7_IRQn;
		displays[5] = d;
	}
	if (u->port == UART4) {
		d->stream = DMA1_Stream4;
		d->chanlDMA = DMA_Channel_4;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		irq = DMA1_Stream4_IRQn;
		displays[3] = d;
	}
	if (u->port == UART5) {
		d->stream = DMA1_Stream7;
		d->chanlDMA = DMA_Channel_4;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		irq = DMA1_Stream7_IRQn;
		displays[4] = d;
	}

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = (1 << DisplRstPin);
	GPIO_Init(DisplRstPort, &GPIO_InitStruct);
	GPIO_ResetBits(DisplRstPort, (1 << DisplRstPin));
	GPIO_InitStruct.GPIO_Pin = (1 << DisplWakeUpPin);
	GPIO_Init(DisplWakeUpPort, &GPIO_InitStruct);
	GPIO_ResetBits(DisplWakeUpPort, (1 << DisplWakeUpPin));
	vTaskDelay(1);
	GPIO_SetBits(DisplWakeUpPort, (1 << DisplWakeUpPin));
	vTaskDelay(50);
	GPIO_ResetBits(DisplWakeUpPort, (1 << DisplWakeUpPin));

	DMA_InitTypeDef DMA_InitStructure;

	DMA_Cmd(d->stream, DISABLE);
	DMA_DeInit(d->stream);

	DMA_InitStructure.DMA_Channel = d->chanlDMA;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &d->uart->port->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr =
			(uint32_t) &displayData.displayBuf[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = 10;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //DMA_MemoryDataSize_Word; // See: http://we.easyelectronics.ru/STM32/programmnyy-dekoder-mp3-perehod-na-platformu-stm32f407.html
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(d->stream, &DMA_InitStructure);
	//DMA_ITConfig(d->stream, DMA_IT_TC, ENABLE);
	//DMA_FlowControllerConfig(d->stream, DMA_FlowCtrl_Peripheral);

	/* Запуск передачи */
	//DMA_MemoryTargetConfig(d->stream, (uint32_t) &d->data->displayBuf[0], DMA_Memory_0);	//d->stream->M0AR = (uint32_t) &d->data->displayBuf[0];
	//DMA_SetCurrDataCounter(d->stream, (uint16_t)d->data->len);//d->stream->NDTR = d->data->len;
	//USART_DMACmd(d->uart, USART_DMAReq_Tx, ENABLE);
	//DMA_Cmd(d->stream, ENABLE);//d->stream->CR |= DMA_SxCR_EN;
	/* Запуск передачи */

	//DMA1_Channel4->CMAR = (uint32_t) &TxData[0];
	//DMA1_Channel4->CNDTR = size;
	//USART1->CR3 |= USART_DMAReq_Tx;
	//DMA1_Channel4->CCR |= DMA_CCR_EN;
	NVIC_SetPriority(irq, 5);
	NVIC_EnableIRQ(irq);
	DMA_ITConfig(d->stream, DMA_IT_TC, ENABLE);

	USART_Cmd(u->port, ENABLE);
	d->xMutRx = xSemaphoreCreateMutex();
	if (d->xMutRx == NULL) {
		xRes = pdFAIL;
	}
	if (xTaskCreate(taskDisplayRx, "Displ_Rx", 500, (void * )u, 1,
			NULL) != pdPASS)
		xRes = pdFAIL;

	return xRes;
}

/* Usat1
 * Usat6
 *  DMA2_Stream7
 *  DMA2_Stream7 */
void DMA2_Stream7_IRQHandler() {
	if (displays[0] != NULL) {
		if (DMA_GetITStatus(displays[0]->stream, DMA_IT_TCIF7) == SET) {
			DMA_ClearITPendingBit(displays[0]->stream, DMA_IT_TCIF7);
			DMA_Hendler(displays[0]);
		}
	}
	if (displays[5] != NULL) {
		if (DMA_GetITStatus(displays[5]->stream, DMA_IT_TCIF7) == SET) {
			DMA_ClearITPendingBit(displays[5]->stream, DMA_IT_TCIF7);
			DMA_Hendler(displays[5]);
		}
	}
}

/* Usat2
 *  DMA1_Stream6 */
void DMA1_Stream6_IRQHandler() {
	if (displays[1] != NULL) {
		if (DMA_GetITStatus(displays[1]->stream, DMA_IT_TCIF6) == SET) {
			DMA_ClearITPendingBit(displays[1]->stream, DMA_IT_TCIF6);
			DMA_Hendler(displays[1]);
		}
	}
}

/* Usat3
 *  DMA1_Stream3 */
void DMA1_Stream3_IRQHandler() {
	if (displays[2] != NULL) {
		if (DMA_GetITStatus(displays[2]->stream, DMA_IT_TCIF3) == SET) {
			DMA_ClearITPendingBit(displays[2]->stream, DMA_IT_TCIF3);
			DMA_Hendler(displays[2]);
		}
	}
}

/* Usat4
 *  DMA1_Stream4 */
void DMA1_Stream4_IRQHandler() {
	if (displays[3] != NULL) {
		if (DMA_GetITStatus(displays[3]->stream, DMA_IT_TCIF4) == SET) {
			DMA_ClearITPendingBit(displays[3]->stream, DMA_IT_TCIF4);
			DMA_Hendler(displays[3]);
		}
	}
}

/* Usat5
 *  DMA1_Stream7 */
void DMA1_Stream7_IRQHandler() {
	if (displays[4] != NULL) {
		if (DMA_GetITStatus(displays[4]->stream, DMA_IT_TCIF7) == SET) {
			DMA_ClearITPendingBit(displays[4]->stream, DMA_IT_TCIF7);
			DMA_Hendler(displays[4]);
		}
	}
}

/* Usat6
 *  DMA2_Stream7 */
/*
 void DMA2_Stream7_IRQHandler(){

 }*/

void DMA_Hendler(display_t* d) {
	static portBASE_TYPE xPrio;
	USART_DMACmd(d->uart->port, USART_DMAReq_Tx, DISABLE);
	DMA_Cmd(d->stream, DISABLE);
	/* Послать мьютекс */
	xPrio = pdFALSE;
	xPrio = xSemaphoreGiveFromISR(d->uart->xReciever, &xPrio);
	if (xPrio == pdTRUE)
		portEND_SWITCHING_ISR(&xPrio);
}

extern uart_t uart1;
void taskDisplayRx(void *arg) {
	display_t *display = (display_t *) arg;
	portCHAR ch1, ch2;
	for (;;) {
		ch1 = GetCharUart(display->uart);
		ch2 = GetCharUart(display->uart);
		if ((ch1 == 'O') && (ch2 == 'K')) {
			PutStringUart(&uart1, "OK");
			xSemaphoreGive(display->xMutRx);
		}
	}
}

/* ------------------------------------------------------ */

extern xQueueHandle xQueueDisplay;
static tasksMessage_t xMsgIn;

/* Command			0xA5	0xXX 0xXX	0xXX		0xXX..		0xCC 0x33 0xC3 0x3C		0xXX  */
/*					head	lent		Command		Data		End frame				Parity */

static const char HANDSHAKE[] = { 0xA5, 0x00, 0x09, DISPAY_CMD_HANDSHAKE, 0xCC,
		0x33, 0xC3, 0x3C, 0xAC };
static const char SET_NAND[] = { 0xA5, 0x00, 0x09, DISPAY_CMD_SET_STORAGE, STORAGE_NAND, 0xCC, 0x33, 0xC3,
		0x3C, 0xA8 }; /* Set Nand Flash */
static const char SET_SD[] = { 0xA5, 0x00, 0x09, DISPAY_CMD_SET_STORAGE, STORAGE_SD, 0xCC, 0x33, 0xC3,
		0x3C, 0xA9 }; /* Set Sd Flash */
static const char CLEAR[] = { 0xA5, 0x00, 0x09, DISPAY_CMD_DRAW_CLEAR, 0xCC,
		0x33, 0xC3, 0x3C, 0x82 };

void taskDisplay(void *arg) {
	display_t *display = (display_t *) arg;

	xSemaphoreTake(display->uart->xMutTx, portMAX_DELAY);
	display->data->len = sizeof(SET_NAND);
	memcpy((char*) display->data->displayBuf, (char*) SET_NAND,
			display->data->len);
	DMA_MemoryTargetConfig(display->stream,
			(uint32_t) &display->data->displayBuf[0], DMA_Memory_0);//display->stream->M0AR = (uint32_t) &display->data->displayBuf[0];
	DMA_SetCurrDataCounter(display->stream, (uint16_t) display->data->len);	//display->stream->NDTR = display->data->len;
	USART_DMACmd(display->uart->port, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(display->stream, ENABLE);
	xSemaphoreTake(display->xMutRx, 3000);

	xSemaphoreGive(display->uart->xMutTx);
	portCHAR cmd;

	for (;;) {
		xQueueReceive(xQueueDisplay, &xMsgIn, portMAX_DELAY);
		cmd = xMsgIn.data[0];
		switch (cmd) {
		case DISPAY_CMD_HANDSHAKE:
			display->data->len = sizeof(HANDSHAKE);
			memcpy((char*) display->data->displayBuf, (char*) HANDSHAKE,
					display->data->len);
			break;
		default:
			break;
		}
		if (display->data->len != 0) {
			xSemaphoreTake(display->uart->xMutTx, portMAX_DELAY);
			DMA_MemoryTargetConfig(display->stream,
					(uint32_t) &display->data->displayBuf[0], DMA_Memory_0);//display->stream->M0AR = (uint32_t) &display->data->displayBuf[0];
			DMA_SetCurrDataCounter(display->stream,
					(uint16_t) display->data->len);	//display->stream->NDTR = display->data->len;
			USART_DMACmd(display->uart->port, USART_DMAReq_Tx, ENABLE);
			DMA_Cmd(display->stream, ENABLE);
			xSemaphoreTake(display->xMutRx, 3000);
			xSemaphoreGive(display->uart->xMutTx);
			display->data->len = 0;
		}
		/* display->data->len = sizeof(SET_NAND);
		 memcpy((char*) display->data->displayBuf, (char*) SET_NAND,
		 display->data->len);
		 DMA_MemoryTargetConfig(display->stream,
		 (uint32_t) &display->data->displayBuf[0], DMA_Memory_0);//display->stream->M0AR = (uint32_t) &display->data->displayBuf[0];
		 DMA_SetCurrDataCounter(display->stream, (uint16_t) display->data->len);	//display->stream->NDTR = display->data->len;
		 USART_DMACmd(display->uart->port, USART_DMAReq_Tx, ENABLE);
		 DMA_Cmd(display->stream, ENABLE);
		 vTaskDelay(1000); */
	}

}
