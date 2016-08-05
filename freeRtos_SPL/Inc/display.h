/*
 * display.h
 *
 *  Created on: 4 авг. 2016 г.
 *      Author: yurock
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include "main.h"
#include "uart.h"


/* Комманды дисплею */
/* System control */
#define DISPAY_CMD_HANDSHAKE			0x00	/* Опрос Дисплея */
#define DISPAY_CMD_SET_BAUD				0x01	/* Установка скорости UART дисплея */
#define DISPAY_CMD_GET_BAUD				0x02	/* Получить текущую скорость UART */
#define DISPAY_CMD_GET_STORAGE			0x06	/* Получить значение используемой памяти */
#define DISPAY_CMD_SET_STORAGE			0x07	/* Установить значение используемой памяти */
#define DISPAY_CMD_SLEEP_MODE			0x08	/* Перевод в режим sleep */
#define DISPAY_CMD_REFRESH				0x0A	/* Обновление дисплея */
#define DISPAY_CMD_GET_DIRECTION		0x0C	/* Получить значение переворота */
#define DISPAY_CMD_SET_DIRECTION		0x0D	/* Установить значение переворота */
#define DISPAY_CMD_IMPORT_FONT			0x0E	/* Импортировать шрифт */
#define DISPAY_CMD_IMPORT_IMAGE			0x0F	/* Импортировать изображение */

/* Display paramrter configuration */
#define DISPAY_CMD_SET_COLOR			0x10	/* Установка уветов */
#define DISPAY_CMD_GET_COLOR			0x11	/* Получить текущие установки цвета */
#define DISPAY_CMD_GET_EN_FONT			0x1C	/* Получить размер английских символов */
#define DISPAY_CMD_GET_CN_FONT			0x1D	/* Получить размер китайских символов */
#define DISPAY_CMD_SET_EN_FONT			0x1E	/* Установить размер английских символов */
#define DISPAY_CMD_SET_CN_FONT			0x1F	/* Установить размер китайских символов */

/* Basic drawing */
#define DISPAY_CMD_DRAW_POINT			0x20
#define DISPAY_CMD_DRAW_LINE			0x22
#define DISPAY_CMD_DRAW_FILL_RECTANGLE	0x24
#define DISPAY_CMD_DRAW_RECTANGLE		0x25
#define DISPAY_CMD_DRAW_CIRCLE			0x26
#define DISPAY_CMD_DRAW_FILL_CIRCLE		0x27
#define DISPAY_CMD_DRAW_TRIANGLE		0x28
#define DISPAY_CMD_DRAW_FILL_TRIANGLE	0x29
#define DISPAY_CMD_DRAW_CLEAR			0x2E
#define DISPAY_CMD_DRAW_TEXT			0x30
#define DISPAY_CMD_DRAW_PICTURES		0x70


typedef struct {
	char displayBuf[1035];
	uint32_t len;
} DisplayData_t;

typedef struct{
	uart_t *uart;
	DMA_Stream_TypeDef *stream;
	uint32_t chanlDMA;
	DisplayData_t *data;
	xSemaphoreHandle xMutRx;
} display_t;

BaseType_t display_init(display_t *d, uart_t *u, USART_TypeDef *port);




void taskDisplay(void *arg);

#endif /* INC_DISPLAY_H_ */
