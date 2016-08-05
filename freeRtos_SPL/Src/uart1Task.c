/*
 * uart1Task.c
 *
 *  Created on: 1 авг. 2016 г.
 *      Author: yurock
 */
#include "uart1Task.h"
#include "stm32_ub_fatfs.h"
#include "ff.h"
#include "sd.h"
#include "crc_calc.h"

/* ---------------------------------------------------------- */

//char textBuf[128];
extern xQueueHandle xQueueMain;

static uart_msg_t uart_rx_buf;

static void Usart1Rx(void * argument);
//static void CONSOLE_printFiles(uart_t *u);

/* ---------------------------------------------------------- */

BaseType_t initUartTask(uart_t *u) {
	BaseType_t result = pdPASS;
	result = xTaskCreate(Usart1Rx, "Uart1_Rx", 1000, (void * )u, 2, NULL);

	return result;
}

//const short * helloStr = "Тест УАРТ\n";
/* Usart1Rx function */
char TxData[32];
static portSHORT Curent_CRC16;
static tasksMessage_t xMsg;

void Usart1Rx(void * argument) {
	/* USER CODE BEGIN Usart1Rx */
	uart_t *u = (uart_t *) argument;
	portCHAR ch;
	//sprintf(TxData, helloStr, 0);
	//PutStringUart(u, TxData);

	sprintf(TxData, "Start Task for uart1\n");
	PutStringUart(u, TxData);

	/*	if (SDIO_isSDCardAccessible() == true) {
	 if (SDIO_mountSDCard() == true) {
	 sprintf(textBuf, "SDCard is mounted!\n");
	 PutStringUart(u, textBuf);
	 //getFreeSize(&uart1);
	 SDIO_scanFiles();
	 CONSOLE_printFiles(u);

	 } else {
	 sprintf(textBuf, "Mount SD Card Failed!\n\r");
	 PutStringUart(u, textBuf);
	 }

	 } else {
	 sprintf(textBuf, "No SD Card!\n\r");
	 PutStringUart(u, textBuf);
	 }*/
	UBaseType_t i;
	for (;;) {
		for (;;) {
			ch = GetCharUart(u);
			if (ch == 0x2) {	// Начало сообщения
				i = 0;
				break;
			}
			if (ch == 0x15) {
				//Transmit_buf(Task_struct->u);
				continue;
			}
			if (ch == 0x6) {
				continue;
			}
			if (ch == 0x00)
				continue;
		}
		while (i < SIZE_DATA_BUF - 1) {
			ch = GetCharUart(u);
			if (ch == 0x3)
				break;
			uart_rx_buf.data[i] = ch;
			i++;
		}
		uart_rx_buf.size = decode_message((uint8_t *) &uart_rx_buf.data, i);
		Curent_CRC16 = CalcCRC((unsigned char *) &uart_rx_buf.data,
				uart_rx_buf.size);
		memcpy((unsigned char *) xMsg.data, (unsigned char *) &uart_rx_buf.data,
				uart_rx_buf.size);
		xMsg.size = uart_rx_buf.size;
		//xOut_msg_RS232.task_id = TASK_CONSOLE;
		//xOut_msg_RS232.interface = UART_0;
		//xOut_msg_RS232.mutex_ptr = &Task_struct->mutexes[TASK_PROTOKOL];
		//xOut_msg_RS232.strim = (stream_t*) (Task_struct->u);
		//message_queue_put(Task_struct->tasks.RS232, &xOut_msg_RS232);
		if(xQueueSend(xQueueMain, &xMsg, 0) == pdFAIL)
			xQueueOverwrite(xQueueMain, &xMsg);

		//PutCharUart(u, ch);
	}
	/* USER CODE END Usart1Rx */
}

#ifdef qwe
extern char SDIO_currentPath[SDIO_CURRENT_PATH_MAX_LENGTH];
extern uint16_t SDIO_filesNum;
extern struct SDIO_fileStruct SDIO_files[SDIO_FILES_TO_VIEW_MAX];

void CONSOLE_printFiles(uart_t *u) {
	static char i;
	static char _consoleChar;
	static char _selectedFileIndex = 0;

	FATFS *fs;
	static DWORD fre_clust, fre_sect, tot_sect;
	//char buf[100];

	/* Get volume information and free clusters of drive 1 */
	FRESULT res = f_getfree("0:", &fre_clust, &fs);
	if (res == FR_OK) {
		/* Get total sectors and free sectors */
		tot_sect = (fs->n_fatent - 2) * fs->csize;
		fre_sect = fre_clust * fs->csize;

		/* Print the free space (assuming 512 bytes/sector) */
		sprintf(textBuf, "%10u KiB total drive space.\n%10u KiB available.\n\n",
				tot_sect / 2, fre_sect / 2);

		PutStringUart(u, textBuf);
		//vTaskDelay(10);
	} else {
		sprintf(textBuf, "res = %d\n\n", res);
		PutStringUart(u, textBuf);
	}

	sprintf(textBuf, "SD:%s\n", SDIO_currentPath);
	PutStringUart(u, textBuf);

	for (i = 0; i < SDIO_filesNum; i++) {
		//if (i == _selectedFileIndex)
		//	printf(CONSOLE_TEXT_COLOR_LIGHT_CYAN);
		//else
		//	printf(CONSOLE_TEXT_COLOR_DEFAUL);

		if (SDIO_files[i].isDirectory) {
			sprintf(textBuf, "dir: %s", SDIO_files[i].fileName);
			PutStringUart(u, textBuf);
			//printf("dir: %s", SDIO_files[i].fileName);
		} else {
			sprintf(textBuf, "fil: %s", SDIO_files[i].fileName);
			PutStringUart(u, textBuf);
			//printf("fil: %s", SDIO_files[i].fileName);
		}
		sprintf(textBuf, "\n");
		PutStringUart(u, textBuf);
		//printf("\n\r");
	}

	//printf(CONSOLE_HIDE_CURSOR);
}
#endif
