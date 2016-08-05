/*
 * mainTask.c
 *
 *  Created on: 5 авг. 2016 г.
 *      Author: yurock
 */
#include "main.h"
#include "display.h"

extern xQueueHandle xQueueMain;
extern xQueueHandle xQueueDisplay;
static tasksMessage_t xMsgIn;
static tasksMessage_t xMsgOut;

void vTaskProtokol(void* arg){
	portCHAR cmd;

	for(;;){
		xQueueReceive(xQueueMain, &xMsgIn, portMAX_DELAY);
		cmd = xMsgIn.data[0];
		switch(cmd){
		case DISPAY_CMD:
			memcpy(&xMsgOut.data[0], &xMsgIn.data[1], xMsgIn.size -1);
			xMsgOut.size = xMsgIn.size - 1;
			//xMsgOut = xMsgIn;
			xQueueSend(xQueueDisplay, &xMsgOut, portMAX_DELAY);
			break;
		default:
			break;
		}
	}

}
