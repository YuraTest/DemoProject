/*
 * main.h
 *
 *  Created on: 28 июл. 2016 г.
 *      Author: yurock
 */

#ifndef INC_MAIN_H_
#define INC_MAIN_H_

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>		// для memcpy


#define GreenLed_Pin GPIO_Pin_12
#define GreenLed_GPIO_Port GPIOD
#define OrangeLed_Pin GPIO_Pin_13
#define OrangeLed_GPIO_Port GPIOD
#define RedLed_Pin GPIO_Pin_14
#define RedLed_GPIO_Port GPIOD
#define BlueLed_Pin GPIO_Pin_15
#define BlueLed_GPIO_Port GPIOD

/*  UART1 */
#define UART1_TxPin			GPIO_PinSource6
#define UART1_TxPort		GPIOB
#define UART1_RxPin			GPIO_PinSource7
#define UART1_RxPort		GPIOB

/*  UART2 */
#define UART2_TxPin			GPIO_PinSource6
#define UART2_TxPort		GPIOB
#define UART2_RxPin			GPIO_PinSource7
#define UART2_RxPort		GPIOB


/*  UART3 */
#define UART3_TxPin			GPIO_PinSource6
#define UART3_TxPort		GPIOB
#define UART3_RxPin			GPIO_PinSource7
#define UART3_RxPort		GPIOB

/*  UART4 */
#define UART4_TxPin			GPIO_PinSource6
#define UART4_TxPort		GPIOB
#define UART4_RxPin			GPIO_PinSource7
#define UART4_RxPort		GPIOB

/*  UART5 */
#define UART5_TxPin			GPIO_PinSource6
#define UART5_TxPort		GPIOB
#define UART5_RxPin			GPIO_PinSource7
#define UART5_RxPort		GPIOB

/*  UART6 */
#define UART6_TxPin			GPIO_PinSource6
#define UART6_TxPort		GPIOB
#define UART6_RxPin			GPIO_PinSource7
#define UART6_RxPort		GPIOB


/* Display Reset Pin */
#define DisplRstPin			GPIO_PinSource6
#define DisplRstPort		GPIOB

/* Display WakeUp Pin */
#define DisplWakeUpPin		GPIO_PinSource6
#define DisplWakeUpPort		GPIOB


#define DISPAY_CMD				0x30
#define SIZE_DATA_BUF			50

typedef struct{
	portSHORT taskId;
	portSHORT size;
	portCHAR data[SIZE_DATA_BUF];
} tasksMessage_t;

void SysTickHandler(void);

#endif /* INC_MAIN_H_ */
