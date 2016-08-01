/*
 * perirh_init.c
 *
 *  Created on: 31 июл. 2016 г.
 *      Author: yurock
 */
#include "periph_init.h"

void initGPIO(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	/* Leds */

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GreenLed_Pin | OrangeLed_Pin | RedLed_Pin
			| BlueLed_Pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Low_Speed;
	GPIO_Init(GPIOD, &GPIO_InitStruct);


	/* UART1 */


}
