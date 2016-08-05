/*
 * crc_calc.h
 *
 *  Created on: 06 мая 2015 г.
 *      Author: Yacinin.YuV
 */

#ifndef CRC_CALC_H_
#define CRC_CALC_H_

#include "stm32f4xx.h"

//void CreateTable(void);
unsigned short CalcCRC(unsigned char *Tlg, unsigned int N);

uint32_t code_message(uint8_t *src, uint8_t *dest, uint32_t size);
uint32_t decode_message(uint8_t *buf, uint32_t size);

#endif /* CRC_CALC_H_ */
