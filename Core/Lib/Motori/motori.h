/*
 * motori.h
 *
 *  Created on: Apr 2, 2025
 *      Author: KORISNIK
 */

#ifndef LIB_MOTORI_MOTORI_H_
#define LIB_MOTORI_MOTORI_H_
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_gpio.h"

// Deklaracija funkcije
void motor1_init(uint8_t smijer, uint16_t faktor_ispune);
void motor2_init(uint8_t smijer, uint16_t faktor_ispune);



#endif /* LIB_MOTORI_MOTORI_H_ */
