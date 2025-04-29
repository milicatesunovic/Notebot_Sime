/*
 * motori.c
 *
 *  Created on: Apr 2, 2025
 *      Author: KORISNIK
 */
#include "motori.h"

extern TIM_HandleTypeDef htim16; // Eksterni tajmer, definisan u main.c
extern TIM_HandleTypeDef htim15;


void motor1_init(uint8_t smijer, uint16_t faktor_ispune) {

	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

	TIM16->CCR1 = faktor_ispune;

	if (smijer == 1) {
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
	} else if (smijer == 2) {
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
	}


}

void motor2_init(uint8_t smijer, uint16_t faktor_ispune) {

	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

	TIM15->CCR1=faktor_ispune;

	if (smijer == 1) {
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
	} else if (smijer == 2) {
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
	}


}

