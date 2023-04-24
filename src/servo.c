/*
 * servo.c
 *
 *  Created on: 05.12.2020
 *      Author: User
 */

#include "servo.h"
#include "stm32f10x_tim.h"

void set_position(uint16_t position, uint8_t mode)
{
	uint16_t val; //zmienna na wartosc wypelnienia sygnalu PWM

	if(position > POS_MAX) //ograniczenie pozycji maksymalnej
		position = POS_MAX;
	else if(position < POS_MIN) //ograniczenie pozycji minimalnej
		position = POS_MIN;

	if(mode) //wybranie kierunku i obliczenie wypelnienia sygnalu PWM
		val = PWM_MIN + ((position - POS_MIN) * STEP) / 1000;
	else
		val = PWM_MAX - ((position - POS_MIN) * STEP) / 1000;

	TIM_SetCompare3(TIM4, val);	//zadanie wypelnienia PWM na rejestr
}
