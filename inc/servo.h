/*
 * servo.h
 *
 *  Created on: 05.12.2020
 *      Author: User
 */

#ifndef SERVO_H_
#define SERVO_H_

#include <stdint.h>

#define POS_MIN 0		//[stopnie/10]
#define POS_MAX 1800	//[stopnie/10]
#define PWM_MIN 500		//[us]
#define PWM_MAX 2500	//[us]

#define STEP ((1000 * (PWM_MAX - PWM_MIN)) / (POS_MAX - POS_MIN))

void set_position(uint16_t position, uint8_t mode);

#endif /* SERVO_H_ */
