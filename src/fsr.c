/*
 * fsr.c
 *
 *  Created on: 16.08.2021
 *      Author: User
 */

#include "fsr.h"

float volt2force(uint16_t voltage)
{
	if(voltage < 1500)
		return 0.06666 * voltage;
	else if(voltage < 2000)
		return 0.3 * voltage - 350;
	else if(voltage < 2250)
		return 0.6 * voltage - 950;
	else if(voltage < 2500)
		return 2.4 * voltage - 5000;
	else if(voltage < 2550)
		return 4.0 * voltage - 9000;
	else if(voltage < 2650)
		return 10.0 * voltage - 24300;
	else
		return 16.0 * voltage - 40200;
}
