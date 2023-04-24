/*
 * send.h
 *
 *  Created on: 09.06.2021
 *      Author: User
 */

#ifndef SEND_H_
#define SEND_H_

#include <stdio.h>
#include <stdint.h>

void send_char(char c);				//funkcja wysylajaca pojedynczy znak do PC

void send_string(const char* s);	//funkcja do wysylania ciagu znakow (string) do PC

#endif /* SEND_H_ */
