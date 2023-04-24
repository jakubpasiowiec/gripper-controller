/*
 * send.c
 *
 *  Created on: 09.06.2021
 *      Author: User
 */

#include "send.h"
#include "stm32f10x_usart.h"


void send_char(char c)			//funkcja wysylajaca pojedynczy znak do PC
{
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, c);	//wysylanie znaku do PC poprzez USART
}

void send_string(const char* s)	//funkcja do wysylania ciagu znakow (string) do PC
{
	while (*s)
		send_char(*s++);
}

int __io_putchar(int c)			//funkcja wykonywana przy uzywaniu printf()
{
	if (c=='\n')
		send_char('\r');
	send_char(c);
	return c;
}
