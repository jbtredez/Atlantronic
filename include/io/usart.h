#ifndef USART_H
#define USART_H

//! @file usart.h
//! @brief Usart
//! @author Jean-Baptiste Trédez

#include "cpu/cpu.h"

enum usart_id
{
	USART3_HALF_DUPLEX,
	USART2_RXTX
};

enum usart_frequency
{
	USART_1000000,
	USART_750000,
	USART_19200
};

void usart_open( enum usart_id id, enum usart_frequency frequency);

void usart_write(unsigned char* buf, uint16_t size);

uint16_t usart_read(unsigned char* buf, uint16_t size);

#endif
