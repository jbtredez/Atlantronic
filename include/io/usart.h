#ifndef USART_H
#define USART_H

//! @file usart.h
//! @brief Usart
//! @author Jean-Baptiste Trédez

#include "cpu/cpu.h"
#include "FreeRTOS.h"
#include "task.h"

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

void usart_set_read_dma_buffer(unsigned char* buf);
void usart_set_read_dma_size(uint16_t size);

//! @return 0 si tout va bien, -1 en cas d'échec de la lecture
int8_t usart_wait_read();

void usart_set_write_dma_buffer(unsigned char* buf);
void usart_send_dma_buffer(uint16_t size);

#endif
