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
	USART3_FULL_DUPLEX,
	UART4_HALF_DUPLEX
};

enum usart_frequency
{
	USART_1000000,
	USART_750000,
	USART_500000,
	USART_250000,
	USART_115200,
	USART_57600,
	USART_19200
};

void usart_open( enum usart_id id, enum usart_frequency frequency);

void usart_set_read_dma_buffer(enum usart_id id, unsigned char* buf);
void usart_set_read_dma_size(enum usart_id id, uint16_t size);

//! @return 0 si tout va bien, code d'erreur sinon
uint32_t usart_wait_read(enum usart_id id, portTickType timeout);

void usart_set_write_dma_buffer(enum usart_id id, unsigned char* buf);
void usart_send_dma_buffer(enum usart_id id, uint16_t size);

#endif
