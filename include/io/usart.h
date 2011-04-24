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

void usart_open( enum usart_id id, uint32_t frequency);

void usart_set_read_dma_buffer(enum usart_id id, unsigned char* buf);
void usart_set_read_dma_size(enum usart_id id, uint16_t size);

//! @return 0 si tout va bien, code d'erreur sinon
uint32_t usart_wait_read(enum usart_id id, portTickType timeout);

void usart_set_write_dma_buffer(enum usart_id id, unsigned char* buf);
void usart_send_dma_buffer(enum usart_id id, uint16_t size);

#endif
