#ifndef USART_H
#define USART_H

//! @file usart.h
//! @brief Usart
//! @author Atlantronic

#include "kernel/cpu/cpu.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"

#ifdef __cplusplus
extern "C" {
#endif

enum usart_id
{
	USART1_FULL_DUPLEX = 0,
	USART2_FULL_DUPLEX,
	USART3_FULL_DUPLEX,
	UART4_FULL_DUPLEX,
	UART5_HALF_DUPLEX,
	//USART6_HALF_DUPLEX
};

#define USART_MAX_DEVICE    (UART5_HALF_DUPLEX+1)

#define ERR_USART_TIMEOUT       0x01
#define ERR_USART_READ_SR_FE    0x02
#define ERR_USART_READ_SR_NE    0x04
#define ERR_USART_READ_SR_ORE   0x08

//!< ouverture usart
int usart_open(enum usart_id id, uint32_t frequency);

void usart_set_read_dma_buffer(enum usart_id id, unsigned char* buf);
void usart_set_read_dma_size(enum usart_id id, uint16_t size);

//! @return 0 si tout va bien, code d'erreur (champ de bit) sinon :
//! ERR_USART_TIMEOUT     : timeout
//! ERR_USART_READ_SR_FE  : desynchro, bruit ou octet "break"
//! ERR_USART_READ_SR_NE  : bruit
//! ERR_USART_READ_SR_ORE : overrun
uint32_t usart_wait_read(enum usart_id id, portTickType timeout);

void usart_set_write_dma_buffer(enum usart_id id, unsigned char* buf);
void usart_send_dma_buffer(enum usart_id id, uint16_t size);

void usart_set_frequency(enum usart_id id, uint32_t frequency);

#ifdef __cplusplus
}
#endif

#endif
