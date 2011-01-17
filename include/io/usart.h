#ifndef USART_H
#define USART_H

//! @file usart.h
//! @brief Usart
//! @author Jean-Baptiste Trédez

#include "cpu/cpu.h"

void usart_write(unsigned char* buf, uint16_t size);

uint16_t usart_read(unsigned char* buf, uint16_t size);

#endif
