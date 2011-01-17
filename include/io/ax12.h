#ifndef AX12_H
#define AX12_H

//! @file ax12.h
//! @brief Gestion AX12
//! @author Jean-Baptiste Trédez

#include "io/usart.h"

void ax12_ping(uint8_t id);
void ax12_read(uint8_t id, uint8_t start, uint8_t length);
void ax12_write8(uint8_t id, uint8_t offset, uint8_t data);
void ax12_write16(uint8_t id, uint8_t offset, uint16_t data);
void ax12_action(uint8_t id);
void ax12_reset(uint8_t id);

#endif
