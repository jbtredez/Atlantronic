#ifndef AX12_H
#define AX12_H

//! @file ax12.h
//! @brief Gestion AX12
//! @author Jean-Baptiste Trédez

#include "io/usart.h"

void ax12_ping(uint8_t id);
uint8_t ax12_read8(uint8_t id, uint8_t offset);
uint16_t ax12_read16(uint8_t id, uint8_t offset);
void ax12_write8(uint8_t id, uint8_t offset, uint8_t data);
void ax12_write16(uint8_t id, uint8_t offset, uint16_t data);
void ax12_action(uint8_t id);
void ax12_reset(uint8_t id);

#define AX12_ID              0x03
#define AX12_LED             0x19
#define AX12_GOAL_POSITION   0x1e
#define AX12_MOVING_SPEED    0x20

// masques
#define AX12_MAX_MOVING_SPEED    0x3ff
#define AX12_MAX_GOAL_POSITION   0x3ff

void ax12_set_led(uint8_t id, uint8_t on);
void ax12_set_moving_speed(uint8_t id, uint16_t speed);
void ax12_set_goal_position(uint8_t id, uint16_t goal);
void ax12_set_id(uint8_t old_id, uint8_t id);

#endif
