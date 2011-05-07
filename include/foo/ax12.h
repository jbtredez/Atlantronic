#ifndef AX12_H
#define AX12_H

//! @file ax12.h
//! @brief Gestion AX12
//! @author Atlantronic

#include "kernel/driver/usart.h"

void ax12_ping(uint8_t id);
void ax12_action(uint8_t id);
void ax12_reset(uint8_t id);

#define AX12_ID                   0x03
#define AX12_TORQUE_LIMIT_EEPROM  0x0e
#define AX12_TORQUE_ENABLE        0x18
#define AX12_LED                  0x19
#define AX12_GOAL_POSITION        0x1e
#define AX12_MOVING_SPEED         0x20
#define AX12_TORQUE_LIMIT         0x22

// masques
#define AX12_MAX_MOVING_SPEED    0x3ff
#define AX12_MAX_GOAL_POSITION   0x3ff
#define AX12_MAX_GOAL_POSITION   0x3ff
#define AX12_MAX_TORQUE_LIMIT    0x3ff

void ax12_set_led(uint8_t id, uint8_t on);
void ax12_set_moving_speed(uint8_t id, uint16_t speed);
void ax12_set_goal_position(uint8_t id, uint16_t goal);
void ax12_set_id(uint8_t old_id, uint8_t id);
void ax12_set_torque_limit(uint8_t id, uint16_t torque_limit);
void ax12_set_torque_limit_eeprom(uint8_t id, uint16_t torque_limit);
void ax12_set_torque_enable(uint8_t id, uint8_t enable);

#endif
