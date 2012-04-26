#ifndef AX12_H
#define AX12_H

//! @file ax12.h
//! @brief Gestion AX12
//! @author Atlantronic

#include "kernel/driver/usart.h"
#include "ax12_id.h"

#define AX12_ID                   0x03
#define AX12_TORQUE_LIMIT_EEPROM  0x0e
#define AX12_ALARM_SHUTDOWN       0x12
#define AX12_TORQUE_ENABLE        0x18
#define AX12_LED                  0x19
#define AX12_GOAL_POSITION        0x1e
#define AX12_MOVING_SPEED         0x20
#define AX12_TORQUE_LIMIT         0x22
#define AX12_PRESENT_POSITION     0x24

// masques
#define AX12_MAX_MOVING_SPEED    0x3ff
#define AX12_MAX_TORQUE_LIMIT    0x3ff

#define AX12_CMD_SCAN                    0x01
#define AX12_CMD_SET_ID                  0x02
#define AX12_CMD_SET_GOAL_POSITION       0x03

struct ax12_cmd_param
{
	uint8_t cmd_id;         //!< id de la commande
	uint8_t id;             //!< id de l'ax12
	int32_t param;          //!< parametre
};

#define ERR_AX12_SEND_CHECK             0x80
#define ERR_AX12_PROTO                  0x81
#define ERR_AX12_CHECKSUM               0x82

#define AX12_INPUT_VOLTAGE_ERROR_MASK    0x01
#define AX12_ANGLE_LIMIT_ERROR_MASK      0x02
#define AX12_OVERHEATING_ERROR_MASK      0x04
#define AX12_RANGE_ERROR_MASK            0x08
#define AX12_CHECKSUM_ERROR_MASK         0x10
#define AX12_OVERLOAD_ERROR_MASK         0x20
#define AX12_INSTRUCTION_ERROR_MASK      0x40

struct ax12_error
{
	//!< bit 7 à 1 : ERR_AX12_SEND_CHECK, ERR_AX12_PROTO ou ERR_AX12_CHECKSUM
	//!< bit 7 à 0 : erreur usart sur les 4 bits de poids faible
	uint8_t transmit_error;
	//!< erreur interne ax12 (champ de bit)
	uint8_t internal_error;
};

//!< affichage d'une erreur ax12
void ax12_print_error(int id, struct ax12_error err);

//!< configuration des limites d'un ax12 (limites mécaniques par exemple pour ne pas forcer)
void ax12_set_goal_limit(uint8_t id, uint16_t min, uint16_t max);

//!< ping un ax12.
struct ax12_error ax12_ping(uint8_t id);
struct ax12_error ax12_action(uint8_t id);
struct ax12_error ax12_reset(uint8_t id);

struct ax12_error ax12_set_led(uint8_t id, uint8_t on);
struct ax12_error ax12_set_moving_speed(uint8_t id, uint16_t speed);

//!< deplacement de l'ax12 vers l'angle alpha (en 2^-26 tours)
struct ax12_error ax12_set_goal_position(uint8_t id, int32_t alpha);

struct ax12_error ax12_set_torque_limit(uint8_t id, uint16_t torque_limit);
struct ax12_error ax12_set_torque_limit_eeprom(uint8_t id, uint16_t torque_limit);
struct ax12_error ax12_set_torque_enable(uint8_t id, uint8_t enable);

uint16_t ax12_get_position(uint8_t id, struct ax12_error* error);

struct ax12_error ax12_write8(uint8_t id, uint8_t offset, uint8_t data);

#endif
