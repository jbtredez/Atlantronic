#ifndef AX12_H
#define AX12_H

//! @file ax12.h
//! @brief Gestion AX12
//! @author Atlantronic

#include "kernel/driver/usart.h"
#include "ax12_id.h"

enum
{
	DYNAMIXEL_MODEL_NUMBER_L = 0,
	DYNAMIXEL_MODEL_NUMBER_H,
	DYNAMIXEL_FIRMWARE_VERSION,
	DYNAMIXEL_ID,
	DYNAMIXEL_BAUD_RATE,
	DYNAMIXEL_RETURN_DELAY_TIME,
	DYNAMIXEL_CW_ANGLE_LIMIT_L,
	DYNAMIXEL_CW_ANGLE_LIMIT_H,
	DYNAMIXEL_CCW_ANGLE_LIMIT_L,
	DYNAMIXEL_CCW_ANGLE_LIMIT_H,
	DYNAMIXEL_RESERVED1,
	DYNAMIXEL_HIGHEST_LIMIT_TEMPERATURE,
	DYNAMIXEL_LOWEST_LIMIT_VOLTAGE,
	DYNAMIXEL_HIGHEST_LIMIT_VOLTAGE,
	DYNAMIXEL_MAX_TORQUE_L,
	DYNAMIXEL_MAX_TORQUE_H,
	DYNAMIXEL_STATUS_RETURN_LEVEL,
	DYNAMIXEL_ALARM_LED,
	DYNAMIXEL_ALARM_SHUTDOWN,
	DYNAMIXEL_RESERVED2,
	DYNAMIXEL_DOWN_CALIBRATION_L,
	DYNAMIXEL_DOWN_CALIBRATION_H,
	DYNAMIXEL_UP_CALIBRATION_L,
	DYNAMIXEL_UP_CALIBRATION_H,
	DYNAMIXEL_TORQUE_ENABLE,
	DYNAMIXEL_LED,
	DYNAMIXEL_CW_COMPLIANCE_MARGIN,
	DYNAMIXEL_CCW_COMPLIANCE_MARGIN,
	DYNAMIXEL_CW_COMPLIANCE_SLOPE,
	DYNAMIXEL_CCW_COMPLIANCE_SLOPE,
	DYNAMIXEL_GOAL_POSITION_L,
	DYNAMIXEL_GOAL_POSITION_H,
	DYNAMIXEL_MOVING_SPEED_L,
	DYNAMIXEL_MOVING_SPEED_H,
	DYNAMIXEL_TORQUE_LIMIT_L,
	DYNAMIXEL_TORQUE_LIMIT_H,
	DYNAMIXEL_PRESENT_POSITION_L,
	DYNAMIXEL_PRESENT_POSITION_H,
	DYNAMIXEL_PRESENT_SPEED_L,
	DYNAMIXEL_PRESENT_SPEED_H,
	DYNAMIXEL_PRESENT_LOAD_L,
	DYNAMIXEL_PRESENT_LOAD_H,
	DYNAMIXEL_PRESENT_VOLTAGE,
	DYNAMIXEL_PRESENT_TEMPERATURE,
	DYNAMIXEL_REGISTRED_INSTRUCTION,
	DYNAMIXEL_RESERVED3,
	DYNAMIXEL_MOVING,
	DYNAMIXEL_LOCK,
	DYNAMIXEL_PUNCH_L,
	DYNAMIXEL_PUNCH_H
};

// masques
#define AX12_MAX_MOVING_SPEED    0x3ff
#define AX12_MAX_TORQUE_LIMIT    0x3ff

#define AX12_CMD_SCAN                    0x01
#define AX12_CMD_SET_ID                  0x02
#define AX12_CMD_SET_GOAL_POSITION       0x03
#define AX12_CMD_GET_POSITION            0x04

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

//!< angle donné en 2^-26 tours
int32_t ax12_get_position(uint8_t id, struct ax12_error* error);

struct ax12_error ax12_write8(uint8_t id, uint8_t offset, uint8_t data);

#endif
