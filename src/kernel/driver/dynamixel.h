#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

//! @file dynamixel.h
//! @brief Gestion Dynamixel (ax12 et rx24)
//! @author Atlantronic

#include "kernel/driver/usart.h"
#include "dynamixel_id.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/queue.h"
#include "kernel/semphr.h"

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
#define DYNAMIXEL_MAX_MOVING_SPEED    0x3ff
#define DYNAMIXEL_MAX_TORQUE_LIMIT    0x3ff

#define DYNAMIXEL_CMD_SCAN                    0x01
#define DYNAMIXEL_CMD_SET_ID                  0x02
#define DYNAMIXEL_CMD_SET_GOAL_POSITION       0x03
#define DYNAMIXEL_CMD_GET_POSITION            0x04

#define DYNAMIXEL_TYPE_AX12         12
#define DYNAMIXEL_TYPE_RX24         24

struct dynamixel_cmd_param
{
	uint8_t type;           //!< type de dynamixel (ax12 ou rx24)
	uint8_t cmd_id;         //!< id de la commande
	uint8_t id;             //!< id du dynamixel
	int32_t param;          //!< parametre
};

#define ERR_DYNAMIXEL_SEND_CHECK             0x80
#define ERR_DYNAMIXEL_PROTO                  0x81
#define ERR_DYNAMIXEL_CHECKSUM               0x82

#define DYNAMIXEL_INPUT_VOLTAGE_ERROR_MASK    0x01
#define DYNAMIXEL_ANGLE_LIMIT_ERROR_MASK      0x02
#define DYNAMIXEL_OVERHEATING_ERROR_MASK      0x04
#define DYNAMIXEL_RANGE_ERROR_MASK            0x08
#define DYNAMIXEL_CHECKSUM_ERROR_MASK         0x10
#define DYNAMIXEL_OVERLOAD_ERROR_MASK         0x20
#define DYNAMIXEL_INSTRUCTION_ERROR_MASK      0x40

#define DYNAMIXEL_ARG_MAX                 3

struct dynamixel_error
{
	//!< bit 7 à 4 : ERR_DYNAMIXEL_SEND_CHECK, ERR_DYNAMIXEL_PROTO ou ERR_DYNAMIXEL_CHECKSUM
	//!< bit 4 à 0 : erreur usart sur les 4 bits de poids faible
	uint8_t transmit_error;
	//!< erreur interne dynamixel (champ de bit)
	uint8_t internal_error;
};

struct Dynamixel
{
	uint16_t min_goal;
	uint16_t max_goal;
	uint16_t goal_pos;
	uint16_t pos;
	struct dynamixel_error last_error;
};

struct dynamixel_status
{
	struct dynamixel_error error;
	uint8_t argc;
	uint8_t arg[DYNAMIXEL_ARG_MAX];
};

struct dynamixel_request
{
	uint8_t id;
	uint8_t instruction;
	uint8_t argc;
	uint8_t arg[DYNAMIXEL_ARG_MAX];
	struct dynamixel_status status;
};

class DynamixelManager
{
	public:
		int init(const char* name, enum usart_id usart_id, int half_duplex, uint32_t frequency, int max_devices_id);

		//!< affichage d'une erreur
		void print_error(int id, struct dynamixel_error err);

		struct dynamixel_error ping(uint8_t id);
		struct dynamixel_error action(uint8_t id);
		struct dynamixel_error reset(uint8_t id);

		struct dynamixel_error set_led(uint8_t id, uint8_t on);
		struct dynamixel_error set_moving_speed(uint8_t id, uint16_t speed);
		struct dynamixel_error set_goal_position(uint8_t id, float theta);  //!< deplacement vers l'angle theta (en rd)
		struct dynamixel_error set_torque_limit(uint8_t id, uint16_t torque_limit);
		struct dynamixel_error set_torque_limit_eeprom(uint8_t id, uint16_t torque_limit);
		struct dynamixel_error set_torque_enable(uint8_t id, uint8_t enable);
		void set_goal_limit(uint8_t id, float min, float max);  //!< configuration des limites d'un dynamixel (limites mécaniques par exemple pour ne pas forcer)

		float get_position(uint8_t id, struct dynamixel_error* error); //!< angle donné en rd

		uint8_t read8(uint8_t id, uint8_t offset, struct dynamixel_error* error);
		uint16_t read16(uint8_t id, uint8_t offset, struct dynamixel_error* error);

		struct dynamixel_error write8(uint8_t id, uint8_t offset, uint8_t data);
		struct dynamixel_error write16(uint8_t id, uint8_t offset, uint16_t data);

	protected:
		static void task_wrapper(void* arg);
		void task();
		void send(struct dynamixel_request *req);

		// variables alignees pour le dma
		uint8_t write_dma_buffer[6 + DYNAMIXEL_ARG_MAX] __attribute__ ((aligned (16)));
		uint8_t read_dma_buffer[2*(6 + DYNAMIXEL_ARG_MAX)] __attribute__ ((aligned (16)));
		int usart_half_duplex;
		enum usart_id usart;
		xSemaphoreHandle mutex;
		xSemaphoreHandle usart_mutex;

		// donnes des dynamixel d'id 1 a max_devices_id-1
		int max_devices_id;
		Dynamixel* devices;
};

extern DynamixelManager ax12;
extern DynamixelManager rx24;

#endif
