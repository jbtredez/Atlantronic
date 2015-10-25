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

#ifndef WEAK_DYNAMIXEL
#define WEAK_DYNAMIXEL __attribute__((weak, alias("nop_function") ))
#endif

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

#define DYNAMIXEL_FLAG_TARGET_REACHED         0x01
#define DYNAMIXEL_FLAG_STUCK                  0x02
#define DYNAMIXEL_FLAG_CONTROL_OFF            0x04
#define DYNAMIXEL_FLAG_TORQUE_TO_UPDATE       0x08

#define DYNAMIXEL_TYPE_AX12         12
#define DYNAMIXEL_TYPE_RX24         24

#define ERR_DYNAMIXEL_SEND_CHECK             0x80
#define ERR_DYNAMIXEL_PROTO                  0x81
#define ERR_DYNAMIXEL_CHECKSUM               0x82
#define ERR_DYNAMIXEL_POWER_OFF              0x83

#define DYNAMIXEL_INPUT_VOLTAGE_ERROR_MASK    0x01
#define DYNAMIXEL_ANGLE_LIMIT_ERROR_MASK      0x02
#define DYNAMIXEL_OVERHEATING_ERROR_MASK      0x04
#define DYNAMIXEL_RANGE_ERROR_MASK            0x08
#define DYNAMIXEL_CHECKSUM_ERROR_MASK         0x10
#define DYNAMIXEL_OVERLOAD_ERROR_MASK         0x20
#define DYNAMIXEL_INSTRUCTION_ERROR_MASK      0x40

#define DYNAMIXEL_ARG_MAX                 3

#define DYNAMIXEL_POS_TO_RD          (150 * M_PI / (0x1ff * 180.0f))
#define DYNAMIXEL_RD_TO_POS          (0x1ff * 180 / (150 * M_PI))
#define DYNAMIXEL_MAX_MOVING_SPEED_RD      11.938f       // 114 rpm
#define DYNAMIXEL_RDS_TO_SPEED       (DYNAMIXEL_MAX_MOVING_SPEED_RD / 0x3ff)

#define DYNAMIXEL_MAX_ON_BUS         10

struct DynamixelError
{
	//!< bit 7 à 4 : ERR_DYNAMIXEL_SEND_CHECK, ERR_DYNAMIXEL_PROTO ou ERR_DYNAMIXEL_CHECKSUM
	//!< bit 4 à 0 : erreur usart sur les 4 bits de poids faible
	uint8_t transmit_error;
	//!< erreur interne dynamixel (champ de bit)
	uint8_t internal_error;
} __attribute((packed));

class DynamixelManager;
class Dynamixel
{
	public:
		int init(DynamixelManager* manager, int id);

		uint8_t id;                               //!< id
		uint16_t min_goal;                        //!< position min
		uint16_t max_goal;                        //!< position max
		uint16_t goal_pos;                        //!< position desiree
		uint16_t pos;                             //!< position actuelle
		uint16_t target_reached_threshold;        //!< tolerance pour target reached
		uint16_t max_torque;                      //!< couple max
		uint16_t flags;                           //!< flags - champ de bit ( DYNAMIXEL_FLAG_TARGET_REACHED, DYNAMIXEL_FLAG_STUCK)
		uint32_t timeStartMoving_ms;              //!< temps en ms du debut du mouvement
		DynamixelError last_error;        //!< derniere erreur

		DynamixelError set_goal_position(float theta);  //!< deplacement vers l'angle theta (en rd)
		DynamixelError set_torque_limit(float torque_limit);
		void set_goal_limit(float min, float max);  //!< configuration des limites d'un dynamixel (limites mécaniques par exemple pour ne pas forcer)
		void set_target_reached_threshold(float threshold);
		bool isFlagActive(uint32_t mask);
		float get_position(DynamixelError* error); //!< angle donné en rd

	protected:
		DynamixelManager* m_manager;
};

struct DynamixelStatus
{
	DynamixelError error;
	uint8_t argc;
	uint8_t arg[DYNAMIXEL_ARG_MAX];
};

struct DynamixelRequest
{
	uint8_t id;
	uint8_t instruction;
	uint8_t argc;
	uint8_t arg[DYNAMIXEL_ARG_MAX];
	struct DynamixelStatus status;
};

struct DynamixelUsbDeviceData
{
	uint8_t id;
	uint16_t pos;           //!< position
	uint16_t flags;         //!< flags
	DynamixelError error; //!< erreurs
} __attribute((packed));

struct DynamixelUsbData
{
	DynamixelUsbDeviceData dynamixel[DYNAMIXEL_MAX_ON_BUS];
} __attribute((packed));

class DynamixelManager
{
	public:
		int init(const char* name, enum usart_id usart_id, uint32_t frequency, int max_devices_id, uint8_t type);

		int registerDynamixel(Dynamixel* dynamixel);

		//!< affichage d'une erreur
		void print_error(int id, DynamixelError err);

		DynamixelError ping(uint8_t id);
		DynamixelError action(uint8_t id);
		DynamixelError reset(uint8_t id);

		// TODO a deplacer dans la classe Dynamixel
		DynamixelError setLed(uint8_t id, uint8_t on);
		DynamixelError set_moving_speed(uint8_t id, float speed);
		DynamixelError set_torque_limit_eeprom(uint8_t id, float torque_limit);
		DynamixelError set_torque_enable(uint8_t id, uint8_t enable);
		DynamixelError set_cw_angle_limit(uint8_t id, uint16_t val);
		DynamixelError set_ccw_angle_limit(uint8_t id, uint16_t val);

		inline int getType()
		{
			return m_type;
		}

		inline void enable()
		{
			m_disabled = false;
		}

		inline void disable()
		{
			m_disabled = true;
		}

		void updateUsbData(DynamixelUsbData* dynamixel);

	protected:
		static void task_wrapper(void* arg);
		void task();
		void send(struct DynamixelRequest *req);
		uint8_t read8(uint8_t id, uint8_t offset, DynamixelError* error);
		uint16_t read16(uint8_t id, uint8_t offset, DynamixelError* error);
		DynamixelError write8(uint8_t id, uint8_t offset, uint8_t data);
		DynamixelError write16(uint8_t id, uint8_t offset, uint16_t data);

		static void cmd(void* arg, void* data);
		void cmd_scan();
		void cmd_set_id(uint8_t old_id, uint8_t id);

		// variables alignees pour le dma
		uint8_t m_writeDmaBuffer[6 + DYNAMIXEL_ARG_MAX] __attribute__ ((aligned (16)));
		uint8_t m_readDmaBuffer[2*(6 + DYNAMIXEL_ARG_MAX)] __attribute__ ((aligned (16)));
		enum usart_id m_usart;
		xSemaphoreHandle m_mutex;
		xSemaphoreHandle m_usartMutex;

		int m_devicesCount;
		Dynamixel* m_devices[DYNAMIXEL_MAX_ON_BUS];
		bool m_disabled;
		uint8_t m_type;
};

// ------------------ interface usb ------------------
enum
{
	DYNAMIXEL_CMD_SCAN = 1,
	DYNAMIXEL_CMD_SET_ID,
	DYNAMIXEL_CMD_SET_BAUDRATE,
	DYNAMIXEL_CMD_SET_MANAGER_BAUDRATE,
	DYNAMIXEL_CMD_SET_GOAL_POSITION,
	DYNAMIXEL_CMD_SET_SPEED,
	DYNAMIXEL_CMD_SET_MAX_TORQUE,
	DYNAMIXEL_CMD_SET_TARGET_REACHED_THRESHOLD,
	DYNAMIXEL_CMD_GET_POSITION,
	DYNAMIXEL_CMD_ENABLE_ENDLESS_TURN_MODE,
	DYNAMIXEL_CMD_DISABLE_ENDLESS_TURN_MODE,
};

struct dynamixel_cmd_param
{
	uint8_t cmd_id;         //!< id de la commande
	uint8_t id;             //!< id du dynamixel
	uint8_t reserved;       //!< reserve
	float param;            //!< parametre
} __attribute((packed));

#endif
