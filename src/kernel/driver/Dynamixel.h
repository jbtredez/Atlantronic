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

#define DYNAMIXEL_INSTRUCTION_PING             0x01
#define DYNAMIXEL_INSTRUCTION_READ_DATA        0x02
#define DYNAMIXEL_INSTRUCTION_WRITE_DATA       0x03
#define DYNAMIXEL_INSTRUCTION_REG_WRITE        0x04
#define DYNAMIXEL_INSTRUCTION_ACTION           0x05
#define DYNAMIXEL_INSTRUCTION_RESET            0x06
#define DYNAMIXEL_INSTRUCTION_SYNC_WRITE       0x83

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

struct DynamixelError
{
	//!< bit 7 à 4 : ERR_DYNAMIXEL_SEND_CHECK, ERR_DYNAMIXEL_PROTO ou ERR_DYNAMIXEL_CHECKSUM
	//!< bit 4 à 0 : erreur usart sur les 4 bits de poids faible
	uint8_t transmit_error;
	//!< erreur interne dynamixel (champ de bit)
	uint8_t internal_error;
} __attribute((packed));

struct DynamixelUsbDeviceData
{
	uint8_t id;
	uint16_t pos;           //!< position
	uint16_t flags;         //!< flags
	DynamixelError error; //!< erreurs
} __attribute((packed));

class DynamixelManager;

class Dynamixel
{
	public:
		int init(DynamixelManager* manager, int id, bool autoUpdate = true);

		DynamixelError setGoalPosition(float theta);           //!< deplacement vers l'angle theta (en rd)
		DynamixelError setTorqueLimit(float torque_limit);
		void setGoalLimits(float min, float max);              //!< configuration des limites d'un dynamixel (limites mécaniques par exemple pour ne pas forcer)
		void setTargetReachedThreshold(float threshold);
		bool isFlagActive(uint32_t mask);
		float getPosition(DynamixelError* error);              //!< angle donné en rd

		DynamixelError setLed(uint8_t on);
		void setId(uint8_t newId);
		DynamixelError setMovingSpeed(float speed);
		DynamixelError setTorqueLimitEeprom(float torque_limit);
		DynamixelError setTorqueEnable(uint8_t enable);
		DynamixelError setCwAngleLimit(uint16_t val);
		DynamixelError setCcwAngleLimit(uint16_t val);
		DynamixelError setBaudRate(uint8_t bd);

		void update();
		inline uint8_t id()
		{
			return m_id;
		}
		void updateUsbData(DynamixelUsbDeviceData* dynamixelDeviceData);

	protected:
		uint8_t read8(uint8_t offset, DynamixelError* error);
		uint16_t read16(uint8_t offset, DynamixelError* error);
		DynamixelError write8(uint8_t offset, uint8_t data);
		DynamixelError write16(uint8_t offset, uint16_t data);

		uint16_t m_minGoal;                        //!< position min
		uint16_t m_maxGoal;                        //!< position max

		xSemaphoreHandle m_mutex;                  //!< mutex d'acces aux donnes ci-dessous
		uint8_t m_id;                              //!< id
		uint16_t m_goalPos;                        //!< position desiree
		uint16_t m_pos;                            //!< position actuelle
		uint16_t m_targetReachedThreshold;         //!< tolerance pour target reached
		uint16_t m_maxTorque;                      //!< couple max
		uint16_t m_flags;                          //!< flags - champ de bit ( DYNAMIXEL_FLAG_TARGET_REACHED, DYNAMIXEL_FLAG_STUCK)
		uint32_t m_timeStartMovingMs;              //!< temps en ms du debut du mouvement
		DynamixelError m_lastError;                //!< derniere erreur
		DynamixelManager* m_manager;
};

#endif
