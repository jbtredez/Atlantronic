#ifndef DYNAMIXEL_MANAGER_H
#define DYNAMIXEL_MANAGER_H

//! @file DynamixelManager.h
//! @brief Gestion Dynamixel (ax12 et rx24)
//! @author Atlantronic

#include "kernel/driver/usart.h"
#include "dynamixel_id.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/queue.h"
#include "kernel/semphr.h"
#include "Dynamixel.h"

#define DYNAMIXEL_MAX_ON_BUS         9

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

struct DynamixelUsbData
{
	DynamixelUsbDeviceData dynamixel[DYNAMIXEL_MAX_ON_BUS];
} __attribute((packed));

class DynamixelManager
{
	public:
		int init(const char* name, enum usart_id usart_id, uint32_t frequency, int max_devices_id, uint8_t type);

		int registerDynamixel(Dynamixel* dynamixel);
		Dynamixel* findDynamixel(int id);

		//!< affichage d'une erreur
		void print_error(int id, DynamixelError err);

		DynamixelError ping(uint8_t id);
		DynamixelError action(uint8_t id);
		DynamixelError reset(uint8_t id);

		inline int getType()
		{
			return m_type;
		}

		void updateUsbData(DynamixelUsbData* dynamixel);
		void send(DynamixelRequest *req);

		bool m_disabled;

	protected:
		static void task_wrapper(void* arg);
		void task();

		static void cmd(void* arg, void* data);
		void cmd_scan();

		// variables alignees pour le dma
		uint8_t m_writeDmaBuffer[6 + DYNAMIXEL_ARG_MAX] __attribute__ ((aligned (16)));
		uint8_t m_readDmaBuffer[2*(6 + DYNAMIXEL_ARG_MAX)] __attribute__ ((aligned (16)));
		enum usart_id m_usart;
		xSemaphoreHandle m_mutex;
		xSemaphoreHandle m_usartMutex;

		int m_devicesCount;
		Dynamixel* m_devices[DYNAMIXEL_MAX_ON_BUS];
		uint8_t m_type;
};

// ------------------ interface usb ------------------
enum
{
// commandes manager
	DYNAMIXEL_CMD_SCAN = 1,
	DYNAMIXEL_CMD_SET_MANAGER_BAUDRATE,
// commandes dynamixel
	DYNAMIXEL_CMD_SET_ID,
	DYNAMIXEL_CMD_SET_BAUDRATE,
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
