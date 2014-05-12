#ifndef CAN_OPEN_H
#define CAN_OPEN_H

#include "kernel/driver/can.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"

#ifndef WEAK_CANOPEN
#define WEAK_CANOPEN __attribute__((weak, alias("nop_function") ))
#endif

enum
{
	CANOPEN_NMT = 0,
	CANOPEN_SYNC,
	CANOPEN_EMERGENCY,
	CANOPEN_RX_PDO1,
	CANOPEN_TX_PDO1,
	CANOPEN_RX_PDO2,
	CANOPEN_TX_PDO2,
	CANOPEN_RX_PDO3,
	CANOPEN_TX_PDO3,
	CANOPEN_RX_PDO4,
	CANOPEN_TX_PDO4,
	CANOPEN_SDO_RES,
	CANOPEN_SDO_REQ,
	CANOPEN_RESERVED_13,
	CANOPEN_BOOTUP,
};

enum
{
	SDO_STATUS_OK = 0,
	SDO_STATUS_KO,
	SDO_STATUS_TRANSMITING,
	SDO_STATUS_UNKNOWN,
};

enum
{
	NMT_OPERATIONAL        =  0x01,
	NMT_STOP               =  0x02,
	NMT_PRE_OPERATIONAL    =  0x80,
	NMT_RESET_AP           =  0x81,
	NMT_RESET_COM          =  0x82,
};

struct canopen_configuration
{
	uint16_t index;
	uint8_t subindex;
	uint8_t size;
	uint32_t data;
};

class CanopenNode
{
	public:
		CanopenNode();

		uint8_t nodeid;
		uint8_t state;
		uint8_t conf_id;
		uint8_t conf_size;
		const struct canopen_configuration* static_conf;
		xSemaphoreHandle sem;
		int sdo_status;
		systime last_sdo_time;
		systime last_communication_time;
		systime last_reset_node_time;

		virtual void rx_pdo(struct can_msg *msg, int type);

		virtual void update(portTickType absTimeout);
		int wait_update_until(portTickType t);
		void resetNode();
};

void canopen_update(portTickType absTimeout) WEAK_CANOPEN;

int canopen_register_node(CanopenNode* node);

int canopen_reset_node(int node);

int canopen_sdo_write(int node, int size, int index, int subindex, uint32_t data);

int canopen_sync();

#endif
