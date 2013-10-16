#ifndef CAN_OPEN_H
#define CAN_OPEN_H

#include "kernel/driver/can.h"

typedef void (*can_callback)(struct can_msg *msg, int nodeid, int type);

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

struct canopen_configuration
{
	uint16_t index;
	uint8_t subindex;
	uint8_t size;
	uint32_t data;
};

int canopen_register_node(int node, const struct canopen_configuration* static_conf, uint8_t conf_size, can_callback callback);

int canopen_reset_node(int node);

int canopen_sdo_write(int node, int size, int index, int subindex, uint32_t data);

int canopen_sync();

#endif
