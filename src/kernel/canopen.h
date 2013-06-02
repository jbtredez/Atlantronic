#ifndef CAN_OPEN_H
#define CAN_OPEN_H

#include "kernel/driver/can.h"

typedef void (*can_callback)(struct can_msg *msg, int nodeid, int type);

#define CANOPEN_RX_PDO1          0x180
#define CANOPEN_RX_PDO2          0x280
#define CANOPEN_RX_PDO3          0x380

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