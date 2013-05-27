#ifndef CAN_OPEN_H
#define CAN_OPEN_H

#include "kernel/driver/can.h"

typedef void (*can_callback)(struct can_msg *msg, int id, int type);

enum canopen_nmt
{
	NMT_RESET,
	NMT_PRE_OPERATIONAL,
	NMT_OPERATIONAL,
	NMT_STOP
};

#define CANOPEN_BOOTUP   0x700
#define CANOPEN_TX_PDO1  0x180

int canopen_register_node(int node, can_callback callback);

int canopen_reset_node(int node);

int canopen_op_mode(int node);

//uint32_t canopen_sdo_read(int node, int index, int subindex);

#endif
