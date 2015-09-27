#ifndef CAN_MIP_NODE_H
#define CAN_MIP_NODE_H

#include "kernel/driver/can.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"

#ifndef WEAK_CAN_MIP
#define WEAK_CAN_MIP __attribute__((weak, alias("nop_function") ))
#endif

#define CAN_MIP_CMD_SYNC                   0x20

class CanMipNode
{
	public:
		CanMipNode();

		uint8_t nodeId;

		xSemaphoreHandle sem;
		systime last_communication_time;
//		systime last_reset_node_time;

		virtual void rxMsg(struct can_msg *msg);
		virtual void update(portTickType absTimeout);
		int wait_update_until(portTickType t);
//		void resetNode();
};

void can_mip_update(portTickType absTimeout) WEAK_CAN_MIP;

int can_mip_register_node(CanMipNode* node);

int can_mip_reset_node(int node);

#endif
