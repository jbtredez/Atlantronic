#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/queue.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
#define WEAK_CAN_MIP
#include "kernel/can_mip.h"

#define CAN_STACK_SIZE             250
#define CAN_READ_QUEUE_SIZE         50
#define CAN_MAX_NODE                 6

static void can_task(void *arg);
static xQueueHandle can_read_queue;
static struct CanMipNode* can_mip_nodes[CAN_MAX_NODE];
static int can_mip_max_node;

static int can_task_module_init(void)
{
	can_read_queue = xQueueCreate(CAN_READ_QUEUE_SIZE, sizeof(struct can_msg));

	if(can_read_queue == 0)
	{
		return -1;
	}

	int res = can_open(CAN_410, can_read_queue);

	if( res )
	{
		return ERR_INIT_CAN;
	}

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(can_task, "can", CAN_STACK_SIZE, NULL, PRIORITY_TASK_CAN, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CAN;
	}

	return 0;
}

module_init(can_task_module_init, INIT_CAN);

static void can_task(void *arg)
{
	(void) arg;
	struct can_msg msg;

	// reset de tout les noeuds can au boot
//	log(LOG_INFO, "reset all nodes");
//	can_mip_reset_node(0);

	while(1)
	{
		if(xQueueReceive(can_read_queue, &msg, portMAX_DELAY))
		{
			// traces CAN pour le debug
			//usb_add(USB_CAN_TRACE, &msg, sizeof(msg));

			// TODO a voir / defauts
			//fault(ERR_CAN_READ_QUEUE_FULL, FAULT_CLEAR);

			if( msg.id == 0x79 )
			{
				int nodeid = msg.data[1];
				int i = 0;
				while( i < can_mip_max_node && can_mip_nodes[i]->nodeId != nodeid )
				{
					i++;
				}

				if( i < can_mip_max_node )
				{
					// c'est un noeud can enregistre
					can_mip_nodes[i]->rxMsg(&msg);
				}
			}
		}
	}
}

//! enregistrement d'un noeud can
//! fonction non protegee - utiliser dans la phase d'init uniquement
int can_mip_register_node(CanMipNode* node)
{
	int res = -1;

	if( can_mip_max_node < CAN_MAX_NODE )
	{
		can_mip_nodes[can_mip_max_node] = node;
		//node->state = NMT_RESET_COM;
		//node->conf_id = 0;
		can_mip_max_node++;
		res = 0;
	}

	return res;
}

/*int can_mip_reset_node(int node)
{
	struct can_msg msg;

	msg.id = 0x01 + (node << 3);
	msg.size = 2;
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;
	msg.data[0] = 0x60;
	msg.data[1] = 0x01;

	return 0;//can_write(&msg, 0);
}*/

int can_mip_sync()
{
	struct can_msg msg;

	//log(LOG_INFO, "sync");
	msg.id = 0x01;
	msg.size = 1;
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;
	msg.data[0] = CAN_MIP_CMD_SYNC;

	return can_write(&msg, 0);
}

void can_mip_update(portTickType absTimeout)
{
	for(int i = 0; i < can_mip_max_node; i++)
	{
		// mise a 0 de la semaphore avant le sync
		xSemaphoreTake(can_mip_nodes[i]->sem, 0);
	}

	can_mip_sync();

	for(int i = 0; i < can_mip_max_node; i++)
	{
		can_mip_nodes[i]->update(absTimeout);
	}
}

CanMipNode::CanMipNode()
{
	nodeId = 0;
	vSemaphoreCreateBinary(sem);
	xSemaphoreTake(sem, 0);
}

void CanMipNode::update(portTickType /*absTimeout*/)
{

}

void CanMipNode::rxMsg(struct can_msg */*msg*/)
{
	last_communication_time = systick_get_time();
}
/*
void CanMipNode::resetNode()
{
	//log_format(LOG_INFO, "reset node %x", nodeId);
	last_reset_node_time = systick_get_time();
	can_mip_reset_node(nodeId);
}*/

//! attente de la mise a jour du noeud
//! @return 0 si c'est bon -1 si timeout
int CanMipNode::wait_update_until(portTickType t)
{
	int res = 0;

	systime currentTime = systick_get_time();
	int timeout = t - currentTime.ms;
	if( timeout < 0)
	{
		timeout = 0;
	}

	if( xSemaphoreTake(sem, timeout) == pdFALSE )
	{
		res = -1;
	}

	return res;
}
