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

	while(1)
	{
		if(xQueueReceive(can_read_queue, &msg, portMAX_DELAY))
		{
			// traces CAN pour le debug
			usb_add(USB_CAN_TRACE, &msg, sizeof(msg));

			// TODO a voir / defauts
			//fault(ERR_CAN_READ_QUEUE_FULL, FAULT_CLEAR);
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

int can_mip_reset_node(int node)
{
	struct can_msg msg;

	msg.id = 0x01 + (node << 3);
	msg.size = 2;
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;
	msg.data[0] = 0x60;
	msg.data[1] = 0x01;

	return can_write(&msg, 0);
}

int can_mip_sync()
{
	struct can_msg msg;

	msg.id = 0x01;
	msg.size = 1;
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;
	msg.data[0] = 0x20;

	return can_write(&msg, 0);
}

void can_mip_update(portTickType absTimeout)
{
	//bool all_nmt_op = true;
	for(int i = 0; i < can_mip_max_node; i++)
	{
		//if( canopen_nodes[i]->state != NMT_OPERATIONAL )
		//{
			//all_nmt_op = false;
		//}
		// mise a 0 de la semaphore avant le sync
		xSemaphoreTake(can_mip_nodes[i]->sem, 0);
	}

	//if( all_nmt_op )
	//{
		log(LOG_INFO, "sync");
		can_mip_sync();
	//}

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
