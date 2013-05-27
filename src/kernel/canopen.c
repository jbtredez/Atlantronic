#include "canopen.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/queue.h"
#include "kernel/event.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"

#define CAN_STACK_SIZE  250
#define CAN_READ_QUEUE_SIZE      50
#define CAN_MAX_NODE          4

static void can_task(void *arg);
static xQueueHandle can_read_queue;
static int can_max_node;

struct canopen_node
{
	char id;
	can_callback callback;
};

struct canopen_node canopen_nodes[CAN_MAX_NODE];

static int canopen_module_init(void)
{
	can_read_queue = xQueueCreate(CAN_READ_QUEUE_SIZE, sizeof(struct can_msg));

	if(can_read_queue == 0)
	{
		return -1;
	}

	int res = can_open(CAN_1000, can_read_queue);

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

module_init(canopen_module_init, INIT_CAN);

static void can_task(void *arg)
{
	(void) arg;
	struct can_msg msg;

	// reset de tout les noeuds can au boot
	log(LOG_INFO, "reset all nodes");
	canopen_reset_node(0);

	while(1)
	{
		if(xQueueReceive(can_read_queue, &msg, portMAX_DELAY))
		{
			// traces CAN pour le debug
			usb_add(USB_CAN_TRACE, &msg, sizeof(msg));

			// TODO a voir / defauts
			//fault(ERR_CAN_READ_QUEUE_FULL, FAULT_CLEAR);

			int id = 0;
			int type = 0;
			if( msg.id > 0x180 && msg.id < 0x200)
			{
				// PDO 1
				id = msg.id - 0x180;
				type = CANOPEN_TX_PDO1;
			}
			else if(msg.id > 0x700 && msg.id < 0x780)
			{
				// bootup
				id = msg.id - 0x700;
				type = CANOPEN_BOOTUP;
				log_format(LOG_INFO, "boot up %x", (unsigned int) id);
			}

			// on a trouve l'id
			if( id > 0 )
			{
				int i = 0;
				for( ; i < can_max_node ; i++)
				{
					if( canopen_nodes[i].id == id)
					{
						canopen_nodes[i].callback(&msg, id, type);
					}
				}
			}
		}
	}
}

//! enregistrement d'un noeud can
//! fonction non protegee - utiliser dans la phase d'init uniquement
int canopen_register_node(int node, can_callback callback)
{
	int res = -1;

	if( can_max_node < CAN_MAX_NODE )
	{
		canopen_nodes[can_max_node].id = node;
		canopen_nodes[can_max_node].callback = callback;
		can_max_node++;
		res = 0;
	}

	return res;
}

int canopen_reset_node(int node)
{
	struct can_msg msg;

	msg.id = 0;
	msg.size = 2;
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;
	msg.data[0] = 0x81;
	msg.data[1] = node;

	return can_write(&msg, 0);
}

int canopen_op_mode(int node)
{
	struct can_msg msg;

	msg.id = 0;
	msg.size = 2;
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;
	msg.data[0] = 1;
	msg.data[1] = node;

	return can_write(&msg, 0);
}

#if 0
uint32_t canopen_sdo_read(int node, int index, int subindex)
{
	struct can_msg msg;

	msg.id = 0x600 + node;
	msg.size = 4;
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;
	msg.data[0] = 0x40;
	msg.data[1] = index & 0xff;
	msg.data[2] = (index >> 8) & 0xff;
	msg.data[3] = subindex;

	can_write(&msg, 0);

	return 0;
}
#endif
