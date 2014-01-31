#include "canopen.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/queue.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"

#define CAN_STACK_SIZE             250
#define CAN_READ_QUEUE_SIZE         50
#define CAN_MAX_NODE                 6
#define CANOPEN_SDO_TIMEOUT         20 // en ms

enum
{
	SDO_OK = 0,
	SDO_KO,
	SDO_TRANSMITING,
};

static void can_task(void *arg);
static xQueueHandle can_read_queue;
static int can_max_node;
static struct CanopenNode* canopen_nodes[CAN_MAX_NODE];
static systime canopen_last_sdo_time;
static uint8_t canopen_sdoStatus = SDO_OK;

static int canopen_op_mode(int node);


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

void canopen_update()
{
	bool all_nmt_op = true;
	for(int i = 0; i < can_max_node; i++)
	{
		if( canopen_nodes[i]->state != NMT_OPERATIONAL )
		{
			all_nmt_op = false;
		}
		// mise a 0 de la semaphore avant le sync
		xSemaphoreTake(canopen_nodes[i]->sem, 0);
	}

	if( all_nmt_op )
	{
		//log(LOG_INFO, "sync");
		canopen_sync();
	}

	for(int i = 0; i < can_max_node; i++)
	{
		canopen_nodes[i]->update();
	}
}

static void can_update_node(int id, unsigned int nodeid, int type, struct can_msg* msg)
{
	CanopenNode* node = canopen_nodes[id];
	if( type == CANOPEN_RX_PDO1 || type == CANOPEN_RX_PDO2 || type == CANOPEN_RX_PDO3 )
	{
		// gestion des pdo specifique => callback
		node->rx_pdo(msg, type);
	}
	else if( type == CANOPEN_SDO_RES)
	{
		uint16_t index = msg->data[1] + (msg->data[2] << 8);
		uint8_t subindex = msg->data[3];

		// on fait de la conf
		if( msg->data[0] == 0x60 )
		{
			// reponse "OK" a un SDO write
			const struct canopen_configuration* conf = &node->static_conf[node->conf_id];
			log_format(LOG_INFO, "SDO %x %x:%x OK", nodeid, conf->index, conf->subindex);
			canopen_sdoStatus = SDO_KO;
		}
		else if( msg->data[0] == 0x80 )
		{
			// reponse d'erreur a un SDO (read ou write)
			log_format(LOG_ERROR, "SDO write failed node %x index %x subindex %x error 0x%x%x%x%x",
					nodeid, index, subindex, msg->data[7], msg->data[6], msg->data[5], msg->data[4]);
			canopen_sdoStatus = SDO_KO;
		}
	}
	else if( type == CANOPEN_BOOTUP)
	{
		node->state = NMT_PRE_OPERATIONAL;
		node->conf_id = 0;
	}
}

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
			//usb_add(USB_CAN_TRACE, &msg, sizeof(msg));

			// TODO a voir / defauts
			//fault(ERR_CAN_READ_QUEUE_FULL, FAULT_CLEAR);

			int type = msg.id >> 7;
			unsigned int nodeid = msg.id & 0x7f;
			if(type == CANOPEN_BOOTUP)
			{
				// bootup
				log_format(LOG_INFO, "boot up %x", nodeid);
			}

			int i = 0;
			while( i < can_max_node && canopen_nodes[i]->nodeid != nodeid )
			{
				i++;
			}

			if( i < can_max_node )
			{
				// c'est un noeud can enregistre
				can_update_node(i, nodeid, type, &msg);
			}
		}
	}
}

//! enregistrement d'un noeud can
//! fonction non protegee - utiliser dans la phase d'init uniquement
int canopen_register_node(CanopenNode* node)
{
	int res = -1;

	if( can_max_node < CAN_MAX_NODE )
	{
		canopen_nodes[can_max_node] = node;
		node->state = NMT_RESET_COM;
		node->conf_id = 0;
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

int canopen_sync()
{
	struct can_msg msg;

	msg.id = 0x80;
	msg.size = 0;
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;

	return can_write(&msg, 0);
}

static int canopen_op_mode(int node)
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

int canopen_sdo_write(int node, int size, int index, int subindex, uint32_t data)
{
	struct can_msg msg;

	//log_format(LOG_INFO, "SDO %x %x:%x = %x", nodeid, index, subindex, (unsigned int)data);

	msg.id = 0x80 * CANOPEN_SDO_REQ + node;
	msg.size = 4 + size;
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;
	msg.data[1] = index & 0xff;
	msg.data[2] = (index >> 8) & 0xff;
	msg.data[3] = subindex;
	msg.data[4] = data & 0xff;

	switch(size)
	{
		case 1:
			msg.data[0] = 0x2f;
			break;
		case 2:
			msg.data[0] = 0x2b;
			msg.data[5] = (data >> 8) & 0xff;
			break;
		case 3:
			msg.data[0] = 0x27;
			msg.data[5] = (data >> 8) & 0xff;
			msg.data[6] = (data >> 16) & 0xff;
			break;
		case 4:
			msg.data[0] = 0x23;
			msg.data[5] = (data >> 8) & 0xff;
			msg.data[6] = (data >> 16) & 0xff;
			msg.data[7] = (data >> 24) & 0xff;
			break;
		default:
			return -1;
	}

	can_write(&msg, 0);

	return 0;
}

CanopenNode::CanopenNode()
{
	nodeid = 0;
	state = 0;
	conf_id = 0;
	conf_size = 0;
	static_conf = 0;
	vSemaphoreCreateBinary(sem);
	xSemaphoreTake(sem, 0);
}

void CanopenNode::update()
{
	if( state == NMT_PRE_OPERATIONAL )
	{
		// etat pre op : on regarde si on a des SDO de conf a envoyer
		if( conf_id < conf_size )
		{
			if( canopen_sdoStatus == SDO_TRANSMITING )
			{
				// SDO envoye mais pas encore de reponse
				systime t = systick_get_time();
				systime dt = t - canopen_last_sdo_time;

				if(dt.ms < CANOPEN_SDO_TIMEOUT)
				{
					// rien a faire
					return;
				}
				// sinon, on garde conf_id pour retenter l'envoi
			}
			else
			{
				// SDO_OK ou SDO_KO
				// on passe au sdo suivant
				// TODO rententer 3 fois si KO ?
				conf_id++;
			}

			if( conf_id < conf_size )
			{
				// envoi du sdo
				const struct canopen_configuration* conf = &static_conf[conf_id];
				canopen_last_sdo_time = systick_get_time();
				canopen_sdoStatus = SDO_TRANSMITING;
				canopen_sdo_write(nodeid, conf->size, conf->index, conf->subindex, conf->data);
			}
			else
			{
				log_format(LOG_INFO, "configuration %x end", nodeid);
				// TODO affecter le OP apres confirmation ?
				state = NMT_OPERATIONAL;
				canopen_op_mode(nodeid);
			}
		}
		else
		{
			// TODO affecter le OP apres confirmation ?
			state = NMT_OPERATIONAL;
			// on passe en op si pas de conf ou si cela n'a pas marche plus tot
			canopen_op_mode(nodeid);
		}
	}
}

void CanopenNode::rx_pdo(struct can_msg *msg, int type)
{
	(void) msg;
	(void) type;
}

//! attente de la mise a jour du noeud
//! @return 0 si c'est bon -1 si timeout
int CanopenNode::wait_update_until(portTickType t)
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
