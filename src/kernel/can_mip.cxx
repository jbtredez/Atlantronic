#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/queue.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
#include "kernel/driver/can.h"

#define CAN_STACK_SIZE             250
#define CAN_READ_QUEUE_SIZE         50

static void can_task(void *arg);
static xQueueHandle can_read_queue;

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
