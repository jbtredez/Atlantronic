#include "pince.h"
#include "ax12.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
#include "kernel/event.h"

#define AX12_RIGHT_EMPTY_THRESHOLD   0x200
#define PINCE_STACK_SIZE               300

static void pince_cmd(void* arg);
static void pince_task(void* arg);

// mutex de protection
static xSemaphoreHandle pince_mutex;
static enum pince_cmd_type pince_order;


static int pince_module_init()
{
	xTaskHandle xHandle;

	portBASE_TYPE err = xTaskCreate(pince_task, "pince", PINCE_STACK_SIZE, NULL, PRIORITY_TASK_PINCE, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_PINCE;
	}

	pince_mutex = xSemaphoreCreateMutex();
	
	if( pince_mutex == NULL )
	{
		return ERR_INIT_PINCE;
	}

	usb_add_cmd(USB_CMD_PINCE, &pince_cmd);

	ax12_set_goal_limit(AX12_PINCE_RIGHT, 0x150, 0x320);
	ax12_set_goal_limit(AX12_PINCE_LEFT, 0xdf, 0x2af);

	return 0;
}

module_init(pince_module_init, INIT_PINCE);

static void pince_task(void* arg)
{
	(void) arg;

	// configuration des pinces
	ax12_set_moving_speed(AX12_PINCE_RIGHT, 0x3ff);
	ax12_set_moving_speed(AX12_PINCE_LEFT, 0x3ff);

	ax12_set_torque_limit(AX12_PINCE_RIGHT, 0x300);
	ax12_set_torque_limit(AX12_PINCE_LEFT, 0x300);

	ax12_set_torque_enable(AX12_PINCE_RIGHT, 1);
	ax12_set_torque_enable(AX12_PINCE_LEFT, 1);
//	ax12_write8(AX12_PINCE_RIGHT, AX12_ALARM_SHUTDOWN, 0x04);
//	ax12_write8(AX12_PINCE_LEFT, AX12_ALARM_SHUTDOWN, 0x04);

	while(1)
	{
		enum pince_cmd_type order;
		xSemaphoreTake(pince_mutex, portMAX_DELAY);
		order = pince_order;
		xSemaphoreGive(pince_mutex);

		// TODO en fonction de order et de l'état courant des pinces
		switch(order)
		{
			case PINCE_OPEN:
				ax12_set_goal_position(AX12_PINCE_RIGHT, 15000000);
				ax12_set_goal_position(AX12_PINCE_LEFT, -15000000);
				break;
			case PINCE_CLOSE:
				ax12_set_goal_position(AX12_PINCE_RIGHT, -15000000);
				ax12_set_goal_position(AX12_PINCE_LEFT,   15000000);
				break;
			default:
				break;
		}

		vTaskDelay(ms_to_tick(50));
	}
}

void pince_open()
{
	log(LOG_INFO, "pince_open");
	xSemaphoreTake(pince_mutex, portMAX_DELAY);
	pince_order = PINCE_OPEN;
	xSemaphoreGive(pince_mutex);
}

void pince_close()
{
	log(LOG_INFO, "pince_close");
	xSemaphoreTake(pince_mutex, portMAX_DELAY);
	pince_order = PINCE_CLOSE;
	xSemaphoreGive(pince_mutex);
}

static void pince_cmd(void* arg)
{
	struct pince_cmd_arg* cmd_arg = (struct pince_cmd_arg*) arg;

	xSemaphoreTake(pince_mutex, portMAX_DELAY);
	switch(cmd_arg->type)
	{
		case PINCE_OPEN:
		case PINCE_CLOSE:
			pince_order = cmd_arg->type;
			break;
		default:
			break;
	}
	xSemaphoreGive(pince_mutex);
}