//! @file fault.c
//! @brief Gestion des defauts
//! @author Atlantronic

#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/queue.h"
#include "kernel/semphr.h"
#include "kernel/log.h"
#include "kernel/module.h"
#include "kernel/fault.h"
#include "kernel/driver/usb.h"
#include <stdlib.h>
#include <stdio.h>
#include "gpio.h"

#define FAULT_STACK_SIZE       70

struct fault_status fault_status[FAULT_MAX];

static void fault_task(void* arg);

static xQueueHandle fault_barrier;

static int fault_module_init(void)
{
	int i = 0;
	for( ; i < FAULT_MAX; i++)
	{
		fault_status[i].state = 0;
		fault_status[i].time = 0;
	}

	vSemaphoreCreateBinary(fault_barrier);

	if(fault_barrier == NULL)
	{
		return ERR_INIT_ERROR;
	}

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(fault_task, "fault", FAULT_STACK_SIZE, NULL, PRIORITY_TASK_FAULT, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CAN;
	}

	return 0;
}

module_init(fault_module_init, INIT_FAULT);

static void fault_task(void* arg)
{
	(void) arg;

	usb_add(USB_ERR, fault_status, sizeof(fault_status));

	while(1)
	{
		// timeout pour envoi de temps en temps (programme externe qui est relancé par ex)
		xSemaphoreTake(fault_barrier, ms_to_tick(1000));

		// pas de mutex, on envoi pour le debug, s'il y a une modif en cours, on va renvoyer juste apres
		// remarque : un defaut peut avoir eu le temps de monter et de descendre entre 2 envois
		usb_add(USB_ERR, fault_status, sizeof(fault_status));
	}
}

void fault(enum fault id, unsigned char new_state)
{
	int updated = 0;

	new_state &= 0x01;

	portENTER_CRITICAL();
	if( (fault_status[id].state & 0x01) != new_state)
	{
		if(new_state)
		{
			// mise à 1 du dernier bit (il etait à 0) et incrémentation du compteur sur les 31 premiers bits
			fault_status[id].state = fault_status[id].state + 3;
		}
		else
		{
			// mise à 0 du dernier bit
			fault_status[id].state &= 0xFFFFFFFE;
		}

		fault_status[id].time = systick_get_time().ms;
		updated = 1;
	}
	portEXIT_CRITICAL();

	if(updated)
	{
		xSemaphoreGive(fault_barrier);
	}
}

long fault_from_isr(enum fault id, unsigned char new_state)
{
	long xHigherPriorityTaskWoken = 0;

	if(fault_status[id].state != new_state)
	{
		fault_status[id].state = new_state;
		fault_status[id].time = systick_get_time_from_isr().ms;
		xSemaphoreGiveFromISR(fault_barrier, &xHigherPriorityTaskWoken);
	}

	return xHigherPriorityTaskWoken;
}
