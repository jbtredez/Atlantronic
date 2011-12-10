//! @file error.c
//! @brief Error codes
//! @author Atlantronic

#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/queue.h"
#include "kernel/event.h"
#include "kernel/log.h"
#include <stdlib.h>
#include <stdio.h>
#include "gpio.h"

#include "kernel/module.h"
#include "kernel/error.h"

#define ERROR_QUEUE_SIZE       50
#define ERROR_STACK_SIZE      300

struct error_item
{
	unsigned int id;
	unsigned char state;
	const char* function;
	int line;
};

unsigned char error_state[ERR_MAX];

static void error_task(void* arg);

static xQueueHandle error_queue;

static int error_module_init(void)
{
	int i = 0;
	for( ; i < ERR_MAX; i++)
	{
		error_state[i] = 0;
	}

	error_queue = xQueueCreate(ERROR_QUEUE_SIZE, sizeof(struct error_item));

	if(error_queue == NULL)
	{
		return ERR_INIT_ERROR;
	}

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(error_task, "error", ERROR_STACK_SIZE, NULL, PRIORITY_TASK_ERROR, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CAN;
	}

	return 0;
}

module_init(error_module_init, INIT_ERROR);

static void error_task(void* arg)
{
	(void) arg;

	struct error_item error;

	while(1)
	{
		if(xQueueReceive(error_queue, &error, portMAX_DELAY))
		{
			if( error_state[error.id] != error.state )
			{
				log_error_func_line("error %u state %u", error.function, error.line, error.id, error.state);
				error_state[error.id] = error.state;
			}
		}
	}
}

void _error_check_update(enum fault id, uint32_t err, const char* func, int line)
{
	if(id == err)
	{
		_error(id, ERROR_ACTIVE, func, line);
	}
	else
	{
		_error(id, ERROR_CLEAR, func, line);
	}
}

void _error(enum fault id, unsigned char new_state, const char* func, int line)
{
	struct error_item err;
	err.id = id;
	err.state = new_state;
	err.function = func;
	err.line = line;
	xQueueSendToBack(error_queue, &err, portMAX_DELAY);
}

long _error_from_isr(enum fault id, unsigned char new_state, const char* func, int line)
{
	long xHigherPriorityTaskWoken;
	struct error_item err;
	err.id = id;
	err.state = new_state;
	err.function = func;
	err.line = line;
	if(xQueueSendToBackFromISR(error_queue, &err, &xHigherPriorityTaskWoken) != pdPASS)
	{
		// erreur, file pleine
		// TODO a voir
	}

	return xHigherPriorityTaskWoken;
}
