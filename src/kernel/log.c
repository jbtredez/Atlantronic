//! @file log.c
//! @brief Log Task
//! @author Atlantronic

#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/queue.h"
#include "kernel/log.h"
#include "priority.h"
#include <stdarg.h>
#include "kernel/event.h"
#include "kernel/rcc.h"

#define LOG_BUFER_SIZE     4096

//! @todo réglage au pif
#define LOG_STACK_SIZE       64

static unsigned char log_buffer[LOG_BUFER_SIZE];
static int log_buffer_begin;
static int log_buffer_end;

void log_task(void *);
void test_task(void *); // TODO tests

static int log_module_init()
{
	log_buffer_begin = 0;
	log_buffer_end = 0;

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(log_task, "log", LOG_STACK_SIZE, NULL, PRIORITY_TASK_LOG, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_LOG;
	}

	// TODO tests
	err = xTaskCreate(test_task, "test", LOG_STACK_SIZE, NULL, PRIORITY_TASK_LOG, &xHandle);

	return 0;
}

module_init(log_module_init, INIT_LOG);

void log_add(char* msg, int size)
{
	portENTER_CRITICAL();
	for( ; size--; )
	{
		log_buffer[log_buffer_end] = *msg;
		msg++;
		log_buffer_end = (log_buffer_end + 1) % LOG_BUFER_SIZE;
		if( log_buffer_end == log_buffer_begin )
		{
			// buffer circulaire plein, on ecrase les vieux log
			log_buffer_begin = (log_buffer_begin + 1) % LOG_BUFER_SIZE;
		}
	}
	vTaskSetEvent(EVENT_LOG);
	portEXIT_CRITICAL();
}

void log_format_and_add(const char* msg, ...)
{
	char buffer[LOG_SIZE];

	va_list ap;
	va_start(ap, msg);
	char size = vsnprintf(buffer, LOG_SIZE, msg, ap);
	va_end(ap);

	log_add(buffer, size);
}

// TODO tests
volatile unsigned int int_rdy = 1;
volatile unsigned int log_write_size = 0;
#include "../src/kernel/driver/usb2/usb_lib.h"
#include "../src/kernel/driver/usb2/usb_pwr.h"

void EP1_IN_Callback(void)
{
	log_buffer_begin = (log_buffer_begin + log_write_size) % LOG_BUFER_SIZE;
	int_rdy = 1;
	vTaskSetEventFromISR(EVENT_LOG);
}

//! Log task
void log_task(void * arg)
{
	(void) arg;

	while(1)
	{
		vTaskWaitEvent(EVENT_LOG, portMAX_DELAY);
		if( int_rdy && bDeviceState == CONFIGURED)
		{
			portENTER_CRITICAL();
			if(log_buffer_begin != log_buffer_end)
			{
				int size = log_buffer_end - log_buffer_begin;
				if(size < 0)
				{
					size += LOG_BUFER_SIZE;
				}
				int_rdy = 0;
				log_write_size = size;
				USB_SIL_Write(EP1_IN, log_buffer + log_buffer_begin, size);
			}
			portEXIT_CRITICAL();
		}
	}
}

// TODO tests
char buffer[9] = "test640\n";

void test_task(void * arg)
{
	(void) arg;
	int i = 0;

	while(1)
	{
		log_format_and_add("test %i\n", i);
		log_add(buffer, sizeof(buffer));
		i++;
		vTaskDelay(ms_to_tick(2000));
	}
}