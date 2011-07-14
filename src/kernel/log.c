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
#include "kernel/driver/usb/usb_lib.h"
#include "kernel/driver/usb/usb_pwr.h"

#define LOG_BUFER_SIZE     4096

//! @todo réglage au pif
#define LOG_STACK_SIZE      64
#define LOG_TEST_STACK_SIZE      180

static unsigned char log_buffer[LOG_BUFER_SIZE];
static int log_buffer_begin;
static int log_buffer_end;
static unsigned int log_write_size;

void log_task(void *);
void test_task(void *); // TODO tests
volatile unsigned int int_rdy; // TODO tests
char buffer_test[68] = "test640\n";

static int log_module_init()
{
	log_buffer_begin = 0;
	log_buffer_end = 0;
	log_write_size = 0;
	int_rdy = 1;

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(log_task, "log", LOG_STACK_SIZE, NULL, PRIORITY_TASK_LOG, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_LOG;
	}

	// TODO tests
	err = xTaskCreate(test_task, "test", LOG_TEST_STACK_SIZE, NULL, PRIORITY_TASK_LOG, &xHandle);

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

// attention, coute tres cher en stack
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
void EP1_IN_Callback(void)
{
	int_rdy = 1;
	log_buffer_begin = (log_buffer_begin + log_write_size) % LOG_BUFER_SIZE;
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
				nop();
			}
			portEXIT_CRITICAL();
		}
	}
}

// TODO tests
void test_task(void * arg)
{
	(void) arg;
	int i = 0;

	while(1)
	{
		//log_format_and_add("test %i\n", i);
		log_add(buffer_test, sizeof(buffer_test));
		i++;
		vTaskDelay(ms_to_tick(500));
	}
}