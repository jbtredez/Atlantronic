//! @file end.c
//! @brief Task waiting during the math, will send halt event.
//! @author Atlantronic

#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
#include "gpio.h"
#include "kernel/driver/power.h"

#define END_STACK_SIZE           100
uint32_t end_match_time = 90000; //!< duree du match en ms

static void end_cmd_set_time(void* arg);
static void end_task(void *arg);
volatile int end_match;

static int end_module_init()
{
	end_match = 0;

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(end_task, "end", END_STACK_SIZE, NULL, PRIORITY_TASK_END, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_END;
	}

	usb_add_cmd(USB_CMD_MATCH_TIME, &end_cmd_set_time);

	return 0;
}

module_init(end_module_init, INIT_END);

static void end_cmd_set_time(void* arg)
{
	// temps passé en ms
	uint32_t time = *((uint32_t*) arg);
	if( ! getGo() )
	{
		end_match_time = time;
		log_format(LOG_INFO, "duree du match => %d ms", (int)end_match_time);
	}
}

static void end_task(void *arg)
{
	(void) arg;

	gpio_wait_go();
	uint32_t msg[3];
	struct systime t = systick_get_time();
	msg[0] = t.ms;
	msg[1] = t.ns;
	msg[2] = end_match_time;
	usb_add(USB_GO, &msg, sizeof(msg));
	vTaskDelay(end_match_time);
	end_match = 1;
	log(LOG_INFO, "Fin du match");

	power_set(POWER_OFF_END_MATCH);
	exitModules();
	setLed(0x00);

	vTaskSuspend(0);
}
