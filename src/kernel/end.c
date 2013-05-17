//! @file end.c
//! @brief Task waiting during the math, will send halt event.
//! @author Atlantronic

#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/event.h"
#include "kernel/rcc.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
#include "gpio.h"

#define END_STACK_SIZE           100
uint64_t end_match_tick = 90ULL * 72000000ULL;

static void end_cmd_set_time(void* arg);
static void end_task(void *arg);

static int end_module_init()
{
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
	uint64_t time = *((uint32_t*) arg);
	if( ! (vTaskGetEvent() & EVENT_GO))
	{
		end_match_tick = ms_to_tick( time );
		log_format(LOG_INFO, "duree du match => %d ms", (int)time);
	}
}

static void end_task(void *arg)
{
	(void) arg;

	vTaskWaitEvent(EVENT_GO, portMAX_DELAY);
	uint64_t msg[2];
	msg[0] = systick_get_time();
	msg[1] = end_match_tick;
	usb_add(USB_GO, &msg, sizeof(msg));
	log(LOG_INFO, "GO");
	vTaskDelay(end_match_tick);
	vTaskSetEvent(EVENT_END);
	log(LOG_INFO, "Fin du match");

	exitModules();
	setLed(0x00);

	vTaskDelete(NULL);
}
