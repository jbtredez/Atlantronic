//! @file end.c
//! @brief Task waiting during the math, will send halt event.
//! @author Jean-Baptiste Trédez

#include "module.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event.h"

#define END_STACK_SIZE           50
const uint64_t DUREE_MATCH_TICK = 90ULL * 72000000ULL;

static void end_task(void *arg);

static int end_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(end_task, "end", END_STACK_SIZE, NULL, PRIORITY_TASK_END, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_END;
	}

	return 0;
}

module_init(end_module_init, INIT_END);

static void end_task(void *arg)
{
	(void) arg;

	vTaskWaitEvent(EVENT_GO, portMAX_DELAY);
	vTaskDelay(DUREE_MATCH_TICK);
	vTaskSetEvent(EVENT_END);

	exitModules();
	setLed(0x00);

	vTaskDelete(NULL);
}
