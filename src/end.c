//! @file end.c
//! @brief Task waiting during the math, will send halt event.
//! @author Jean-Baptiste Trédez

#include "module.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event.h"

#define END_STACK_SIZE           50
const uint64_t DUREE_MATCH_TICK = 90ULL * 72000000ULL;
uint32_t color;

static void end_task(void *arg);

static int end_module_init()
{
	color = COLOR_BLUE;

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

	uint32_t old_btn = BTN_2;
	uint32_t btn = BTN_2;

	setLed(0x30);

	while(getBTN(BTN_1) == BTN_1)
	{
		btn = getBTN(BTN_2);

		if(btn != old_btn && btn > 0)
		{
			if(color == COLOR_BLUE)
			{
				color = COLOR_RED;
				setLed(0x06);
			}
			else
			{
				color = COLOR_BLUE;
				setLed(0x30);
			}
		}

		old_btn = btn;
	}

	setLed(0x23F);

	vTaskSetEvent(EVENT_GO);
	vTaskDelay(DUREE_MATCH_TICK);
	vTaskSetEvent(EVENT_END);

	setLed(0x00);

	vTaskDelete(NULL);
}