#include "FreeRTOS.h"
#include "task.h"
#include "module.h"
#include "control/control.h"
#include "io/systick.h"
#include "event.h"
#include "io/gpio.h"
#include "io/systick.h"

//! @todo réglage au pif
#define STRATEGY_TEST_CONTROL_STACK_SIZE       64

static void strategy_test_control_task();
int strategy_test_control_module_init();

int strategy_test_control_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(strategy_test_control_task, "strategy_test_control", STRATEGY_TEST_CONTROL_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	return 0;
}

module_init(strategy_test_control_module_init, INIT_STRATEGY);

static void strategy_test_control_task()
{
	// TODO : pour les tests
	systick_start_match();
	vTaskSetEvent(EVENT_GO);

	if(getColor() == COLOR_BLUE)
	{
#if 1
		control_straight(-1800);
		vTaskWaitEvent(EVENT_CONTROL_READY);
#endif
#if 0
		control_rotate(1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
#endif
#if 0
		//control_straight(1000);
//		vTaskWaitEvent(EVENT_CONTROL_READY);
//		control_straight(550);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(-1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(200);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(-1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(1550);
#endif
		vTaskWaitEvent(EVENT_CONTROL_READY);
	}
	else
	{
/*		control_straight(500);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(-1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(1550);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(550);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(1550);*/
	}
/*	control_goto(1000,-1000);
	vTaskWaitEvent(EVENT_CONTROL_READY);
	control_goto(1500,-1000);
	vTaskWaitEvent(EVENT_CONTROL_READY);
*/
	vTaskDelete(NULL);
}
