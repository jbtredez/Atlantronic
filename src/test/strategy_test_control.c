#include "FreeRTOS.h"
#include "task.h"
#include "module.h"
#include "control/control.h"
#include "io/systick.h"
#include "event.h"
#include "io/gpio.h"
#include "io/systick.h"
#include "location/location.h"
#include "pince.h"

//! @todo réglage au pif
#define STRATEGY_TEST_CONTROL_STACK_SIZE       100

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
//	if(getcolor() == COLOR_BLUE)
	{
		location_set_position(-1400.0f, -850.0f, 0.0f);
	}

	pince_configure();
	
	while(1)
	{
		pince_open();
		vTaskDelay(72000000);
		pince_close();
		vTaskDelay(72000000);
	}

	pince_open();

	vTaskWaitEvent(EVENT_GO, portMAX_DELAY);

	//control_straight(800);
	location_set_position(-1410.0f, -850.0f, 0.0f);
	control_goto(-350-350.0/2.0f, -1050+200);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	control_goto(-850*0, -525*0);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
//	control_rotate(1.57);
//	vTaskWaitEvent(EVENT_CONTROL_READY);
//	control_straight(850);

/*	control_straight(850-350);
	vTaskWaitEvent(EVENT_CONTROL_READY);
	control_rotate(1.57/2.0f);
	vTaskWaitEvent(EVENT_CONTROL_READY);
	control_straight(1000);
	vTaskWaitEvent(EVENT_CONTROL_READY);
	control_rotate(1.57/2.0f);
	vTaskWaitEvent(EVENT_CONTROL_READY);
	control_straight(1100);
	vTaskWaitEvent(EVENT_CONTROL_READY);
*/
//	control_rotate(10570);
//	vTaskWaitEvent(EVENT_CONTROL_READY);
		
//	control_straight(1800);
#if 0
	if(getcolor() == COLOR_BLUE)
	{
		control_straight(1800);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(1000);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(10570);
		vTaskWaitEvent(EVENT_CONTROL_READY);
#if 0
		control_straight(550);
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
#endif
	vTaskDelete(NULL);
}
