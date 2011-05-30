#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "control/control.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "kernel/rcc.h"
#include "pince.h"
#include "foo/control/control_pince.h"

//! @todo réglage au pif
#define TEST_PINCE_STACK_SIZE       100

static void test_pince_task();
int test_pince_module_init();

int test_pince_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(test_pince_task, "t_pince", TEST_PINCE_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	return 0;
}

module_init(test_pince_module_init, INIT_TEST_PINCE);

static void test_pince_task()
{
	vTaskWaitEvent(EVENT_GO, portMAX_DELAY);

	pince_configure();
	pince_open();
	vTaskDelay(ms_to_tick(1000));

	while(1)
	{
		//control_pince_independant(70, 70);
		control_pince_dual(70, 0);
		vTaskWaitEvent(EVENT_CONTROL_PINCE_READY, ms_to_tick(3000));
		pince_close();
		vTaskDelay(ms_to_tick(200));
		//control_pince_independant(2500, 2500);
		control_pince_dual(3000, 0);
		vTaskWaitEvent(EVENT_CONTROL_PINCE_READY, ms_to_tick(3000));
		pince_open();
		vTaskDelay(ms_to_tick(400));
	}

	vTaskDelete(NULL);
}

