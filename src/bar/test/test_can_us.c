#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/event.h"

#include "gpio.h"
#include "kernel/rcc.h"

//! @todo réglage au pif
#define TEST_CAN_STACK_SIZE       100

static void test_hokuyo_task();
int test_hokuyo_module_init();


int test_can_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(test_can_task, "t_can", TEST_CAN_STACK_SIZE, NULL, PRIORITY_TASK_CAN, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TEST;
	}

	return 0;
}

module_init(test_can_module_init, INIT_TEST_DEPLACEMENT);

static void test_can_task()
{
	uint8_t i = 0;

	
	while(1)
	{
	   for(i=0; i<US_MAX; i++)
	   {
	      us_send(CAN_US_EMERGENCY_ID, i);
	      vTaskDelay(ms_to_tick(1000));
	   }
	    
	}

	vTaskDelete(NULL);
}
