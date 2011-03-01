//! @file ax12.c
//! @brief Tests : task 1
//! @author Jean-Baptiste Trédez

#include "module.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"

#define TASK1_STACK_SIZE          100
#define PRIORITY_TEST_TASK1        1

static void task1_task(void *arg);


static int task1_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(task1_task, "task1", TASK1_STACK_SIZE, NULL, PRIORITY_TEST_TASK1, &xHandle);

	if(err != pdPASS)
	{
		return -1;
	}

	return 0;
}

module_init(task1_module_init, INIT_TEST_TASK1);

static void task1_task(void* arg)
{
	(void) arg;

//	while(1) ;
	#if 1
	while(1)
	{
		int64_t i;
		static volatile float x = 0.3587;
		for(i = 0; i < 10000LL; i++)
		{
			x++;
		}
	}
	#endif
}
