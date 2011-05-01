//! @file end.c
//! @brief Task waiting during the math, will send halt event.
//! @author Jean-Baptiste Trédez

#include "module.h"
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE
#include "event.h"

#define TASK2_STACK_SIZE           100
#define PRIORITY_TEST_TASK2        1

//const uint64_t DUREE_MATCH_TICK = 1000000000LL+300LL;
const uint64_t DUREE_MATCH_TICK = 90ULL * 72000000ULL;

static void task2_task(void *arg);

static int task2_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(task2_task, "task2", TASK2_STACK_SIZE, NULL, PRIORITY_TEST_TASK2, &xHandle);

	if(err != pdPASS)
	{
		return -1;
	}

	return 0;
}

module_init(task2_module_init, INIT_TEST_TASK2);

static void task2_task(void *arg)
{
	(void) arg;
	
	while(1) ;
//vTaskSuspend(NULL);
	//vTaskWaitEvent(EVENT_GO);
	//vTaskDelayUntil(DUREE_MATCH_TICK);
	//vTaskSetEvent(EVENT_END);
	
	//vTaskDelete(NULL);
}