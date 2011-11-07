#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

// configMAX_PRIORITIES in priority.h
// configKERNEL_INTERRUPT_PRIORITY in priority.h
// configMAX_SYSCALL_INTERRUPT_PRIORITY in priority.h
#include "priority.h"

#define configUSE_PREEMPTION			1
#define configUSE_IDLE_HOOK				0
#define configUSE_TICK_HOOK				0
#define configTICK_RATE_HZ				( ( portTickType ) 1000 )
#define configCPU_CLOCK_HZ				( ( unsigned long ) 72000000 )  
#define configPERIPHERAL_CLOCK_HZ		( ( unsigned long ) 40000000UL )
#define configMINIMAL_STACK_SIZE		( ( unsigned portSHORT ) 50 )
#define configISR_STACK_SIZE			( 400 )
#define configTOTAL_HEAP_SIZE			( ( size_t ) 32*1024 )
#define configMAX_TASK_NAME_LEN			( 8 )
#define configUSE_TRACE_FACILITY    	0
#define configIDLE_SHOULD_YIELD			0
#define configUSE_CO_ROUTINES 			0
#define configUSE_MUTEXES				1
#define configUSE_COUNTING_SEMAPHORES	1
#define configUSE_ALTERNATIVE_API		0
#define configUSE_RECURSIVE_MUTEXES		1
#define configCHECK_FOR_STACK_OVERFLOW	0
#define configUSE_APPLICATION_TASK_TAG	0
#define configQUEUE_REGISTRY_SIZE		0
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )
#define configGENERATE_RUN_TIME_STATS		1

#define INCLUDE_vTaskPrioritySet        	0
#define INCLUDE_uxTaskPriorityGet       	0
#define INCLUDE_vTaskDelete             	1
#define INCLUDE_vTaskCleanUpResources   	0
#define INCLUDE_vTaskSuspend            	1
#define INCLUDE_vTaskDelayUntil				1
#define INCLUDE_vTaskDelay					1
#define INCLUDE_uxTaskGetStackHighWaterMark 0
#define INCLUDE_xTaskGetSchedulerState		0

#endif
