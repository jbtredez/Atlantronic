#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

// entête du pic si on compile pour le pic
#ifdef __GCC_PIC32__
	#include <p32xxxx.h>
#endif

// configMAX_PRIORITIES in priority.h
#include "priority.h"

#define configUSE_PREEMPTION			1
#define configUSE_IDLE_HOOK				1 // laisser à 1 (simu sur pc => nanosleep dans la tache idle ; coroutines => ordonnancement dans la tache idle)
#define configUSE_TICK_HOOK				1 // laisser à 1 : gestion du temps le temps par vApplicationTickHook
#define configTICK_RATE_HZ				( ( portTickType ) 1000 )
#define configCPU_CLOCK_HZ				( ( unsigned long ) 72000000 )  
#define configPERIPHERAL_CLOCK_HZ		( ( unsigned long ) 40000000UL )
#define configMINIMAL_STACK_SIZE		( ( unsigned portSHORT ) 190 )
#define configISR_STACK_SIZE			( 400 )
#define configTOTAL_HEAP_SIZE			( ( size_t ) 17*1024 )
#define configMAX_TASK_NAME_LEN			( 8 )
#define configUSE_TRACE_FACILITY    	0
#define configIDLE_SHOULD_YIELD			1
#define configUSE_CO_ROUTINES 			0
#define configUSE_MUTEXES				1
#define configUSE_COUNTING_SEMAPHORES	1
#define configUSE_ALTERNATIVE_API		0
#define configUSE_RECURSIVE_MUTEXES		1
#define configCHECK_FOR_STACK_OVERFLOW	0 // laisser à 0 pour une simu sur pc
#define configUSE_APPLICATION_TASK_TAG	1
#define configQUEUE_REGISTRY_SIZE		0
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )
#define configKERNEL_INTERRUPT_PRIORITY			240
#define configMAX_SYSCALL_INTERRUPT_PRIORITY	191
#define configGENERATE_RUN_TIME_STATS		0

#define INCLUDE_vTaskPrioritySet        	1
#define INCLUDE_uxTaskPriorityGet       	1
#define INCLUDE_vTaskDelete             	1
#define INCLUDE_vTaskCleanUpResources   	1
#define INCLUDE_vTaskSuspend            	1
#define INCLUDE_vTaskDelayUntil				1
#define INCLUDE_vTaskDelay					1
#define INCLUDE_uxTaskGetStackHighWaterMark 0 // ne pas utiliser sur la simu sur pc
#define INCLUDE_xTaskGetSchedulerState		1

#endif
