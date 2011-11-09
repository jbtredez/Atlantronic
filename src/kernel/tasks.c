//! @file tasks.c
//! @brief noyau freeRTOS v6.0.5 modifié
//!
//! Afin de voir les modifications réalisées :
//!    git diff 23f9696a820a86ec5f00f780aa1bc4031f80c4e5 src/rtos/tasks.c

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/StackMacros.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/*
 * Macro to define the amount of stack available to the idle task.
 */
#define tskIDLE_STACK_SIZE	configMINIMAL_STACK_SIZE

/*
 * Task control block.  A task control block (TCB) is allocated to each task,
 * and stores the context of the task.
 */
typedef struct tskTaskControlBlock
{
	volatile portSTACK_TYPE	*pxTopOfStack;		/*< Points to the location of the last item placed on the tasks stack.  THIS MUST BE THE FIRST MEMBER OF THE STRUCT. */

	#if ( portUSING_MPU_WRAPPERS == 1 )
		xMPU_SETTINGS xMPUSettings;				/*< The MPU settings are defined as part of the port layer.  THIS MUST BE THE SECOND MEMBER OF THE STRUCT. */
	#endif	
	
	xListItem				xGenericListItem;	/*< List item used to place the TCB in ready and blocked queues. */
	xListItem				xEventListItem;		/*< List item used to place the TCB in event lists. */
	unsigned portBASE_TYPE	uxPriority;			/*< The priority of the task where 0 is the lowest priority. */
	portSTACK_TYPE			*pxStack;			/*< Points to the start of the stack. */
	char				    pcTaskName[ configMAX_TASK_NAME_LEN ];/*< Descriptive name given to the task when created.  Facilitates debugging only. */

	volatile unsigned portBASE_TYPE event;
	unsigned portBASE_TYPE eventMask;

	#if ( portSTACK_GROWTH > 0 )
		portSTACK_TYPE *pxEndOfStack;			/*< Used for stack overflow checking on architectures where the stack grows up from low memory. */
	#endif

	#if ( portCRITICAL_NESTING_IN_TCB == 1 )
		unsigned portBASE_TYPE uxCriticalNesting;
	#endif

	#if ( configUSE_TRACE_FACILITY == 1 )
		unsigned portBASE_TYPE	uxTCBNumber;	/*< This is used for tracing the scheduler and making debugging easier only. */
	#endif

	#if ( configUSE_MUTEXES == 1 )
		unsigned portBASE_TYPE uxBasePriority;	/*< The priority last assigned to the task - used by the priority inheritance mechanism. */
	#endif

	#if ( configUSE_APPLICATION_TASK_TAG == 1 )
		pdTASK_HOOK_CODE pxTaskTag;
	#endif

#if ( configGENERATE_RUN_TIME_STATS == 1 )
	uint64_t                last_start_tick_count;
	uint64_t                cpu_time_used;
#endif
} tskTCB;


/*
 * Some kernel aware debuggers require data to be viewed to be global, rather
 * than file scope.
 */
#ifdef portREMOVE_STATIC_QUALIFIER
	#define static
#endif

/*lint -e956 */
PRIVILEGED_DATA tskTCB * volatile pxCurrentTCB = NULL;

/* Lists for ready and blocked tasks. --------------------*/

PRIVILEGED_DATA static xList pxReadyTasksLists[ configMAX_PRIORITIES ];	/*< Prioritised ready tasks. */
PRIVILEGED_DATA static xList xDelayedTaskList;							/*< Delayed tasks. */
PRIVILEGED_DATA static xList xPendingReadyList;							/*< Tasks that have been readied while the scheduler was suspended.  They will be moved to the ready queue when the scheduler is resumed. */

#if ( INCLUDE_vTaskDelete == 1 )

	PRIVILEGED_DATA static volatile xList xTasksWaitingTermination;		/*< Tasks that have been deleted - but the their memory not yet freed. */
	PRIVILEGED_DATA static volatile unsigned portBASE_TYPE uxTasksDeleted = ( unsigned portBASE_TYPE ) 0;

#endif

#if ( INCLUDE_vTaskSuspend == 1 )

	PRIVILEGED_DATA static xList xSuspendedTaskList;					/*< Tasks that are currently suspended. */

#endif

/* File private variables. --------------------------------*/
PRIVILEGED_DATA static volatile unsigned portBASE_TYPE uxCurrentNumberOfTasks 	= ( unsigned portBASE_TYPE ) 0;
PRIVILEGED_DATA static volatile portTickType xTickCount 						= ( portTickType ) 0;
PRIVILEGED_DATA static unsigned portBASE_TYPE uxTopUsedPriority	 				= tskIDLE_PRIORITY;
PRIVILEGED_DATA static volatile unsigned portBASE_TYPE uxTopReadyPriority 		= tskIDLE_PRIORITY;
PRIVILEGED_DATA static volatile signed portBASE_TYPE xSchedulerRunning 			= pdFALSE;
PRIVILEGED_DATA static volatile unsigned portBASE_TYPE uxSchedulerSuspended	 	= ( unsigned portBASE_TYPE ) pdFALSE;
PRIVILEGED_DATA static volatile unsigned portBASE_TYPE uxMissedTicks 			= ( unsigned portBASE_TYPE ) 0;
PRIVILEGED_DATA static volatile unsigned portBASE_TYPE uxMissedEvents 			= ( unsigned portBASE_TYPE ) 0;
PRIVILEGED_DATA static volatile portBASE_TYPE xMissedYield 						= ( portBASE_TYPE ) pdFALSE;
PRIVILEGED_DATA static unsigned portBASE_TYPE uxTaskNumber 						= ( unsigned portBASE_TYPE ) 0;

/* Debugging and trace facilities private variables and macros. ------------*/

/*
 * The value used to fill the stack of a task when the task is created.  This
 * is used purely for checking the high water mark for tasks.
 */
#define tskSTACK_FILL_BYTE	( 0xa5 )

/*
 * Macros used by vListTask to indicate which state a task is in.
 */
#define tskBLOCKED_CHAR		( ( signed char ) 'B' )
#define tskREADY_CHAR		( ( signed char ) 'R' )
#define tskDELETED_CHAR		( ( signed char ) 'D' )
#define tskSUSPENDED_CHAR	( ( signed char ) 'S' )

/*
 * Macros and private variables used by the trace facility.
 */
#if ( configUSE_TRACE_FACILITY == 1 )

	#define tskSIZE_OF_EACH_TRACE_LINE			( ( unsigned long ) ( sizeof( unsigned long ) + sizeof( unsigned long ) ) )
	PRIVILEGED_DATA static volatile signed char * volatile pcTraceBuffer;
	PRIVILEGED_DATA static signed char *pcTraceBufferStart;
	PRIVILEGED_DATA static signed char *pcTraceBufferEnd;
	PRIVILEGED_DATA static signed portBASE_TYPE xTracing = pdFALSE;
	static unsigned portBASE_TYPE uxPreviousTask = 255;
	PRIVILEGED_DATA static char pcStatusString[ 50 ];

#endif

/*-----------------------------------------------------------*/

/*
 * Macro that writes a trace of scheduler activity to a buffer.  This trace
 * shows which task is running when and is very useful as a debugging tool.
 * As this macro is called each context switch it is a good idea to undefine
 * it if not using the facility.
 */
#if ( configUSE_TRACE_FACILITY == 1 )

	#define vWriteTraceToBuffer()																	\
	{																								\
		if( xTracing )																				\
		{																							\
			if( uxPreviousTask != pxCurrentTCB->uxTCBNumber )										\
			{																						\
				if( ( pcTraceBuffer + tskSIZE_OF_EACH_TRACE_LINE ) < pcTraceBufferEnd )				\
				{																					\
					uxPreviousTask = pxCurrentTCB->uxTCBNumber;										\
					*( unsigned long * ) pcTraceBuffer = ( unsigned long ) xTickCount;		\
					pcTraceBuffer += sizeof( unsigned long );									\
					*( unsigned long * ) pcTraceBuffer = ( unsigned long ) uxPreviousTask;	\
					pcTraceBuffer += sizeof( unsigned long );									\
				}																					\
				else																				\
				{																					\
					xTracing = pdFALSE;																\
				}																					\
			}																						\
		}																							\
	}

#else

	#define vWriteTraceToBuffer()

#endif
/*-----------------------------------------------------------*/

/*
 * Place the task represented by pxTCB into the appropriate ready queue for
 * the task.  It is inserted at the end of the list.  One quirk of this is
 * that if the task being inserted is at the same priority as the currently
 * executing task, then it will only be rescheduled after the currently
 * executing task has been rescheduled.
 */
#define prvAddTaskToReadyQueue( pxTCB )																			\
{																												\
	if( pxTCB->uxPriority > uxTopReadyPriority )																\
	{																											\
		uxTopReadyPriority = pxTCB->uxPriority;																	\
	}																											\
	vListInsertEnd( ( xList * ) &( pxReadyTasksLists[ pxTCB->uxPriority ] ), &( pxTCB->xGenericListItem ) );	\
}
/*-----------------------------------------------------------*/

/*
 * Several functions take an xTaskHandle parameter that can optionally be NULL,
 * where NULL is used to indicate that the handle of the currently executing
 * task should be used in place of the parameter.  This macro simply checks to
 * see if the parameter is NULL and returns a pointer to the appropriate TCB.
 */
#define prvGetTCBFromHandle( pxHandle ) ( ( pxHandle == NULL ) ? ( tskTCB * ) pxCurrentTCB : ( tskTCB * ) pxHandle )


/* File private functions. --------------------------------*/

/*
 * Utility to ready a TCB for a given task.  Mainly just copies the parameters
 * into the TCB structure.
 */
static void prvInitialiseTCBVariables( tskTCB *pxTCB, const char * const pcName, unsigned portBASE_TYPE uxPriority, const xMemoryRegion * const xRegions, unsigned short usStackDepth ) PRIVILEGED_FUNCTION;

/*
 * Utility to ready all the lists used by the scheduler.  This is called
 * automatically upon the creation of the first task.
 */
static void prvInitialiseTaskLists( void ) PRIVILEGED_FUNCTION;

/*
 * The idle task, which as all tasks is implemented as a never ending loop.
 * The idle task is automatically created and added to the ready lists upon
 * creation of the first user task.
 *
 * The portTASK_FUNCTION_PROTO() macro is used to allow port/compiler specific
 * language extensions.  The equivalent prototype for this function is:
 *
 * void prvIdleTask( void *pvParameters );
 *
 */
static portTASK_FUNCTION_PROTO( prvIdleTask, pvParameters );

/*
 * Utility to free all memory allocated by the scheduler to hold a TCB,
 * including the stack pointed to by the TCB.
 *
 * This does not free memory allocated by the task itself (i.e. memory
 * allocated by calls to pvPortMalloc from within the tasks application code).
 */
#if ( ( INCLUDE_vTaskDelete == 1 ) || ( INCLUDE_vTaskCleanUpResources == 1 ) )

	static void prvDeleteTCB( tskTCB *pxTCB ) PRIVILEGED_FUNCTION;

#endif

/*
 * Used only by the idle task.  This checks to see if anything has been placed
 * in the list of tasks waiting to be deleted.  If so the task is cleaned up
 * and its TCB deleted.
 */
static void prvCheckTasksWaitingTermination( void ) PRIVILEGED_FUNCTION;

/*
 * Allocates memory from the heap for a TCB and associated stack.  Checks the
 * allocation was successful.
 */
static tskTCB *prvAllocateTCBAndStack( unsigned short usStackDepth, portSTACK_TYPE *puxStackBuffer ) PRIVILEGED_FUNCTION;

/*
 * Called from vTaskList.  vListTasks details all the tasks currently under
 * control of the scheduler.  The tasks may be in one of a number of lists.
 * prvListTaskWithinSingleList accepts a list and details the tasks from
 * within just that list.
 *
 * THIS FUNCTION IS INTENDED FOR DEBUGGING ONLY, AND SHOULD NOT BE CALLED FROM
 * NORMAL APPLICATION CODE.
 */
#if ( configUSE_TRACE_FACILITY == 1 )

	static void prvListTaskWithinSingleList( const signed char *pcWriteBuffer, xList *pxList, signed char cStatus ) PRIVILEGED_FUNCTION;

#endif

/*
 * When a task is created, the stack of the task is filled with a known value.
 * This function determines the 'high water mark' of the task stack by
 * determining how much of the stack remains at the original preset value.
 */
#if ( ( configUSE_TRACE_FACILITY == 1 ) || ( INCLUDE_uxTaskGetStackHighWaterMark == 1 ) )

	static unsigned short usTaskCheckFreeStackSpace( const unsigned char * pucStackByte ) PRIVILEGED_FUNCTION;

#endif

unsigned portBASE_TYPE xTaskUpdateEvent(xList* pxTaskList, uint32_t mask, int wake);

/*lint +e956 */



/*-----------------------------------------------------------
 * TASK CREATION API documented in task.h
 *----------------------------------------------------------*/

signed portBASE_TYPE xTaskGenericCreate( pdTASK_CODE pxTaskCode, const char * const pcName, unsigned short usStackDepth, void *pvParameters, unsigned portBASE_TYPE uxPriority, xTaskHandle *pxCreatedTask, portSTACK_TYPE *puxStackBuffer, const xMemoryRegion * const xRegions )
{
signed portBASE_TYPE xReturn;
tskTCB * pxNewTCB;

	/* Allocate the memory required by the TCB and stack for the new task,
	checking that the allocation was successful. */
	pxNewTCB = prvAllocateTCBAndStack( usStackDepth, puxStackBuffer );

	if( pxNewTCB != NULL )
	{
		portSTACK_TYPE *pxTopOfStack;

		#if( portUSING_MPU_WRAPPERS == 1 )
			/* Should the task be created in privileged mode? */
			portBASE_TYPE xRunPrivileged;
			if( ( uxPriority & portPRIVILEGE_BIT ) != 0x00 )
			{
				xRunPrivileged = pdTRUE;
			}
			else
			{
				xRunPrivileged = pdFALSE;
			}
			uxPriority &= ~portPRIVILEGE_BIT;
		#endif /* portUSING_MPU_WRAPPERS == 1 */

		/* Calculate the top of stack address.  This depends on whether the
		stack grows from high memory to low (as per the 80x86) or visa versa.
		portSTACK_GROWTH is used to make the result positive or negative as
		required by the port. */
		#if( portSTACK_GROWTH < 0 )
		{
			pxTopOfStack = pxNewTCB->pxStack + ( usStackDepth - 1 );
			pxTopOfStack = ( portSTACK_TYPE * ) ( ( ( unsigned long ) pxTopOfStack ) & ( ( unsigned long ) ~portBYTE_ALIGNMENT_MASK  ) );
		}
		#else
		{
			pxTopOfStack = pxNewTCB->pxStack;

			/* If we want to use stack checking on architectures that use
			a positive stack growth direction then we also need to store the
			other extreme of the stack space. */
			pxNewTCB->pxEndOfStack = pxNewTCB->pxStack + ( usStackDepth - 1 );
		}
		#endif

		/* Setup the newly allocated TCB with the initial state of the task. */
		prvInitialiseTCBVariables( pxNewTCB, pcName, uxPriority, xRegions, usStackDepth );

		/* Initialize the TCB stack to look as if the task was already running,
		but had been interrupted by the scheduler.  The return address is set
		to the start of the task function. Once the stack has been initialised
		the	top of stack variable is updated. */
		#if( portUSING_MPU_WRAPPERS == 1 )
		{
			pxNewTCB->pxTopOfStack = pxPortInitialiseStack( pxTopOfStack, pxTaskCode, pvParameters, xRunPrivileged );
		}
		#else
		{
			pxNewTCB->pxTopOfStack = pxPortInitialiseStack( pxTopOfStack, pxTaskCode, pvParameters );
		}
		#endif

		/* We are going to manipulate the task queues to add this task to a
		ready list, so must make sure no interrupts occur. */
		portENTER_CRITICAL();
		{
			uxCurrentNumberOfTasks++;
			if( uxCurrentNumberOfTasks == ( unsigned portBASE_TYPE ) 1 )
			{
				/* As this is the first task it must also be the current task. */
				pxCurrentTCB =  pxNewTCB;

				/* This is the first task to be created so do the preliminary
				initialisation required.  We will not recover if this call
				fails, but we will report the failure. */
				prvInitialiseTaskLists();
			}
			else
			{
				/* If the scheduler is not already running, make this task the
				current task if it is the highest priority task to be created
				so far. */
				if( xSchedulerRunning == pdFALSE )
				{
					if( pxCurrentTCB->uxPriority <= uxPriority )
					{
						pxCurrentTCB = pxNewTCB;
					}
				}
			}

			/* Remember the top priority to make context switching faster.  Use
			the priority in pxNewTCB as this has been capped to a valid value. */
			if( pxNewTCB->uxPriority > uxTopUsedPriority )
			{
				uxTopUsedPriority = pxNewTCB->uxPriority;
			}

			#if ( configUSE_TRACE_FACILITY == 1 )
			{
				/* Add a counter into the TCB for tracing only. */
				pxNewTCB->uxTCBNumber = uxTaskNumber;
			}
			#endif
			uxTaskNumber++;

			prvAddTaskToReadyQueue( pxNewTCB );

			xReturn = pdPASS;

			traceTASK_CREATE( pxNewTCB );
		}
		portEXIT_CRITICAL();
	}
	else
	{
		xReturn = errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
		traceTASK_CREATE_FAILED( pxNewTCB );
	}

	if( xReturn == pdPASS )
	{
		if( ( void * ) pxCreatedTask != NULL )
		{
			/* Pass the TCB out - in an anonymous way.  The calling function/
			task can use this as a handle to delete the task later if
			required.*/
			*pxCreatedTask = ( xTaskHandle ) pxNewTCB;
		}

		if( xSchedulerRunning != pdFALSE )
		{
			/* If the created task is of a higher priority than the current task
			then it should run now. */
			if( pxCurrentTCB->uxPriority < uxPriority )
			{
				portYIELD_WITHIN_API();
			}
		}
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelete == 1 )

	void vTaskDelete( xTaskHandle pxTaskToDelete )
	{
	tskTCB *pxTCB;

		portENTER_CRITICAL();
		{
			/* Ensure a yield is performed if the current task is being
			deleted. */
			if( pxTaskToDelete == pxCurrentTCB )
			{
				pxTaskToDelete = NULL;
			}

			/* If null is passed in here then we are deleting ourselves. */
			pxTCB = prvGetTCBFromHandle( pxTaskToDelete );

			/* Remove task from the ready list and place in the	termination list.
			This will stop the task from be scheduled.  The idle task will check
			the termination list and free up any memory allocated by the
			scheduler for the TCB and stack. */
			vListRemove( &( pxTCB->xGenericListItem ) );

			/* Is the task waiting on an event also? */
			if( pxTCB->xEventListItem.pvContainer )
			{
				vListRemove( &( pxTCB->xEventListItem ) );
			}

			vListInsertEnd( ( xList * ) &xTasksWaitingTermination, &( pxTCB->xGenericListItem ) );

			/* Increment the ucTasksDeleted variable so the idle task knows
			there is a task that has been deleted and that it should therefore
			check the xTasksWaitingTermination list. */
			++uxTasksDeleted;

			/* Increment the uxTaskNumberVariable also so kernel aware debuggers
			can detect that the task lists need re-generating. */
			uxTaskNumber++;

			traceTASK_DELETE( pxTCB );
		}
		portEXIT_CRITICAL();

		/* Force a reschedule if we have just deleted the current task. */
		if( xSchedulerRunning != pdFALSE )
		{
			if( ( void * ) pxTaskToDelete == NULL )
			{
				portYIELD_WITHIN_API();
			}
		}
	}

#endif






/*-----------------------------------------------------------
 * TASK CONTROL API documented in task.h
 *----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelayUntil == 1 )

	void vTaskDelayUntil( portTickType wake_time )
	{
	portBASE_TYPE xAlreadyYielded;

		vTaskSuspendAll();
		{
			if(  wake_time > xTickCount )
			{
				traceTASK_DELAY_UNTIL();

				/* We must remove ourselves from the ready list before adding
				ourselves to the blocked list as the same list item is used for
				both lists. */
				vListRemove( ( xListItem * ) &( pxCurrentTCB->xGenericListItem ) );

				/* The list item will be inserted in wake time order. */
				listSET_LIST_ITEM_VALUE( &( pxCurrentTCB->xGenericListItem ), wake_time );

				vListInsert( ( xList * ) &xDelayedTaskList, ( xListItem * ) &( pxCurrentTCB->xGenericListItem ) );
			}
		}
		xAlreadyYielded = xTaskResumeAll();

		/* Force a reschedule if xTaskResumeAll has not already done so, we may
		have put ourselves to sleep. */
		if( !xAlreadyYielded )
		{
			portYIELD_WITHIN_API();
		}
	}

#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelay == 1 )

	void vTaskDelay( portTickType xTicksToDelay )
	{
	portTickType xTimeToWake;
	signed portBASE_TYPE xAlreadyYielded = pdFALSE;

		/* A delay time of zero just forces a reschedule. */
		if( xTicksToDelay > ( portTickType ) 0 )
		{
			vTaskSuspendAll();
			{
				traceTASK_DELAY();

				/* A task that is removed from the event list while the
				scheduler is suspended will not get placed in the ready
				list or removed from the blocked list until the scheduler
				is resumed.

				This task cannot be in an event list as it is the currently
				executing task. */

				/* Calculate the time to wake - this may overflow but this is
				not a problem. */
				xTimeToWake = xTickCount + xTicksToDelay;

				/* We must remove ourselves from the ready list before adding
				ourselves to the blocked list as the same list item is used for
				both lists. */
				vListRemove( ( xListItem * ) &( pxCurrentTCB->xGenericListItem ) );

				/* The list item will be inserted in wake time order. */
				listSET_LIST_ITEM_VALUE( &( pxCurrentTCB->xGenericListItem ), xTimeToWake );

				vListInsert( ( xList * ) &xDelayedTaskList, ( xListItem * ) &( pxCurrentTCB->xGenericListItem ) );
			}
			xAlreadyYielded = xTaskResumeAll();
		}

		/* Force a reschedule if xTaskResumeAll has not already done so, we may
		have put ourselves to sleep. */
		if( !xAlreadyYielded )
		{
			portYIELD_WITHIN_API();
		}
	}

#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskPriorityGet == 1 )

	unsigned portBASE_TYPE uxTaskPriorityGet( xTaskHandle pxTask )
	{
	tskTCB *pxTCB;
	unsigned portBASE_TYPE uxReturn;

		portENTER_CRITICAL();
		{
			/* If null is passed in here then we are changing the
			priority of the calling function. */
			pxTCB = prvGetTCBFromHandle( pxTask );
			uxReturn = pxTCB->uxPriority;
		}
		portEXIT_CRITICAL();

		return uxReturn;
	}

#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskPrioritySet == 1 )

	void vTaskPrioritySet( xTaskHandle pxTask, unsigned portBASE_TYPE uxNewPriority )
	{
	tskTCB *pxTCB;
	unsigned portBASE_TYPE uxCurrentPriority, xYieldRequired = pdFALSE;

		/* Ensure the new priority is valid. */
		if( uxNewPriority >= configMAX_PRIORITIES )
		{
			uxNewPriority = configMAX_PRIORITIES - 1;
		}

		portENTER_CRITICAL();
		{
			if( pxTask == pxCurrentTCB )
			{
				pxTask = NULL;
			}

			/* If null is passed in here then we are changing the
			priority of the calling function. */
			pxTCB = prvGetTCBFromHandle( pxTask );

			traceTASK_PRIORITY_SET( pxTask, uxNewPriority );

			#if ( configUSE_MUTEXES == 1 )
			{
				uxCurrentPriority = pxTCB->uxBasePriority;
			}
			#else
			{
				uxCurrentPriority = pxTCB->uxPriority;
			}
			#endif

			if( uxCurrentPriority != uxNewPriority )
			{
				/* The priority change may have readied a task of higher
				priority than the calling task. */
				if( uxNewPriority > uxCurrentPriority )
				{
					if( pxTask != NULL )
					{
						/* The priority of another task is being raised.  If we
						were raising the priority of the currently running task
						there would be no need to switch as it must have already
						been the highest priority task. */
						xYieldRequired = pdTRUE;
					}
				}
				else if( pxTask == NULL )
				{
					/* Setting our own priority down means there may now be another
					task of higher priority that is ready to execute. */
					xYieldRequired = pdTRUE;
				}



				#if ( configUSE_MUTEXES == 1 )
				{
					/* Only change the priority being used if the task is not
					currently using an inherited priority. */
					if( pxTCB->uxBasePriority == pxTCB->uxPriority )
					{
						pxTCB->uxPriority = uxNewPriority;
					}

					/* The base priority gets set whatever. */
					pxTCB->uxBasePriority = uxNewPriority;
				}
				#else
				{
					pxTCB->uxPriority = uxNewPriority;
				}
				#endif

				listSET_LIST_ITEM_VALUE( &( pxTCB->xEventListItem ), ( configMAX_PRIORITIES - ( portTickType ) uxNewPriority ) );

				/* If the task is in the blocked or suspended list we need do
				nothing more than change it's priority variable. However, if
				the task is in a ready list it needs to be removed and placed
				in the queue appropriate to its new priority. */
				if( listIS_CONTAINED_WITHIN( &( pxReadyTasksLists[ uxCurrentPriority ] ), &( pxTCB->xGenericListItem ) ) )
				{
					/* The task is currently in its ready list - remove before adding
					it to it's new ready list.  As we are in a critical section we
					can do this even if the scheduler is suspended. */
					vListRemove( &( pxTCB->xGenericListItem ) );
					prvAddTaskToReadyQueue( pxTCB );
				}

				if( xYieldRequired == pdTRUE )
				{
					portYIELD_WITHIN_API();
				}
			}
		}
		portEXIT_CRITICAL();
	}

#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskSuspend == 1 )

	void vTaskSuspend( xTaskHandle pxTaskToSuspend )
	{
	tskTCB *pxTCB;

		portENTER_CRITICAL();
		{
			/* Ensure a yield is performed if the current task is being
			suspended. */
			if( pxTaskToSuspend == pxCurrentTCB )
			{
				pxTaskToSuspend = NULL;
			}

			/* If null is passed in here then we are suspending ourselves. */
			pxTCB = prvGetTCBFromHandle( pxTaskToSuspend );

			traceTASK_SUSPEND( pxTCB );

			/* Remove task from the ready/delayed list and place in the	suspended list. */
			vListRemove( &( pxTCB->xGenericListItem ) );

			/* Is the task waiting on an event also? */
			if( pxTCB->xEventListItem.pvContainer )
			{
				vListRemove( &( pxTCB->xEventListItem ) );
			}

			vListInsertEnd( ( xList * ) &xSuspendedTaskList, &( pxTCB->xGenericListItem ) );
		}
		portEXIT_CRITICAL();

		/* We may have just suspended the current task. */
		if( ( void * ) pxTaskToSuspend == NULL )
		{
			portYIELD_WITHIN_API();
		}
	}

#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskSuspend == 1 )

	signed portBASE_TYPE xTaskIsTaskSuspended( xTaskHandle xTask )
	{
	portBASE_TYPE xReturn = pdFALSE;
	const tskTCB * const pxTCB = ( tskTCB * ) xTask;

		/* Is the task we are attempting to resume actually in the
		suspended list? */
		if( listIS_CONTAINED_WITHIN( &xSuspendedTaskList, &( pxTCB->xGenericListItem ) ) != pdFALSE )
		{
			/* Has the task already been resumed from within an ISR? */
			if( listIS_CONTAINED_WITHIN( &xPendingReadyList, &( pxTCB->xEventListItem ) ) != pdTRUE )
			{
				/* Is it in the suspended list because it is in the
				Suspended state?  It is possible to be in the suspended
				list because it is blocked on a task with no timeout
				specified. */
				if( listIS_CONTAINED_WITHIN( NULL, &( pxTCB->xEventListItem ) ) == pdTRUE )
				{
					xReturn = pdTRUE;
				}
			}
		}

		return xReturn;
	}

#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskSuspend == 1 )

	void vTaskResume( xTaskHandle pxTaskToResume )
	{
	tskTCB *pxTCB;

		/* Remove the task from whichever list it is currently in, and place
		it in the ready list. */
		pxTCB = ( tskTCB * ) pxTaskToResume;

		/* The parameter cannot be NULL as it is impossible to resume the
		currently executing task. */
		if( ( pxTCB != NULL ) && ( pxTCB != pxCurrentTCB ) )
		{
			portENTER_CRITICAL();
			{
				if( xTaskIsTaskSuspended( pxTCB ) == pdTRUE )
				{
					traceTASK_RESUME( pxTCB );

					/* As we are in a critical section we can access the ready
					lists even if the scheduler is suspended. */
					vListRemove(  &( pxTCB->xGenericListItem ) );
					prvAddTaskToReadyQueue( pxTCB );

					/* We may have just resumed a higher priority task. */
					if( pxTCB->uxPriority >= pxCurrentTCB->uxPriority )
					{
						/* This yield may not cause the task just resumed to run, but
						will leave the lists in the correct state for the next yield. */
						portYIELD_WITHIN_API();
					}
				}
			}
			portEXIT_CRITICAL();
		}
	}

#endif

/*-----------------------------------------------------------*/

#if ( ( INCLUDE_xTaskResumeFromISR == 1 ) && ( INCLUDE_vTaskSuspend == 1 ) )

	portBASE_TYPE xTaskResumeFromISR( xTaskHandle pxTaskToResume )
	{
	portBASE_TYPE xYieldRequired = pdFALSE;
	tskTCB *pxTCB;

		pxTCB = ( tskTCB * ) pxTaskToResume;

		if( xTaskIsTaskSuspended( pxTCB ) == pdTRUE )
		{
			traceTASK_RESUME_FROM_ISR( pxTCB );

			if( uxSchedulerSuspended == ( unsigned portBASE_TYPE ) pdFALSE )
			{
				xYieldRequired = ( pxTCB->uxPriority >= pxCurrentTCB->uxPriority );
				vListRemove(  &( pxTCB->xGenericListItem ) );
				prvAddTaskToReadyQueue( pxTCB );
			}
			else
			{
				/* We cannot access the delayed or ready lists, so will hold this
				task pending until the scheduler is resumed, at which point a
				yield will be performed if necessary. */
				vListInsertEnd( ( xList * ) &( xPendingReadyList ), &( pxTCB->xEventListItem ) );
			}
		}

		return xYieldRequired;
	}

#endif




/*-----------------------------------------------------------
 * PUBLIC SCHEDULER CONTROL documented in task.h
 *----------------------------------------------------------*/


void vTaskStartScheduler( void )
{
portBASE_TYPE xReturn;

	/* Add the idle task at the lowest priority. */
	xReturn = xTaskCreate( prvIdleTask, "idle", tskIDLE_STACK_SIZE, ( void * ) NULL, ( tskIDLE_PRIORITY | portPRIVILEGE_BIT ), ( xTaskHandle * ) NULL );

	if( xReturn == pdPASS )
	{
		/* Interrupts are turned off here, to ensure a tick does not occur
		before or during the call to xPortStartScheduler().  The stacks of
		the created tasks contain a status word with interrupts switched on
		so interrupts will automatically get re-enabled when the first task
		starts to run.

		STEPPING THROUGH HERE USING A DEBUGGER CAN CAUSE BIG PROBLEMS IF THE
		DEBUGGER ALLOWS INTERRUPTS TO BE PROCESSED. */
		portDISABLE_INTERRUPTS();

		xSchedulerRunning = pdTRUE;
		xTickCount = ( portTickType ) 0;

		/* Setting up the timer tick is hardware specific and thus in the
		portable interface. */
		if( xPortStartScheduler() )
		{
			/* Should not reach here as if the scheduler is running the
			function will not return. */
		}
		else
		{
			/* Should only reach here if a task calls xTaskEndScheduler(). */
		}
	}
}
/*-----------------------------------------------------------*/

void vTaskEndScheduler( void )
{
	/* Stop the scheduler interrupts and call the portable scheduler end
	routine so the original ISRs can be restored if necessary.  The port
	layer must ensure interrupts enable	bit is left in the correct state. */
	portDISABLE_INTERRUPTS();
	xSchedulerRunning = pdFALSE;
	vPortEndScheduler();
}
/*----------------------------------------------------------*/

void vTaskSuspendAll( void )
{
	/* A critical section is not required as the variable is of type
	portBASE_TYPE. */
	++uxSchedulerSuspended;
}
/*----------------------------------------------------------*/

signed portBASE_TYPE xTaskResumeAll( void )
{
register tskTCB *pxTCB;
signed portBASE_TYPE xAlreadyYielded = pdFALSE;

	/* It is possible that an ISR caused a task to be removed from an event
	list while the scheduler was suspended.  If this was the case then the
	removed task will have been added to the xPendingReadyList.  Once the
	scheduler has been resumed it is safe to move all the pending ready
	tasks from this list into their appropriate ready list. */
	portENTER_CRITICAL();
	{
		--uxSchedulerSuspended;

		if( uxSchedulerSuspended == ( unsigned portBASE_TYPE ) pdFALSE )
		{
			if( uxCurrentNumberOfTasks > ( unsigned portBASE_TYPE ) 0 )
			{
				portBASE_TYPE xYieldRequired = pdFALSE;

				/* Move any readied tasks from the pending list into the
				appropriate ready list. */
				while( ( pxTCB = ( tskTCB * ) listGET_OWNER_OF_HEAD_ENTRY( &xPendingReadyList ) ) != NULL )
				{
					vListRemove( &( pxTCB->xEventListItem ) );
					vListRemove( &( pxTCB->xGenericListItem ) );
					prvAddTaskToReadyQueue( pxTCB );

					/* If we have moved a task that has a priority higher than
					the current task then we should yield. */
					if( pxTCB->uxPriority >= pxCurrentTCB->uxPriority )
					{
						xYieldRequired = pdTRUE;
					}
				}

				if( uxMissedEvents )
				{
					int i;
					for(i = 0; i < configMAX_PRIORITIES; i++)
					{
						xTaskUpdateEvent(&pxReadyTasksLists[i], uxMissedEvents, 0);
					}
					
					xYieldRequired |= xTaskUpdateEvent(&xDelayedTaskList, uxMissedEvents, 1);
					xYieldRequired |= xTaskUpdateEvent(&xSuspendedTaskList, uxMissedEvents, 1);

					uxMissedEvents = 0;
				}

				/* If any ticks occurred while the scheduler was suspended then
				they should be processed now.  This ensures the tick count does not
				slip, and that any delayed tasks are resumed at the correct time. */
				if( uxMissedTicks > ( unsigned portBASE_TYPE ) 0 )
				{
					vTaskIncrementTick();

					/* As we have processed some ticks it is appropriate to yield
					to ensure the highest priority task that is ready to run is
					the task actually running. */
					#if configUSE_PREEMPTION == 1
					{
						xYieldRequired = pdTRUE;
					}
					#endif
				}

				if( ( xYieldRequired == pdTRUE ) || ( xMissedYield == pdTRUE ) )
				{
					xAlreadyYielded = pdTRUE;
					xMissedYield = pdFALSE;
					portYIELD_WITHIN_API();
				}
			}
		}
	}
	portEXIT_CRITICAL();

	return xAlreadyYielded;
}

unsigned portBASE_TYPE uxTaskGetNumberOfTasks( void )
{
	/* A critical section is not required because the variables are of type
	portBASE_TYPE. */
	return uxCurrentNumberOfTasks;
}
/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )

	void vTaskList( signed char *pcWriteBuffer )
	{
	unsigned portBASE_TYPE uxQueue;

		/* This is a VERY costly function that should be used for debug only.
		It leaves interrupts disabled for a LONG time. */

		vTaskSuspendAll();
		{
			/* Run through all the lists that could potentially contain a TCB and
			report the task name, state and stack high water mark. */

			pcWriteBuffer[ 0 ] = ( signed char ) 0x00;
			strcat( ( char * ) pcWriteBuffer, ( const char * ) "\r\n" );

			uxQueue = uxTopUsedPriority + 1;

			do
			{
				uxQueue--;

				if( !listLIST_IS_EMPTY( &( pxReadyTasksLists[ uxQueue ] ) ) )
				{
					prvListTaskWithinSingleList( pcWriteBuffer, ( xList * ) &( pxReadyTasksLists[ uxQueue ] ), tskREADY_CHAR );
				}
			}while( uxQueue > ( unsigned short ) tskIDLE_PRIORITY );

			if( !listLIST_IS_EMPTY( &xDelayedTaskList ) )
			{
				prvListTaskWithinSingleList( pcWriteBuffer, ( xList * ) &xDelayedTaskList, tskBLOCKED_CHAR );
			}

			#if( INCLUDE_vTaskDelete == 1 )
			{
				if( !listLIST_IS_EMPTY( &xTasksWaitingTermination ) )
				{
					prvListTaskWithinSingleList( pcWriteBuffer, ( xList * ) &xTasksWaitingTermination, tskDELETED_CHAR );
				}
			}
			#endif

			#if ( INCLUDE_vTaskSuspend == 1 )
			{
				if( !listLIST_IS_EMPTY( &xSuspendedTaskList ) )
				{
					prvListTaskWithinSingleList( pcWriteBuffer, ( xList * ) &xSuspendedTaskList, tskSUSPENDED_CHAR );
				}
			}
			#endif
		}
		xTaskResumeAll();
	}

#endif

/*----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )

	void vTaskStartTrace( signed char * pcBuffer, unsigned long ulBufferSize )
	{
		portENTER_CRITICAL();
		{
			pcTraceBuffer = ( signed char * )pcBuffer;
			pcTraceBufferStart = pcBuffer;
			pcTraceBufferEnd = pcBuffer + ( ulBufferSize - tskSIZE_OF_EACH_TRACE_LINE );
			xTracing = pdTRUE;
		}
		portEXIT_CRITICAL();
	}

#endif
/*----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )

	unsigned long ulTaskEndTrace( void )
	{
	unsigned long ulBufferLength;

		portENTER_CRITICAL();
			xTracing = pdFALSE;
		portEXIT_CRITICAL();

		ulBufferLength = ( unsigned long ) ( pcTraceBuffer - pcTraceBufferStart );

		return ulBufferLength;
	}

#endif



/*-----------------------------------------------------------
 * SCHEDULER INTERNALS AVAILABLE FOR PORTING PURPOSES
 * documented in task.h
 *----------------------------------------------------------*/


void vTaskIncrementTick( void )
{
	extern volatile int64_t systick_time;

	/* Called by the portable layer each time a tick interrupt occurs.
	Increments the tick then checks to see if the new tick value will cause any
	tasks to be unblocked. */
	if( uxSchedulerSuspended == ( unsigned portBASE_TYPE ) pdFALSE )
	{
		// pas d'overflow, on est sur 64bits
		xTickCount = systick_time;
		uxMissedTicks = 0;

		/* See if this tick has made a timeout expire. */
		tskTCB *pxTCB;

		while( ( pxTCB = ( tskTCB * ) listGET_OWNER_OF_HEAD_ENTRY( &xDelayedTaskList ) ) != NULL )
		{
			if( xTickCount < listGET_LIST_ITEM_VALUE( &( pxTCB->xGenericListItem ) ) )
			{
				break;
			}
			vListRemove( &( pxTCB->xGenericListItem ) );
			/* Is the task waiting on an event also? */
			if( pxTCB->xEventListItem.pvContainer )
			{
				vListRemove( &( pxTCB->xEventListItem ) );
			}
			prvAddTaskToReadyQueue( pxTCB );
		}
	}
	else
	{
		uxMissedTicks = 1;
	}

	traceTASK_INCREMENT_TICK( xTickCount );
}

/*-----------------------------------------------------------*/

#if ( ( INCLUDE_vTaskCleanUpResources == 1 ) && ( INCLUDE_vTaskSuspend == 1 ) )

	void vTaskCleanUpResources( void )
	{
	unsigned short usQueue;
	volatile tskTCB *pxTCB;

		usQueue = ( unsigned short ) uxTopUsedPriority + ( unsigned short ) 1;

		/* Remove any TCB's from the ready queues. */
		do
		{
			usQueue--;

			while( !listLIST_IS_EMPTY( &( pxReadyTasksLists[ usQueue ] ) ) )
			{
				listGET_OWNER_OF_NEXT_ENTRY( pxTCB, &( pxReadyTasksLists[ usQueue ] ) );
				vListRemove( ( xListItem * ) &( pxTCB->xGenericListItem ) );

				prvDeleteTCB( ( tskTCB * ) pxTCB );
			}
		}while( usQueue > ( unsigned short ) tskIDLE_PRIORITY );

		/* Remove any TCB's from the delayed queue. */
		while( !listLIST_IS_EMPTY( &xDelayedTaskList ) )
		{
			listGET_OWNER_OF_NEXT_ENTRY( pxTCB, &xDelayedTaskList );
			vListRemove( ( xListItem * ) &( pxTCB->xGenericListItem ) );

			prvDeleteTCB( ( tskTCB * ) pxTCB );
		}

		while( !listLIST_IS_EMPTY( &xSuspendedTaskList ) )
		{
			listGET_OWNER_OF_NEXT_ENTRY( pxTCB, &xSuspendedTaskList );
			vListRemove( ( xListItem * ) &( pxTCB->xGenericListItem ) );

			prvDeleteTCB( ( tskTCB * ) pxTCB );
		}
	}

#endif
/*-----------------------------------------------------------*/

#if ( configUSE_APPLICATION_TASK_TAG == 1 )

	void vTaskSetApplicationTaskTag( xTaskHandle xTask, pdTASK_HOOK_CODE pxTagValue )
	{
	tskTCB *xTCB;

		/* If xTask is NULL then we are setting our own task hook. */
		if( xTask == NULL )
		{
			xTCB = ( tskTCB * ) pxCurrentTCB;
		}
		else
		{
			xTCB = ( tskTCB * ) xTask;
		}

		/* Save the hook function in the TCB.  A critical section is required as
		the value can be accessed from an interrupt. */
		portENTER_CRITICAL();
			xTCB->pxTaskTag = pxTagValue;
		portEXIT_CRITICAL();
	}

#endif
/*-----------------------------------------------------------*/

#if ( configUSE_APPLICATION_TASK_TAG == 1 )

	pdTASK_HOOK_CODE xTaskGetApplicationTaskTag( xTaskHandle xTask )
	{
	tskTCB *xTCB;
	pdTASK_HOOK_CODE xReturn;

		/* If xTask is NULL then we are setting our own task hook. */
		if( xTask == NULL )
		{
			xTCB = ( tskTCB * ) pxCurrentTCB;
		}
		else
		{
			xTCB = ( tskTCB * ) xTask;
		}

		/* Save the hook function in the TCB.  A critical section is required as
		the value can be accessed from an interrupt. */
		portENTER_CRITICAL();
			xReturn = xTCB->pxTaskTag;
		portEXIT_CRITICAL();

		return xReturn;
	}

#endif
/*-----------------------------------------------------------*/

#if ( configUSE_APPLICATION_TASK_TAG == 1 )

	portBASE_TYPE xTaskCallApplicationTaskHook( xTaskHandle xTask, void *pvParameter )
	{
	tskTCB *xTCB;
	portBASE_TYPE xReturn;

		/* If xTask is NULL then we are calling our own task hook. */
		if( xTask == NULL )
		{
			xTCB = ( tskTCB * ) pxCurrentTCB;
		}
		else
		{
			xTCB = ( tskTCB * ) xTask;
		}

		if( xTCB->pxTaskTag != NULL )
		{
			xReturn = xTCB->pxTaskTag( pvParameter );
		}
		else
		{
			xReturn = pdFAIL;
		}

		return xReturn;
	}

#endif
/*-----------------------------------------------------------*/
void vTaskSwitchContext( void )
{
	if( uxSchedulerSuspended != ( unsigned portBASE_TYPE ) pdFALSE )
	{
		/* The scheduler is currently suspended - do not allow a context
		switch. */
		xMissedYield = pdTRUE;
		return;
	}

	traceTASK_SWITCHED_OUT();

#if ( configGENERATE_RUN_TIME_STATS == 1 )
	pxCurrentTCB->cpu_time_used += systick_get_time_from_isr() - pxCurrentTCB->last_start_tick_count;
#endif

	taskFIRST_CHECK_FOR_STACK_OVERFLOW();
	taskSECOND_CHECK_FOR_STACK_OVERFLOW();

	// TODO : gérer un temps max pour les taches de même priorité
	tskTCB *pxTCB;
	while(xDelayedTaskList.uxNumberOfItems)
	{
		pxTCB = xDelayedTaskList.xListEnd.pxNext->pvOwner;

		if( systick_reconfigure_from_isr(pxTCB->xGenericListItem.xItemValue) == 0)
		{
			break;
		}

		// on doit réveiller la tache tout de suite et programmer le systick pour une autre date
		vListRemove( &( pxTCB->xGenericListItem ) );

		// Is the task waiting on an event also?
		if( pxTCB->xEventListItem.pvContainer )
		{
			vListRemove( &( pxTCB->xEventListItem ) );
		}
		prvAddTaskToReadyQueue( pxTCB );
	}

	/* Find the highest priority queue that contains ready tasks. */

	while( listLIST_IS_EMPTY( &( pxReadyTasksLists[ uxTopReadyPriority ] ) ) )
	{
		--uxTopReadyPriority;
	}

	/* listGET_OWNER_OF_NEXT_ENTRY walks through the list, so the tasks of the
	same priority get an equal share of the processor time. */
	listGET_OWNER_OF_NEXT_ENTRY( pxCurrentTCB, &( pxReadyTasksLists[ uxTopReadyPriority ] ) );

#if ( configGENERATE_RUN_TIME_STATS == 1 )
	pxCurrentTCB->last_start_tick_count = systick_get_time_from_isr();
#endif

	traceTASK_SWITCHED_IN();
	vWriteTraceToBuffer();
}
/*-----------------------------------------------------------*/

void vTaskPlaceOnEventList( const xList * const pxEventList, portTickType xTicksToWait )
{
portTickType xTimeToWake;

	/* THIS FUNCTION MUST BE CALLED WITH INTERRUPTS DISABLED OR THE
	SCHEDULER SUSPENDED. */

	/* Place the event list item of the TCB in the appropriate event list.
	This is placed in the list in priority order so the highest priority task
	is the first to be woken by the event. */
	vListInsert( ( xList * ) pxEventList, ( xListItem * ) &( pxCurrentTCB->xEventListItem ) );

	/* We must remove ourselves from the ready list before adding ourselves
	to the blocked list as the same list item is used for both lists.  We have
	exclusive access to the ready lists as the scheduler is locked. */
	vListRemove( ( xListItem * ) &( pxCurrentTCB->xGenericListItem ) );


	#if ( INCLUDE_vTaskSuspend == 1 )
	{
		if( xTicksToWait == portMAX_DELAY )
		{
			/* Add ourselves to the suspended task list instead of a delayed task
			list to ensure we are not woken by a timing event.  We will block
			indefinitely. */
			vListInsertEnd( ( xList * ) &xSuspendedTaskList, ( xListItem * ) &( pxCurrentTCB->xGenericListItem ) );
		}
		else
		{
			/* Calculate the time at which the task should be woken if the event does
			not occur.  This may overflow but this doesn't matter. */
			xTimeToWake = xTickCount + xTicksToWait;

			listSET_LIST_ITEM_VALUE( &( pxCurrentTCB->xGenericListItem ), xTimeToWake );

			vListInsert( ( xList * ) &xDelayedTaskList, ( xListItem * ) &( pxCurrentTCB->xGenericListItem ) );
		}
	}
	#else
	{
			/* Calculate the time at which the task should be woken if the event does
			not occur.  This may overflow but this doesn't matter. */
			xTimeToWake = xTickCount + xTicksToWait;

			listSET_LIST_ITEM_VALUE( &( pxCurrentTCB->xGenericListItem ), xTimeToWake );

			vListInsert( ( xList * ) &xDelayedTaskList, ( xListItem * ) &( pxCurrentTCB->xGenericListItem ) );
	}
	#endif
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xTaskRemoveFromEventList( const xList * const pxEventList )
{
tskTCB *pxUnblockedTCB;
portBASE_TYPE xReturn;

	/* THIS FUNCTION MUST BE CALLED WITH INTERRUPTS DISABLED OR THE
	SCHEDULER SUSPENDED.  It can also be called from within an ISR. */

	/* The event list is sorted in priority order, so we can remove the
	first in the list, remove the TCB from the delayed list, and add
	it to the ready list.

	If an event is for a queue that is locked then this function will never
	get called - the lock count on the queue will get modified instead.  This
	means we can always expect exclusive access to the event list here. */
	pxUnblockedTCB = ( tskTCB * ) listGET_OWNER_OF_HEAD_ENTRY( pxEventList );
	vListRemove( &( pxUnblockedTCB->xEventListItem ) );

	if( uxSchedulerSuspended == ( unsigned portBASE_TYPE ) pdFALSE )
	{
		vListRemove( &( pxUnblockedTCB->xGenericListItem ) );
		prvAddTaskToReadyQueue( pxUnblockedTCB );
	}
	else
	{
		/* We cannot access the delayed or ready lists, so will hold this
		task pending until the scheduler is resumed. */
		vListInsertEnd( ( xList * ) &( xPendingReadyList ), &( pxUnblockedTCB->xEventListItem ) );
	}

	if( pxUnblockedTCB->uxPriority >= pxCurrentTCB->uxPriority )
	{
		/* Return true if the task removed from the event list has
		a higher priority than the calling task.  This allows
		the calling task to know if it should force a context
		switch now. */
		xReturn = pdTRUE;
	}
	else
	{
		xReturn = pdFALSE;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

void vTaskSetTimeOutState( xTimeOutType * const pxTimeOut )
{
	pxTimeOut->xTimeOnEntering = xTickCount;
}
/*-----------------------------------------------------------*/

portBASE_TYPE xTaskCheckForTimeOut( xTimeOutType * const pxTimeOut, portTickType * const pxTicksToWait )
{
portBASE_TYPE xReturn;

	portENTER_CRITICAL();
	{
		#if ( INCLUDE_vTaskSuspend == 1 )
			/* If INCLUDE_vTaskSuspend is set to 1 and the block time specified is
			the maximum block time then the task should block indefinitely, and
			therefore never time out. */
			if( *pxTicksToWait == portMAX_DELAY )
			{
				xReturn = pdFALSE;
			}
			else /* We are not blocking indefinitely, perform the checks below. */
		#endif

		if( ( ( portTickType ) ( ( portTickType ) xTickCount - ( portTickType ) pxTimeOut->xTimeOnEntering ) ) < ( portTickType ) *pxTicksToWait )
		{
			/* Not a genuine timeout. Adjust parameters for time remaining. */
			*pxTicksToWait -= ( ( portTickType ) xTickCount - ( portTickType ) pxTimeOut->xTimeOnEntering );
			vTaskSetTimeOutState( pxTimeOut );
			xReturn = pdFALSE;
		}
		else
		{
			xReturn = pdTRUE;
		}
	}
	portEXIT_CRITICAL();

	return xReturn;
}
/*-----------------------------------------------------------*/

void vTaskMissedYield( void )
{
	xMissedYield = pdTRUE;
}

/*
 * -----------------------------------------------------------
 * The Idle task.
 * ----------------------------------------------------------
 *
 * The portTASK_FUNCTION() macro is used to allow port/compiler specific
 * language extensions.  The equivalent prototype for this function is:
 *
 * void prvIdleTask( void *pvParameters );
 *
 */

static portTASK_FUNCTION( prvIdleTask, pvParameters )
{
	/* Stop warnings. */
	( void ) pvParameters;

	for( ;; )
	{
		/* See if any tasks have been deleted. */
		prvCheckTasksWaitingTermination();

		#if ( configUSE_PREEMPTION == 0 )
		{
			/* If we are not using preemption we keep forcing a task switch to
			see if any other task has become available.  If we are using
			preemption we don't need to do this as any task becoming available
			will automatically get the processor anyway. */
			taskYIELD();
		}
		#endif

		#if ( ( configUSE_PREEMPTION == 1 ) && ( configIDLE_SHOULD_YIELD == 1 ) )
		{
			/* When using preemption tasks of equal priority will be
			timesliced.  If a task that is sharing the idle priority is ready
			to run then the idle task should yield before the end of the
			timeslice.

			A critical region is not required here as we are just reading from
			the list, and an occasional incorrect value will not matter.  If
			the ready list at the idle priority contains more than one task
			then a task other than the idle task is ready to execute. */
			if( listCURRENT_LIST_LENGTH( &( pxReadyTasksLists[ tskIDLE_PRIORITY ] ) ) > ( unsigned portBASE_TYPE ) 1 )
			{
				taskYIELD();
			}
		}
		#endif

		#if ( configUSE_IDLE_HOOK == 1 )
		{
			extern void vApplicationIdleHook( void );

			/* Call the user defined function from within the idle task.  This
			allows the application designer to add background functionality
			without the overhead of a separate task.
			NOTE: vApplicationIdleHook() MUST NOT, UNDER ANY CIRCUMSTANCES,
			CALL A FUNCTION THAT MIGHT BLOCK. */
			vApplicationIdleHook();
		}
		#endif
	}
} /*lint !e715 pvParameters is not accessed but all task functions require the same prototype. */







/*-----------------------------------------------------------
 * File private functions documented at the top of the file.
 *----------------------------------------------------------*/



static void prvInitialiseTCBVariables( tskTCB *pxTCB, const char * const pcName, unsigned portBASE_TYPE uxPriority, const xMemoryRegion * const xRegions, unsigned short usStackDepth )
{
	/* Store the function name in the TCB. */
	#if configMAX_TASK_NAME_LEN > 1
	{
		/* Don't bring strncpy into the build unnecessarily. */
		strncpy( pxTCB->pcTaskName, pcName, ( unsigned short ) configMAX_TASK_NAME_LEN );
	}
	#endif
	pxTCB->pcTaskName[ ( unsigned short ) configMAX_TASK_NAME_LEN - ( unsigned short ) 1 ] = 0;

	/* This is used as an array index so must ensure it's not too large.  First
	remove the privilege bit if one is present. */
	if( uxPriority >= configMAX_PRIORITIES )
	{
		uxPriority = configMAX_PRIORITIES - 1;
	}

	pxTCB->uxPriority = uxPriority;
	#if ( configUSE_MUTEXES == 1 )
	{
		pxTCB->uxBasePriority = uxPriority;
	}
	#endif

	vListInitialiseItem( &( pxTCB->xGenericListItem ) );
	vListInitialiseItem( &( pxTCB->xEventListItem ) );

	/* Set the pxTCB as a link back from the xListItem.  This is so we can get
	back to	the containing TCB from a generic item in a list. */
	listSET_LIST_ITEM_OWNER( &( pxTCB->xGenericListItem ), pxTCB );

	/* Event lists are always in priority order. */
	listSET_LIST_ITEM_VALUE( &( pxTCB->xEventListItem ), configMAX_PRIORITIES - ( portTickType ) uxPriority );
	listSET_LIST_ITEM_OWNER( &( pxTCB->xEventListItem ), pxTCB );

	pxTCB->event = 0;
	pxTCB->eventMask = 0;

	#if ( portCRITICAL_NESTING_IN_TCB == 1 )
	{
		pxTCB->uxCriticalNesting = ( unsigned portBASE_TYPE ) 0;
	}
	#endif

	#if ( configUSE_APPLICATION_TASK_TAG == 1 )
	{
		pxTCB->pxTaskTag = NULL;
	}
	#endif

#if ( configGENERATE_RUN_TIME_STATS == 1 )
	pxTCB->cpu_time_used = 0UL;
	pxTCB->last_start_tick_count = 0;
#endif

	#if ( portUSING_MPU_WRAPPERS == 1 )
	{
		vPortStoreTaskMPUSettings( &( pxTCB->xMPUSettings ), xRegions, pxTCB->pxStack, usStackDepth );
	}
	#else
	{
		( void ) xRegions;
		( void ) usStackDepth;
	}
	#endif
}
/*-----------------------------------------------------------*/

#if ( portUSING_MPU_WRAPPERS == 1 )

	void vTaskAllocateMPURegions( xTaskHandle xTaskToModify, const xMemoryRegion * const xRegions )
	{
	tskTCB *pxTCB;
	
		if( xTaskToModify == pxCurrentTCB )
		{
			xTaskToModify = NULL;
		}

		/* If null is passed in here then we are deleting ourselves. */
		pxTCB = prvGetTCBFromHandle( xTaskToModify );

        vPortStoreTaskMPUSettings( &( pxTCB->xMPUSettings ), xRegions, NULL, 0 );
	}
	/*-----------------------------------------------------------*/
#endif

static void prvInitialiseTaskLists( void )
{
unsigned portBASE_TYPE uxPriority;

	for( uxPriority = 0; uxPriority < configMAX_PRIORITIES; uxPriority++ )
	{
		vListInitialise( &( pxReadyTasksLists[ uxPriority ] ) );
	}

	vListInitialise( &xDelayedTaskList );
	vListInitialise( &xPendingReadyList );

	#if ( INCLUDE_vTaskDelete == 1 )
	{
		vListInitialise( ( xList * ) &xTasksWaitingTermination );
	}
	#endif

	#if ( INCLUDE_vTaskSuspend == 1 )
	{
		vListInitialise( &xSuspendedTaskList );
	}
	#endif
}
/*-----------------------------------------------------------*/

static void prvCheckTasksWaitingTermination( void )
{
	#if ( INCLUDE_vTaskDelete == 1 )
	{
		portBASE_TYPE xListIsEmpty;

		/* ucTasksDeleted is used to prevent vTaskSuspendAll() being called
		too often in the idle task. */
		if( uxTasksDeleted > ( unsigned portBASE_TYPE ) 0 )
		{
			vTaskSuspendAll();
				xListIsEmpty = listLIST_IS_EMPTY( &xTasksWaitingTermination );
			xTaskResumeAll();

			if( !xListIsEmpty )
			{
				tskTCB *pxTCB;

				portENTER_CRITICAL();
				{
					pxTCB = ( tskTCB * ) listGET_OWNER_OF_HEAD_ENTRY( &xTasksWaitingTermination );
					vListRemove( &( pxTCB->xGenericListItem ) );
					--uxCurrentNumberOfTasks;
					--uxTasksDeleted;
				}
				portEXIT_CRITICAL();

				prvDeleteTCB( pxTCB );
			}
		}
	}
	#endif
}
/*-----------------------------------------------------------*/

static tskTCB *prvAllocateTCBAndStack( unsigned short usStackDepth, portSTACK_TYPE *puxStackBuffer )
{
tskTCB *pxNewTCB;

	/* Allocate space for the TCB.  Where the memory comes from depends on
	the implementation of the port malloc function. */
	pxNewTCB = ( tskTCB * ) pvPortMalloc( sizeof( tskTCB ) );

	if( pxNewTCB != NULL )
	{
		/* Allocate space for the stack used by the task being created.
		The base of the stack memory stored in the TCB so the task can
		be deleted later if required. */
		pxNewTCB->pxStack = ( portSTACK_TYPE * ) pvPortMallocAligned( ( ( ( size_t )usStackDepth ) * sizeof( portSTACK_TYPE ) ), puxStackBuffer );

		if( pxNewTCB->pxStack == NULL )
		{
			/* Could not allocate the stack.  Delete the allocated TCB. */
			vPortFree( pxNewTCB );
			pxNewTCB = NULL;
		}
		else
		{
			/* Just to help debugging. */
			memset( pxNewTCB->pxStack, tskSTACK_FILL_BYTE, usStackDepth * sizeof( portSTACK_TYPE ) );
		}
	}

	return pxNewTCB;
}
/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )

	static void prvListTaskWithinSingleList( const signed char *pcWriteBuffer, xList *pxList, signed char cStatus )
	{
	volatile tskTCB *pxNextTCB, *pxFirstTCB;
	unsigned short usStackRemaining;

		/* Write the details of all the TCB's in pxList into the buffer. */
		listGET_OWNER_OF_NEXT_ENTRY( pxFirstTCB, pxList );
		do
		{
			listGET_OWNER_OF_NEXT_ENTRY( pxNextTCB, pxList );
			#if ( portSTACK_GROWTH > 0 )
			{
				usStackRemaining = usTaskCheckFreeStackSpace( ( unsigned char * ) pxNextTCB->pxEndOfStack );
			}
			#else
			{
				usStackRemaining = usTaskCheckFreeStackSpace( ( unsigned char * ) pxNextTCB->pxStack );
			}
			#endif			
			
			sprintf( pcStatusString, ( char * ) "%s\t\t%c\t%u\t%u\t%u\r\n", pxNextTCB->pcTaskName, cStatus, ( unsigned int ) pxNextTCB->uxPriority, usStackRemaining, ( unsigned int ) pxNextTCB->uxTCBNumber );
			strcat( ( char * ) pcWriteBuffer, ( char * ) pcStatusString );

		} while( pxNextTCB != pxFirstTCB );
	}

#endif
/*-----------------------------------------------------------*/

#if ( ( configUSE_TRACE_FACILITY == 1 ) || ( INCLUDE_uxTaskGetStackHighWaterMark == 1 ) )

	static unsigned short usTaskCheckFreeStackSpace( const unsigned char * pucStackByte )
	{
	register unsigned short usCount = 0;

		while( *pucStackByte == tskSTACK_FILL_BYTE )
		{
			pucStackByte -= portSTACK_GROWTH;
			usCount++;
		}

		usCount /= sizeof( portSTACK_TYPE );

		return usCount;
	}

#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskGetStackHighWaterMark == 1 )

	unsigned portBASE_TYPE uxTaskGetStackHighWaterMark( xTaskHandle xTask )
	{
	tskTCB *pxTCB;
	unsigned char *pcEndOfStack;
	unsigned portBASE_TYPE uxReturn;

		pxTCB = prvGetTCBFromHandle( xTask );

		#if portSTACK_GROWTH < 0
		{
			pcEndOfStack = ( unsigned char * ) pxTCB->pxStack;
		}
		#else
		{
			pcEndOfStack = ( unsigned char * ) pxTCB->pxEndOfStack;
		}
		#endif

		uxReturn = ( unsigned portBASE_TYPE ) usTaskCheckFreeStackSpace( pcEndOfStack );

		return uxReturn;
	}

#endif
/*-----------------------------------------------------------*/

#if ( ( INCLUDE_vTaskDelete == 1 ) || ( INCLUDE_vTaskCleanUpResources == 1 ) )

	static void prvDeleteTCB( tskTCB *pxTCB )
	{
		/* Free up the memory allocated by the scheduler for the task.  It is up to
		the task to free any memory allocated at the application level. */
		vPortFreeAligned( pxTCB->pxStack );
		vPortFree( pxTCB );
	}

#endif


/*-----------------------------------------------------------*/

#if ( INCLUDE_xTaskGetCurrentTaskHandle == 1 )

	xTaskHandle xTaskGetCurrentTaskHandle( void )
	{
	xTaskHandle xReturn;

		/* A critical section is not required as this is not called from
		an interrupt and the current TCB will always be the same for any
		individual execution thread. */
		xReturn = pxCurrentTCB;

		return xReturn;
	}

#endif

/*-----------------------------------------------------------*/

#if ( INCLUDE_xTaskGetSchedulerState == 1 )

	portBASE_TYPE xTaskGetSchedulerState( void )
	{
	portBASE_TYPE xReturn;

		if( xSchedulerRunning == pdFALSE )
		{
			xReturn = taskSCHEDULER_NOT_STARTED;
		}
		else
		{
			if( uxSchedulerSuspended == ( unsigned portBASE_TYPE ) pdFALSE )
			{
				xReturn = taskSCHEDULER_RUNNING;
			}
			else
			{
				xReturn = taskSCHEDULER_SUSPENDED;
			}
		}

		return xReturn;
	}

#endif
/*-----------------------------------------------------------*/

#if ( configUSE_MUTEXES == 1 )

	void vTaskPriorityInherit( xTaskHandle * const pxMutexHolder )
	{
	tskTCB * const pxTCB = ( tskTCB * ) pxMutexHolder;

		if( pxTCB->uxPriority < pxCurrentTCB->uxPriority )
		{
			/* Adjust the mutex holder state to account for its new priority. */
			listSET_LIST_ITEM_VALUE( &( pxTCB->xEventListItem ), configMAX_PRIORITIES - ( portTickType ) pxCurrentTCB->uxPriority );

			/* If the task being modified is in the ready state it will need to
			be moved in to a new list. */
			if( listIS_CONTAINED_WITHIN( &( pxReadyTasksLists[ pxTCB->uxPriority ] ), &( pxTCB->xGenericListItem ) ) )
			{
				vListRemove( &( pxTCB->xGenericListItem ) );

				/* Inherit the priority before being moved into the new list. */
				pxTCB->uxPriority = pxCurrentTCB->uxPriority;
				prvAddTaskToReadyQueue( pxTCB );
			}
			else
			{
				/* Just inherit the priority. */
				pxTCB->uxPriority = pxCurrentTCB->uxPriority;
			}
		}
	}

#endif
/*-----------------------------------------------------------*/

#if ( configUSE_MUTEXES == 1 )

	void vTaskPriorityDisinherit( xTaskHandle * const pxMutexHolder )
	{
	tskTCB * const pxTCB = ( tskTCB * ) pxMutexHolder;

		if( pxMutexHolder != NULL )
		{
			if( pxTCB->uxPriority != pxTCB->uxBasePriority )
			{
				/* We must be the running task to be able to give the mutex back.
				Remove ourselves from the ready list we currently appear in. */
				vListRemove( &( pxTCB->xGenericListItem ) );

				/* Disinherit the priority before adding ourselves into the new
				ready list. */
				pxTCB->uxPriority = pxTCB->uxBasePriority;
				listSET_LIST_ITEM_VALUE( &( pxTCB->xEventListItem ), configMAX_PRIORITIES - ( portTickType ) pxTCB->uxPriority );
				prvAddTaskToReadyQueue( pxTCB );
			}
		}
	}

#endif
/*-----------------------------------------------------------*/

#if ( portCRITICAL_NESTING_IN_TCB == 1 )

	void vTaskEnterCritical( void )
	{
		portDISABLE_INTERRUPTS();

		if( xSchedulerRunning != pdFALSE )
		{
			pxCurrentTCB->uxCriticalNesting++;
		}
	}

#endif
/*-----------------------------------------------------------*/

#if ( portCRITICAL_NESTING_IN_TCB == 1 )

void vTaskExitCritical( void )
{
	if( xSchedulerRunning != pdFALSE )
	{
		if( pxCurrentTCB->uxCriticalNesting > 0 )
		{
			pxCurrentTCB->uxCriticalNesting--;

			if( pxCurrentTCB->uxCriticalNesting == 0 )
			{
				portENABLE_INTERRUPTS();
			}
		}
	}
}

#endif
/*-----------------------------------------------------------*/

//! @todo description
uint32_t vTaskWaitEvent(uint32_t mask, portTickType timeout)
{
	portTickType xTimeToWake;

	portENTER_CRITICAL();
	pxCurrentTCB->eventMask = mask;
	if(! (pxCurrentTCB->event & mask) )
	{
		// We must remove ourselves from the ready list before adding ourselves
		// to the blocked list as the same list item is used for both lists.  We have
		// exclusive access to the ready lists as the scheduler is locked.
		vListRemove( ( xListItem * ) &( pxCurrentTCB->xGenericListItem ) );

		if( timeout == portMAX_DELAY )
		{
			// Add ourselves to the suspended task list instead of a delayed task
			// list to ensure we are not woken by a timing event.  We will block
			// indefinitely.
			vListInsertEnd( ( xList * ) &xSuspendedTaskList, ( xListItem * ) &( pxCurrentTCB->xGenericListItem ) );
		}
		else
		{
			// Calculate the time at which the task should be woken if the event does
			// not occur.  This may overflow but this doesn't matter.
			xTimeToWake = xTickCount + timeout;

			listSET_LIST_ITEM_VALUE( &( pxCurrentTCB->xGenericListItem ), xTimeToWake );

			vListInsert( ( xList * ) &xDelayedTaskList, ( xListItem * ) &( pxCurrentTCB->xGenericListItem ) );
		}

		portYIELD_WITHIN_API();
	}

	portEXIT_CRITICAL();

	// lecture de event atomique
	return pxCurrentTCB->event;
}

//! @todo description
unsigned portBASE_TYPE xTaskUpdateEvent(xList* pxTaskList, uint32_t mask, int wake)
{
	tskTCB *pxTcb;

	xListItem *pxListItem;
	xListItem *next;
	unsigned portBASE_TYPE xHigherPriorityTaskWoken = 0;

	if( pxTaskList->uxNumberOfItems )
	{
		pxListItem = (xListItem *) pxTaskList->xListEnd.pxNext;
		do
		{
			pxTcb = pxListItem->pvOwner;
			next = (xListItem*) pxListItem->pxNext;

			pxTcb->event |= mask;
			if( wake && pxTcb->event & pxTcb->eventMask)
			{
				if( pxTcb->uxPriority >= pxCurrentTCB->uxPriority )
				{
					xHigherPriorityTaskWoken |= 1;
				}
				vListRemove( &( pxTcb->xGenericListItem ) );
				prvAddTaskToReadyQueue( pxTcb );

				pxTcb->eventMask = 0;
			}

			pxListItem = next;
		}while( pxListItem != (xListItem*) &pxTaskList->xListEnd);
	}

	return xHigherPriorityTaskWoken;
}

//! Le but est de mettre à jour les variables event de toutes les taches
//! et de placer toutes les taches qui attendent un des évènements
//! parmi "mask" dans la liste "ready".
//! Une fois ce travail réalisé, si on a réveillé une tache plus prioritaire,
//! il faut laisser la main.
void vTaskSetEvent(uint32_t mask)
{
	unsigned portBASE_TYPE xHigherPriorityTaskWoken = 0;
	unsigned portBASE_TYPE xAlreadyYielded = 0;
	int i;

	vTaskSuspendAll();
	for(i = 0; i < configMAX_PRIORITIES; i++)
	{
		xTaskUpdateEvent(&pxReadyTasksLists[i], mask, 0);
	}

	xHigherPriorityTaskWoken |= xTaskUpdateEvent(&xDelayedTaskList, mask, 1);
	xHigherPriorityTaskWoken |= xTaskUpdateEvent(&xSuspendedTaskList, mask, 1);
	xAlreadyYielded = xTaskResumeAll();

	if( !xAlreadyYielded && xHigherPriorityTaskWoken)
	{
		portYIELD_WITHIN_API();
	}
}

//! cf vTaskSetEvent mais depuis une interruption
unsigned portBASE_TYPE vTaskSetEventFromISR(uint32_t mask)
{
	unsigned portBASE_TYPE i;
	unsigned portBASE_TYPE xHigherPriorityTaskWoken = 0;

	if( uxSchedulerSuspended == 0)
	{
		for(i = 0; i < configMAX_PRIORITIES; i++)
		{
			xTaskUpdateEvent(&pxReadyTasksLists[i], mask, 0);
		}

		xHigherPriorityTaskWoken |= xTaskUpdateEvent(&xDelayedTaskList, mask, 1);
		xHigherPriorityTaskWoken |= xTaskUpdateEvent(&xSuspendedTaskList, mask, 1);
	}
	else
	{
		uxMissedEvents |= mask;
	}

	return xHigherPriorityTaskWoken;
}

//! @todo description
void vTaskClearEvent(uint32_t mask)
{
	portENTER_CRITICAL();
	pxCurrentTCB->event &= ~mask;
	portEXIT_CRITICAL();
}

//! @todo description
uint32_t vTaskGetEvent()
{
	// lecture atomique de event
	return pxCurrentTCB->event;
}
