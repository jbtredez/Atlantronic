//! @file systick.c
//! @brief Systick module
//! @author Atlantronic

#include "kernel/systick.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/FreeRTOSConfig.h"
#include "kernel/portmacro.h"

#define portNVIC_INT_CTRL			( ( volatile unsigned long *) 0xe000ed04 )
#define portNVIC_PENDSVSET			0x10000000

static volatile int64_t systick_time;
static volatile int64_t systick_time_start_match;
void isr_systick( void );
extern void vTaskIncrementTick( );

extern void vTaskStartScheduler(void);

static int systick_module_init()
{
	log(LOG_INFO, "Lancement de l'ordonanceur");

	// 4 bits for pre-emption priority, 0 bit sub priority
	SCB->AIRCR = 0x05FA0300;

	vTaskStartScheduler();

	return ERR_SYSTICK;
}

module_init(systick_module_init, INIT_SYSTICK);

//! interruption systick, on declenche l'IT de changement de contexte
//! on desactive les IT avant de toucher au systick pour ne pas entrer
//! en concurence avec l'IT de changement de contexte
void isr_systick( void )
{
	// preemption
	*(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;

	portSET_INTERRUPT_MASK_FROM_ISR();
	systick_time += SysTick->LOAD;
	vTaskIncrementTick();
	portCLEAR_INTERRUPT_MASK_FROM_ISR( 0 );
}

int64_t systick_get_time()
{
	int64_t t;
	portENTER_CRITICAL();
	t = systick_get_time_from_isr();
	portEXIT_CRITICAL();

	return t;
}

int64_t systick_get_time_from_isr()
{
	return systick_time + SysTick->LOAD - SysTick->VAL;
}

int64_t systick_get_match_time()
{
	int64_t t;
	portENTER_CRITICAL();
	t = systick_get_time_from_isr() - systick_time_start_match;
	portEXIT_CRITICAL();

	return t;
}

void systick_start_match_from_isr()
{
	if( systick_time_start_match == 0)
	{
		systick_time_start_match = systick_get_time_from_isr();
	}
}

void systick_start_match()
{
	portENTER_CRITICAL();
	systick_start_match_from_isr();
	portEXIT_CRITICAL();
}
