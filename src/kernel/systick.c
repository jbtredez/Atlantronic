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

static struct systime systick_time;
static struct systime systick_time_start_match;
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
__attribute__((optimize("-O2"))) void isr_systick( void )
{
	// preemption
	*(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;

	portSET_INTERRUPT_MASK_FROM_ISR();
	systick_time.ms++;
	vTaskIncrementTick();
	portCLEAR_INTERRUPT_MASK_FROM_ISR( 0 );
}

struct systime systick_get_time()
{
	struct systime t;
	portENTER_CRITICAL();
	t = systick_get_time_from_isr();
	portEXIT_CRITICAL();

	return t;
}

__attribute__((optimize("-O2"))) struct systime systick_get_time_from_isr()
{
	// formule pour eviter les debordements sur 32 bits
	systick_time.ns = (1000 * (SysTick->LOAD - SysTick->VAL)) / RCC_SYSCLK_MHZ;
	return systick_time;
}

struct systime systick_get_match_time()
{
	struct systime t;
	portENTER_CRITICAL();
	t = systick_get_time_from_isr();
	t = timediff(t, systick_time_start_match);
	portEXIT_CRITICAL();

	return t;
}

void systick_start_match_from_isr()
{
	if( systick_time_start_match.ms == 0 && systick_time_start_match.ns == 0)
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

struct systime timediff(const struct systime t2, const struct systime t1)
{
	struct systime t;
	if( t2.ns - t1.ns < 0 )
	{
		t.ms = t2.ms - t1.ms - 1;
		t.ns = 1000000 + t2.ns - t1.ns;
	}
	else
	{
		t.ms = t2.ms - t1.ms;
		t.ns = t2.ns - t1.ns;
	}
	return t;
}

struct systime timeadd(const struct systime t1, const struct systime t2)
{
	struct systime t;
	t.ms = t1.ms + t2.ms;
	t.ns = t1.ns + t2.ns;
	if( t.ns > 1000000 )
	{
		t.ns -= 1000000;
		t.ms++;
	}

	return t;
}
