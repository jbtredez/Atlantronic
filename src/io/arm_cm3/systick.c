//! @file systick.c
//! @brief Systick module
//! @author Jean-Baptiste Trédez

#include "io/systick.h"
#include "module.h"
#include "rtos/FreeRTOSConfig.h"
#include "portmacro.h"

#define portNVIC_INT_CTRL			( ( volatile unsigned long *) 0xe000ed04 )
#define portNVIC_SYSPRI2			( ( volatile unsigned long *) 0xe000ed20 )
#define portNVIC_PENDSVSET			0x10000000
#define portNVIC_PENDSV_PRI			( ( ( unsigned long ) configKERNEL_INTERRUPT_PRIORITY ) << 16 )
#define portNVIC_SYSTICK_PRI		( ( ( unsigned long ) configKERNEL_INTERRUPT_PRIORITY ) << 24 )

// --> 1 cycle pour le rechargement du systick (on a compté systick_last_load_used + 1)
// --> SYSTICK_REPROGRAM_TIME-1 cycles (tests) entre la lecture du SysTick->VAL  (systick_time -= SysTick->VAL) et le rearmement du timer
#define SYSTICK_REPROGRAM_TIME     10

volatile int32_t systick_last_load_used;
volatile int64_t systick_time;
volatile int64_t systick_time_start_match;
void isr_systick( void );

extern void vTaskIncrementTick( void );

static int systick_module_init()
{
	systick_time_start_match = 0;

	// Make PendSV, CallSV and SysTick the same priroity as the kernel
	*(portNVIC_SYSPRI2) |= portNVIC_PENDSV_PRI;
	*(portNVIC_SYSPRI2) |= portNVIC_SYSTICK_PRI;
//	NVIC_SetPriority(PendSV_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
//	NVIC_SetPriority(SysTick_IRQn-1, (1<<__NVIC_PRIO_BITS) - 1);

	systick_time = 0;
	SysTick->VAL = 0x00;
	systick_last_load_used = SYSTICK_MAXCOUNT;
	SysTick->LOAD = SYSTICK_MAXCOUNT;
	SysTick->CTRL = (1 << SYSTICK_CLKSOURCE) | (1<<SYSTICK_ENABLE) | (1<<SYSTICK_TICKINT);
	SysTick->VAL = 0x00; // recharge du systick au prochain cycle
	systick_last_load_used = SysTick->LOAD;

	vTaskStartScheduler();

	return ERR_SYSTICK;
}

module_init(systick_module_init, INIT_SYSTICK);

int systick_reconfigure(uint64_t tick)
{
	systick_time += systick_last_load_used + SYSTICK_REPROGRAM_TIME;
	int64_t delta = tick - systick_time + SysTick->VAL;
	if(delta < 100)
	{
		// le temps de faire la fin du context switch, on sera bon.
		systick_time = systick_time - systick_last_load_used - SYSTICK_REPROGRAM_TIME;
		return -1;
	}

	// section "continue" sans branchement pour calculer le nombre de cycle entre la lecture
	// de val et la reprogramation de sysclk afin de ne pas perdre de tick
	systick_time -= SysTick->VAL;
	
	SysTick->LOAD = (tick - systick_time) & SYSTICK_MAXCOUNT;
	SysTick->VAL = 0x00;   // recharge du systick au prochain cycle
	systick_last_load_used = SysTick->LOAD;
	SysTick->LOAD = SYSTICK_MAXCOUNT;

	return 0;
}

void isr_systick( void )
{
	*(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;

	portSET_INTERRUPT_MASK();

	systick_time += systick_last_load_used;
	systick_last_load_used = SYSTICK_MAXCOUNT;
	vTaskIncrementTick();

	portCLEAR_INTERRUPT_MASK();
}

int64_t systick_get_time()
{
	int64_t t;
	portENTER_CRITICAL();
	t = systick_time + systick_last_load_used - systick_time_start_match - SysTick->VAL;
	portEXIT_CRITICAL();

	return t;
}

int64_t systick_get_match_time()
{
	int64_t t;
	portENTER_CRITICAL();
	t = systick_time + systick_last_load_used - SysTick->VAL;
	portEXIT_CRITICAL();

	return t;
}

void systick_start_match()
{
	portENTER_CRITICAL();
	if( systick_time_start_match == 0)
	{
		systick_time_start_match = systick_time + systick_last_load_used - SysTick->VAL;
	}
	portEXIT_CRITICAL();
}