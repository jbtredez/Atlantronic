//! @file systick.c
//! @brief Systick module
//! @author Atlantronic

#include "kernel/systick.h"
#include "kernel/module.h"
#include "kernel/FreeRTOSConfig.h"
#include "kernel/portmacro.h"
#include "kernel/log.h"

#define portNVIC_INT_CTRL			( ( volatile unsigned long *) 0xe000ed04 )
#define portNVIC_PENDSVSET			0x10000000


// --> 1 cycle pour le rechargement du systick (on a compté systick_last_load_used + 1)
// --> SYSTICK_REPROGRAM_TIME-1 cycles (tests) entre la lecture du SysTick->VAL  (systick_time -= SysTick->VAL) et le rearmement du timer
// TODO a voir
#define SYSTICK_REPROGRAM_TIME     0

volatile int32_t systick_last_load_used;
volatile int64_t systick_time;
volatile int64_t systick_time_start_match;
void isr_systick( void );

extern void vTaskStartScheduler(void);
extern void vTaskIncrementTick( void );

static int systick_module_init()
{
	log_info("Lancement de l'ordonanceur");

	systick_time_start_match = 0;

	NVIC_SetPriority(PendSV_IRQn, PRIORITY_IRQ_PENDSV);
	NVIC_SetPriority(SysTick_IRQn, PRIORITY_IRQ_SYSTICK);

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

//! fonction qui ne doit être utilisée que par l'interruption de l'ordonanceur
//! qui désactive les autres IT touchant au systick
int systick_reconfigure_from_isr(uint64_t tick)
{
	int32_t val = SysTick->VAL;
	if( SysTick->CTRL & SysTick_CTRL_COUNTFLAG)
	{
		systick_time += systick_last_load_used;
		systick_last_load_used = SYSTICK_MAXCOUNT;
		val = SysTick->VAL;
	}

	systick_time += systick_last_load_used + SYSTICK_REPROGRAM_TIME - val;
	int64_t delta = tick - systick_time;
	if(delta < 500)
	{
		// le temps de faire la fin du context switch, on sera bon.
		systick_time -= systick_last_load_used + SYSTICK_REPROGRAM_TIME - val;
		return -1;
	}
	
	SysTick->LOAD = (tick - systick_time) & SYSTICK_MAXCOUNT;
	SysTick->VAL = 0x00;   // recharge du systick au prochain cycle
	systick_last_load_used = SysTick->LOAD;
	SysTick->LOAD = SYSTICK_MAXCOUNT;

	return 0;
}

//! interruption systick, on declenche l'IT de changement de contexte
//! on desactive les IT avant de toucher au systick pour ne pas entrer
//! en concurence avec l'IT de changement de contexte
void isr_systick( void )
{
	*(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;

	portSET_INTERRUPT_MASK();
	if( SysTick->CTRL & SysTick_CTRL_COUNTFLAG)
	{
		systick_time += systick_last_load_used;
		systick_last_load_used = SYSTICK_MAXCOUNT;
	}

	vTaskIncrementTick();

	portCLEAR_INTERRUPT_MASK();
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
	uint32_t val = SysTick->VAL;
	if( SysTick->CTRL & SysTick_CTRL_COUNTFLAG)
	{
		systick_time += systick_last_load_used;
		systick_last_load_used = SYSTICK_MAXCOUNT;
		val = SysTick->VAL;
	}

	return systick_time + systick_last_load_used - val;
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
