//! @file systick.c
//! @brief Systick module
//! @author Atlantronic

#include "kernel/systick.h"
#include "kernel/module.h"
#ifdef STM32F10X_CL
#include "kernel/FreeRTOSConfig.h"
#include "kernel/portmacro.h"
#include "kernel/log.h"
#endif
#ifdef STM32F4XX
#include "kernel2/FreeRTOSConfig.h"
#include "kernel2/portmacro.h"
#endif

#define portNVIC_INT_CTRL			( ( volatile unsigned long *) 0xe000ed04 )
#define portNVIC_PENDSVSET			0x10000000
#ifdef STM32F10X_CL
// --> 1 cycle pour le rechargement du systick (on a compté systick_last_load_used + 1)
// --> SYSTICK_REPROGRAM_TIME-1 cycles (tests) entre la lecture du SysTick->VAL  (systick_time -= SysTick->VAL) et le rearmement du timer
// TODO a voir
#define SYSTICK_REPROGRAM_TIME     0
static volatile int32_t systick_last_load_used;
#endif

static volatile int64_t systick_time;
static volatile int64_t systick_time_start_match;
void isr_systick( void );

#ifdef STM32F10X_CL
extern void vTaskIncrementTick( portTickType time );
#endif
#ifdef STM32F4XX
extern void vTaskIncrementTick( );
#endif

extern void vTaskStartScheduler(void);

static int systick_module_init()
{
#ifdef STM32F10X_CL
	log(LOG_INFO, "Lancement de l'ordonanceur");
#endif

#ifdef STM32F10X_CL
	NVIC_SetPriority(PendSV_IRQn, PRIORITY_IRQ_PENDSV);
	#define SysTick_LOAD_RELOAD_Msk        SYSTICK_MAXCOUNT
	#define SysTick_CTRL_CLKSOURCE_Msk     (1 << SYSTICK_CLKSOURCE)
	#define SysTick_CTRL_TICKINT_Msk       (1 << SYSTICK_TICKINT)
	#define SysTick_CTRL_ENABLE_Msk        (1 << SYSTICK_ENABLE)

	NVIC_SetPriority(SysTick_IRQn, PRIORITY_IRQ_SYSTICK);

	SysTick->VAL = 0x00;
	systick_last_load_used = SysTick_LOAD_RELOAD_Msk;
	SysTick->LOAD = SysTick_LOAD_RELOAD_Msk;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
	SysTick->VAL = 0x00; // recharge du systick au prochain cycle
	systick_last_load_used = SysTick->LOAD;
#endif
	// 4 bits for pre-emption priority, 0 bit sub priority
	SCB->AIRCR = 0x05FA0300;

	vTaskStartScheduler();

	return ERR_SYSTICK;
}

module_init(systick_module_init, INIT_SYSTICK);
#ifdef STM32F10X_CL
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
#endif
//! interruption systick, on declenche l'IT de changement de contexte
//! on desactive les IT avant de toucher au systick pour ne pas entrer
//! en concurence avec l'IT de changement de contexte
void isr_systick( void )
{
	// preemption
	*(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;

#ifdef STM32F10X_CL
	portSET_INTERRUPT_MASK();
	if( SysTick->CTRL & SysTick_CTRL_COUNTFLAG)
	{
		systick_time += systick_last_load_used;
		systick_last_load_used = SYSTICK_MAXCOUNT;
	}
	vTaskIncrementTick(systick_time);
	portCLEAR_INTERRUPT_MASK();
#endif

#ifdef STM32F4XX
	portSET_INTERRUPT_MASK_FROM_ISR();
	systick_time += 168000;
	vTaskIncrementTick();
	portCLEAR_INTERRUPT_MASK_FROM_ISR( 0 );
#endif
}

int64_t systick_get_time()
{
	int64_t t;
	portENTER_CRITICAL();
	t = systick_get_time_from_isr();
	portEXIT_CRITICAL();

	return t;
}
#if STM32F10X_CL
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
#endif
#ifdef STM32F4XX
int64_t systick_get_time_from_isr()
{
	return systick_time + 168000 - SysTick->VAL;
}
#endif

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
