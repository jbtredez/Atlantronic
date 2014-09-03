#include "exti.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"
#include "kernel/FreeRTOS.h"

long exti_unwanted()
{
	while( 1 ) ;
}

ExtiIsr exti_isr[16];

int exti_module_init()
{
	unsigned int i;
	for(i = 0; i < sizeof(exti_isr) / sizeof(exti_isr[0]); i++)
	{
		exti_isr[i] = exti_unwanted;
	}

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	NVIC_SetPriority(EXTI0_IRQn, PRIORITY_IRQ_EXTI0);
	NVIC_SetPriority(EXTI1_IRQn, PRIORITY_IRQ_EXTI1);
	NVIC_SetPriority(EXTI2_IRQn, PRIORITY_IRQ_EXTI2);
	NVIC_SetPriority(EXTI3_IRQn, PRIORITY_IRQ_EXTI3);
	NVIC_SetPriority(EXTI4_IRQn, PRIORITY_IRQ_EXTI4);
	NVIC_SetPriority(EXTI9_5_IRQn, PRIORITY_IRQ_EXTI9_5);
	NVIC_SetPriority(EXTI15_10_IRQn, PRIORITY_IRQ_EXTI15_10);

	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	return 0;
}

module_init(exti_module_init, INIT_GPIO);


int exti_register(ExtiPort port, uint8_t pin, uint8_t type, ExtiIsr isr)
{
	if( pin >= 16 )
	{
		return -1;
	}

	if( exti_isr[pin] != exti_unwanted )
	{
		return -1;
	}

	exti_isr[pin] = isr;

	uint32_t lineMask = (1 << pin);
	int exticr_id = pin >> 2;
	int exticr_part = (pin & 0x03)*4;

	SYSCFG->EXTICR[exticr_id] |= port << exticr_part;

	EXTI->IMR |= lineMask;

	if( type & EXTI_TYPE_DOWN )
	{
		EXTI->FTSR |= lineMask;
	}

	if( type & EXTI_TYPE_UP )
	{
		EXTI->RTSR |= lineMask;
	}

	return 0;
}

void isr_exti0(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;

	portSET_INTERRUPT_MASK_FROM_ISR();

	if( EXTI->PR & EXTI_PR_PR0)
	{
		EXTI->PR = EXTI_PR_PR0;
		xHigherPriorityTaskWoken = exti_isr[0]();
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_exti1(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;

	portSET_INTERRUPT_MASK_FROM_ISR();

	if( EXTI->PR & EXTI_PR_PR1)
	{
		EXTI->PR = EXTI_PR_PR1;
		xHigherPriorityTaskWoken = exti_isr[1]();
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_exti2(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;

	portSET_INTERRUPT_MASK_FROM_ISR();

	if( EXTI->PR & EXTI_PR_PR2)
	{
		EXTI->PR = EXTI_PR_PR2;
		xHigherPriorityTaskWoken = exti_isr[2]();
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_exti3(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;

	portSET_INTERRUPT_MASK_FROM_ISR();

	if( EXTI->PR & EXTI_PR_PR3)
	{
		EXTI->PR = EXTI_PR_PR3;
		xHigherPriorityTaskWoken = exti_isr[3]();
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_exti4(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;

	portSET_INTERRUPT_MASK_FROM_ISR();

	if( EXTI->PR & EXTI_PR_PR4)
	{
		EXTI->PR = EXTI_PR_PR4;
		xHigherPriorityTaskWoken = exti_isr[4]();
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_exti9_5(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;

	portSET_INTERRUPT_MASK_FROM_ISR();

	if( EXTI->PR & EXTI_PR_PR5)
	{
		EXTI->PR = EXTI_PR_PR5;
		xHigherPriorityTaskWoken = exti_isr[5]();
	}
	if( EXTI->PR & EXTI_PR_PR6)
	{
		EXTI->PR = EXTI_PR_PR6;
		xHigherPriorityTaskWoken |= exti_isr[6]();
	}
	if( EXTI->PR & EXTI_PR_PR7)
	{
		EXTI->PR = EXTI_PR_PR7;
		xHigherPriorityTaskWoken |= exti_isr[7]();
	}
	if( EXTI->PR & EXTI_PR_PR8)
	{
		EXTI->PR = EXTI_PR_PR8;
		xHigherPriorityTaskWoken |= exti_isr[8]();
	}
	if( EXTI->PR & EXTI_PR_PR9)
	{
		EXTI->PR = EXTI_PR_PR9;
		xHigherPriorityTaskWoken |= exti_isr[9]();
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_exti15_10(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;

	portSET_INTERRUPT_MASK_FROM_ISR();

	if( EXTI->PR & EXTI_PR_PR10)
	{
		EXTI->PR = EXTI_PR_PR10;
		xHigherPriorityTaskWoken = exti_isr[10]();
	}
	if( EXTI->PR & EXTI_PR_PR11)
	{
		EXTI->PR = EXTI_PR_PR11;
		xHigherPriorityTaskWoken |= exti_isr[11]();
	}
	if( EXTI->PR & EXTI_PR_PR12)
	{
		EXTI->PR = EXTI_PR_PR12;
		xHigherPriorityTaskWoken |= exti_isr[12]();
	}
	if( EXTI->PR & EXTI_PR_PR13)
	{
		EXTI->PR = EXTI_PR_PR13;
		xHigherPriorityTaskWoken |= exti_isr[13]();
	}
	if( EXTI->PR & EXTI_PR_PR14)
	{
		EXTI->PR = EXTI_PR_PR14;
		xHigherPriorityTaskWoken |= exti_isr[14]();
	}
	if( EXTI->PR & EXTI_PR_PR15)
	{
		EXTI->PR = EXTI_PR_PR15;
		xHigherPriorityTaskWoken |= exti_isr[15]();
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}
