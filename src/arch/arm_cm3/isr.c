//! @file isr.c
//! @brief isr
//! @author Jean-Baptiste Trédez

#include "io/debug.h"
#include "rtos/FreeRTOSConfig.h"

void isr_reset(void) __attribute__ ((naked));
static void isr_nmi(void);
static void isr_hard_fault(void);
static void isr_mpu_fault(void);
static void isr_bus_fault(void);
static void isr_usage_fault(void);
static void isr_debug_monitor(void);
static void isr_unexpected(void);
static void isr_context_switch( void ) __attribute__ ((naked));

void isr_usart3(void) __attribute__((weak, alias("isr_unexpected") ));

extern void isr_systick(void);
extern void vPortSVCHandler( void );
extern int main(void);

extern unsigned long __text_end__;
extern unsigned long __data_start__;
extern unsigned long __data_end__;
extern unsigned long __bss_start__;
extern unsigned long __bss_end__;
extern unsigned long _stack_top;

__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
	// core level - cortex-m3
	(void (*)(void))(&_stack_top),          // The initial stack pointer
	isr_reset,                              // The reset handler
	isr_nmi,                                // The NMI handler
	isr_hard_fault,                         // The hard fault handler
	isr_mpu_fault,                          // The MPU fault handler
	isr_bus_fault,                          // The bus fault handler
	isr_usage_fault,                        // The usage fault handler
	0,                                      // Reserved
	0,                                      // Reserved
	0,                                      // Reserved
	0,                                      // Reserved
	vPortSVCHandler,                        // SVCall handler
	isr_debug_monitor,                      // Debug monitor handler
	0,                                      // Reserved
	isr_context_switch,                     // The PendSV handler
	isr_systick,                            // The SysTick handler

	// chip level
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_usart3,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
  (void*)0,
  (void*)0,
  (void*)0,
  (void*)0,
  (void*)0,
  (void*)0,
  (void*)0,
//  (void*)0xF108F85F // RAM boot.
};

void isr_reset(void)
{
	unsigned long *pulSrc, *pulDest;

	//
	// Copy the data segment initializers from flash to SRAM.
	//
	pulSrc = &__text_end__;
	for(pulDest = &__data_start__; pulDest < &__data_end__; )
	{
		*pulDest++ = *pulSrc++;
	}

    //
    // Zero fill the bss segment.  This is done with inline assembly since this
    // will clear the value of pulDest if it is not kept in a register.
    //
    __asm volatile
    (
		"    ldr     r0, =__bss_start__          \n"
		"    ldr     r1, =__bss_end__            \n"
		"    mov     r2, #0                      \n"
		"    .thumb_func                         \n"
		"zero_loop:                              \n"
		"        cmp     r0, r1                  \n"
		"        it      lt                      \n"
		"        strlt   r2, [r0], #4            \n"
		"        blt     zero_loop                 "
	);

	main();
}

void isr_context_switch( void )
{
	/* This is a naked function. */

	__asm volatile
	(
		"	mrs r0, psp							\n"
		"										\n"
		"	ldr	r3, pxCurrentTCBConst			\n" /* Get the location of the current TCB. */
		"	ldr	r2, [r3]						\n"
		"										\n"
		"	stmdb r0!, {r4-r11}					\n" /* Save the remaining registers. */
		"	str r0, [r2]						\n" /* Save the new top of stack into the first member of the TCB. */
		"										\n"
		"	stmdb sp!, {r3, lr} 				\n"
		"	mov r0, %0							\n"
		"	msr basepri, r0						\n"
		"	bl vTaskSwitchContext				\n"
		"	mov r0, #0							\n"
		"	msr basepri, r0						\n"
		"	ldmia sp!, {r3, lr} 				\n"
		"										\n"	/* Restore the context, including the critical nesting count. */
		"	ldr r1, [r3]						\n"
		"	ldr r0, [r1]						\n" /* The first item in pxCurrentTCB is the task top of stack. */
		"	ldmia r0!, {r4-r11}					\n" /* Pop the registers. */
		"	msr psp, r0							\n"
		"	bx lr								\n"
		"										\n"
		"	.align 2							\n"
		"pxCurrentTCBConst: .word pxCurrentTCB	\n"
		::"i"(configMAX_SYSCALL_INTERRUPT_PRIORITY)
	);
}

static void isr_nmi(void)
{
	while(debug_mode())
	{

	}
}

static void isr_hard_fault(void)
{
	while(debug_mode())
	{

	}
}

static void isr_mpu_fault(void)
{
	while(debug_mode())
	{

	}
}

static void isr_bus_fault(void)
{
	while(debug_mode())
	{

	}
}

static void isr_usage_fault(void)
{
	while(debug_mode())
	{

	}
}

static void isr_debug_monitor(void)
{
	while(debug_mode())
	{

	}
}

static void isr_unexpected(void)
{
	while(debug_mode())
	{

	}
}
