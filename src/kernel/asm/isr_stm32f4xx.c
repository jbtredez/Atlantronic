//! @file isr.c
//! @brief isr
//! @author Atlantronic

#include "kernel/FreeRTOS.h"
#include "priority.h"

struct stack_t
{
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;
	uint32_t psr;
};

#define SCB_CFSR_MEMMANAG_IACCVIOL        0x01
#define SCB_CFSR_MEMMANAG_DACCVIOL        0x02
#define SCB_CFSR_MEMMANAG_MUNSTKERR       0x08
#define SCB_CFSR_MEMMANAG_MSTKERR         0x10
#define SCB_CFSR_MEMMANAG_MLSPERR         0x20
#define SCB_CFSR_MEMMANAG_MMARVALID       0x80
#define SCB_CFSR_BUS_IBUSERR             0x100
#define SCB_CFSR_BUS_PRECISERR           0x200
#define SCB_CFSR_BUS_IMPRECISERR         0x400
#define SCB_CFSR_BUS_UNSTKERR            0x800
#define SCB_CFSR_BUS_STKERR             0x1000
#define SCB_CFSR_BUS_LSPERR             0x2000
#define SCB_CFSR_BUS_BFARVALID          0x8000
#define SCB_CFSR_USAGE_UNDEFINSTR      0x10000
#define SCB_CFSR_USAGE_INVSTATE        0x20000
#define SCB_CFSR_USAGE_INVPC           0x40000
#define SCB_CFSR_USAGE_NOCP            0x80000
#define SCB_CFSR_USAGE_UNALIGNED      0x100000
#define SCB_CFSR_USAGE_DIVBYZERO      0x200000

void isr_reset(void) __attribute__ ((naked)); //!< fonction de reset (point d'entrée)
static void isr_nmi(void); //!< interruption nmi
static void isr_hard_fault(void) __attribute__ ((naked)); //!< interruption "hard fault"
static void isr_mpu_fault(void); //!< interruption d'erreur sur le mpu
static void isr_bus_fault(void); //!< interruption d'erreur sur le bus
static void isr_usage_fault(void); //!< interruption d'erreur "usage fault"
static void isr_svc( void ) __attribute__ (( naked )); //!< interruption svc (lancement de la première tache)
static void isr_debug_monitor(void); //!< interruption debug monitor
static void isr_unexpected(void); //!< interruption imprévue
static void isr_context_switch( void ) __attribute__ ((naked)); //!< changement de contexte
void isr_systick(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption systick

// interruptions dma
void isr_dma1_stream0(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma1_0
void isr_dma1_stream1(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma1_1
void isr_dma1_stream2(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma1_3
void isr_dma1_stream3(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma1_5
void isr_dma1_stream4(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma1_4
void isr_dma1_stream5(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma1_5
void isr_dma1_stream6(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma1_6
void isr_dma1_stream7(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma1_7

void isr_dma2_stream0(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma2_0
void isr_dma2_stream1(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma2_1
void isr_dma2_stream4(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma2_4
void isr_dma2_stream3(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma2_3
void isr_dma2_stream6(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma2_6

void isr_spi1(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption spi1

void isr_usart2(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption usart2
void isr_usart3(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption usart3
void isr_uart4(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption uart4
void isr_uart5(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption uart5
void isr_usart6(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption usart6

void isr_otg_fs(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption usb otg
void isr_can1_tx(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption transmission CAN1
void isr_can1_rx0(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption reception CAN1
void isr_can1_sce(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption status change error CAN1

void isr_exti3(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption exti3
void isr_exti9_5(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption exti 5 a 9
void isr_exti15_10(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption exti 10 a 15

void isr_tim6(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption tim6

extern void __main(void) __attribute__((noreturn)); //!< fonction main à lancer une fois les segments data et bss initialisés en sram

extern unsigned long _sidata;
extern unsigned long _sdata; //!< debut du segment data en sram (segment à remplir au reset) (cf arm-elf.ld)
extern unsigned long _edata; //!< fin du segment data en sram (segment à remplir au reset) (cf arm-elf.ld)
extern unsigned long _sbss; //!< debut du segment bss en sram (segment à initialiser à zéro) (cf arm-elf.ld)
extern unsigned long _ebss; //!< fin du segment bss en sram (segment à initialiser à zéro) (cf arm-elf.ld)
extern unsigned long _estack; //!< haut de la ram (-16 par précaution) => début de la stack principale (cf arm-elf.ld)
extern void (*__preinit_array_start []) (void); //!< debut du tableau de pointeur de fonction preinit (c++)
extern void (*__preinit_array_end []) (void); //!< fin du tableau de pointeur de fonction preinit (c++)
extern void (*__init_array_start []) (void); //!< debut du tableau de pointeur de fonction init (c++)
extern void (*__init_array_end []) (void); //!< fin du tableau de pointeur de fonction init (c++)

__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
	// processeur - cortex-m4
	(void (*)(void))(&_estack),          // Initialisation du pointer de la stack principale
	isr_reset, 
	isr_nmi,
	isr_hard_fault,
	isr_mpu_fault,
	isr_bus_fault,
	isr_usage_fault,
	0,                                      // Reserved
	0,                                      // Reserved
	0,                                      // Reserved
	0,                                      // Reserved
	isr_svc,
	isr_debug_monitor,
	0,                                      // Reserved
	isr_context_switch,
	isr_systick,

	// périphériques
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_exti3,
	isr_unexpected,
	isr_dma1_stream0,
	isr_dma1_stream1,
	isr_dma1_stream2,
	isr_dma1_stream3,
	isr_dma1_stream4,
	isr_dma1_stream5,
	isr_dma1_stream6,
	isr_unexpected,
	isr_can1_tx,
	isr_can1_rx0,
	isr_unexpected,
	isr_can1_sce,
	isr_exti9_5,
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
	isr_spi1,
	isr_unexpected,
	isr_unexpected,
	isr_usart2,
	isr_usart3,
	isr_exti15_10,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_dma1_stream7,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_uart4,
	isr_uart5,
	isr_tim6,
	isr_unexpected,
	isr_dma2_stream0,
	isr_dma2_stream1,
	isr_unexpected,
	isr_dma2_stream3,
	isr_dma2_stream4,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_otg_fs,
	isr_unexpected,
	isr_dma2_stream6,
	isr_unexpected,
	isr_usart6,
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
};

void isr_reset(void)
{
	unsigned long *pulSrc, *pulDest;
	int i;

	// activation des IT isr_mpu_fault, isr_bus_fault, isr_usage_fault
	//SCB->SHCSR |= 0x00007000;
	//NVIC_SetPriority(UsageFault_IRQn, PRIORITY_IRQ_USAGE_FAULT);

	//
	// Copy the data segment initializers from flash to SRAM.
	//
	pulSrc = &_sidata;
	for(pulDest = &_sdata; pulDest < &_edata; )
	{
		*pulDest++ = *pulSrc++;
	}

	//
	// Zero fill the bss segment.  This is done with inline assembly since this
	// will clear the value of pulDest if it is not kept in a register.
	//
	__asm volatile
	(
		"    ldr     r0, =_sbss                  \n"
		"    ldr     r1, =_ebss                  \n"
		"    mov     r2, #0                      \n"
		"    .thumb_func                         \n"
		"zero_loop:                              \n"
		"        cmp     r0, r1                  \n"
		"        it      lt                      \n"
		"        strlt   r2, [r0], #4            \n"
		"        blt     zero_loop                 "
	);

	// appel des constructeurs c++
	int count = __preinit_array_end - __preinit_array_start;
	for (i = 0; i < count; i++)
	{
		__preinit_array_start[i] ();
	}

	count = __init_array_end - __init_array_start;
	for (i = 0; i < count; i++)
	{
		__init_array_start[i] ();
	}

	__main();
}

void isr_svc( void )
{
	__asm volatile
	(
		"	ldr	r3, pxCurrentTCBConst2		\n" /* Restore the context. */
		"	ldr r1, [r3]					\n" /* Use pxCurrentTCBConst to get the pxCurrentTCB address. */
		"	ldr r0, [r1]					\n" /* The first item in pxCurrentTCB is the task top of stack. */
		"	ldmia r0!, {r4-r11, r14}		\n" /* Pop the registers that are not automatically saved on exception entry and the critical nesting count. */
		"	msr psp, r0						\n" /* Restore the task stack pointer. */
		"	mov r0, #0 						\n"
		"	msr	basepri, r0					\n"
		"	bx r14							\n"
		"									\n"
		"	.align 2						\n"
		"pxCurrentTCBConst2: .word pxCurrentTCB				\n"
	);
}

void isr_context_switch( void )
{
	__asm volatile
	(
		"	mrs r0, psp							\n"
		"										\n"
		"	ldr	r3, pxCurrentTCBConst			\n" /* Get the location of the current TCB. */
		"	ldr	r2, [r3]						\n"
		"										\n"
		"	tst r14, #0x10						\n" /* Is the task using the FPU context?  If so, push high vfp registers. */
		"	it eq								\n"
		"	vstmdbeq r0!, {s16-s31}				\n"
		"										\n"
		"	stmdb r0!, {r4-r11, r14}			\n" /* Save the core registers. */
		"										\n"
		"	str r0, [r2]						\n" /* Save the new top of stack into the first member of the TCB. */
		"										\n"
		"	stmdb sp!, {r3, r14}				\n"
		"	mov r0, %0 							\n"
		"	msr basepri, r0						\n"
		"	bl vTaskSwitchContext				\n"
		"	mov r0, #0							\n"
		"	msr basepri, r0						\n"
		"	ldmia sp!, {r3, r14}				\n"
		"										\n"
		"	ldr r1, [r3]						\n" /* The first item in pxCurrentTCB is the task top of stack. */
		"	ldr r0, [r1]						\n"
		"										\n"
		"	ldmia r0!, {r4-r11, r14}			\n" /* Pop the core registers. */
		"										\n"
		"	tst r14, #0x10						\n" /* Is the task using the FPU context?  If so, pop the high vfp registers too. */
		"	it eq								\n"
		"	vldmiaeq r0!, {s16-s31}				\n"
		"										\n"
		"	msr psp, r0							\n"
		"	bx r14								\n"
		"										\n"
		"	.align 2							\n"
		"pxCurrentTCBConst: .word pxCurrentTCB	\n"
		::"i"(configMAX_SYSCALL_INTERRUPT_PRIORITY)
	);
}

static void isr_nmi(void)
{
	while( 1 )
	{

	}
}

void isr_hard_fault_stack(struct stack_t* fault_stack)
{
	(void)fault_stack;
	// regarder fault_stack->pc pour voir d'ou vient le probleme

	if( SCB->CFSR | SCB_CFSR_MEMMANAG_IACCVIOL )
	{
		// Instruction access violation
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_MEMMANAG_DACCVIOL )
	{
		// Data access violation
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_MEMMANAG_MUNSTKERR)
	{
		// Memory Management Fault on unstacking for a return from exception
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_MEMMANAG_MSTKERR )
	{
		// Memory Management Fault on stacking for exception entry
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_MEMMANAG_MLSPERR )
	{
		// Memory Management Fault during floating point lazy state preservation
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_MEMMANAG_MMARVALID )
	{
		// Memory Management Fault Address Register (SCB->MMFAR) valid flag
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_BUS_IBUSERR )
	{
		// Instruction bus error
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_BUS_PRECISERR )
	{
		// Precise data bus error
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_BUS_IMPRECISERR )
	{
		// Imprecise data bus error
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_BUS_UNSTKERR )
	{
		// BusFault on unstacking for a return from exception
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_BUS_STKERR )
	{
		// BusFault on stacking for exception entry
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_BUS_LSPERR )
	{
		// Bus Fault during floating point lazy state preservation
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_BUS_BFARVALID )
	{
		// Bus Fault Address Register (SCB->BFAR) valid flag
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_USAGE_UNDEFINSTR )
	{
		// Undefined instruction Usage Fault
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_USAGE_INVSTATE )
	{
		// Invalid state Usage Fault
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_USAGE_INVPC )
	{
		// Invalid PC load Usage Fault, caused by an invalid EXC_RETURN value
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_USAGE_NOCP )
	{
		// No coprocessor Usage Fault. The processor does not support coprocessor instructions
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_USAGE_UNALIGNED )
	{
		// Unaligned access
		// TODO : action ?
	}

	if( SCB->CFSR | SCB_CFSR_USAGE_DIVBYZERO )
	{
		// Divide by zero
		// TODO : action ?
	}

//	isr_pwm_reset();

	while( 1 )
	{

	}
}

static void isr_hard_fault(void)
{
	__asm volatile
	(
		" tst lr, #4                     \n"
		" ite eq                         \n"
		" mrseq r0, msp                  \n"
		" mrsne r0, psp                  \n"
		" ldr r1, [r0, #24]              \n"
		" ldr r2, mem_handler_const      \n"
		" bx r2                          \n"
		" mem_handler_const: .word isr_hard_fault_stack\n"
	);
}

static void isr_mpu_fault(void)
{
	while( 1 )
	{

	}
}

static void isr_bus_fault(void)
{
	while( 1 )
	{

	}
}

static void isr_usage_fault(void)
{
	while( 1 )
	{

	}
}

static void isr_debug_monitor(void)
{
	while( 1 )
	{

	}
}

static void isr_unexpected(void)
{
	while( 1 )
	{

	}
}

