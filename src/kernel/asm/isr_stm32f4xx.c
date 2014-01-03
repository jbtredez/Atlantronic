//! @file isr.c
//! @brief isr
//! @author Atlantronic

#include "kernel/FreeRTOS.h"

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
	isr_unexpected,
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
	isr_spi1,
	isr_unexpected,
	isr_unexpected,
	isr_usart2,
	isr_usart3,
	isr_unexpected,
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
	isr_unexpected,
	isr_unexpected,
	isr_dma2_stream0,
	isr_dma2_stream1,
	isr_unexpected,
	isr_dma2_stream3,
	isr_unexpected,
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

static void isr_hard_fault(void)
{
	while( 1 )
	{

	}
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

