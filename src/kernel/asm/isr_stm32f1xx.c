//! @file isr.c
//! @brief isr
//! @author Atlantronic

#include "kernel/FreeRTOSConfig.h"
#include "kernel/fault.h"
#include "gpio.h"

#define BAR_NUMBER       0x00
#define FOO_NUMBER       0x03

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

static void isr_cpu_down_safety(void); //!< tout va mal, on sauve les meubles
void isr_pwm_reset(void) __attribute__((weak )); //!< sécurité : le module pwm met les moteurs à l'arrêt

void isr_usart3(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption usart3
void isr_uart4(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption uart4
void isr_dma1_channel1(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma1, canal 1
void isr_dma1_channel2(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma1, canal 2
void isr_dma1_channel3(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma1, canal 3
void isr_dma2_channel3(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma2, canal 3
void isr_dma2_channel5(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma2, canal 5
void isr_adc(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption adc
void isr_exti0(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption exti 0
void isr_exti1(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption exti 1
void isr_exti2(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption exti 2
void isr_exti3(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption exti 3
void isr_exti4(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption exti 4
void isr_exti9_5(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption exti 5 à 9
void isr_exti15_10(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption exti 10 à 15
void isr_can1_tx(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption can1, tx
void isr_can1_rx0(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption can1, rx0
void isr_otg_fs(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption usb otg

static int32_t check_card_number(); //!< fonction de vérification du numéro de carte. Retourne 0 si c'est ok
extern void __main(void) __attribute__((noreturn)); //!< fonction main à lancer une fois les segments data et bss initialisés en sram

extern unsigned long _sidata;
extern unsigned long _sdata; //!< debut du segment data en sram (segment à remplir au reset) (cf arm-elf.ld)
extern unsigned long _edata; //!< fin du segment data en sram (segment à remplir au reset) (cf arm-elf.ld)
extern unsigned long _sbss; //!< debut du segment bss en sram (segment à initialiser à zéro) (cf arm-elf.ld)
extern unsigned long _ebss; //!< fin du segment bss en sram (segment à initialiser à zéro) (cf arm-elf.ld)
extern unsigned long _estack; //!< haut de la ram (-16 par précaution) => début de la stack principale (cf arm-elf.ld)

__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
	// processeur - cortex-m3
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
	isr_exti0,
	isr_exti1,
	isr_exti2,
	isr_exti3,
	isr_exti4,
	isr_dma1_channel1,
	isr_dma1_channel2,
	isr_dma1_channel3,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_adc,
	isr_can1_tx,
	isr_can1_rx0,
	isr_unexpected,
	isr_unexpected,
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
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_usart3,
	isr_exti15_10,
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
	isr_uart4,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_dma2_channel3,
	isr_unexpected,
	isr_dma2_channel5,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_otg_fs
//  (void*)0xF108F85F // RAM boot.
};

static int32_t check_card_number()
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	uint8_t num = (GPIOA->IDR >> 2 ) & 0x03;

	#if defined( __foo__ )
	return ( num == FOO_NUMBER?0:1 );
	#elif defined( __bar__ )
	return ( num == BAR_NUMBER?0:1 );
	#else
	#error unknown card
	#endif
}

void isr_reset(void)
{
	unsigned long *pulSrc, *pulDest;

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

	// on va attendre que les tensions soient ok sur les pin
	int i = 100000;
	for( ; i--; )
	{
		nop();
	}

	if( check_card_number() )
	{
		while(1)
		{
			// bug : numéro de version incorrect
		}
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
		"   mrs r0, psp                         \n"
		"                                       \n"
		"   ldr r3, pxCurrentTCBConst           \n" // Get the location of the current TCB.
		"   ldr r2, [r3]                        \n"
		"                                       \n"
		"   stmdb r0!, {r4-r11, r14}            \n" // Save the remaining registers.
		"   str r0, [r2]                        \n" // Save the new top of stack into the first member of the TCB.
		"                                       \n"
		"   stmdb sp!, {r3, r14}                \n"
		"   mov r0, %0                          \n"
		"   msr basepri, r0                     \n"
		"   bl vTaskSwitchContext               \n"
		"   mov r0, #0                          \n"
		"   msr basepri, r0                     \n"
		"   ldmia sp!, {r3, r14}                \n"
		"                                       \n" // Restore the context, including the critical nesting count.
		"   ldr r1, [r3]                        \n"
		"   ldr r0, [r1]                        \n" // The first item in pxCurrentTCB is the task top of stack.
		"   ldmia r0!, {r4-r11, r14}            \n" // Pop the registers.
		"   msr psp, r0                         \n"
		"   bx lr                               \n"
		"                                       \n"
		"   .align 2                            \n"
		"pxCurrentTCBConst: .word pxCurrentTCB  \n"
		::"i"(configMAX_SYSCALL_INTERRUPT_PRIORITY)
	);
}

static void isr_nmi(void)
{
	setLed(ERR_NMI);
	isr_cpu_down_safety();
}

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

void halt_faulty(struct stack_t *faulty_stack)
{
	(void)faulty_stack;
	// Inspect faulty_stack->pc to locate the offending instruction.

	setLed(ERR_HARD_FAULT);
	isr_pwm_reset();

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
		" mem_handler_const: .word halt_faulty\n"
	);
}

static void isr_mpu_fault(void)
{
	setLed(ERR_MPU_FAULT);
	isr_cpu_down_safety();
}

static void isr_bus_fault(void)
{
	setLed(ERR_BUS_FAULT);
	isr_cpu_down_safety();
}

static void isr_usage_fault(void)
{
	setLed(ERR_USAGE_FAULT);
	isr_cpu_down_safety();
}

static void isr_debug_monitor(void)
{
	setLed(ERR_DEBUG_MONITOR);
	isr_cpu_down_safety();
}

static void isr_unexpected(void)
{
	setLed(ERR_UNEXPECTED_ISR);
	isr_cpu_down_safety();
}

void isr_pwm_reset(void)
{
	// surchargée par le module pwm si présent
}

static void isr_cpu_down_safety(void)
{
	isr_pwm_reset();

	while( 1 )
	{

	}
}
