//! @file isr.c
//! @brief isr
//! @author Jean-Baptiste Trédez

#include "rtos/FreeRTOSConfig.h"
#include "error.h"
#include "io/gpio.h"

void isr_reset(void) __attribute__ ((naked)); //!< fonction de reset (point d'entrée)
static void isr_nmi(void); //!< interruption nmi
static void isr_hard_fault(void); //!< interruption "hard fault"
static void isr_mpu_fault(void); //!< interruption d'erreur sur le mpu
static void isr_bus_fault(void); //!< interruption d'erreur sur le bus
static void isr_usage_fault(void); //!< interruption d'erreur "usage fault"
static void isr_svc( void ) __attribute__ (( naked )); //!< interruption svc (lancement de la première tache)
static void isr_debug_monitor(void); //!< interruption debug monitor
static void isr_unexpected(void); //!< interruption imprévue
static void isr_context_switch( void ) __attribute__ ((naked)); //!< changement de contexte


static void isr_cpu_down_safety(void); //!< tout va mal, on sauve les meubles
void isr_pwm_reset(void) __attribute__((weak )); //!< sécurité : le module pwm met les moteurs à l'arrêt

void isr_usart3(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption usart3
void isr_dma1_channel1(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma1, canal 1
void isr_dma1_channel2(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma1, canal 2
void isr_dma1_channel3(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption dma1, canal 3
void isr_adc(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption adc
void isr_exit9_5(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption exit 5 à 9
void isr_exit15_10(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption exit 10 à 15
void isr_can1_tx(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption can1, tx
void isr_can1_rx0(void) __attribute__((weak, alias("isr_unexpected") )); //!< interruption can1, rx0

extern void isr_systick(void); //!< interruption systick (déclarée dans systick.c)
extern int main(void); //!< fonction main à lancer une fois les segments data et bss initialisés en sram

extern unsigned long __text_end__; //!< fin du segment text (flash) = debut du segment data (flash) (cf arm-elf.ld)
extern unsigned long __data_start__; //!< debut du segment data en sram (segment à remplir au reset) (cf arm-elf.ld)
extern unsigned long __data_end__; //!< fin du segment data en sram (segment à remplir au reset) (cf arm-elf.ld)
extern unsigned long __bss_start__; //!< debut du segment bss en sram (segment à initialiser à zéro) (cf arm-elf.ld)
extern unsigned long __bss_end__; //!< fin du segment bss en sram (segment à initialiser à zéro) (cf arm-elf.ld)
extern unsigned long _stack_top; //!< haut de la ram (-16 par précaution) => début de la stack principale (cf arm-elf.ld)

__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
	// processeur - cortex-m3
	(void (*)(void))(&_stack_top),          // Stack pointer initial principal
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
	isr_context_switch,                     // The PendSV handler => context_switch
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
	isr_exit9_5,
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
	isr_exit15_10,
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

// doc utile : 0xfffffff1 : retour handler mode
//             0xfffffffd : retour thread mode, et utilisation de process stack
//             0xfffffff9 : retour thread mode, et utilisation de main stack
void isr_svc( void )
{
	__asm volatile
	(
		"   ldr r3, pxCurrentTCBConst2          \n" // Restore the context.
		"   ldr r1, [r3]                        \n" // Use pxCurrentTCBConst to get the pxCurrentTCB address.
		"   ldr r0, [r1]                        \n" // The first item in pxCurrentTCB is the task top of stack.
		"   ldmia r0!, {r4-r11}                 \n" // Pop the registers that are not automatically saved on exception entry and the critical nesting count.
		"   msr psp, r0                         \n" // Restore the task stack pointer.
		"   mov r0, #0                          \n"
		"   msr basepri, r0                     \n"
		"   ldr lr, =0xfffffffd                 \n" // Au branchement sur 0xfffffffd, on retourne en thread mode avec utilisation de la stack process (on a mis le bon pointeur msr psp, r0). On va dépiler automatiquement r0,r1,r2,r3, r12, lr et brancher sur le pc.
		"   bx lr                               \n"
		"                                       \n"
		"   .align 2                            \n"
		"pxCurrentTCBConst2: .word pxCurrentTCB	\n"
	);
}

void isr_context_switch( void )
{
	/* This is a naked function. */

	__asm volatile
	(
		"   mrs r0, psp                         \n"
		"                                       \n"
		"   ldr r3, pxCurrentTCBConst           \n" // Get the location of the current TCB.
		"   ldr r2, [r3]                        \n"
		"                                       \n"
		"   stmdb r0!, {r4-r11}                 \n" // Save the remaining registers.
		"   str r0, [r2]                        \n" // Save the new top of stack into the first member of the TCB.
		"                                       \n"
		"   stmdb sp!, {r3, lr}                 \n"
		"   mov r0, %0                          \n"
		"   msr basepri, r0                     \n"
		"   bl vTaskSwitchContext               \n"
		"   mov r0, #0                          \n"
		"   msr basepri, r0                     \n"
		"   ldmia sp!, {r3, lr}                 \n"
		"                                       \n" // Restore the context, including the critical nesting count.
		"   ldr r1, [r3]                        \n"
		"   ldr r0, [r1]                        \n" // The first item in pxCurrentTCB is the task top of stack.
		"   ldmia r0!, {r4-r11}                 \n" // Pop the registers.
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

static void isr_hard_fault(void)
{
	setLed(ERR_HARD_FAULT);
	isr_cpu_down_safety();
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
