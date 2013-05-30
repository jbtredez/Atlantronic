//! @file isr.c
//! @brief isr
//! @author Atlantronic

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
};

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

	__main();
}

//! doc utile : 0xfffffff1 : retour handler mode
//!             0xfffffffd : retour thread mode, et utilisation de process stack
//!             0xfffffff9 : retour thread mode, et utilisation de main stack
void isr_svc( void )
{
	while( 1 )
	{

	}
}

void isr_context_switch( void )
{
	while( 1 )
	{

	}
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

