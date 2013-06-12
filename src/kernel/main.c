//! @file main.c
//! @brief Programme principal
//! @author Atlantronic

#include "kernel/module.h"
#include "gpio.h"
#include <stdint.h>
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"

//! pour ne pas confondre avec le main de la libc newlib
void __main() __attribute__((noreturn));
void kernel_panic(uint8_t err) __attribute__((noreturn));

void kernel_panic(uint8_t err)
{
	(void) err;
	while(1)
	{
		//setLed(LED_RED);
	}
}

void __main()
{
	uint8_t err = initModules();

	// on n'arrive normalement jamais ici sur la cible
	kernel_panic(err);
}

void vApplicationMallocFailedHook( void )
{
	taskDISABLE_INTERRUPTS();
	kernel_panic(0);
}

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	taskDISABLE_INTERRUPTS();
	kernel_panic(0);
}
