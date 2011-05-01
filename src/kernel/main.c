//! @file main.c
//! @brief Programme principal
//! @author Jean-Baptiste Trédez

#include "kernel/module.h"
#include "kernel/log.h"
#include "gpio.h"

//! pour ne pas confondre avec le main de la libc newlib
void __main() __attribute__((noreturn));
void init_panic(uint8_t init) __attribute__((noreturn));

void init_panic(uint8_t init)
{
	setLed(init);

	while(1)
	{

	}
}

void __main()
{
	uint8_t error = initModules();

	// on n'arrive normalement jamais ici sur la cible
	init_panic(error);
}

