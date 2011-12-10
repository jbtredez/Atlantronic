//! @file main.c
//! @brief Programme principal
//! @author Atlantronic

#include "kernel/module.h"
#include "kernel/log.h"
#include "gpio.h"

//! pour ne pas confondre avec le main de la libc newlib
void __main() __attribute__((noreturn));
void init_panic(uint8_t err) __attribute__((noreturn));

void init_panic(uint8_t err)
{
	setLed(err);

	while(1)
	{

	}
}

void __main()
{
	uint8_t err = initModules();

	// on n'arrive normalement jamais ici sur la cible
	init_panic(err);
}

