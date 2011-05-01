//! @file main.c
//! @brief Programme principal
//! @author Jean-Baptiste Trédez

#include "kernel/module.h"
#include "kernel/log.h"
#include "gpio.h"

void init_panic(uint8_t init)
{
	setLed(init);

	while(1)
	{

	}
}

int main()
{
	uint8_t error = initModules();

	// on n'arrive normalement jamais ici sur la cible
	init_panic(error);	

	return 0;
}

