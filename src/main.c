//! @file main.c
//! @brief Programme principal
//! @author Jean-Baptiste Trédez

#include "module.h"
#include "log.h"
#include "io/gpio.h"

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

