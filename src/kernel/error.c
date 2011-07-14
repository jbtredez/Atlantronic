//! @file error.c
//! @brief Error codes
//! @author Atlantronic

#include <stdlib.h>
#include <stdio.h>
#include "gpio.h"

#include "kernel/module.h"
#include "kernel/error.h"

#define ERROR_MAX              10

static uint16_t error_array[ERROR_MAX];
static volatile uint8_t error_index = 0;

// TODO : voir / reentrance
void error_raise(uint16_t error_number)
{
	error_array[ error_index ] = error_number;
	setLed (error_number);

	error_index++;
	if(error_index >= ERROR_MAX)
	{
		error_index = 0;
	}
}
