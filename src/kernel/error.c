#include <stdlib.h>
#include <stdio.h>
#include "gpio.h"

#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/error.h"

#define NB_ERROR 10

static uint16_t error_array[NB_ERROR];
static volatile uint8_t error_index = 0;

void error_raise(uint16_t error_number)
{
    error_array[ error_index ] = error_number;
    setLed (error_number);
    
    error_index++;
    if(error_index>=NB_ERROR)
      error_index = 0;
}

#if 0 
//for test only
void main()
{
  int i=0;
  for(i=0;i<34;i++)
    error_raise(0x330);
}
#endif