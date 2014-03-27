#include "power.h"
#include "gpio.h"
#include "kernel/log.h"

static int power_state = POWER_OFF;

void power_set(int powerEventMask)
{
	int old_state = power_state;
	power_state |= powerEventMask;
	if( power_state )
	{
		gpio_power_off();
	}
	if(old_state !=  power_state)
	{
		if( power_state & POWER_OFF_UNDERVOLTAGE )
		{
			log(LOG_INFO, "undervoltage");
		}
		log(LOG_INFO, "power off");
	}
}

void power_clear(int powerEventMask)
{
	int old_state = power_state;
	power_state &= ~powerEventMask;
	if( ! power_state )
	{
		gpio_power_on();
	}
	if(old_state !=  power_state)
	{
		log(LOG_INFO, "power on");
	}
}
