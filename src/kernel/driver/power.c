#include "power.h"
#include "gpio.h"
#include "kernel/log.h"

static int power_state = POWER_ON;

void power_set(int powerEventMask)
{
	int old_state = power_state;
	power_state |= powerEventMask;
	if( power_state )
	{
		gpio_power_off();
	}

	int diff = power_state ^ old_state;
	if( diff & POWER_OFF_UNDERVOLTAGE )
	{
		log(LOG_ERROR, "power off - undervoltage");
	}
	if( diff & POWER_OFF_END_MATCH )
	{
		log(LOG_INFO, "power off - end match");
	}
	if( diff & POWER_OFF )
	{
		log(LOG_INFO, "power off");
	}
}

void power_clear(int powerEventMask)
{
	int old_state = power_state;
	power_state &= ~powerEventMask;

	int diff = powerEventMask & old_state;
	if( diff & POWER_OFF_UNDERVOLTAGE )
	{
		log(LOG_INFO, "power clear - undervoltage");
	}
	if( diff & POWER_OFF_END_MATCH )
	{
		log(LOG_INFO, "power clear - end match");
	}
	if( diff & POWER_OFF )
	{
		log(LOG_INFO, "power clear - off");
	}

	if( ! power_state )
	{
		gpio_power_on();
		if(old_state != power_state)
		{
			log(LOG_INFO, "power on");
		}
	}
}
