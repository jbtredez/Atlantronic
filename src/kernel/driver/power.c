#include "power.h"
#include "gpio.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/driver/pwm.h"

int power_state = POWER_OFF;
static void power_cmd(void* arg);

int power_module_init()
{
	usb_add_cmd(USB_CMD_POWER, power_cmd);

	return 0;
}

module_init(power_module_init, INIT_POWER);

void power_set(int powerEventMask)
{
	int old_state = power_state;
	power_state |= powerEventMask;
	if( power_state )
	{
		pwm_disable();
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
	if( diff & POWER_OFF_AU )
	{
		log(LOG_ERROR, "power off - AU");
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
	if( diff & POWER_OFF_AU )
	{
		log(LOG_INFO, "power clear - AU");
	}

	if( ! power_state )
	{
		pwm_enable();
		gpio_power_on();
		if(old_state != power_state)
		{
			log(LOG_INFO, "power on");
		}
	}
}

static void power_cmd(void* arg)
{
	struct power_cmd_arg* cmd_arg = (struct power_cmd_arg*) arg;
	if( cmd_arg->power_off )
	{
		power_set(POWER_OFF);
	}
	else
	{
		power_clear(POWER_OFF);
	}
}
