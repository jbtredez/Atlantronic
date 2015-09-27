#include "power.h"
#include "kernel/driver/gpio.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/driver/pwm.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"

int power_state = POWER_ON;
static void power_cmd(void* arg);

#define POWER_GPIO_ENABLE               GPIOD
#define POWER_GPIO_ENABLE_PIN              11

static inline void gpio_power_on()
{
	gpio_reset_pin(POWER_GPIO_ENABLE, POWER_GPIO_ENABLE_PIN);
}

static inline void gpio_power_off()
{
	gpio_set_pin(POWER_GPIO_ENABLE, POWER_GPIO_ENABLE_PIN);
}

int power_module_init()
{
	// puissance on/off
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

	gpio_pin_init(POWER_GPIO_ENABLE, POWER_GPIO_ENABLE_PIN, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // on/off

	usb_add_cmd(USB_CMD_POWER, power_cmd);

	gpio_power_on();

	return 0;
}

void power_module_exit()
{
	// petite attente pour laisser un peu de rab au dynamixel
	vTaskDelay(300);
	power_set(POWER_OFF_END_MATCH);
}

module_init(power_module_init, INIT_POWER);
module_exit(power_module_exit, EXIT_POWER);

void power_set(int powerEventMask)
{
	int old_state = power_state;
	power_state |= powerEventMask;
	if( power_state )
	{
		pwm_disable();
		if( power_state & ~POWER_OFF_HEARTBEAT )
		{
			gpio_power_off();
		}
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
	if( diff & POWER_OFF_HEARTBEAT)
	{
		log(LOG_ERROR, "power off - HeartBeat");
	}
	if( diff & POWER_OFF_MIP_MOTOR)
	{
		log(LOG_ERROR, "power off - mip motor");
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
	if( diff & POWER_OFF_HEARTBEAT)
	{
		log(LOG_ERROR, "power clear - HeartBeat");
	}
	if( diff & POWER_OFF_MIP_MOTOR)
	{
		log(LOG_ERROR, "power clear - mip motor");
	}

	if( ! (power_state & ~POWER_OFF_HEARTBEAT) )
	{
		gpio_power_on();
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
