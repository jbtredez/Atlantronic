#include "power.h"
#include "kernel/driver/gpio.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/driver/pwm.h"

int power_state = POWER_ON;
static void power_cmd(void* arg);

#if defined(__discovery__)
static GPIO_TypeDef* power_gpio_enable = GPIOB;
static uint32_t power_pin_enable = 2;
#elif defined(__disco__)
static GPIO_TypeDef* power_gpio_enable = GPIOD;
static uint32_t power_pin_enable = 11;
#else
#error unknown card
#endif

static inline void gpio_power_on()
{
	gpio_reset_pin(power_gpio_enable, power_pin_enable);
}

static inline void gpio_power_off()
{
	gpio_set_pin(power_gpio_enable, power_pin_enable);
}

int power_module_init()
{
	// puissance on/off
#if defined(__discovery__)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
#elif defined(__disco__)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
#else
#error unknown card
#endif

	gpio_pin_init(power_gpio_enable, power_pin_enable, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // on/off

	usb_add_cmd(USB_CMD_POWER, power_cmd);

	gpio_power_on();

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
