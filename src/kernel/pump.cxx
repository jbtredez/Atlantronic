#define WEAK_PUMP
#include "pump.h"
#include "kernel/module.h"
#include "kernel/driver/pwm.h"
#include "kernel/driver/adc.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"

// pompe 12V
#define PUMP_VMAX        12

static void pump_cmd(void*);

static Pump pump[PUMP_MAX] =
{
	Pump(PWM_PUMP_1),
	Pump(PWM_PUMP_2),
	Pump(PWM_PUMP_3),
	Pump(PWM_PUMP_4)
};

int pump_module_init()
{
	usb_add_cmd(USB_CMD_PUMP, pump_cmd);

	return 0;
}

module_init(pump_module_init, INIT_PUMP);

void pump_update()
{
	for(int i = 0; i < PUMP_MAX; i++)
	{
		pump[i].update();
	}
}

void Pump::set(float percent)
{
	if(percent < 0)
	{
		val = 0;
	}
	else if ( percent > 1)
	{
		val = 1;
	}
	else
	{
		val = percent;
	}
}

void Pump::update()
{
	pwm_set(pwm_id, val * PUMP_VMAX / adc_filtered_data.vBat);
}

//------------------ interface usb -------------------
void pump_cmd(void* arg)
{
	struct pump_cmd_arg* cmd_arg = (struct pump_cmd_arg*) arg;
	int id = cmd_arg->id;
	if( id < PUMP_MAX)
	{
		pump[id].set(cmd_arg->val/100.0f);
	}
	else
	{
		log_format(LOG_INFO, "unknown pump id %d", id);
	}
}
