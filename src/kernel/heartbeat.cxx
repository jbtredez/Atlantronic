#include "kernel/driver/power.h"
#include "heartbeat.h"
#include "kernel/module.h"
#include "kernel/log.h"

#define HEARTBEAT_TIMEOUT                 5000

static systime heartbeat_last_time;
static int heartbeat_disabled = 0;

static void heartbeat_cmd(void* arg);

int heartbeat_module_init()
{
	usb_add_cmd(USB_CMD_HEARTBEAT, heartbeat_cmd);

	return 0;
}

module_init(heartbeat_module_init, INIT_HEARTBEAT);

static void heartbeat_cmd(void* arg)
{
	struct heartbeat_cmd_arg* cmd_arg = (struct heartbeat_cmd_arg*) arg;

	if( cmd_arg->type == HEARTBEAT_UPDATE )
	{
		heartbeat_disabled = 0;
		heartbeat_last_time = systick_get_time();
	}
	else if( cmd_arg->type == HEARTBEAT_DISABLE )
	{
		heartbeat_disabled = 1;
	}
}

void heartbeat_update()
{
	systime t = systick_get_time();
	if( (t - heartbeat_last_time).ms > HEARTBEAT_TIMEOUT && ! heartbeat_disabled)
	{
		power_set(POWER_OFF_HEARTBEAT);
	}
	else
	{
		power_clear(POWER_OFF_HEARTBEAT);
	}
}
