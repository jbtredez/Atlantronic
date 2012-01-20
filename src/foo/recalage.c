#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "control/control.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "gpio.h"
#include "location/location.h"
#include "kernel/rcc.h"
#include "kernel/robot_parameters.h"
#include "pince.h"
#include "kernel/us_def.h"

void recalage()
{
#if 0
	pince_configure();
	pince_close();

	if(getcolor() == COLOR_BLUE)
	{
		location_set_position(-880.0f, -880.0f, PI/2.0f);
	}
	else
	{
		location_set_position(880.0f, -880.0f, PI/2.0f);
	}

	control_straight_to_wall(-200);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	control_straight(200 + PARAM_NP_X);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);

	if(getcolor() == COLOR_BLUE)
	{
		control_rotate(-PI/2.0f);
	}
	else
	{
		control_rotate(PI/2.0f);
	}

	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	control_straight_to_wall(-800);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	control_straight(110);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
#endif
}
