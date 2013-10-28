#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/can_motor.h"
#include "kernel/can/can_id.h"
#include "kernel/canopen.h"
#include "control.h"

#define CONTROL_STACK_SIZE       350

static void control_task(void* arg);

static int control_module_init()
{
	portBASE_TYPE err = xTaskCreate(control_task, "control", CONTROL_STACK_SIZE, NULL, PRIORITY_TASK_CONTROL, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	return 0;
}

module_init(control_module_init, INIT_CONTROL);

static void control_task(void* arg)
{
	(void) arg;
	systime t1;
	systime t2;
	int res;

	uint32_t wake_time = 0;

	while(1)
	{
		//log(LOG_INFO, "sync");
		t1 = systick_get_time();

		if((can_motor[CAN_MOTOR_DRIVING1].status_word & 0x6f) != 0x27)
		{
			goto wait;
		}

		can_motor[CAN_MOTOR_DRIVING1].wait_update(0);
		canopen_sync();
		res = can_motor[CAN_MOTOR_DRIVING1].wait_update(ms_to_tick(2));

		if( ! res )
		{
			// mise à jour de la position
			//location_update();

			// recuperation des entrées AN
			//adc_get(&control_an);

			//control_compute();

			can_motor[CAN_MOTOR_DRIVING1].set_speed(0x00);
			can_motor[CAN_MOTOR_DRIVING2].set_speed(0);
			can_motor[CAN_MOTOR_DRIVING3].set_speed(0);
			can_motor[CAN_MOTOR_STEERING1].set_speed(0);
			can_motor[CAN_MOTOR_STEERING2].set_speed(0);
			can_motor[CAN_MOTOR_STEERING3].set_speed(0);
			t2 = systick_get_time();
			systime dt = t2 - t1;
			log_format(LOG_INFO, "dt %d us", (int)(dt.ms*1000 + dt.ns/1000));
		}

wait:
		vTaskDelayUntil(&wake_time, CONTROL_PERIOD);
	}
}
