#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"

#include "elevator.h"
#include "kernel/driver/stepper_driver.h"
#include "kernel/driver/io.h"
#include "kernel/driver/usb.h"

static StepperDriver elevator_motor(GPIO_11, GPIO_10, 5, 100, 400, 400);

#define ELEVATOR_STACK_SIZE       350
#define ELEVATOR_PERIOD             2
#define ELEVATOR_DT             0.002

static void elevator_task(void* arg);
static void elevator_cmd(void* arg);

static int elevator_module_init()
{
	portBASE_TYPE err = xTaskCreate(elevator_task, "elevator", ELEVATOR_STACK_SIZE, NULL, PRIORITY_TASK_ELEVATOR, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	usb_add_cmd(USB_CMD_ELEVATOR, elevator_cmd);
	return 0;
}

module_init(elevator_module_init, INIT_ELEVATOR);

static void elevator_task(void* /*arg*/)
{
	uint32_t wake_time = 0;

	while(1)
	{
		elevator_motor.step(ELEVATOR_DT);
		vTaskDelayUntil(&wake_time, ELEVATOR_PERIOD);
	}
}

void elevator_set_position(float pos)
{
	// TODO verif plage de position + butee
	elevator_motor.setPosition(pos);
}

static void elevator_cmd(void* arg)
{
	struct elevator_cmd_arg* cmd_arg = (struct elevator_cmd_arg*) arg;
	elevator_set_position(cmd_arg->pos);
}