#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"

#include "elevator.h"
#include "kernel/driver/stepper_driver.h"
#include "kernel/driver/io.h"

static StepperDriver elevator_motor(GPIO_11, GPIO_10);

#define ELEVATOR_STACK_SIZE       350
#define ELEVATOR_PERIOD             2
#define ELEVATOR_DT             0.002

static void elevator_task(void* arg);

static int elevator_module_init()
{
	portBASE_TYPE err = xTaskCreate(elevator_task, "elevator", ELEVATOR_STACK_SIZE, NULL, PRIORITY_TASK_ELEVATOR, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

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
