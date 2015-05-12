#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"

#define WEAK_ELEVATOR
#include "elevator.h"
#include "kernel/driver/stepper_driver.h"
#include "kernel/driver/io.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"

#define ELEVATOR_STACK_SIZE       350
#define ELEVATOR_PERIOD             1
#define ELEVATOR_DT             0.001

#define ELEVATOR_MIN                0
#define ELEVATOR_MAX              150
#define ELEVATOR_MAX_SPEED        100

static StepperDriver elevatorMotor(IO_ELEVATOR_STEP, IO_ELEVATOR_DIR, 5, ELEVATOR_MAX_SPEED, 1000, 1000);


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
	int switchActiveLastCycle = 1;

	elevatorMotor.setMaxSpeed(20);
	elevatorMotor.setPosition(-50);

	for(int i = 0; i < 5000 && gpio_get(IO_ELEVATOR_SWITCH); i++)
	{
		elevatorMotor.step(ELEVATOR_DT);
		vTaskDelayUntil(&wake_time, ELEVATOR_PERIOD);
	}

	// on est en butee
	elevatorMotor.setCurrentPosition(0);
	elevatorMotor.setPosition(35);
	elevatorMotor.setMaxSpeed(ELEVATOR_MAX_SPEED);
	log(LOG_INFO, "elevator initialized");

	while(1)
	{
		int switchActive = ! gpio_get(IO_ELEVATOR_SWITCH);
		if( switchActive && ! switchActiveLastCycle )
		{
			// on vient de passer en butee
			elevatorMotor.setCurrentPosition(0);
			if( elevatorMotor.getWantedPosition() < 0 )
			{
				elevatorMotor.setPosition(0);
			}
		}
		switchActiveLastCycle = switchActive;

		elevatorMotor.step(ELEVATOR_DT);
		vTaskDelayUntil(&wake_time, ELEVATOR_PERIOD);
	}
}

void elevator_set_position(float pos)
{
	if( pos < ELEVATOR_MIN )
	{
		log_format(LOG_ERROR, "pos out of range : %d < %d, moving to min", (int)pos, (int)ELEVATOR_MIN);
		pos = ELEVATOR_MIN;
	}
	else if( pos > ELEVATOR_MAX )
	{
		log_format(LOG_ERROR, "pos out of range : %d > %d, moving to max", (int)pos, (int)ELEVATOR_MAX);
		pos = ELEVATOR_MAX;
	}

	if( pos == ELEVATOR_MIN )
	{
		// on souhaite aller jusqu'au switch
		pos = ELEVATOR_MIN - 2;
	}

	elevatorMotor.setPosition(pos);
}

float elevator_get_position()
{
	return elevatorMotor.getCurrentPosition();
}

static void elevator_cmd(void* arg)
{
	struct elevator_cmd_arg* cmd_arg = (struct elevator_cmd_arg*) arg;
	elevator_set_position(cmd_arg->pos);
}
