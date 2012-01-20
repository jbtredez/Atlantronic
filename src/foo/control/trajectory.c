//! @file trajectory.h
//! @brief Generation de trajectoire
//! @author Atlantronic

#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/rcc.h"
#include "control/trajectory.h"

#define TRAJECTORY_STACK_SIZE       350

void trajectory_cmd(void* arg);
static void trajectory_task(void* arg);

static int trajectory_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(trajectory_task, "traj", TRAJECTORY_STACK_SIZE, NULL, PRIORITY_TASK_CONTROL, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	usb_add_cmd(USB_CMD_TRAJECTORY, &trajectory_cmd);

	return 0;
}

module_init(trajectory_module_init, INIT_TRAJECTORY);

static void trajectory_task(void* arg)
{
	while(1)
	{
		vTaskDelay(ms_to_tick(50));
	}
}

void trajectory_cmd(void* arg)
{
	struct trajectory_cmd_arg* cmd_arg = (struct trajectory_cmd_arg*) arg;

	switch(cmd_arg->type)
	{
		case TRAJECTORY_STRAIGHT:
			trajectory_straight(cmd_arg->dist);
			break;
		case TRAJECTORY_STRAIGHT_TO_WALL:
			trajectory_straight_to_wall(cmd_arg->dist);
			break;
		case TRAJECTORY_ROTATE:
			trajectory_rotate(cmd_arg->alpha);
			break;
		case TRAJECTORY_ROTATE_TO:
			trajectory_rotate_to(cmd_arg->alpha);
			break;
		case TRAJECTORY_GOTO:
			control_goto_near(cmd_arg->x, cmd_arg->y, cmd_arg->alpha, cmd_arg->dist, cmd_arg->way);
			break;
		default:
			control_free();
			break;
	}
}

void trajectory_straight_to_wall(float dist)
{
	// TODO
}

void trajectory_rotate(float angle)
{
	struct vect_pos dest = location_get_position();
	dest.alpha += angle;
	control_goto_near(dest.x, dest.y, dest.alpha, 0, TRAJECTORY_ANY_WAY);
}

void trajectory_rotate_to(float angle)
{
	struct vect_pos dest = location_get_position();
	dest.alpha += control_find_rotate(dest.alpha, angle);
	control_goto_near(dest.x, dest.y, dest.alpha, 0, TRAJECTORY_ANY_WAY);
}

void trajectory_straight(float dist)
{
	struct vect_pos dest = location_get_position();
	dest.x += dest.ca * dist;
	dest.y += dest.sa * dist;
	control_goto_near(dest.x, dest.y, dest.alpha, 0, TRAJECTORY_ANY_WAY);
}

void trajectory_goto_near(float x, float y, float dist, enum trajectory_way sens)
{
	control_goto_near(x, y, 0, dist, sens);
}
