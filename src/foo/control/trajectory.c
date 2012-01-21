//! @file trajectory.h
//! @brief Generation de trajectoire
//! @author Atlantronic

#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/queue.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "control/trajectory.h"

#define TRAJECTORY_STACK_SIZE       350
#define TRAJECTORY_QUEUE_SIZE        10

void trajectory_cmd(void* arg);
static void trajectory_task(void* arg);
static xQueueHandle trajectory_queue;

static int trajectory_module_init()
{
	xTaskHandle xHandle;

	trajectory_queue = xQueueCreate(TRAJECTORY_QUEUE_SIZE, sizeof(struct trajectory_cmd_arg));

	if(trajectory_queue == 0)
	{
		return ERR_INIT_TRAJECTORY;
	}

	portBASE_TYPE err = xTaskCreate(trajectory_task, "traj", TRAJECTORY_STACK_SIZE, NULL, PRIORITY_TASK_CONTROL, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TRAJECTORY;
	}

	usb_add_cmd(USB_CMD_TRAJECTORY, &trajectory_cmd);

	return 0;
}

module_init(trajectory_module_init, INIT_TRAJECTORY);

static void trajectory_task(void* arg)
{
	(void) arg;

	struct trajectory_cmd_arg cmd;
	struct vect_pos dest;

	while(1)
	{
		if(xQueueReceive(trajectory_queue, &cmd, portMAX_DELAY))
		{
			dest = location_get_position();

			switch(cmd.type)
			{
				case TRAJECTORY_STRAIGHT:
					dest.x += dest.ca * cmd.dist;
					dest.y += dest.sa * cmd.dist;
					control_goto_near(dest.x, dest.y, dest.alpha, 0, TRAJECTORY_ANY_WAY);
					break;
				case TRAJECTORY_STRAIGHT_TO_WALL:
					dest.x += dest.ca * cmd.dist;
					dest.y += dest.sa * cmd.dist;
					control_goto_near(dest.x, dest.y, dest.alpha, 0, TRAJECTORY_ANY_WAY);
					break;
				case TRAJECTORY_ROTATE:
					dest.alpha += cmd.alpha;
					control_goto_near(dest.x, dest.y, dest.alpha, 0, TRAJECTORY_ANY_WAY);
					break;
				case TRAJECTORY_ROTATE_TO:
					dest.alpha += control_find_rotate(dest.alpha, cmd.alpha);
					control_goto_near(dest.x, dest.y, dest.alpha, 0, cmd.way);
					break;
				case TRAJECTORY_GOTO:
					control_goto_near(cmd.x, cmd.y, dest.alpha, cmd.dist, cmd.way);
					break;
				default:
					control_free();
					break;
			}
		}
	}
}

void trajectory_cmd(void* arg)
{
	if( xQueueSendToBack(trajectory_queue, arg, 0) != pdPASS)
	{
		log(LOG_ERROR, "Queue de trajectoire pleine");
	}
}

void trajectory_straight_to_wall(float dist)
{
	struct trajectory_cmd_arg cmd =
	{
		.type = TRAJECTORY_STRAIGHT_TO_WALL,
		.dist = dist,
	};

	xQueueSendToBack(trajectory_queue, &cmd, portMAX_DELAY);
}

void trajectory_rotate(float angle)
{
	struct trajectory_cmd_arg cmd =
	{
		.type = TRAJECTORY_ROTATE,
		.alpha = angle,
	};

	xQueueSendToBack(trajectory_queue, &cmd, portMAX_DELAY);
}

void trajectory_rotate_to(float angle)
{
	struct trajectory_cmd_arg cmd =
	{
		.type = TRAJECTORY_ROTATE_TO,
		.alpha = angle,
	};

	xQueueSendToBack(trajectory_queue, &cmd, portMAX_DELAY);
}

void trajectory_straight(float dist)
{
	struct trajectory_cmd_arg cmd =
	{
		.type = TRAJECTORY_STRAIGHT,
		.dist = dist,
	};

	xQueueSendToBack(trajectory_queue, &cmd, portMAX_DELAY);
}

void trajectory_goto_near(float x, float y, float dist, enum trajectory_way way)
{
	struct trajectory_cmd_arg cmd =
	{
		.type = TRAJECTORY_STRAIGHT,
		.x = x,
		.y = y,
		.dist = dist,
		.way = way,
	};

	xQueueSendToBack(trajectory_queue, &cmd, portMAX_DELAY);
}
