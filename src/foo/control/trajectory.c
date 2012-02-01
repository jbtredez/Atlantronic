//! @file trajectory.h
//! @brief Generation de trajectoire
//! @author Atlantronic

#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/event.h"
#include "kernel/robot_parameters.h"
#include "control/trajectory.h"
#include "detection.h"
#include <math.h>

#define TRAJECTORY_STACK_SIZE       350

enum trajectory_state
{
	TRAJECTORY_NONE,
	TRAJECTORY_TO_DEST,
	TRAJECTORY_BASIC_AVOIDANCE,
};

void trajectory_cmd(void* arg);
static void trajectory_task(void* arg);
static void trajectory_compute(enum trajectory_state next_state);

// requete pour la tache trajectory + mutex
struct trajectory_cmd_arg trajectory_request;
static xSemaphoreHandle trajectory_mutex;

// donnees privees a la tache
static struct vect_pos trajectory_dest;
static float trajectory_approx_dist;
static enum trajectory_way trajectory_way;
static enum trajectory_cmd_type trajectory_type;
static enum trajectory_state trajectory_state;

static int trajectory_module_init()
{
	xTaskHandle xHandle;

	portBASE_TYPE err = xTaskCreate(trajectory_task, "traj", TRAJECTORY_STACK_SIZE, NULL, PRIORITY_TASK_CONTROL, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TRAJECTORY;
	}

	trajectory_mutex = xSemaphoreCreateMutex();

	if(trajectory_mutex == NULL)
	{
		return ERR_INIT_TRAJECTORY;
	}

	usb_add_cmd(USB_CMD_TRAJECTORY, &trajectory_cmd);

	trajectory_state = TRAJECTORY_NONE;

	return 0;
}

module_init(trajectory_module_init, INIT_TRAJECTORY);

static void trajectory_task(void* arg)
{
	(void) arg;

	uint32_t ev;

	while(1)
	{
		ev = vTaskWaitEvent(EVENT_CONTROL_COLSISION | EVENT_CONTROL_TARGET_REACHED | EVENT_CONTROL_TARGET_NOT_REACHED | EVENT_TRAJECTORY_UPDATE, portMAX_DELAY);

		if(ev & EVENT_TRAJECTORY_UPDATE)
		{
			trajectory_state = TRAJECTORY_NONE;
			xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

			trajectory_dest = location_get_position();

			trajectory_type = trajectory_request.type;

			switch(trajectory_request.type)
			{
				case TRAJECTORY_STRAIGHT:
					trajectory_dest.x += trajectory_dest.ca * trajectory_request.dist;
					trajectory_dest.y += trajectory_dest.sa * trajectory_request.dist;
					trajectory_approx_dist = 0;
					if(trajectory_request.dist > 0)
					{
						trajectory_way = TRAJECTORY_FORWARD;
					}
					else
					{
						trajectory_way = TRAJECTORY_BACKWARD;
					}
					break;
				case TRAJECTORY_STRAIGHT_TO_WALL:
					trajectory_dest.x += trajectory_dest.ca * trajectory_request.dist;
					trajectory_dest.y += trajectory_dest.sa * trajectory_request.dist;
					trajectory_approx_dist = 0;
					trajectory_way = TRAJECTORY_ANY_WAY;
					break;
				case TRAJECTORY_ROTATE:
					trajectory_dest.alpha += trajectory_request.alpha;
					trajectory_dest.ca = cosf(trajectory_dest.alpha);
					trajectory_dest.sa = sinf(trajectory_dest.alpha);
					trajectory_approx_dist = 0;
					trajectory_way = TRAJECTORY_ANY_WAY;
					break;
				case TRAJECTORY_ROTATE_TO:
					trajectory_dest.alpha += control_find_rotate(trajectory_dest.alpha, trajectory_request.alpha);
					trajectory_dest.ca = cosf(trajectory_dest.alpha);
					trajectory_dest.sa = sinf(trajectory_dest.alpha);
					trajectory_approx_dist = 0;
					trajectory_way = trajectory_request.way;
					break;
				case TRAJECTORY_GOTO:
					trajectory_dest.x = trajectory_request.x;
					trajectory_dest.y = trajectory_request.y;
					trajectory_dest.alpha = trajectory_request.alpha;
					trajectory_dest.ca = cosf(trajectory_dest.alpha);
					trajectory_dest.sa = sinf(trajectory_dest.alpha);
					trajectory_approx_dist = trajectory_request.dist;
					trajectory_way = trajectory_request.way;
					break;
				case TRAJECTORY_FREE:
				default:
					trajectory_type = TRAJECTORY_FREE;
					break;
			}

			vTaskClearEvent(EVENT_TRAJECTORY_UPDATE);
			xSemaphoreGive(trajectory_mutex);

			if(trajectory_type != TRAJECTORY_FREE)
			{
				trajectory_compute(TRAJECTORY_TO_DEST);
			}
			else
			{
				control_free();
			}
		}

		if(ev & EVENT_CONTROL_COLSISION)
		{
			log(LOG_INFO, "collision");
			vTaskClearEvent(EVENT_CONTROL_COLSISION);
			trajectory_compute(TRAJECTORY_BASIC_AVOIDANCE);
		}

		if(ev & EVENT_CONTROL_TARGET_REACHED)
		{
			switch(trajectory_state)
			{
				default:
				case TRAJECTORY_NONE:
					break;
				case TRAJECTORY_TO_DEST:
					log(LOG_INFO, "target reached");
					break;
				case TRAJECTORY_BASIC_AVOIDANCE:
					trajectory_compute(TRAJECTORY_BASIC_AVOIDANCE);
					break;
			}

			vTaskClearEvent(EVENT_CONTROL_TARGET_REACHED);
		}

		if( ev & EVENT_CONTROL_TARGET_NOT_REACHED)
		{
			log(LOG_ERROR, "target not reached");
			vTaskClearEvent(EVENT_CONTROL_TARGET_NOT_REACHED);
		}
	}
}

static void trajectory_compute(enum trajectory_state next_state)
{
	struct vect_pos pos = location_get_position();

	if(next_state == TRAJECTORY_TO_DEST)
	{
		control_goto_near(trajectory_dest.x, trajectory_dest.y, trajectory_dest.alpha, trajectory_approx_dist, trajectory_way);
	}
	else if( next_state == TRAJECTORY_BASIC_AVOIDANCE)
	{
		struct fx_vect2 a_table;
		struct fx_vect2 b_table;
		struct fx_vect2 a_robot;
		struct fx_vect2 b_robot;
		detection_get_front_seg(&a_table, &b_table);
		fx_vect2_table_to_robot(&pos, &a_table, &a_robot);
		fx_vect2_table_to_robot(&pos, &b_table, &b_robot);
		float dist = (a_robot.x >> 16) - PARAM_LEFT_CORNER_X - 150;

		pos.x += dist * pos.ca;
		pos.y += dist * pos.sa;
		control_goto_near(pos.x, pos.y, pos.alpha, 0, TRAJECTORY_BACKWARD);
	}

	trajectory_state = next_state;
}

void trajectory_cmd(void* arg)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	memcpy(&trajectory_request, arg, sizeof(struct trajectory_cmd_arg));
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_free()
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_FREE;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_straight_to_wall(float dist)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_STRAIGHT_TO_WALL;
	trajectory_request.dist = dist;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_rotate(float angle)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_ROTATE;
	trajectory_request.alpha = angle;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_rotate_to(float angle)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_ROTATE_TO;
	trajectory_request.alpha = angle;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_straight(float dist)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_STRAIGHT;
	trajectory_request.dist = dist;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_goto_near(float x, float y, float dist, enum trajectory_way way)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_STRAIGHT;
	trajectory_request.x = x;
	trajectory_request.y = y;
	trajectory_request.dist = dist;
	trajectory_request.way = way;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}
