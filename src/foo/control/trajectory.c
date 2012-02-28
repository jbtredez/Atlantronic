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
#include "kernel/math/trigo.h"
#include "control/trajectory.h"
#include "detection.h"
#include "graph.h"

#define TRAJECTORY_STACK_SIZE       350

enum trajectory_state
{
	TRAJECTORY_NONE,
	TRAJECTORY_TO_DEST,
	TRAJECTORY_BASIC_AVOIDANCE,
	TRAJECTORY_USE_GRAPH,
};

void trajectory_cmd(void* arg);
static void trajectory_task(void* arg);
static void trajectory_compute(enum trajectory_state next_state);
static int trajectory_find_way_to_graph();

// requete pour la tache trajectory + mutex
struct trajectory_cmd_arg trajectory_request;
static xSemaphoreHandle trajectory_mutex;

// donnees privees a la tache
static struct fx_vect_pos trajectory_pos; //!< position du robot au moment du reveil de la tache
static struct fx_vect_pos trajectory_dest;
static int32_t trajectory_approx_dist;
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
		ev = vTaskWaitEvent(EVENT_CONTROL_COLSISION | EVENT_CONTROL_TARGET_REACHED | EVENT_CONTROL_TARGET_NOT_REACHED | EVENT_TRAJECTORY_UPDATE | EVENT_DETECTION_UPDATED, portMAX_DELAY);

		trajectory_pos = location_get_position();

		if(ev & EVENT_TRAJECTORY_UPDATE)
		{
			trajectory_state = TRAJECTORY_NONE;
			xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

			trajectory_dest = trajectory_pos;
			trajectory_type = trajectory_request.type;

			switch(trajectory_request.type)
			{
				case TRAJECTORY_STRAIGHT:
					trajectory_dest.x += ((int64_t)trajectory_dest.ca * (int64_t)trajectory_request.dist) >> 30;
					trajectory_dest.y += ((int64_t)trajectory_dest.sa * (int64_t)trajectory_request.dist) >> 30;
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
					trajectory_dest.x += ((int64_t)trajectory_dest.ca * (int64_t)trajectory_request.dist) >> 30;
					trajectory_dest.y += ((int64_t)trajectory_dest.sa * (int64_t)trajectory_request.dist) >> 30;
					trajectory_approx_dist = 0;
					trajectory_way = TRAJECTORY_ANY_WAY;
					break;
				case TRAJECTORY_ROTATE:
					trajectory_dest.alpha += trajectory_request.alpha;
					trajectory_dest.ca = fx_cos(trajectory_dest.alpha);
					trajectory_dest.sa = fx_sin(trajectory_dest.alpha);
					trajectory_approx_dist = 0;
					trajectory_way = TRAJECTORY_ANY_WAY;
					break;
				case TRAJECTORY_ROTATE_TO:
					trajectory_dest.alpha += control_find_rotate(trajectory_dest.alpha, trajectory_request.alpha);
					trajectory_dest.ca = fx_cos(trajectory_dest.alpha);
					trajectory_dest.sa = fx_sin(trajectory_dest.alpha);
					trajectory_approx_dist = 0;
					trajectory_way = trajectory_request.way;
					break;
				case TRAJECTORY_GOTO:
					trajectory_dest.x = trajectory_request.x;
					trajectory_dest.y = trajectory_request.y;
					trajectory_dest.alpha = trajectory_request.alpha;
					trajectory_dest.ca = fx_cos(trajectory_dest.alpha);
					trajectory_dest.sa = fx_sin(trajectory_dest.alpha);
					trajectory_approx_dist = trajectory_request.dist;
					trajectory_way = trajectory_request.way;
					break;
				case TRAJECTORY_GOTO_GRAPH:
					break;
				case TRAJECTORY_FREE:
				default:
					trajectory_type = TRAJECTORY_FREE;
					break;
			}

			vTaskClearEvent(EVENT_TRAJECTORY_UPDATE);
			xSemaphoreGive(trajectory_mutex);

			if( trajectory_type == TRAJECTORY_GOTO_GRAPH)
			{
				trajectory_find_way_to_graph();
			}

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
			/*if( trajectory_type == TRAJECTORY_STRAIGHT)
			{
				trajectory_compute(TRAJECTORY_BASIC_AVOIDANCE);
			}
			else if( trajectory_type == TRAJECTORY_GOTO)
			{*/
				trajectory_compute(TRAJECTORY_USE_GRAPH);
			//}
			vTaskClearEvent(EVENT_CONTROL_COLSISION);
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
				case TRAJECTORY_USE_GRAPH:
					trajectory_compute(TRAJECTORY_USE_GRAPH);
					break;
			}

			vTaskClearEvent(EVENT_CONTROL_TARGET_REACHED);
		}

		if( ev & EVENT_CONTROL_TARGET_NOT_REACHED)
		{
			log(LOG_ERROR, "target not reached");
			vTaskClearEvent(EVENT_CONTROL_TARGET_NOT_REACHED);
		}

		if( ev & EVENT_DETECTION_UPDATED)
		{
			struct fx_vect2 a;
			struct fx_vect2 b;
			detection_compute_front_object(&trajectory_pos, &a, &b);
			control_set_front_object(&a, 100<<16);
			vTaskClearEvent(EVENT_DETECTION_UPDATED);
		}
	}
}

static int trajectory_find_way_to_graph()
{
	// passe en stack, pas trop de noeuds
	struct graph_node_dist node_dist[GRAPH_NUM_NODE];
	struct fx_vect2 p = {trajectory_pos.x, trajectory_pos.y};

	graph_compute_node_distance(p, node_dist);

	struct fx_vect2 a_table;
	struct fx_vect2 b_table;
	struct fx_vect_pos pos = trajectory_pos;
	int32_t xmin;
	int i;
	int id = 0;

	for( i = 0 ; i < GRAPH_NUM_NODE; i++)
	{
		id = node_dist[i].id;
		int32_t dx = graph_node[id].pos.x - pos.x;
		int32_t dy = graph_node[id].pos.y - pos.y;
		pos.alpha = fx_atan2(dy, dx);
		pos.ca = fx_cos(pos.alpha);
		pos.sa = fx_sin(pos.alpha);
		xmin = detection_compute_front_object(&pos, &a_table, &b_table) >> 16;
		if(node_dist[i].dist < xmin)
		{
			break;
		}
	}

	log_format(LOG_INFO, "point graph : %d", id);

	trajectory_dest.x = graph_node[node_dist[0].id].pos.x;
	trajectory_dest.y = graph_node[node_dist[0].id].pos.y;
	trajectory_dest.alpha = 0;
	trajectory_dest.ca = 1;
	trajectory_dest.sa = 0;

	return 0;
}

static void trajectory_compute(enum trajectory_state next_state)
{
	struct fx_vect_pos pos = trajectory_pos;

	if(next_state == TRAJECTORY_TO_DEST)
	{
		control_goto_near(trajectory_dest.x, trajectory_dest.y, trajectory_dest.alpha, trajectory_approx_dist, CONTROL_LINE_XY, trajectory_way);
	}
	else if( next_state == TRAJECTORY_BASIC_AVOIDANCE)
	{
		if( trajectory_state == TRAJECTORY_TO_DEST)
		{
			struct fx_vect2 a_table;
			struct fx_vect2 b_table;
			struct fx_vect2 a_robot;
			detection_compute_front_object(&pos, &a_table, &b_table);
			fx_vect2_table_to_robot(&pos, &a_table, &a_robot);
			int32_t dist = a_robot.x - PARAM_LEFT_CORNER_X - (150 << 16);

			pos.x += ((int64_t)dist * (int64_t)pos.ca) >> 30;
			pos.y += ((int64_t)dist * (int64_t)pos.sa) >> 30;
			control_goto_near(pos.x, pos.y, pos.alpha, 0, CONTROL_LINE_XY, TRAJECTORY_BACKWARD);
		}
	}
	else if( next_state == TRAJECTORY_USE_GRAPH)
	{
		// TODO
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

void trajectory_straight_to_wall(int32_t dist)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_STRAIGHT_TO_WALL;
	trajectory_request.dist = dist;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_rotate(int32_t angle)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_ROTATE;
	trajectory_request.alpha = angle;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_rotate_to(int32_t angle)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_ROTATE_TO;
	trajectory_request.alpha = angle;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_straight(int32_t dist)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_STRAIGHT;
	trajectory_request.dist = dist;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_goto_near(int32_t x, int32_t y, int32_t dist, enum trajectory_way way)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_GOTO;
	trajectory_request.x = x;
	trajectory_request.y = y;
	trajectory_request.dist = dist;
	trajectory_request.way = way;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_goto_graph_node(uint32_t node_id, int32_t dist, enum trajectory_way way)
{
	if( node_id >= GRAPH_NUM_NODE)
	{
		log_format(LOG_ERROR, "node_id inconnu : %zd", node_id);
		return;
	}

	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_GOTO;
	trajectory_request.x = graph_node[node_id].pos.x;
	trajectory_request.y = graph_node[node_id].pos.y;
	trajectory_request.dist = dist;
	trajectory_request.way = way;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_goto_graph()
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_GOTO_GRAPH;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}