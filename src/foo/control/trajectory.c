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
#define TRAJECTORY_APPROX_DIST     (100<<16)      //!< distance d'approche d'un objet

enum trajectory_state
{
	TRAJECTORY_NONE,
	TRAJECTORY_TO_DEST,
	TRAJECTORY_USE_GRAPH,
};

void trajectory_cmd(void* arg);
static void trajectory_task(void* arg);
static void trajectory_compute();
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
static enum trajectory_avoidance_type trajectory_avoidance_type;
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

static void trajectory_update()
{
	struct trajectory_cmd_arg cmd_arg;

	// mutex pour trajectory_request
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	cmd_arg = trajectory_request;
	vTaskClearEvent(EVENT_TRAJECTORY_UPDATE);
	xSemaphoreGive(trajectory_mutex);

	trajectory_state = TRAJECTORY_NONE;
	trajectory_dest = trajectory_pos;
	trajectory_approx_dist = 0;
	trajectory_way = TRAJECTORY_ANY_WAY;
	trajectory_avoidance_type = cmd_arg.avoidance_type;
	trajectory_type = cmd_arg.type;

	switch(cmd_arg.type)
	{
		case TRAJECTORY_STRAIGHT:
		case TRAJECTORY_STRAIGHT_TO_WALL:
			trajectory_dest.x += ((int64_t)trajectory_dest.ca * (int64_t)cmd_arg.dist) >> 30;
			trajectory_dest.y += ((int64_t)trajectory_dest.sa * (int64_t)cmd_arg.dist) >> 30;
			if(cmd_arg.dist > 0)
			{
				trajectory_way = TRAJECTORY_FORWARD;
			}
			else
			{
				trajectory_way = TRAJECTORY_BACKWARD;
			}
			break;
		case TRAJECTORY_ROTATE:
			trajectory_dest.alpha += cmd_arg.alpha;
			trajectory_dest.ca = fx_cos(trajectory_dest.alpha);
			trajectory_dest.sa = fx_sin(trajectory_dest.alpha);
			break;
		case TRAJECTORY_ROTATE_TO:
			trajectory_dest.alpha += control_find_rotate(trajectory_dest.alpha, cmd_arg.alpha);
			trajectory_dest.ca = fx_cos(trajectory_dest.alpha);
			trajectory_dest.sa = fx_sin(trajectory_dest.alpha);
			break;
		case TRAJECTORY_GOTO_XY:
			trajectory_dest.x = cmd_arg.x;
			trajectory_dest.y = cmd_arg.y;
			trajectory_approx_dist = cmd_arg.dist;
			trajectory_way = cmd_arg.way;
			break;
		case TRAJECTORY_GOTO_XYA:
			trajectory_dest.x = cmd_arg.x;
			trajectory_dest.y = cmd_arg.y;
			trajectory_dest.alpha = cmd_arg.alpha;
			trajectory_dest.ca = fx_cos(trajectory_dest.alpha);
			trajectory_dest.sa = fx_sin(trajectory_dest.alpha);
			trajectory_approx_dist = cmd_arg.dist;
			trajectory_way = cmd_arg.way;
			break;
		case TRAJECTORY_GOTO_GRAPH:
			trajectory_find_way_to_graph();
			break;
		case TRAJECTORY_FREE:
		default:
			trajectory_type = TRAJECTORY_FREE;
			control_free();
			return;
	}

	trajectory_state = TRAJECTORY_TO_DEST;
	trajectory_compute();
}

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
			trajectory_update();
		}

		if(ev & EVENT_CONTROL_COLSISION)
		{
			vTaskClearEvent(EVENT_CONTROL_COLSISION);

			switch(trajectory_avoidance_type)
			{
				default:
				case TRAJECTORY_AVOIDANCE_STOP:
					log(LOG_INFO, "collision -> stop");
					trajectory_state = TRAJECTORY_NONE;
					break;
				case TRAJECTORY_AVOIDANCE_GRAPH:
					log(LOG_INFO, "collision -> graph");
					trajectory_state = TRAJECTORY_USE_GRAPH;
					trajectory_compute();
					break;
			}
		}

		if(ev & EVENT_CONTROL_TARGET_REACHED)
		{
			vTaskClearEvent(EVENT_CONTROL_TARGET_REACHED);

			switch(trajectory_state)
			{
				default:
				case TRAJECTORY_NONE:
					break;
				case TRAJECTORY_TO_DEST:
					log(LOG_INFO, "target reached");
					break;
				case TRAJECTORY_USE_GRAPH:
					trajectory_state = TRAJECTORY_USE_GRAPH;
					trajectory_compute();
					break;
			}
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
			control_set_front_object(&a, TRAJECTORY_APPROX_DIST);
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
		// TODO prendre en compte la rotation sur place en plus de la ligne droite
		// 10mm de marge / control
		if(node_dist[i].dist < xmin - PARAM_LEFT_CORNER_X - TRAJECTORY_APPROX_DIST - (10<<16))
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

static void trajectory_compute()
{
	if(trajectory_state == TRAJECTORY_TO_DEST)
	{
		enum control_type control_type = CONTROL_LINE_XYA;
		switch(trajectory_type)
		{
			case TRAJECTORY_FREE:
				break;
			case TRAJECTORY_STRAIGHT:
			case TRAJECTORY_STRAIGHT_TO_WALL:
			case TRAJECTORY_GOTO_XY:
			case TRAJECTORY_GOTO_GRAPH:
				control_type = CONTROL_LINE_XY;
				break;
			case TRAJECTORY_ROTATE:
			case TRAJECTORY_ROTATE_TO:
				control_type = CONTROL_LINE_A;
				break;
			default:
			case TRAJECTORY_GOTO_XYA:
				control_type = CONTROL_LINE_XYA;
				break;
		};
		control_goto_near(trajectory_dest.x, trajectory_dest.y, trajectory_dest.alpha, trajectory_approx_dist, control_type, trajectory_way);
	}
	else if( trajectory_state == TRAJECTORY_USE_GRAPH)
	{
		// TODO
		trajectory_find_way_to_graph();
	}
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
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_straight_to_wall(int32_t dist)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_STRAIGHT_TO_WALL;
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	trajectory_request.dist = dist;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_rotate(int32_t angle)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_ROTATE;
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	trajectory_request.alpha = angle;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_rotate_to(int32_t angle)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_ROTATE_TO;
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	trajectory_request.alpha = angle;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_straight(int32_t dist)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_STRAIGHT;
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	trajectory_request.dist = dist;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_goto_near_xy(int32_t x, int32_t y, int32_t dist, enum trajectory_way way, enum trajectory_avoidance_type avoidance_type)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_GOTO_XY;
	trajectory_request.avoidance_type = avoidance_type;
	trajectory_request.x = x;
	trajectory_request.y = y;
	trajectory_request.dist = dist;
	trajectory_request.way = way;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_goto_near(int32_t x, int32_t y, int32_t alpha, int32_t dist, enum trajectory_way way, enum trajectory_avoidance_type avoidance_type)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_GOTO_XYA;
	trajectory_request.avoidance_type = avoidance_type;
	trajectory_request.x = x;
	trajectory_request.y = y;
	trajectory_request.alpha = alpha;
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

	trajectory_request.type = TRAJECTORY_GOTO_XY;
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
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
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}