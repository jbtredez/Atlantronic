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
#define TRAJECTORY_APPROX_DIST     (50<<16)      //!< distance d'approche d'un objet

void trajectory_cmd(void* arg);
static void trajectory_task(void* arg);
static void trajectory_compute();
static int trajectory_find_way_to_graph(struct fx_vect_pos pos);

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
static int32_t trajectory_hokuyo_enable_check; //!< utilisation ou non des hokuyos
static int32_t trajectory_static_check_enable; //!< verification des éléments statiques
static struct graph_dijkstra_info trajectory_dijkstra_info[GRAPH_NUM_NODE];
static uint8_t trajectory_graph_valid_links[GRAPH_NUM_LINK];
static uint8_t trajectory_current_graph_id;
static uint8_t trajectory_last_graph_id;

static int trajectory_module_init()
{
	xTaskHandle xHandle;

	portBASE_TYPE err = xTaskCreate(trajectory_task, "traj", TRAJECTORY_STACK_SIZE, NULL, PRIORITY_TASK_TRAJECTORY, &xHandle);

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

	trajectory_state = TRAJECTORY_STATE_NONE;
	trajectory_hokuyo_enable_check = 1;
	trajectory_static_check_enable = 1;

	return 0;
}

module_init(trajectory_module_init, INIT_TRAJECTORY);

static void trajectory_update()
{
	struct trajectory_cmd_arg cmd_arg;
	int id;

	// mutex pour trajectory_request
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	cmd_arg = trajectory_request;
	vTaskClearEvent(EVENT_TRAJECTORY_UPDATE);
	xSemaphoreGive(trajectory_mutex);

	trajectory_state = TRAJECTORY_STATE_NONE;
	trajectory_dest = trajectory_pos;
	trajectory_approx_dist = 0;
	trajectory_way = TRAJECTORY_ANY_WAY;
	trajectory_avoidance_type = cmd_arg.avoidance_type;
	trajectory_type = cmd_arg.type;

	switch(cmd_arg.type)
	{
		case TRAJECTORY_STRAIGHT_TO_WALL:
			control_back_to_wall();
			return;
		case TRAJECTORY_STRAIGHT:
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
			id = trajectory_find_way_to_graph(trajectory_pos);
			log_format(LOG_INFO, "point graph : %d", id);

			trajectory_dest.x = graph_node[id].pos.x;
			trajectory_dest.y = graph_node[id].pos.y;
			trajectory_dest.alpha = 0;
			trajectory_dest.ca = 1;
			trajectory_dest.sa = 0;
			break;
		case TRAJECTORY_FREE:
		default:
			trajectory_type = TRAJECTORY_FREE;
			control_free();
			return;
	}

	trajectory_state = TRAJECTORY_STATE_MOVING_TO_DEST;

	// verification : trajectoire possible ?
	if(trajectory_static_check_enable && (cmd_arg.type == TRAJECTORY_GOTO_XY || cmd_arg.type == TRAJECTORY_GOTO_XYA))
	{
		// TODO : check fait pour la marche avant, tolérance plus grande si on le fait en marche arrière ?
		// on regarde si c'est possible / objets statiques de la table
		// si c'est pas possible, on passe par le graph
		struct fx_vect2 a_table;
		struct fx_vect2 b_table;

		// position actuelle avec alignement (marche avant) avec la destination
		struct fx_vect_pos pos = trajectory_pos;
		int32_t dx = trajectory_dest.x - trajectory_pos.x;
		int32_t dy = trajectory_dest.y - trajectory_pos.y;
		pos.alpha = fx_atan2(dy, dx);
		pos.ca = fx_cos(pos.alpha);
		pos.sa = fx_sin(pos.alpha);

		int32_t xmin = detection_compute_front_object(DETECTION_STATIC_OBJ, &pos, &a_table, &b_table) >> 16;

		// passage en mm pour faire les calculs sur 32bits
		dx = dx >> 16;
		dy = dy >> 16;
		int32_t dist2 = dx * dx +  dy * dy;
		int32_t xmin2 = xmin * xmin;
		if( xmin2 < dist2)
		{
			// trajectoire impossible, on passe par le graph
			log(LOG_INFO, "goto - obstacle statique, utilisation du graph");
			trajectory_state = TRAJECTORY_STATE_MOVING_TO_GRAPH;
		}
	}

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
					// pas d'évitement, fin de la trajectoire
					log(LOG_INFO, "collision -> stop");
					trajectory_state = TRAJECTORY_STATE_COLISION;
					vTaskSetEvent(EVENT_TRAJECTORY_END);
					break;
				case TRAJECTORY_AVOIDANCE_GRAPH:
					log(LOG_INFO, "collision -> graph");
					// si on n'est pas sur le graph
					if(trajectory_state != TRAJECTORY_STATE_USING_GRAPH)
					{
						trajectory_state = TRAJECTORY_STATE_MOVING_TO_GRAPH;
					}
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
				case TRAJECTORY_STATE_NONE:
				case TRAJECTORY_STATE_TARGET_REACHED:
				case TRAJECTORY_STATE_TARGET_NOT_REACHED:
				case TRAJECTORY_STATE_COLISION:
					break;
				case TRAJECTORY_STATE_MOVING_TO_DEST:
					log(LOG_INFO, "target reached");
					trajectory_state = TRAJECTORY_STATE_TARGET_REACHED;
					vTaskSetEvent(EVENT_TRAJECTORY_END);
					break;
				case TRAJECTORY_STATE_MOVING_TO_GRAPH:
					trajectory_state = TRAJECTORY_STATE_USING_GRAPH;
					trajectory_compute();
					break;
				case TRAJECTORY_STATE_USING_GRAPH:
					trajectory_compute();
					break;
			}
		}

		if( ev & EVENT_CONTROL_TARGET_NOT_REACHED)
		{
			vTaskClearEvent(EVENT_CONTROL_TARGET_NOT_REACHED);

			log(LOG_ERROR, "target not reached");
			trajectory_state = TRAJECTORY_STATE_TARGET_NOT_REACHED;
			vTaskSetEvent(EVENT_TRAJECTORY_END);
		}

		if( ev & EVENT_DETECTION_UPDATED)
		{
			vTaskClearEvent(EVENT_DETECTION_UPDATED);

			struct fx_vect2 a = {1 << 30, 1 << 30};
			if( trajectory_hokuyo_enable_check )
			{
				struct fx_vect2 b;
				detection_compute_front_object(DETECTION_DYNAMIC_OBJ, &trajectory_pos, &a, &b);
			}
			control_set_front_object(&a, TRAJECTORY_APPROX_DIST);
		}
	}
}

static int trajectory_find_way_to_graph(struct fx_vect_pos pos)
{
	// passe en stack, pas trop de noeuds
	struct graph_node_dist node_dist[GRAPH_NUM_NODE];
	struct fx_vect2 p = {pos.x, pos.y};

	graph_compute_node_distance(p, node_dist);

	struct fx_vect2 a_table;
	struct fx_vect2 b_table;
	int32_t xmin;
	int i;
	int id = 0;
	int32_t dist;
	enum detection_type detect_type = DETECTION_FULL;

	if(!trajectory_static_check_enable && !trajectory_hokuyo_enable_check)
	{
		// on y va direct, pas de gestion d'obstacles
		return node_dist[0].id;
	}

	if(trajectory_static_check_enable && ! trajectory_hokuyo_enable_check)
	{
		detect_type = DETECTION_STATIC_OBJ;
	}
	else if(!trajectory_static_check_enable && trajectory_hokuyo_enable_check)
	{
		detect_type = DETECTION_DYNAMIC_OBJ;
	}

	for( i = 0 ; i < GRAPH_NUM_NODE; i++)
	{
		id = node_dist[i].id;
		int32_t dx = graph_node[id].pos.x - pos.x;
		int32_t dy = graph_node[id].pos.y - pos.y;
		pos.alpha = fx_atan2(dy, dx);
		pos.ca = fx_cos(pos.alpha);
		pos.sa = fx_sin(pos.alpha);
		xmin = detection_compute_front_object(detect_type, &pos, &a_table, &b_table);
		// TODO prendre en compte la rotation sur place en plus de la ligne droite
		// 10mm de marge / control
		dist = node_dist[i].dist;
		dist <<= 16;
		if( dist < xmin - PARAM_LEFT_CORNER_X - TRAJECTORY_APPROX_DIST - (10<<16))
		{
			break;
		}
	}

	// rien de réalisable pour le moment, on va vers le plus proche
	if(i == GRAPH_NUM_NODE)
	{
		return node_dist[0].id;
	}

	return id;
}

static void trajectory_compute()
{
	if( trajectory_state == TRAJECTORY_STATE_MOVING_TO_GRAPH)
	{
		trajectory_current_graph_id = trajectory_find_way_to_graph(trajectory_pos);
		log_format(LOG_INFO, "point entrée graph : %d", trajectory_current_graph_id);

		control_goto_near(graph_node[trajectory_current_graph_id].pos.x, graph_node[trajectory_current_graph_id].pos.y, 0, 0, CONTROL_LINE_XY, TRAJECTORY_ANY_WAY);
	}
	else if( trajectory_state == TRAJECTORY_STATE_USING_GRAPH)
	{
		trajectory_last_graph_id = trajectory_find_way_to_graph(trajectory_dest);
		if(trajectory_current_graph_id == trajectory_last_graph_id)
		{
			trajectory_state = TRAJECTORY_STATE_MOVING_TO_DEST;
		}
		else
		{
			int i = 0;
			struct fx_vect_pos pos;
			struct fx_vect2 a_table;
			struct fx_vect2 b_table;
			int32_t xmin;

			if(!trajectory_static_check_enable && !trajectory_hokuyo_enable_check)
			{
				// pas de gestion d'obstacles
			}
			else
			{
				enum detection_type detect_type = DETECTION_FULL;
				if(trajectory_static_check_enable && ! trajectory_hokuyo_enable_check)
				{
					detect_type = DETECTION_STATIC_OBJ;
				}
				else if(!trajectory_static_check_enable && trajectory_hokuyo_enable_check)
				{
					detect_type = DETECTION_DYNAMIC_OBJ;
				}

				for(i=0; i< GRAPH_NUM_LINK; i++)
				{
					pos.x = graph_node[graph_link[i].a].pos.x;
					pos.y = graph_node[graph_link[i].a].pos.y;
					pos.alpha = graph_link[i].alpha;
					pos.ca = fx_cos(pos.alpha);
					pos.sa = fx_sin(pos.alpha);
					xmin = detection_compute_front_object(detect_type, &pos, &a_table, &b_table) >> 16;
					if(graph_link[i].dist < xmin)
					{
						trajectory_graph_valid_links[i] = 1;
					}
					else
					{
						trajectory_graph_valid_links[i] = 0;
					}
				}
			}

			log_format(LOG_INFO, "graph_dijkstra de %d à %d", trajectory_current_graph_id, trajectory_last_graph_id);
			int res = graph_dijkstra(trajectory_current_graph_id, trajectory_last_graph_id, trajectory_dijkstra_info, trajectory_graph_valid_links);
			if(res)
			{
				log_format(LOG_INFO, "aucun chemin trouvé");
				// TODO erreur
			}
			else
			{
				int i = trajectory_last_graph_id;
				while(trajectory_dijkstra_info[i].prev_node != trajectory_current_graph_id)
				{
					log_format(LOG_INFO, "chemin - graph : %d <- %d", i, trajectory_dijkstra_info[i].prev_node);
					i = trajectory_dijkstra_info[i].prev_node;
				}
				log_format(LOG_INFO, "chemin - graph : %d <- %d", i, trajectory_dijkstra_info[i].prev_node);
				trajectory_current_graph_id = i;
				control_goto_near(graph_node[i].pos.x, graph_node[i].pos.y, 0, 0, CONTROL_LINE_XY, TRAJECTORY_FORWARD);
			}
		}
	}

	if(trajectory_state == TRAJECTORY_STATE_MOVING_TO_DEST)
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
	vTaskClearEvent(EVENT_TRAJECTORY_END);
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_straight_to_wall()
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_STRAIGHT_TO_WALL;
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	vTaskClearEvent(EVENT_TRAJECTORY_END);
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_rotate(int32_t angle)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_ROTATE;
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	trajectory_request.alpha = angle;
	vTaskClearEvent(EVENT_TRAJECTORY_END);
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_rotate_to(int32_t angle)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_ROTATE_TO;
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	trajectory_request.alpha = angle;
	vTaskClearEvent(EVENT_TRAJECTORY_END);
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_straight(int32_t dist)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_STRAIGHT;
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	trajectory_request.dist = dist;
	vTaskClearEvent(EVENT_TRAJECTORY_END);
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
	vTaskClearEvent(EVENT_TRAJECTORY_END);
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
	vTaskClearEvent(EVENT_TRAJECTORY_END);
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
	vTaskClearEvent(EVENT_TRAJECTORY_END);
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

void trajectory_goto_graph()
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);

	trajectory_request.type = TRAJECTORY_GOTO_GRAPH;
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	vTaskClearEvent(EVENT_TRAJECTORY_END);
	vTaskSetEvent(EVENT_TRAJECTORY_UPDATE);

	xSemaphoreGive(trajectory_mutex);
}

enum trajectory_state trajectory_get_state()
{
	return trajectory_state;
}

void trajectory_disable_static_check()
{
	trajectory_static_check_enable = 0;
}

void trajectory_enable_static_check()
{
	trajectory_static_check_enable = 1;
}

void trajectory_disable_hokuyo()
{
	trajectory_hokuyo_enable_check = 0;
}

void trajectory_enable_hokuyo()
{
	trajectory_hokuyo_enable_check = 1;
}