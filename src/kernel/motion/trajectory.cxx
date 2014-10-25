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
#include "kernel/robot_parameters.h"
#include "trajectory.h"
#include "kernel/detection.h"
#include "kernel/motion/graph.h"
#include <stdlib.h>

#define TRAJECTORY_STACK_SIZE       350
#define TRAJECTORY_APPROX_DIST     (150<<16)      //!< distance d'approche d'un objet
#define TRAJECTORY_QUEUE_SIZE        20

// evenements de trajectory
enum
{
// def dans control
//	CONTROL_TARGET_REACHED = 0,   //!< cible atteinte
//	CONTROL_TARGET_NOT_REACHED,   //!< cible non atteinte
//	CONTROL_COLSISION,            //!< collision
//	CONTROL_TIMEOUT,              //!< timeout
	TRAJECTORY_DETECTION_UPDATE = 50,
	TRAJECTORY_UPDATE,
};

void trajectory_cmd(void* arg);
static void trajectory_task(void* arg);
static void trajectory_compute(uint32_t event);
static void trajectory_continue();
static int trajectory_find_way_to_graph(VectPlan pos);
//static void trajectory_control_callback(enum control_status status);
static void trajectory_detection_callback();
static float trajectory_find_rotate(float debut, float fin);

static float trajectory_detect_dist_min;
static xQueueHandle trajectory_queue;

// requete pour la tache trajectory + mutex
struct trajectory_cmd_arg trajectory_request;
static xSemaphoreHandle trajectory_mutex;

// donnees privees a la tache
static VectPlan trajectory_pos; //!< position du robot au moment du reveil de la tache
static VectPlan trajectory_dest;
static int32_t trajectory_approx_dist;
static enum trajectory_way trajectory_way;
static enum trajectory_cmd_type trajectory_type;
static enum trajectory_avoidance_type trajectory_avoidance_type;
static enum trajectory_state trajectory_state;
static int32_t trajectory_hokuyo_enable_check; //!< utilisation ou non des hokuyos
static int32_t trajectory_static_check_enable; //!< verification des éléments statiques
static struct graph_dijkstra_info trajectory_dijkstra_info[GRAPH_NUM_NODE];
//static uint8_t trajectory_graph_valid_links[GRAPH_NUM_LINK];
static uint8_t trajectory_current_graph_id;
static uint8_t trajectory_last_graph_id;
// TODO rendre parametrable
KinematicsParameters trajectory_linear_param = {1000, 1000, 1000};
KinematicsParameters trajectory_angular_param = {1, 1, 1};

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

	trajectory_queue = xQueueCreate(TRAJECTORY_QUEUE_SIZE, 1);

	if( ! trajectory_queue )
	{
		return ERR_INIT_TRAJECTORY;
	}

	usb_add_cmd(USB_CMD_TRAJECTORY, &trajectory_cmd);

	trajectory_state = TRAJECTORY_STATE_NONE;
	trajectory_hokuyo_enable_check = 1;
	trajectory_static_check_enable = 1;
	trajectory_detect_dist_min = PARAM_RIGHT_CORNER_X;
// TODO
//	control_register_event_callback(trajectory_control_callback);
	detection_register_callback(trajectory_detection_callback);

	return 0;
}

module_init(trajectory_module_init, INIT_TRAJECTORY);

static float trajectory_find_rotate(float debut, float fin)
{
	float theta = fmodf(fin - debut, 2*M_PI);

	// retour dans [ -M_PI ; M_PI ]
	if(theta < M_PI)
	{
		theta += 2*M_PI;
	}
	else if(theta > M_PI)
	{
		theta -= 2*M_PI;
	}

	return theta;
}

static void trajectory_update()
{
	struct trajectory_cmd_arg cmd_arg;
	int id;

	// mutex pour trajectory_request
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	cmd_arg = trajectory_request;
	xSemaphoreGive(trajectory_mutex);

	trajectory_state = TRAJECTORY_STATE_NONE;
	trajectory_dest = trajectory_pos;
	trajectory_approx_dist = 0;
	trajectory_way = TRAJECTORY_ANY_WAY;
	trajectory_avoidance_type = (enum trajectory_avoidance_type)cmd_arg.avoidance_type;
	trajectory_type = (enum trajectory_cmd_type)cmd_arg.type;

	switch(trajectory_type)
	{
		case TRAJECTORY_STRAIGHT_TO_WALL:
			//control_back_to_wall(); // TODO
			return;
		case TRAJECTORY_STRAIGHT:
			trajectory_dest.x += cosf(trajectory_dest.theta) * cmd_arg.dist;
			trajectory_dest.y += sinf(trajectory_dest.theta) * cmd_arg.dist;
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
			trajectory_dest.theta += cmd_arg.dest.theta;
			break;
		case TRAJECTORY_ROTATE_TO:
			trajectory_dest.theta += trajectory_find_rotate(trajectory_dest.theta, cmd_arg.dest.theta);
			break;
		case TRAJECTORY_GOTO_XY:
			trajectory_dest.x = cmd_arg.dest.x;
			trajectory_dest.y = cmd_arg.dest.y;
			trajectory_approx_dist = cmd_arg.dist;
			trajectory_way = (enum trajectory_way)cmd_arg.way;
			break;
		case TRAJECTORY_GOTO_XYA:
			trajectory_dest = cmd_arg.dest;
			trajectory_approx_dist = cmd_arg.dist;
			trajectory_way = (enum trajectory_way)cmd_arg.way;
			break;
		case TRAJECTORY_GOTO_GRAPH:
			id = trajectory_find_way_to_graph(trajectory_pos);
			log_format(LOG_INFO, "point graph : %d", id);

			trajectory_dest.x = graph_node[id].pos.x;
			trajectory_dest.y = graph_node[id].pos.y;
			trajectory_dest.theta = 0;
			break;
		case TRAJECTORY_FREE:
		default:
			trajectory_type = TRAJECTORY_FREE;
			motion_enable(false);
			return;
	}

	trajectory_state = TRAJECTORY_STATE_MOVING_TO_DEST;

	// verification : trajectoire possible ?
	if(trajectory_static_check_enable && (cmd_arg.type == TRAJECTORY_GOTO_XY || cmd_arg.type == TRAJECTORY_GOTO_XYA))
	{
		// TODO : check fait pour la marche avant, tolérance plus grande si on le fait en marche arrière ?
		// on regarde si c'est possible / objets statiques de la table
		// si c'est pas possible, on passe par le graph
		Vect2 a_table;
		Vect2 b_table;

		// position actuelle avec alignement (marche avant) avec la destination
		VectPlan pos = trajectory_pos;
		float dx = trajectory_dest.x - trajectory_pos.x;
		float dy = trajectory_dest.y - trajectory_pos.y;
		pos.theta = atan2f(dy, dx);

		float xmin = detection_compute_front_object(DETECTION_STATIC_OBJ, pos, &a_table, &b_table, trajectory_detect_dist_min);

		// passage en mm pour faire les calculs sur 32bits
		float dist2 = dx * dx +  dy * dy;
		float xmin2 = xmin * xmin;
		if( xmin2 < dist2)
		{
			// trajectoire impossible, on passe par le graph
			log(LOG_INFO, "goto - obstacle statique, utilisation du graph");
			trajectory_state = TRAJECTORY_STATE_MOVING_TO_GRAPH;
		}
	}

	trajectory_compute(0);
}
#if 0
static void trajectory_control_callback(enum control_status status)
{
	unsigned char event = status;
	xQueueSend(trajectory_queue, &event, 0);
}
#endif
static void trajectory_detection_callback()
{
	unsigned char event = TRAJECTORY_DETECTION_UPDATE;
	xQueueSend(trajectory_queue, &event, 0);
}

static void trajectory_task(void* arg)
{
	(void) arg;

	unsigned char event;

	while(1)
	{
		xQueueReceive(trajectory_queue, &event, portMAX_DELAY);

		trajectory_pos = location_get_position();

		if(event == TRAJECTORY_UPDATE)
		{
			trajectory_update();
		}
		else if(event == MOTION_COLSISION)
		{
			switch(trajectory_avoidance_type)
			{
				default:
				case TRAJECTORY_AVOIDANCE_STOP:
					// pas d'évitement, fin de la trajectoire
					log(LOG_INFO, "collision -> stop");
					trajectory_state = TRAJECTORY_STATE_COLISION;
					break;
				case TRAJECTORY_AVOIDANCE_GRAPH:
					log(LOG_INFO, "collision -> graph");
					// si on n'est pas sur le graph
					if(trajectory_state != TRAJECTORY_STATE_USING_GRAPH)
					{
						trajectory_state = TRAJECTORY_STATE_MOVING_TO_GRAPH;
					}
					trajectory_compute(event);
					break;
			}
		}
		else if(event == MOTION_TARGET_REACHED)
		{
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
					break;
				case TRAJECTORY_STATE_MOVING_TO_GRAPH:
					trajectory_state = TRAJECTORY_STATE_USING_GRAPH;
					trajectory_compute(event);
					break;
				case TRAJECTORY_STATE_USING_GRAPH:
					trajectory_continue();
					break;
			}
		}
		else if( event == MOTION_TARGET_NOT_REACHED)
		{
			log(LOG_ERROR, "target not reached");
			if( trajectory_state != TRAJECTORY_STATE_USING_GRAPH )
			{
				trajectory_state = TRAJECTORY_STATE_TARGET_NOT_REACHED;
			}
			else
			{
				// on continue sur le graph
				trajectory_compute(event);
			}
		}
		else if( event == TRAJECTORY_DETECTION_UPDATE)
		{
			Vect2 a(1e30, 1e30);
			if( trajectory_hokuyo_enable_check )
			{
				Vect2 b;
				detection_compute_front_object(DETECTION_DYNAMIC_OBJ, trajectory_pos, &a, &b, trajectory_detect_dist_min);
				VectPlan pos_ar = trajectory_pos;
				pos_ar.theta += M_PI;
				//detection_compute_front_object(DETECTION_DYNAMIC_OBJ, &pos_ar, &c, &b);
			}

			// TODO
			//control_set_front_object(&a, TRAJECTORY_APPROX_DIST);
			//control_set_back_object(&c, TRAJECTORY_APPROX_DIST);
		}
		// TODO voir / CONTROL_TIMEOUT
	}
}

static int trajectory_find_way_to_graph(VectPlan pos)
{
	// passe en stack, pas trop de noeuds
	struct graph_node_dist node_dist[GRAPH_NUM_NODE];
	struct Vect2 p(pos.x, pos.y);

	graph_compute_node_distance(p, node_dist);

	Vect2 a_table;
	Vect2 b_table;
	float xmin;
	int i;
	int id = 0;
	float dist;
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
		float dx = graph_node[id].pos.x - pos.x;
		float dy = graph_node[id].pos.y - pos.y;
		pos.theta = atan2f(dy, dx);
		xmin = detection_compute_front_object(detect_type, pos, &a_table, &b_table, trajectory_detect_dist_min);
		// TODO prendre en compte la rotation sur place en plus de la ligne droite
		// 10mm de marge / control
		dist = node_dist[i].dist;
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

static void trajectory_compute(uint32_t event)
{
	(void) event;
	if( trajectory_state == TRAJECTORY_STATE_MOVING_TO_GRAPH)
	{
		trajectory_current_graph_id = trajectory_find_way_to_graph(trajectory_pos);
		log_format(LOG_INFO, "point entrée graph : %d", trajectory_current_graph_id);

		VectPlan dest(graph_node[trajectory_current_graph_id].pos.x, graph_node[trajectory_current_graph_id].pos.y, 0);
		motion_goto(dest, VectPlan(), TRAJECTORY_ANY_WAY, MOTION_AXIS_XYA,trajectory_linear_param, trajectory_angular_param);
		// TODO goto_near
		//control_goto_near(graph_node[trajectory_current_graph_id].pos.x, graph_node[trajectory_current_graph_id].pos.y, 0, 0, CONTROL_LINE_XY, TRAJECTORY_ANY_WAY);
	}
#if 0
	else if( trajectory_state == TRAJECTORY_STATE_USING_GRAPH)
	{
		trajectory_last_graph_id = trajectory_find_way_to_graph(trajectory_dest);
		if(trajectory_current_graph_id == trajectory_last_graph_id)
		{
			// on est au point du graphe le plus proche de notre destination 
			trajectory_state = TRAJECTORY_STATE_MOVING_TO_DEST;
		}
		else
		{
			// on utilise le graphe, et on a un event traj echouée, ou ennemi sur le chemin
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
					xmin = detection_compute_front_object(detect_type, &pos, &a_table, &b_table, trajectory_detect_dist_min) >> 16;
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


			if((event == CONTROL_COLSISION)&&(trajectory_state == TRAJECTORY_STATE_USING_GRAPH))
			{
				//si on recompute la position en cours de parcours du graph pour collision
				//on supprime l'arc courant pour le calcul
				int i = trajectory_last_graph_id;
				int j;
				while(trajectory_dijkstra_info[i].prev_node != trajectory_current_graph_id)
				{
					i = trajectory_dijkstra_info[i].prev_node;
				}
				for (j=0;j<GRAPH_NUM_LINK; j++)
				{
					if((graph_link[j].a == trajectory_current_graph_id)&&(graph_link[j].b == i))
					{
						trajectory_graph_valid_links[j]=0;
					}
				}
			}
				

			log_format(LOG_INFO, "graph_dijkstra de %d à %d", trajectory_current_graph_id, trajectory_last_graph_id);
			int res = graph_dijkstra(trajectory_current_graph_id, trajectory_last_graph_id, trajectory_dijkstra_info, trajectory_graph_valid_links);
			if(res)
			{
				log_format(LOG_INFO, "aucun chemin trouvé");
				trajectory_state = TRAJECTORY_STATE_TARGET_NOT_REACHED;
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
				control_goto_near(graph_node[i].pos.x, graph_node[i].pos.y, 0, 0, CONTROL_LINE_XY, TRAJECTORY_FORWARD);
			}
		}
	}
#endif
	if(trajectory_state == TRAJECTORY_STATE_MOVING_TO_DEST)
	{
		/*enum control_type control_type = CONTROL_LINE_XYA;
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
		};*/
		motion_goto(trajectory_dest, VectPlan(), TRAJECTORY_ANY_WAY, MOTION_AXIS_XYA, trajectory_linear_param, trajectory_angular_param);
		// TODO goto_near
		//control_goto_near(trajectory_dest.x, trajectory_dest.y, trajectory_dest.alpha, trajectory_approx_dist, control_type, trajectory_way);
	}
}

static void trajectory_continue()
{
	trajectory_last_graph_id = trajectory_find_way_to_graph(trajectory_dest);
	if(trajectory_current_graph_id == trajectory_dijkstra_info[trajectory_last_graph_id].prev_node)
	{
		// on est au point du graphe le plus proche de notre destination 
		trajectory_current_graph_id = trajectory_last_graph_id;
		trajectory_state = TRAJECTORY_STATE_MOVING_TO_DEST;
	}
	else
	{
		// on utilise le graphe, et on a un event traj finie
		int i = trajectory_last_graph_id;
		log(LOG_INFO, "trajectory getting next node");
		while(trajectory_dijkstra_info[trajectory_dijkstra_info[i].prev_node].prev_node != trajectory_current_graph_id)
		{
			i = trajectory_dijkstra_info[i].prev_node;
		}
		trajectory_current_graph_id = trajectory_dijkstra_info[i].prev_node;
		log_format(LOG_INFO, "going from - to : %d -> %d", trajectory_current_graph_id, i);
		motion_goto(VectPlan(graph_node[i].pos.x, graph_node[i].pos.y, 0), VectPlan(), TRAJECTORY_ANY_WAY, MOTION_AXIS_XYA, trajectory_linear_param, trajectory_angular_param);
		// TODO goto_near
		//control_goto_near(graph_node[i].pos.x, graph_node[i].pos.y, 0, 0, CONTROL_LINE_XY, TRAJECTORY_FORWARD);
	}

	if(trajectory_state == TRAJECTORY_STATE_MOVING_TO_DEST)
	{
		/*enum control_type control_type = CONTROL_LINE_XYA;
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
		};*/
		motion_goto(trajectory_dest, VectPlan(), TRAJECTORY_ANY_WAY, MOTION_AXIS_XYA, trajectory_linear_param, trajectory_angular_param);
		// TODO goto_near
		//control_goto_near(trajectory_dest.x, trajectory_dest.y, trajectory_dest.alpha, trajectory_approx_dist, control_type, trajectory_way);
	}
}

void trajectory_cmd(void* arg)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	memcpy(&trajectory_request, arg, sizeof(struct trajectory_cmd_arg));
	xSemaphoreGive(trajectory_mutex);

	unsigned char event = TRAJECTORY_UPDATE;
	xQueueSend(trajectory_queue, &event, 50);
}

void trajectory_free()
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_request.type = TRAJECTORY_FREE;
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	xSemaphoreGive(trajectory_mutex);

	unsigned char event = TRAJECTORY_UPDATE;
	xQueueSend(trajectory_queue, &event, 50);
}

void trajectory_straight_to_wall()
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_request.type = TRAJECTORY_STRAIGHT_TO_WALL;
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	xSemaphoreGive(trajectory_mutex);

	unsigned char event = TRAJECTORY_UPDATE;
	xQueueSend(trajectory_queue, &event, 50);
}

void trajectory_rotate(float theta)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_request.type = TRAJECTORY_ROTATE;
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	trajectory_request.dest.theta = theta;
	xSemaphoreGive(trajectory_mutex);

	unsigned char event = TRAJECTORY_UPDATE;
	xQueueSend(trajectory_queue, &event, 50);
}

void trajectory_rotate_to(float theta)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_request.type = TRAJECTORY_ROTATE_TO;
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	trajectory_request.dest.theta = theta;
	xSemaphoreGive(trajectory_mutex);

	unsigned char event = TRAJECTORY_UPDATE;
	xQueueSend(trajectory_queue, &event, 50);
}

void trajectory_straight(float dist)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_request.type = TRAJECTORY_STRAIGHT;
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	trajectory_request.dist = dist;
	xSemaphoreGive(trajectory_mutex);

	unsigned char event = TRAJECTORY_UPDATE;
	xQueueSend(trajectory_queue, &event, 50);
}

void trajectory_goto_near_xy(float x, float y, float dist, enum trajectory_way way, enum trajectory_avoidance_type avoidance_type)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_request.type = TRAJECTORY_GOTO_XY;
	trajectory_request.avoidance_type = avoidance_type;
	trajectory_request.dest.x = x;
	trajectory_request.dest.y = y;
	trajectory_request.dist = dist;
	trajectory_request.way = way;
	xSemaphoreGive(trajectory_mutex);

	unsigned char event = TRAJECTORY_UPDATE;
	xQueueSend(trajectory_queue, &event, 50);
}

void trajectory_goto_near(VectPlan dest, float dist, enum trajectory_way way, enum trajectory_avoidance_type avoidance_type)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_request.type = TRAJECTORY_GOTO_XYA;
	trajectory_request.avoidance_type = avoidance_type;
	trajectory_request.dest = dest;
	trajectory_request.dist = dist;
	trajectory_request.way = way;
	xSemaphoreGive(trajectory_mutex);

	unsigned char event = TRAJECTORY_UPDATE;
	xQueueSend(trajectory_queue, &event, 50);
}

void trajectory_goto_graph_node(uint32_t node_id, float dist, enum trajectory_way way, enum trajectory_avoidance_type avoidance_type)
{
	if( node_id >= GRAPH_NUM_NODE)
	{
		log_format(LOG_ERROR, "node_id inconnu : %d", (int)node_id);
		return;
	}

	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_request.type = TRAJECTORY_GOTO_XY;
	trajectory_request.avoidance_type = avoidance_type;
	trajectory_request.dest.x = graph_node[node_id].pos.x;
	trajectory_request.dest.y = graph_node[node_id].pos.y;
	trajectory_request.dist = dist;
	trajectory_request.way = way;
	xSemaphoreGive(trajectory_mutex);

	unsigned char event = TRAJECTORY_UPDATE;
	xQueueSend(trajectory_queue, &event, 50);
}

void trajectory_goto_graph()
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_request.type = TRAJECTORY_GOTO_GRAPH;
	trajectory_request.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;
	xSemaphoreGive(trajectory_mutex);

	unsigned char event = TRAJECTORY_UPDATE;
	xQueueSend(trajectory_queue, &event, 50);
}

enum trajectory_state trajectory_get_state()
{
	return trajectory_state;
}
#if 0
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

void trajectory_set_detection_dist_min(int32_t dist_min)
{
	trajectory_detect_dist_min = dist_min;
}

// TODO faire mieux pour eviter le polling
int trajectory_wait(enum trajectory_state wanted_state, uint32_t timeout)
{
	enum trajectory_state state;

	do
	{
		vTaskDelay(1);
		timeout --;
		state = trajectory_state;
	}
	while(state != TRAJECTORY_STATE_COLISION && state != TRAJECTORY_STATE_TARGET_REACHED && state != TRAJECTORY_STATE_TARGET_NOT_REACHED && timeout);

	if( state != wanted_state )
	{
		log_format(LOG_ERROR, "incorrect state : %d", state);
		return -1;
	}

	return 0;
}
#endif
