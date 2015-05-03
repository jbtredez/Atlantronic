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

#define TRAJECTORY_STACK_SIZE       400
#define TRAJECTORY_APPROX_DIST      150      //!< distance d'approche d'un objet
#define TRAJECTORY_PERIOD            10

void trajectory_cmd(void* arg);
static void trajectory_task(void* arg);
static void trajectory_update();
static int trajectory_find_way_to_graph(VectPlan pos, enum detection_type detect_type);
static void trajectory_detection_callback();

// requete pour la tache trajectory + mutex
static struct trajectory_cmd_arg trajectory_request;
static bool trajectory_new_request;
static xSemaphoreHandle trajectory_mutex;

// donnees privees a la tache
static VectPlan trajectory_pos; //!< position du robot au moment du reveil de la tache
static VectPlan trajectory_dest;
static float trajectory_approx_dist;
static enum motion_way trajectory_way;
static enum trajectory_cmd_type trajectory_type;
static enum avoidance_type trajectory_avoidance_type;
static enum trajectory_state trajectory_state;
static int32_t trajectory_hokuyo_enable_check; //!< utilisation ou non des hokuyos
static int32_t trajectory_static_check_enable; //!< verification des éléments statiques
static struct graph_dijkstra_info trajectory_dijkstra_info[GRAPH_NUM_NODE];
static uint8_t trajectory_graph_valid_links[GRAPH_NUM_LINK];
static uint8_t trajectory_graph_way[GRAPH_NUM_NODE];
static int trajectory_graph_way_count;
static uint8_t trajectory_graph_way_id;
KinematicsParameters trajectory_linear_param = {1000, 400, 400};
KinematicsParameters trajectory_angular_param = {3, 5, 5};

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

	detection_register_callback(trajectory_detection_callback);

	return 0;
}

module_init(trajectory_module_init, INIT_TRAJECTORY);

static void trajectory_task(void* arg)
{
	(void) arg;
	uint32_t wake_time = 0;
	enum motion_state motion_state;
	enum motion_status motion_status;
	enum motion_trajectory_step motion_traj_step;
	enum motion_wanted_state motion_wanted_state;

	while(1)
	{
		trajectory_pos = location_get_position();
		motion_get_state(&motion_state, &motion_status, &motion_traj_step, &motion_wanted_state);

		if( trajectory_new_request )
		{
			trajectory_update();
		}

		switch(trajectory_state)
		{
			default:
			case TRAJECTORY_STATE_UPDATING_TRAJECTORY:
			case TRAJECTORY_STATE_NONE:
			case TRAJECTORY_STATE_TARGET_REACHED:
			case TRAJECTORY_STATE_TARGET_NOT_REACHED:
			case TRAJECTORY_STATE_COLISION:
				break;
			case TRAJECTORY_STATE_MOVING_TO_DEST:
				switch( motion_status )
				{
					case MOTION_TARGET_REACHED:
						log(LOG_INFO, "TRAJECTORY_TARGET_REACHED");
						trajectory_state = TRAJECTORY_STATE_TARGET_REACHED;
						break;
					case MOTION_TARGET_NOT_REACHED:
						log(LOG_ERROR, "TRAJECTORY_TARGET_NOT_REACHED");
						trajectory_state = TRAJECTORY_STATE_TARGET_NOT_REACHED;
						break;
					case MOTION_COLSISION:
						switch(trajectory_avoidance_type)
						{
							default:
							case AVOIDANCE_STOP:
								// pas d'évitement, fin de la trajectoire
								log(LOG_INFO, "collision -> stop");
								trajectory_state = TRAJECTORY_STATE_COLISION;
								break;
							case AVOIDANCE_GRAPH:
								log(LOG_INFO, "collision -> graph");
								// si on n'est pas sur le graph
								// TODO retenter n fois puis recalculer une trajectoire ?
								// pour le moment arret
								log(LOG_INFO, "collision -> stop");
								trajectory_state = TRAJECTORY_STATE_COLISION;
								break;
						}
						break;
					default:
					case MOTION_TIMEOUT:
						log(LOG_INFO, "timeout -> target not reached");
						trajectory_state = TRAJECTORY_STATE_TARGET_NOT_REACHED;
						break;
					case MOTION_IN_MOTION:
					case MOTION_UPDATING_TRAJECTORY:
						break;
				}
				break;
			case TRAJECTORY_STATE_MOVE_TO_DEST:
				if( motion_state == MOTION_ENABLED && motion_wanted_state == MOTION_WANTED_STATE_UNKNOWN )
				{
					VectPlan dest = trajectory_dest;
					VectPlan u = trajectory_dest - trajectory_pos;
					float ds = u.norm();
					motion_trajectory_type traj_type = MOTION_AXIS_XYA;
					if( fabsf(ds) > EPSILON )
					{
						dest.x -= trajectory_approx_dist * u.x / ds;
						dest.y -= trajectory_approx_dist * u.y / ds;
					}
					else if( trajectory_type == TRAJECTORY_ROTATE )
					{
						traj_type = MOTION_AXIS_A;
					}

					motion_goto(dest, VectPlan(), trajectory_way, traj_type, trajectory_linear_param, trajectory_angular_param);
					trajectory_state = TRAJECTORY_STATE_MOVING_TO_DEST;
				}
				break;
			case TRAJECTORY_STATE_USING_GRAPH:
				if( motion_state == MOTION_ENABLED && motion_wanted_state == MOTION_WANTED_STATE_UNKNOWN )
				{
					if( trajectory_graph_way_id < trajectory_graph_way_count - 1 )
					{
						trajectory_graph_way_id++;
						int i = trajectory_graph_way[trajectory_graph_way_id];
						log_format(LOG_INFO, "goto graph node %d", i);
						VectPlan dest(graph_node[i].pos.x, graph_node[i].pos.y, 0);
						motion_goto(dest, VectPlan(), WAY_FORWARD, MOTION_AXIS_XY, trajectory_linear_param, trajectory_angular_param);
					}
					else
					{
						motion_goto(trajectory_dest, VectPlan(), trajectory_way, MOTION_AXIS_XYA, trajectory_linear_param, trajectory_angular_param);
						trajectory_state = TRAJECTORY_STATE_MOVING_TO_DEST;
					}
				}
				break;
			case TRAJECTORY_STATE_MOVE_TO_GRAPH:
				if( motion_state == MOTION_ENABLED && motion_wanted_state == MOTION_WANTED_STATE_UNKNOWN)
				{
					trajectory_state = TRAJECTORY_STATE_USING_GRAPH;
					int i = trajectory_graph_way[0];
					VectPlan dest(graph_node[i].pos.x, graph_node[i].pos.y, 0);
					log_format(LOG_INFO, "goto graph node %d : %d %d", i, (int)dest.x, (int)dest.y);
					motion_goto(dest, VectPlan(), WAY_FORWARD, MOTION_AXIS_XY, trajectory_linear_param, trajectory_angular_param);
				}
				break;
		}

		// TODO ne pas attendre si evenement
		vTaskDelayUntil(&wake_time, TRAJECTORY_PERIOD);
	}
}

static void simplify_path()
{
	// elimination de points de passage
	int i = 0;
	for(i = -1; i < trajectory_graph_way_count - 1; i++)
	{
		Vect2 a_table;
		Vect2 b_table;

		// position du noeud avec alignement (marche avant) avec la destination
		VectPlan pos;
		if( i >= 0 )
		{
			int id = trajectory_graph_way[i];
			pos.x = graph_node[id].pos.x;
			pos.y = graph_node[id].pos.y;

		}
		else
		{
			// i == -1 : test elimination du premier point de passage
			pos.x = trajectory_pos.x;
			pos.y = trajectory_pos.y;
		}
		float dx = trajectory_dest.x - pos.x;
		float dy = trajectory_dest.y - pos.y;
		pos.theta = atan2f(dy, dx);

		float xmin = detection_compute_front_object(DETECTION_STATIC_OBJ, pos, &a_table, &b_table);
		float dist2 = dx * dx +  dy * dy;
		float xmin2 = xmin * xmin;
		if( dist2 < xmin2)
		{
			// possibilite d'aller directement a la destination depuis ce point
			//log_format(LOG_INFO, "shortcut to dest from %d", i);
			trajectory_graph_way_count = i+1;
			break;
		}

		int j;
		for(j = trajectory_graph_way_count - 1; j > i+1 ; j--)
		{
			// position du noeud avec alignement (marche avant) avec la destination
			int id2 = trajectory_graph_way[j];
			dx = graph_node[id2].pos.x - pos.x;
			dy = graph_node[id2].pos.y - pos.y;
			pos.theta = atan2f(dy, dx);

			xmin = detection_compute_front_object(DETECTION_STATIC_OBJ, pos, &a_table, &b_table);
			dist2 = dx * dx +  dy * dy;
			xmin2 = xmin * xmin;
			if( dist2 < xmin2)
			{
				// possibilite de prendre un racourci de i a j
				//log_format(LOG_INFO, "shortcut from %d to %d", i, j);
				// on supprime les points intermediaires entre i et j
				int k = i;
				int skip = (j - i - 1);
				for(k = i + 1; k < trajectory_graph_way_count - 1 - skip; k++)
				{
					trajectory_graph_way[k] = trajectory_graph_way[k+skip];
				}
				trajectory_graph_way_count -= skip;
				break;
			}
		}
	}
}

//! calcul des parametres de la trajectoire
//! planification pour eviter les obstacles statiques en passant par le graph
//! on ne prend pas en compte l'adversaire dans cette premiere planification (il aura peut être bouge)
static void trajectory_update()
{
	struct trajectory_cmd_arg req;
	int id;

	// mutex pour trajectory_request
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	req = trajectory_request;
	trajectory_new_request = false;
	xSemaphoreGive(trajectory_mutex);

	trajectory_state = TRAJECTORY_STATE_NONE;
	trajectory_dest = trajectory_pos;
	trajectory_approx_dist = 0;
	trajectory_way = WAY_ANY;
	trajectory_avoidance_type = (enum avoidance_type)req.avoidance_type;
	trajectory_type = (enum trajectory_cmd_type)req.type;

	switch(trajectory_type)
	{
		case TRAJECTORY_STRAIGHT:
			log_format(LOG_INFO, "straight %d", (int)req.dist);
			trajectory_dest.x += cosf(trajectory_dest.theta) * req.dist;
			trajectory_dest.y += sinf(trajectory_dest.theta) * req.dist;
			if(req.dist > 0)
			{
				trajectory_way = WAY_FORWARD;
			}
			else
			{
				trajectory_way = WAY_BACKWARD;
			}
			break;
		case TRAJECTORY_ROTATE:
			log_format(LOG_INFO, "rotate %d", (int)(req.dest.theta * 180 / M_PI));
			trajectory_dest.theta += req.dest.theta;
			break;
		case TRAJECTORY_ROTATE_TO:
			log_format(LOG_INFO, "rotate_to %d", (int)(req.dest.theta * 180 / M_PI));
			trajectory_dest.theta += motion_find_rotate(trajectory_dest.theta, req.dest.theta);
			break;
		case TRAJECTORY_GOTO_XY:
			trajectory_dest.x = req.dest.x;
			trajectory_dest.y = req.dest.y;
			trajectory_dest.theta = atan2f(trajectory_dest.y - trajectory_pos.y, trajectory_dest.x - trajectory_pos.x);
			trajectory_approx_dist = req.dist;
			trajectory_way = (enum motion_way)req.way;
			break;
		case TRAJECTORY_GOTO_XYA:
			trajectory_dest = req.dest;
			trajectory_approx_dist = req.dist;
			trajectory_way = (enum motion_way)req.way;
			break;
		case TRAJECTORY_GOTO_GRAPH:
			id = trajectory_find_way_to_graph(trajectory_pos, DETECTION_STATIC_OBJ);
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

	trajectory_state = TRAJECTORY_STATE_MOVE_TO_DEST;

	// verification : trajectoire possible ?
	if(trajectory_static_check_enable && (req.type == TRAJECTORY_GOTO_XY || req.type == TRAJECTORY_GOTO_XYA))
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

		float xmin = detection_compute_front_object(DETECTION_STATIC_OBJ, pos, &a_table, &b_table);
		float dist2 = dx * dx +  dy * dy;
		float xmin2 = xmin * xmin;
		if( xmin2 < dist2)
		{
			// trajectoire impossible, on passe par le graph
			log(LOG_INFO, "goto - obstacle statique, utilisation du graph");
			trajectory_state = TRAJECTORY_STATE_MOVE_TO_GRAPH;
		}
	}

	if( trajectory_state == TRAJECTORY_STATE_MOVE_TO_GRAPH )
	{
		trajectory_graph_way_count = 1;

		trajectory_graph_way[0] = trajectory_find_way_to_graph(trajectory_pos, DETECTION_STATIC_OBJ);
		int last_graph_id = trajectory_find_way_to_graph(trajectory_dest, DETECTION_STATIC_OBJ);
		if(trajectory_graph_way[0] != last_graph_id)
		{
			// calcul du trajet dans le graph
			// on active tout les chemins
			for(int i = 0; i < GRAPH_NUM_LINK; i++)
			{
				trajectory_graph_valid_links[i] = 1;
			}
			//log_format(LOG_INFO, "graph_dijkstra de %d à %d", trajectory_graph_way[0], trajectory_last_graph_id);
			int res = graph_dijkstra(trajectory_graph_way[0], last_graph_id, trajectory_dijkstra_info, trajectory_graph_valid_links);
			if(res)
			{
				log_format(LOG_INFO, "aucun chemin trouvé de %d à %d", trajectory_graph_way[0], last_graph_id);
				trajectory_state = TRAJECTORY_STATE_TARGET_NOT_REACHED;
			}
			else
			{
				int i = last_graph_id;
				while(trajectory_dijkstra_info[i].prev_node != trajectory_graph_way[0])
				{
					i = trajectory_dijkstra_info[i].prev_node;
					trajectory_graph_way_count++;
				}
				trajectory_graph_way_count++;

				// on refait un parcourt pour mettre dans les points dans l'ordre
				i = last_graph_id;
				int j = trajectory_graph_way_count - 1;
				trajectory_graph_way[j] = i;
				while(trajectory_dijkstra_info[i].prev_node != trajectory_graph_way[0])
				{
					i = trajectory_dijkstra_info[i].prev_node;
					j--;
					trajectory_graph_way[j] = i;
				}
				// affichage debug
				/*for(i=0; i < trajectory_graph_way_count; i++)
				{
					log_format(LOG_INFO, "chemin - graph : %d : %d", i, trajectory_graph_way[i]);
				}*/
			}
		}

		simplify_path();

		log_format(LOG_INFO, "passage par %d points", trajectory_graph_way_count);
		log_format(LOG_INFO, "point entrée graph : %d", trajectory_graph_way[0]);
		int i;
		for( i = 1 ; i < trajectory_graph_way_count - 1; i++)
		{
			log_format(LOG_INFO, "point passage graph : %d", trajectory_graph_way[i]);
		}
		log_format(LOG_INFO, "point sortie graph : %d", trajectory_graph_way[trajectory_graph_way_count-1]);
		trajectory_graph_way_id = 0;
	}
}

static int trajectory_find_way_to_graph(VectPlan pos, enum detection_type detect_type)
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

	if(!trajectory_static_check_enable && !trajectory_hokuyo_enable_check)
	{
		// on y va direct, pas de gestion d'obstacles
		return node_dist[0].id;
	}

	for( i = 0 ; i < GRAPH_NUM_NODE; i++)
	{
		id = node_dist[i].id;
		float dx = graph_node[id].pos.x - pos.x;
		float dy = graph_node[id].pos.y - pos.y;
		pos.theta = atan2f(dy, dx);
		xmin = detection_compute_front_object(detect_type, pos, &a_table, &b_table);
		// TODO prendre en compte la rotation sur place en plus de la ligne droite
		// 10mm de marge / control
		dist = node_dist[i].dist;
		if( dist < xmin - PARAM_LEFT_CORNER_X - TRAJECTORY_APPROX_DIST - 10)
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

static void trajectory_detection_callback()
{
	// TODO
}

static void trajectory_update_request()
{
	if( trajectory_new_request )
	{
		log(LOG_ERROR, "trajectory - overiding trajectory");
	}
	trajectory_new_request = true;
	trajectory_state = TRAJECTORY_STATE_UPDATING_TRAJECTORY;
}

void trajectory_set_kinematics_param(KinematicsParameters linParam, KinematicsParameters angParam)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_linear_param = linParam;
	trajectory_angular_param = angParam;
	xSemaphoreGive(trajectory_mutex);
}

void trajectory_cmd(void* arg)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	memcpy(&trajectory_request, arg, sizeof(struct trajectory_cmd_arg));
	trajectory_update_request();
	xSemaphoreGive(trajectory_mutex);
}

void trajectory_free()
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_request.type = TRAJECTORY_FREE;
	trajectory_request.avoidance_type = AVOIDANCE_STOP;
	trajectory_update_request();
	xSemaphoreGive(trajectory_mutex);
}

void trajectory_rotate(float theta)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_request.type = TRAJECTORY_ROTATE;
	trajectory_request.avoidance_type = AVOIDANCE_STOP;
	trajectory_request.dest.theta = theta;
	trajectory_update_request();
	xSemaphoreGive(trajectory_mutex);
}

void trajectory_rotate_to(float theta)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_request.type = TRAJECTORY_ROTATE_TO;
	trajectory_request.avoidance_type = AVOIDANCE_STOP;
	trajectory_request.dest.theta = theta;
	trajectory_update_request();
	xSemaphoreGive(trajectory_mutex);
}

void trajectory_straight(float dist)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_request.type = TRAJECTORY_STRAIGHT;
	trajectory_request.avoidance_type = AVOIDANCE_STOP;
	trajectory_request.dist = dist;
	trajectory_update_request();
	xSemaphoreGive(trajectory_mutex);
}

void trajectory_goto_near_xy(float x, float y, float dist, enum motion_way way, enum avoidance_type avoidance_type)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_request.type = TRAJECTORY_GOTO_XY;
	trajectory_request.avoidance_type = avoidance_type;
	trajectory_request.dest.x = x;
	trajectory_request.dest.y = y;
	trajectory_request.dist = dist;
	trajectory_request.way = way;
	trajectory_update_request();
	xSemaphoreGive(trajectory_mutex);
}

void trajectory_goto_near(VectPlan dest, float dist, enum motion_way way, enum avoidance_type avoidance_type)
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_request.type = TRAJECTORY_GOTO_XYA;
	trajectory_request.avoidance_type = avoidance_type;
	trajectory_request.dest = dest;
	trajectory_request.dist = dist;
	trajectory_request.way = way;
	trajectory_update_request();
	xSemaphoreGive(trajectory_mutex);
}

void trajectory_goto(VectPlan dest, enum motion_way way, enum avoidance_type avoidance_type)
{
	trajectory_goto_near(dest, 0, way, avoidance_type);
}

void trajectory_goto_graph_node(uint32_t node_id, float dist, enum motion_way way, enum avoidance_type avoidance_type)
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
	trajectory_update_request();
	xSemaphoreGive(trajectory_mutex);
}

void trajectory_goto_graph()
{
	xSemaphoreTake(trajectory_mutex, portMAX_DELAY);
	trajectory_request.type = TRAJECTORY_GOTO_GRAPH;
	trajectory_request.avoidance_type = AVOIDANCE_STOP;
	trajectory_update_request();
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


////////////////////////////////////////////////
/// function    : trajectory_wait()
/// descrition  : Waiting function of trajectory move functions
/// param       : wanted_state = enum trajectory_state
/// param       : timeout = uint32_t time_out (<0 no time-out but buffer overflow!!!!)
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
// TODO faire mieux pour eviter le polling
int trajectory_wait(enum trajectory_state wanted_state, uint32_t timeout)
{
	enum trajectory_state state = trajectory_state;

	while(state != TRAJECTORY_STATE_COLISION && state != TRAJECTORY_STATE_TARGET_REACHED && state != TRAJECTORY_STATE_TARGET_NOT_REACHED && timeout)
	{
		vTaskDelay(1);
		timeout --;
		state = trajectory_state;
	}

	if( state != wanted_state )
	{
		log_format(LOG_ERROR, "incorrect state : %d", state);
		return -1;
	}

	return 0;
}
