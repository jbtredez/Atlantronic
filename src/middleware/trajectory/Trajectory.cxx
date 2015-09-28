//! @file Trajectory.h
//! @brief Generation de trajectoire
//! @author Atlantronic

#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "disco/robot_parameters.h"
#include "Trajectory.h"
#include <stdlib.h>

#define TRAJECTORY_STACK_SIZE       400
#define TRAJECTORY_APPROX_DIST      150      //!< distance d'approche d'un objet
#define TRAJECTORY_PERIOD            10

void trajectoryCmd(void* arg);
static void trajectory_detection_callback();

Trajectory trajectory;

static int trajectory_module_init()
{
	usb_add_cmd(USB_CMD_TRAJECTORY, &trajectoryCmd);

	detection_register_callback(trajectory_detection_callback);

	return trajectory.init();
}

module_init(trajectory_module_init, INIT_TRAJECTORY);

Trajectory::Trajectory()
{
	m_trajectoryState = TRAJECTORY_STATE_NONE;
	m_hokuyoEnableCheck = true;
	m_staticCheckEnable = true;

	m_linearParam.vMax = 700;
	m_linearParam.aMax = 600;
	m_linearParam.dMax = 600;
	m_angularParam.vMax = 3;
	m_angularParam.aMax = 5;
	m_angularParam.dMax = 5;
}

int Trajectory::init()
{
	xTaskHandle xHandle;

	m_mutex = xSemaphoreCreateMutex();

	if(m_mutex == NULL)
	{
		return ERR_INIT_TRAJECTORY;
	}

	portBASE_TYPE err = xTaskCreate(&Trajectory::trajectory_task, "traj", TRAJECTORY_STACK_SIZE, this, PRIORITY_TASK_TRAJECTORY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TRAJECTORY;
	}

	return 0;
}

void Trajectory::trajectory_task(void* arg)
{
	Trajectory* traj = (Trajectory*) arg;
	traj->trajectoryTask();
}

void Trajectory::trajectoryTask()
{
	uint32_t wake_time = 0;
	enum motion_state motion_state;
	enum motion_status motion_status;
	enum motion_trajectory_step motion_traj_step;
	enum motion_wanted_state motion_wanted_state;

	while(1)
	{
		m_pos = location_get_position();
		motion.getState(&motion_state, &motion_status, &motion_traj_step, &motion_wanted_state);

		if( m_newRequest )
		{
			update();
		}

		switch(m_trajectoryState)
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
						m_trajectoryState = TRAJECTORY_STATE_TARGET_REACHED;
						break;
					case MOTION_TARGET_NOT_REACHED:
						log(LOG_ERROR, "TRAJECTORY_TARGET_NOT_REACHED");
						m_trajectoryState = TRAJECTORY_STATE_TARGET_NOT_REACHED;
						break;
					case MOTION_COLSISION:
						switch(m_avoidanceType)
						{
							default:
							case AVOIDANCE_STOP:
								// pas d'évitement, fin de la trajectoire
								log(LOG_INFO, "collision -> stop");
								m_trajectoryState = TRAJECTORY_STATE_COLISION;
								break;
							case AVOIDANCE_GRAPH:
								log(LOG_INFO, "collision -> graph");
								computeGraph(DETECTION_FULL);
								m_trajectoryState = TRAJECTORY_STATE_MOVE_TO_GRAPH;
								break;
						}
						break;
					default:
					case MOTION_TIMEOUT:
						log(LOG_INFO, "timeout -> target not reached");
						m_trajectoryState = TRAJECTORY_STATE_TARGET_NOT_REACHED;
						break;
					case MOTION_IN_MOTION:
					case MOTION_UPDATING_TRAJECTORY:
						break;
				}
				break;
			case TRAJECTORY_STATE_MOVE_TO_DEST:
				if( motion_state == MOTION_ENABLED && motion_wanted_state == MOTION_WANTED_STATE_UNKNOWN )
				{
					VectPlan dest = m_dest;
					VectPlan u = m_dest - m_pos;
					float ds = u.norm();
					motion_trajectory_type traj_type = MOTION_AXIS_XYA;
					if( fabsf(ds) > EPSILON )
					{
						if( m_type == TRAJECTORY_GOTO_XY )
						{
							traj_type = MOTION_AXIS_XY;
						}
						dest.x -= m_approxDist * u.x / ds;
						dest.y -= m_approxDist * u.y / ds;
					}
					else if( m_type == TRAJECTORY_ROTATE )
					{
						traj_type = MOTION_AXIS_A;
					}

					motion.goTo(dest, VectPlan(), m_way, traj_type, m_linearParam, m_angularParam);
					m_trajectoryState = TRAJECTORY_STATE_MOVING_TO_DEST;
				}
				break;
			case TRAJECTORY_STATE_USING_GRAPH:
				if( motion_state == MOTION_ENABLED && motion_wanted_state == MOTION_WANTED_STATE_UNKNOWN )
				{
					if( m_graphWayId < m_graph.m_wayCount - 1 )
					{
						m_graphWayId++;
						int i = m_graph.m_way[m_graphWayId];
						log_format(LOG_INFO, "goto graph node %d", i);
						VectPlan dest(m_graph.getNode(i), 0);
						motion.goTo(dest, VectPlan(), WAY_FORWARD, MOTION_AXIS_XY, m_linearParam, m_angularParam);
					}
					else
					{
						motion_trajectory_type traj_type = MOTION_AXIS_XYA;
						if( m_type == TRAJECTORY_GOTO_XY )
						{
							traj_type = MOTION_AXIS_XY;
						}
						motion.goTo(m_dest, VectPlan(), m_way, traj_type, m_linearParam, m_angularParam);
						m_trajectoryState = TRAJECTORY_STATE_MOVING_TO_DEST;
					}
				}
				break;
			case TRAJECTORY_STATE_MOVE_TO_GRAPH:
				if( motion_state == MOTION_ENABLED && motion_wanted_state == MOTION_WANTED_STATE_UNKNOWN)
				{
					m_trajectoryState = TRAJECTORY_STATE_USING_GRAPH;
					int i = m_graph.m_way[0];
					VectPlan dest(m_graph.getNode(i), 0);
					log_format(LOG_INFO, "goto graph node %d : %d %d", i, (int)dest.x, (int)dest.y);
					motion.goTo(dest, VectPlan(), WAY_FORWARD, MOTION_AXIS_XY, m_linearParam, m_angularParam);
				}
				break;
		}

		// TODO ne pas attendre si evenement
		vTaskDelayUntil(&wake_time, TRAJECTORY_PERIOD);
	}
}

void Trajectory::simplifyPath(enum detection_type type)
{
	// elimination de points de passage
	int i = 0;
	for(i = -1; i < m_graph.m_wayCount - 1; i++)
	{
		Vect2 a_table;
		Vect2 b_table;

		// position du noeud avec alignement (marche avant) avec la destination
		VectPlan pos;
		if( i >= 0 )
		{
			int id = m_graph.m_way[i];
			pos = VectPlan(m_graph.getNode(id), 0);
		}
		else
		{
			// i == -1 : test elimination du premier point de passage
			pos.x = m_pos.x;
			pos.y = m_pos.y;
		}
		float dx = m_dest.x - pos.x;
		float dy = m_dest.y - pos.y;
		pos.theta = atan2f(dy, dx);

		float xmin = detection_compute_front_object(type, pos, &a_table, &b_table);
		float dist2 = dx * dx +  dy * dy;
		float xmin2 = xmin * xmin;
		if( dist2 < xmin2)
		{
			// possibilite d'aller directement a la destination depuis ce point
			//log_format(LOG_INFO, "shortcut to dest from %d", i);
			m_graph.m_wayCount = i+1;
			break;
		}

		int j;
		for(j = m_graph.m_wayCount - 1; j > i+1 ; j--)
		{
			// position du noeud avec alignement (marche avant) avec la destination
			int id2 = m_graph.m_way[j];
			Vect2 p = m_graph.getNode(id2);
			dx = p.x - pos.x;
			dy = p.y - pos.y;
			pos.theta = atan2f(dy, dx);

			xmin = detection_compute_front_object(type, pos, &a_table, &b_table);
			dist2 = dx * dx +  dy * dy;
			xmin2 = xmin * xmin;
			if( dist2 < xmin2)
			{
				// possibilite de prendre un racourci de i a j
				//log_format(LOG_INFO, "shortcut from %d to %d nb %d", i, j, trajectory_graph_way_count);
				// on supprime les points intermediaires entre i et j
				int k = i;
				int skip = (j - i - 1);
				for(k = i + 1; k < m_graph.m_wayCount - skip; k++)
				{
					//log_format(LOG_INFO, "skip %d", trajectory_graph_way[k]);
					m_graph.m_way[k] = m_graph.m_way[k+skip];
				}
				m_graph.m_wayCount -= skip;
				break;
			}
		}
	}
}

void Trajectory::computeGraph(enum detection_type type)
{
	int startId = findWayToGraph(m_pos, type);
	int endId = findWayToGraph(m_dest, type);
	if(startId != endId)
	{
		// calcul du trajet dans le graph

		// on active tout les chemins ou passe le robot
		for(int i = 0; i < GRAPH_NUM_LINK; i++)
		{
			// position du noeud avec alignement (marche avant) avec la destination
			VectPlan pos;
			GraphLink link = m_graph.getLink(i);
			pos = VectPlan(m_graph.getNode(link.a), link.alpha);
			float xmin = detection_compute_front_object(type, pos, NULL, NULL);
			m_graph.setValidLink(i, link.dist < xmin);
			//log_format(LOG_INFO, "lien %d : %d (%d -> %d)", i, trajectory_graph_valid_links[i], (int)graph_link[i].a, (int)graph_link[i].b);
		}
		//log_format(LOG_INFO, "graph_dijkstra de %d à %d", trajectory_graph_way[0], trajectory_last_graph_id);
		int res = m_graph.dijkstra(startId, endId);
		if(res)
		{
			log_format(LOG_INFO, "aucun chemin trouvé de %d à %d", startId, endId);
			m_trajectoryState = TRAJECTORY_STATE_TARGET_NOT_REACHED;
		}
	}

	simplifyPath(type);

	log_format(LOG_INFO, "passage par %d points", m_graph.m_wayCount);
	log_format(LOG_INFO, "point entrée graph : %d", startId);
	int i;
	for( i = 1 ; i < m_graph.m_wayCount - 1; i++)
	{
		log_format(LOG_INFO, "point passage graph : %d", m_graph.m_way[i]);
	}
	log_format(LOG_INFO, "point sortie graph : %d", endId);
	m_graphWayId = 0;
}

//! calcul des parametres de la trajectoire
//! planification pour eviter les obstacles statiques en passant par le graph
//! on ne prend pas en compte l'adversaire dans cette premiere planification (il aura peut être bouge)
void Trajectory::update()
{
	struct trajectory_cmd_arg req;
	int id;

	// mutex pour trajectory_request
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	req = m_request;
	m_newRequest = false;
	xSemaphoreGive(m_mutex);

	m_trajectoryState = TRAJECTORY_STATE_NONE;
	m_dest = m_pos;
	m_approxDist = 0;
	m_way = WAY_ANY;
	m_avoidanceType = (enum avoidance_type)req.avoidance_type;
	m_type = (enum trajectory_cmd_type)req.type;

	switch(m_type)
	{
		case TRAJECTORY_STRAIGHT:
			log_format(LOG_INFO, "straight %d", (int)req.dist);
			m_dest.x += cosf(m_dest.theta) * req.dist;
			m_dest.y += sinf(m_dest.theta) * req.dist;
			if(req.dist > 0)
			{
				m_way = WAY_FORWARD;
			}
			else
			{
				m_way = WAY_BACKWARD;
			}
			break;
		case TRAJECTORY_ROTATE:
			log_format(LOG_INFO, "rotate %d", (int)(req.dest.theta * 180 / M_PI));
			m_dest.theta += req.dest.theta;
			break;
		case TRAJECTORY_ROTATE_TO:
			log_format(LOG_INFO, "rotate_to %d", (int)(req.dest.theta * 180 / M_PI));
			m_dest.theta += motion.findRotate(m_dest.theta, req.dest.theta);
			break;
		case TRAJECTORY_GOTO_XY:
			m_dest.x = req.dest.x;
			m_dest.y = req.dest.y;
			m_dest.theta = atan2f(m_dest.y - m_pos.y, m_dest.x - m_pos.x);
			m_approxDist = req.dist;
			m_way = (enum motion_way)req.way;
			break;
		case TRAJECTORY_GOTO_XYA:
			m_dest = req.dest;
			m_approxDist = req.dist;
			m_way = (enum motion_way)req.way;
			break;
		case TRAJECTORY_GOTO_GRAPH:
			id = findWayToGraph(m_pos, DETECTION_STATIC_OBJ);
			log_format(LOG_INFO, "point graph : %d", id);

			m_dest = VectPlan(m_graph.getNode(id), 0);
			break;
		case TRAJECTORY_FREE:
		default:
			m_type = TRAJECTORY_FREE;
			motion.enable(false);
			return;
	}

	m_trajectoryState = TRAJECTORY_STATE_MOVE_TO_DEST;

	// verification : trajectoire possible ?
	if(m_staticCheckEnable && (req.type == TRAJECTORY_GOTO_XY || req.type == TRAJECTORY_GOTO_XYA))
	{
		// TODO : check fait pour la marche avant, tolérance plus grande si on le fait en marche arrière ?
		// on regarde si c'est possible / objets statiques de la table
		// si c'est pas possible, on passe par le graph
		Vect2 a_table;
		Vect2 b_table;

		// position actuelle avec alignement (marche avant) avec la destination
		VectPlan pos = m_pos;
		float dx = m_dest.x - m_pos.x;
		float dy = m_dest.y - m_pos.y;
		pos.theta = atan2f(dy, dx);

		float xmin = detection_compute_front_object(DETECTION_STATIC_OBJ, pos, &a_table, &b_table);
		float dist2 = dx * dx +  dy * dy;
		float xmin2 = xmin * xmin;
		if( xmin2 < dist2)
		{
			// trajectoire impossible, on passe par le graph
			log(LOG_INFO, "goto - obstacle statique, utilisation du graph");
			m_trajectoryState = TRAJECTORY_STATE_MOVE_TO_GRAPH;
		}
	}

	if( m_trajectoryState == TRAJECTORY_STATE_MOVE_TO_GRAPH )
	{
		computeGraph(DETECTION_STATIC_OBJ);
	}
}

int Trajectory::findWayToGraph(VectPlan pos, enum detection_type detect_type)
{
	// passe en stack, pas trop de noeuds
	struct GraphNodeDist node_dist[GRAPH_NUM_NODE];
	struct Vect2 p(pos.x, pos.y);

	m_graph.computeNodeDistance(p, node_dist);

	Vect2 a_table;
	Vect2 b_table;
	float xmin;
	int i;
	int id = 0;
	float dist;

	if(!m_staticCheckEnable && !m_hokuyoEnableCheck)
	{
		// on y va direct, pas de gestion d'obstacles
		return node_dist[0].id;
	}

	for( i = 0 ; i < GRAPH_NUM_NODE; i++)
	{
		id = node_dist[i].id;
		Vect2 p = m_graph.getNode(id);
		float dx = p.x - pos.x;
		float dy = p.y - pos.y;
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

void Trajectory::updateRequest()
{
	if( m_newRequest )
	{
		log(LOG_ERROR, "trajectory - overiding trajectory");
	}
	m_newRequest = true;
	m_trajectoryState = TRAJECTORY_STATE_UPDATING_TRAJECTORY;
}

void Trajectory::setKinematicsParam(KinematicsParameters linParam, KinematicsParameters angParam)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_linearParam = linParam;
	m_angularParam = angParam;
	xSemaphoreGive(m_mutex);
}

void Trajectory::getKinematicsParam(KinematicsParameters* linParam, KinematicsParameters* angParam)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	*linParam = m_linearParam;
	*angParam = m_angularParam;
	xSemaphoreGive(m_mutex);
}

void trajectoryCmd(void* arg)
{
	xSemaphoreTake(trajectory.m_mutex, portMAX_DELAY);
	memcpy(&trajectory.m_request, arg, sizeof(struct trajectory_cmd_arg));
	trajectory.updateRequest();
	xSemaphoreGive(trajectory.m_mutex);
}

void Trajectory::freeWheel()
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_request.type = TRAJECTORY_FREE;
	m_request.avoidance_type = AVOIDANCE_STOP;
	updateRequest();
	xSemaphoreGive(m_mutex);
}

void Trajectory::rotate(float theta)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_request.type = TRAJECTORY_ROTATE;
	m_request.avoidance_type = AVOIDANCE_STOP;
	m_request.dest.theta = theta;
	updateRequest();
	xSemaphoreGive(m_mutex);
}

void Trajectory::rotateTo(float theta)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_request.type = TRAJECTORY_ROTATE_TO;
	m_request.avoidance_type = AVOIDANCE_STOP;
	m_request.dest.theta = theta;
	updateRequest();
	xSemaphoreGive(m_mutex);
}

void Trajectory::straight(float dist)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_request.type = TRAJECTORY_STRAIGHT;
	m_request.avoidance_type = AVOIDANCE_STOP;
	m_request.dist = dist;
	updateRequest();
	xSemaphoreGive(m_mutex);
}

void Trajectory::goToNearXy(float x, float y, float dist, enum motion_way way, enum avoidance_type avoidance_type)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_request.type = TRAJECTORY_GOTO_XY;
	m_request.avoidance_type = avoidance_type;
	m_request.dest.x = x;
	m_request.dest.y = y;
	m_request.dist = dist;
	m_request.way = way;
	updateRequest();
	xSemaphoreGive(m_mutex);
}

void Trajectory::goToNear(VectPlan dest, float dist, enum motion_way way, enum avoidance_type avoidance_type)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_request.type = TRAJECTORY_GOTO_XYA;
	m_request.avoidance_type = avoidance_type;
	m_request.dest = dest;
	m_request.dist = dist;
	m_request.way = way;
	updateRequest();
	xSemaphoreGive(m_mutex);
}

void Trajectory::goTo(VectPlan dest, enum motion_way way, enum avoidance_type avoidance_type)
{
	goToNear(dest, 0, way, avoidance_type);
}

void Trajectory::goToGraphNode(uint32_t node_id, float dist, enum motion_way way, enum avoidance_type avoidance_type)
{
	if( node_id >= GRAPH_NUM_NODE)
	{
		log_format(LOG_ERROR, "node_id inconnu : %d", (int)node_id);
		return;
	}

	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_request.type = TRAJECTORY_GOTO_XY;
	m_request.avoidance_type = avoidance_type;
	m_request.dest = VectPlan(m_graph.getNode(node_id), 0);
	m_request.dist = dist;
	m_request.way = way;
	updateRequest();
	xSemaphoreGive(m_mutex);
}

void Trajectory::goToGraph()
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_request.type = TRAJECTORY_GOTO_GRAPH;
	m_request.avoidance_type = AVOIDANCE_STOP;
	updateRequest();
	xSemaphoreGive(m_mutex);
}

////////////////////////////////////////////////
/// function    : trajectory_wait()
/// descrition  : Waiting function of trajectory move functions
/// param       : wanted_state = enum trajectory_state
/// param       : timeout = uint32_t time_out (<0 no time-out but buffer overflow!!!!)
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
// TODO faire mieux pour eviter le polling
int Trajectory::wait(enum trajectory_state wanted_state, uint32_t timeout)
{
	enum trajectory_state state = m_trajectoryState;

	while(state != TRAJECTORY_STATE_COLISION && state != TRAJECTORY_STATE_TARGET_REACHED && state != TRAJECTORY_STATE_TARGET_NOT_REACHED && timeout)
	{
		vTaskDelay(1);
		timeout --;
		state = m_trajectoryState;
	}

	if( state != wanted_state )
	{
		log_format(LOG_ERROR, "incorrect state : %d", state);
		return -1;
	}

	return 0;
}
