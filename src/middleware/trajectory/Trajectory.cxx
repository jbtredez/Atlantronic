//! @file Trajectory.h
//! @brief Generation de trajectoire
//! @author Atlantronic

#include <stdlib.h>
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/math/findRotation.h"
#include "Trajectory.h"
#include "kernel/math/poly7.h"
#include "disco/bot.h"

#define TRAJECTORY_STACK_SIZE       400
#define TRAJECTORY_APPROX_DIST      150      //!< distance d'approche d'un objet
#define TRAJECTORY_PERIOD            10

int Trajectory::init(Detection* detection, Motion* motion, Location* location, KinematicsParameters linearParam, KinematicsParameters angularParam)
{
	xTaskHandle xHandle;

	usb_add_cmd(USB_CMD_TRAJECTORY, &trajectoryCmd, this);

	m_location = location;
	m_detection = detection;
	m_detection->registerCallback(detectionCallback, this);
	m_motion = motion;

	m_trajectoryState = TRAJECTORY_STATE_NONE;
	m_hokuyoEnableCheck = true;
	m_staticCheckEnable = true;

	m_linearParam = linearParam;
	m_angularParam = angularParam;
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
	enum motion_state motion_state, motion_wanted_state;
	enum motion_status motion_status;

	while(1)
	{
		m_pos = m_location->getPosition();

		if( m_newRequest )
		{
			update();
		}

		m_motion->getState(&motion_state, &motion_status, &motion_wanted_state);

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
		}

		// TODO ne pas attendre si evenement
		vTaskDelayUntil(&wake_time, TRAJECTORY_PERIOD);
	}
}
void Trajectory::motionAddGoTo(bool newTrajectory, VectPlan dest, VectPlan cp, TrajectoryWay way, TrajectoryType type)
{
	switch(type)
	{
		case TRAJECTORY_AXIS_XYA:
		case TRAJECTORY_AXIS_A:
		case TRAJECTORY_AXIS_XY:
			motionAddGoToStraightRotate(newTrajectory, dest, cp, way, type);
			break;
		case TRAJECTORY_CURVILINEAR_XY:
		case TRAJECTORY_CURVILINEAR_XYA:
			// TODO : attention, ne marche pas pour les rotation pure. Le detecter et faire motionAddGoToStraightRotate à la place
			motionAddGoToCurvilinear(newTrajectory, dest, cp, way, type);
			break;
		default:
			log(LOG_ERROR, "uniknown trajectory type");
			break;
	}
}

void Trajectory::motionAddGoToCurvilinear(bool newTrajectory, VectPlan dest, VectPlan cp, TrajectoryWay way, TrajectoryType type)
{
	VectPlan wantedDest = loc_to_abs(dest, -cp);

	PathPoint pt;
	VectPlan start;

	if( newTrajectory )
	{
		m_motion->clearTrajectory();
		start = m_pos;
		pt.pos = start;
		m_motion->addTrajectoryPoints(&pt, 1);
	}
	else
	{
		start = m_motion->getLastPathPoint();
		pt.pos = start;
	}

	float a1[8];
	float b1[8];
	float a2[8];
	float b2[8];
	VectPlan delta = wantedDest - start;
	float n = delta.norm();
	if( type != TRAJECTORY_CURVILINEAR_XYA )
	{
		// TODO pas super mais si on met n2 a 0, on a un probleme de continuite sur sigma
		wantedDest.theta = atan2f(delta.y, delta.x);
	}

	if( way == WAY_ANY )
	{
		// test en marche avant
		computePoly7Traj(start, wantedDest, a1, b1, n, n);

		// test en marche arriere
		start.theta += M_PI;
		wantedDest.theta += M_PI;
		computePoly7Traj(start, wantedDest, a2, b2, n, n);
		wantedDest.theta -= M_PI;
		start.theta -= M_PI;

		float ds1 = 0;
		float ds2 = 0;
		PathPoint pt1 = pt;
		PathPoint pt2 = pt;
		for(int i = 1; i < 100; i++)
		{
			float t = i / 100.0f;
			float t2 = t*t;
			float t3 = t2 * t;
			float t4 = t3 * t;
			float t5 = t4 * t;
			float t6 = t5 * t;
			float t7 = t6 * t;
			float x = a1[0] + a1[1] * t + a1[2] * t2 + a1[3] * t3 + a1[4] * t4 + a1[5] * t5 + a1[6] * t6 + a1[7] * t7;
			float y = b1[0] + b1[1] * t + b1[2] * t2 + b1[3] * t3 + b1[4] * t4 + b1[5] * t5 + b1[6] * t6 + b1[7] * t7;
			ds1 += (Vect2(pt1.pos) - Vect2(x,y)).norm2();
			pt1.pos.x = x;
			pt1.pos.y = y;
			x = a2[0] + a2[1] * t + a2[2] * t2 + a2[3] * t3 + a2[4] * t4 + a2[5] * t5 + a2[6] * t6 + a2[7] * t7;
			y = b2[0] + b2[1] * t + b2[2] * t2 + b2[3] * t3 + b2[4] * t4 + b2[5] * t5 + b2[6] * t6 + b2[7] * t7;
			ds2 += (Vect2(pt2.pos) - Vect2(x,y)).norm2();
			pt2.pos.x = x;
			pt2.pos.y = y;
		}

		if( ds1 < ds2 )
		{
			way = WAY_FORWARD;
		}
		else
		{
			way = WAY_BACKWARD;
		}
	}
	else if( way == WAY_FORWARD )
	{
		computePoly7Traj(start, wantedDest, a1, b1, n, n);
	}
	else
	{
		// test en marche arriere
		start.theta += M_PI;
		wantedDest.theta += M_PI;
		computePoly7Traj(start, wantedDest, a2, b2, n, n);
		wantedDest.theta -= M_PI;
		start.theta -= M_PI;
	}

	float a[8];
	float b[8];
	if( way == WAY_FORWARD )
	{
		for(int i = 0; i < 8; i++)
		{
			a[i] = a1[i];
			b[i] = b1[i];
		}
	}
	else
	{
		for(int i = 0; i < 8; i++)
		{
			a[i] = a2[i];
			b[i] = b2[i];
		}
	}

	for(int i = 1; i < 99; i++)
	{
		float t = i / 100.0f;
		float t2 = t*t;
		float t3 = t2 * t;
		float t4 = t3 * t;
		float t5 = t4 * t;
		float t6 = t5 * t;
		float t7 = t6 * t;
		float x = a[0] + a[1] * t + a[2] * t2 + a[3] * t3 + a[4] * t4 + a[5] * t5 + a[6] * t6 + a[7] * t7;
		float y = b[0] + b[1] * t + b[2] * t2 + b[3] * t3 + b[4] * t4 + b[5] * t5 + b[6] * t6 + b[7] * t7;
		float dx = a[1] + 2 * a[2] * t + 3 * a[3] * t2 + 4 * a[4] * t3 + 5 * a[5] * t4 + 6 * a[6] * t5 + 7 * a[7] * t6;
		float dy = b[1] + 2 * b[2] * t + 3 * b[3] * t2 + 4 * b[4] * t3 + 5 * b[5] * t4 + 6 * b[6] * t5 + 7 * b[7] * t6;
		pt.pos.theta = atan2f(dy, dx);
		if( way == WAY_BACKWARD )
		{
			pt.pos.theta += M_PI;
		}
		pt.pos.x = x;
		pt.pos.y = y;
		m_motion->addTrajectoryPoints(&pt, 1);
	}

	if( type == TRAJECTORY_CURVILINEAR_XY )
	{
		wantedDest.theta = pt.pos.theta;
	}
	pt.pos = wantedDest;
	m_motion->addTrajectoryPoints(&pt, 1);
}

void Trajectory::motionAddGoToStraightRotate(bool newTrajectory, VectPlan dest, VectPlan cp, TrajectoryWay way, TrajectoryType type)
{
	VectPlan wantedDest = loc_to_abs(dest, -cp);
	float dtheta1 = 0;
	float ds = 0;
	float dtheta2 = 0;
	PathPoint pts[4];
	int nbPts = 0;
	VectPlan start;

	if( newTrajectory )
	{
		m_motion->clearTrajectory();
		start = m_pos;
		pts[0].pos = start;
		nbPts++;
	}
	else
	{
		start = m_motion->getLastPathPoint();
	}

	if( type == TRAJECTORY_AXIS_A)
	{
		wantedDest.x = start.x;
		wantedDest.y = start.y;
	}

	VectPlan ab = wantedDest - start;
	float nab = ab.norm();
	// distance minimale de translation TODO 5mm en dur
	if( nab > 5 )
	{
		float theta1 = atan2f(ab.y, ab.x);
		ds = nab;

		if(way == WAY_FORWARD)
		{
			dtheta1 = findRotation(start.theta, theta1);
			if( type == TRAJECTORY_AXIS_XYA )
			{
				dtheta2 = findRotation(start.theta + dtheta1, wantedDest.theta);
			}
		}
		else if( way == WAY_BACKWARD)
		{
			dtheta1 = findRotation(start.theta, theta1 + M_PI);
			if( type == TRAJECTORY_AXIS_XYA )
			{
				dtheta2 = findRotation(start.theta + dtheta1, wantedDest.theta);
			}
		}
		else
		{
			float dtheta1_forward = findRotation(start.theta, theta1);
			float dtheta1_backward = findRotation(start.theta, theta1 + M_PI);
			float dtheta2_forward = 0;
			float dtheta2_backward = 0;

			if( type == TRAJECTORY_AXIS_XYA )
			{
				dtheta2_forward = findRotation(start.theta + dtheta1_forward, wantedDest.theta);
				dtheta2_backward = findRotation(start.theta + dtheta1_backward, wantedDest.theta);
			}

			if ( fabsf(dtheta1_forward) + fabsf(dtheta2_forward) > fabsf(dtheta1_backward) + fabsf(dtheta2_backward))
			{
				dtheta1 = dtheta1_backward;
				dtheta2 = dtheta2_backward;
			}
			else
			{
				dtheta1 = dtheta1_forward;
				dtheta2 = dtheta2_forward;
			}
		}
	}
	else if( type != TRAJECTORY_AXIS_XY)
	{
		if( type == TRAJECTORY_AXIS_A )
		{
			// rotation demandee explicitement. Pas d'optimisation de la rotation a faire.
			// utile pour calibration odometrie principalement
			dtheta1 = wantedDest.theta - start.theta;
		}
		else
		{
			// optimisation de l'angle de rotation a faire
			dtheta1 = findRotation(start.theta, wantedDest.theta);
		}
	}

	if( fabsf(dtheta1) > EPSILON )
	{
		pts[nbPts].pos.x = start.x;
		pts[nbPts].pos.y = start.y;
		pts[nbPts].pos.theta = start.theta + dtheta1;
		nbPts++;
	}

	if( ds > EPSILON )
	{
		pts[nbPts].pos.x = wantedDest.x;
		pts[nbPts].pos.y = wantedDest.y;
		pts[nbPts].pos.theta = start.theta + dtheta1;
		nbPts++;

		if( fabsf(dtheta2) > EPSILON && type == TRAJECTORY_AXIS_XYA)
		{
			pts[nbPts].pos.x = wantedDest.x;
			pts[nbPts].pos.y = wantedDest.y;
			pts[nbPts].pos.theta = start.theta + dtheta1 + dtheta2;
			nbPts++;
		}
	}
#if 0
	log_format(LOG_INFO, "goto %d %d %d : rotate %d translate %d, rotate %d",
			(int)dest.x, (int)dest.y, (int)(dest.theta*180/M_PI),
			(int)(dtheta1 * 180 / M_PI), (int)ds, (int)(dtheta2 * 180 / M_PI));
#endif
	m_motion->addTrajectoryPoints(pts, nbPts);
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

		float xmin = m_detection->computeFrontObject(type, pos, &a_table, &b_table);
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

			xmin = m_detection->computeFrontObject(type, pos, &a_table, &b_table);
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
			float xmin = m_detection->computeFrontObject(type, pos, NULL, NULL);
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

	bool newTraj = true;
	if( m_graph.m_wayCount > 0)
	{
		log_format(LOG_INFO, "passage par %d points", m_graph.m_wayCount);
		for(int i = 0 ; i < m_graph.m_wayCount; i++)
		{
			log_format(LOG_INFO, "point passage graph : %d", m_graph.m_way[i]);
			VectPlan dest = VectPlan(m_graph.getNode(m_graph.m_way[i]), 0);
			motionAddGoTo(newTraj, dest, VectPlan(), WAY_FORWARD, TRAJECTORY_AXIS_XY);
			newTraj = false;
		}
	}

	TrajectoryType traj_type = TRAJECTORY_AXIS_XYA;
	if( m_type == TRAJECTORY_GOTO_XY )
	{
		traj_type = TRAJECTORY_AXIS_XY;
	}
	motionAddGoTo(newTraj, m_dest, VectPlan(), m_way, traj_type);
	m_motion->startTrajectory(m_linearParam, m_angularParam);
	m_trajectoryState = TRAJECTORY_STATE_MOVING_TO_DEST;
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
			m_dest.theta += findRotation(m_dest.theta, req.dest.theta);
			break;
		case TRAJECTORY_GOTO_XY:
			m_dest.x = req.dest.x;
			m_dest.y = req.dest.y;
			m_dest.theta = atan2f(m_dest.y - m_pos.y, m_dest.x - m_pos.x);
			m_approxDist = req.dist;
			m_way = (TrajectoryWay)req.way;
			break;
		case TRAJECTORY_GOTO_XYA:
			m_dest = req.dest;
			m_approxDist = req.dist;
			m_way = (TrajectoryWay)req.way;
			break;
		case TRAJECTORY_GOTO_GRAPH:
			id = findWayToGraph(m_pos, DETECTION_STATIC_OBJ);
			log_format(LOG_INFO, "point graph : %d", id);

			m_dest = VectPlan(m_graph.getNode(id), 0);
			break;
		case TRAJECTORY_FREE:
		default:
			m_type = TRAJECTORY_FREE;
			m_motion->enable(false);
			return;
	}

	// verification : trajectoire possible ?
	bool useGraph = false;
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

		float xmin = m_detection->computeFrontObject(DETECTION_STATIC_OBJ, pos, &a_table, &b_table);
		float dist2 = dx * dx +  dy * dy;
		float xmin2 = xmin * xmin;
		if( xmin2 < dist2)
		{
			// trajectoire impossible, on passe par le graph
			log(LOG_INFO, "goto - obstacle statique, utilisation du graph");
			computeGraph(DETECTION_STATIC_OBJ);
			useGraph = true;
		}
	}

	if( ! useGraph)
	{
		VectPlan dest = m_dest;
		VectPlan u = m_dest - m_pos;
		float ds = u.norm();
		log_format(LOG_INFO, "Move to dest");
		TrajectoryType traj_type = TRAJECTORY_AXIS_XYA;
		if( fabsf(ds) > EPSILON )
		{
			if( m_type == TRAJECTORY_GOTO_XY )
			{
				traj_type = TRAJECTORY_AXIS_XY;
			}
			dest.x -= m_approxDist * u.x / ds;
			dest.y -= m_approxDist * u.y / ds;
		}
		else if( m_type == TRAJECTORY_ROTATE )
		{
			traj_type = TRAJECTORY_AXIS_A;
		}
		motionAddGoTo(true, dest, VectPlan(), m_way, traj_type);
		m_motion->startTrajectory(m_linearParam, m_angularParam);
		m_trajectoryState = TRAJECTORY_STATE_MOVING_TO_DEST;
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
		xmin = m_detection->computeFrontObject(detect_type, pos, &a_table, &b_table);
		// TODO prendre en compte la rotation sur place en plus de la ligne droite
		// 10mm de marge / control
		dist = node_dist[i].dist;
		if( dist < xmin - Bot::halfLength - TRAJECTORY_APPROX_DIST - 10)
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

void Trajectory::detectionCallback(void* /*arg*/)
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

void Trajectory::trajectoryCmd(void* arg, void* data)
{
	Trajectory* traj = (Trajectory*) arg;
	xSemaphoreTake(traj->m_mutex, portMAX_DELAY);
	memcpy(&traj->m_request, data, sizeof(struct trajectory_cmd_arg));
	traj->updateRequest();
	xSemaphoreGive(traj->m_mutex);
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

void Trajectory::goToNearXy(float x, float y, float dist, TrajectoryWay way, enum avoidance_type avoidance_type)
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

void Trajectory::goToNear(VectPlan dest, float dist, TrajectoryWay way, enum avoidance_type avoidance_type)
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

void Trajectory::goTo(VectPlan dest, TrajectoryWay way, enum avoidance_type avoidance_type)
{
	goToNear(dest, 0, way, avoidance_type);
}

void Trajectory::goToGraphNode(uint32_t node_id, float dist, TrajectoryWay way, enum avoidance_type avoidance_type)
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
		log_format(LOG_ERROR, "incorrect state : %d, wanted %d", state, wanted_state);
		return -1;
	}

	return 0;
}
