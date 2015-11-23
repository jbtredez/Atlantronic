/*
 * TrajectoryMoveToDest.cpp
 *
 *  Created on: 18 nov. 2015
 *      Author: jul
 */

#include <stdlib.h>
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/math/findRotation.h"
#include "disco/robot_parameters.h"
#include "Trajectory.h"
#include "kernel/math/poly7.h"

#define TRAJECTORY_STACK_SIZE       400
#define TRAJECTORY_APPROX_DIST      150      //!< distance d'approche d'un objet
#define TRAJECTORY_PERIOD            10



#include "TrajectoryMoveToDest.h"

TrajectoryMoveToDest::TrajectoryMoveToDest()
StateMachineState("TRAJECTORY_MoveTodest",TRAJECTORY_STATE_MOVE_TO_DEST)
{


}

TrajectoryMoveToDest::~TrajectoryMoveToDest()
{
	// TODO Auto-generated destructor stub
}


void TrajectoryMoveToDest::entry(void* data)
{
	Trajectory* t = (Trajectory*) data;

	//Satisfaction de la volonte operateur ou de la volonte automatique
	t->m_wantedState = TRAJECTORY_STATE_NONE;
	bool new_request = false;
	struct trajectory_cmd_arg req;
	int id;

	// mutex pour trajectory_request
	xSemaphoreTake(t->m_mutex, portMAX_DELAY);
	if(t->m_newRequest == true)
	{
		req = t->m_request;
		t->m_newRequest = false;
		new_request = true;
	}
	xSemaphoreGive(t->m_mutex);

	if(new_request == true)
	{
		//On initialise la postion de destination final
		m_dest.position = req.dest;
		m_dest.approxDist = req.dist;
		m_dest.way = (enum TrajectoryWay)req.way;
		m_dest.avoidance = (enum avoidance_type)req.avoidance_type;
		m_dest.type = (enum trajectory_cmd_type)req.type;
	}




}

void TrajectoryMoveToDest::run(void* data)
{
	Trajectory* t = (Trajectory*) data;
	m_position = t->m_location->getPosition();

	//détermine quelle est le type de mouvement demandé
	switch(m_dest.type)
	{
		//Cas de déplacement rectiligne (on se déplace suivant un angle et une distance)
		case TRAJECTORY_STRAIGHT:
		{
			log_format(LOG_INFO, "straight %d", (int)m_dest.approxDist);
			m_dest.position.x += cosf(robotPosition.theta) * m_dest.approxDist;
			m_dest.position.y += sinf(robotPosition.theta) * m_dest.approxDist;

			if(m_dest.approxDist > 0)
			{
				m_dest.way = WAY_FORWARD;
			}
			else
			{
				m_dest.way = WAY_BACKWARD;
			}
			m_dest.approxDist = 0;
			break;
		}
			//Cas de Rotation simple  (on fait une simple rotation relative)
		case TRAJECTORY_ROTATE:
			log_format(LOG_INFO, "rotate %d", (int)(m_dest.position.theta * 180 / M_PI));
			m_dest.position.theta += m_position.theta;
			break;


			//Cas de Rotation simple  (on fait une simple rotation dans le repere fixe)
			//On calcul la rotation relative pour la convertir la cmd en rotation relative
		case TRAJECTORY_ROTATE_TO:
			log_format(LOG_INFO, "rotate_to %d", (int)(m_dest.position.theta * 180 / M_PI));
			m_dest.position.theta += findRotation(m_position.theta, m_dest.position.theta);
			m_dest.type = TRAJECTORY_ROTATE;
			break;



			//Cas de deplacement à une position X et Y dans un repere fixe sans contrainte de rotation
		case TRAJECTORY_GOTO_XY:
			m_dest.position.theta = atan2f(m_dest.position.y - m_position.y, m_dest.position.x - m_position.x);
			break;

			//Cas de deplacement à une position X et Y dans un repere fixe avec contrainte de rotation
		case TRAJECTORY_GOTO_XYA:
			//Rien à faire tous les paramètres sont déjà sauvegardés
			break;

			//Cas de deplacement à un noeud du graph
		case TRAJECTORY_GOTO_GRAPH:
			///Recherche du point le plus proche du graph
			id = findWayToGraph(m_position, DETECTION_STATIC_OBJ);
			log_format(LOG_INFO, "point graph : %d", id);
			//L'objectif en graph node vers le position
			m_dest = VectPlan(t->m_graph.getNode(id), 0);
			break;
		case TRAJECTORY_FREE:
		default:
			///Roue libre
			t->m_type = TRAJECTORY_FREE;
			t->m_motion->enable(false);
			return;
	}

	// verification : trajectoire possible ?
		bool useGraph = false;
		if(t->m_staticCheckEnable && (m_dest.type == TRAJECTORY_GOTO_XY || m_dest.type == TRAJECTORY_GOTO_XYA))
		{
			// TODO : check fait pour la marche avant, tolérance plus grande si on le fait en marche arrière ?
			// on regarde si c'est possible / objets statiques de la table
			// si c'est pas possible, on passe par le graph
			Vect2 a_table;
			Vect2 b_table;

			// calcul du vecteur de trajectoire du robot pour aller à la position final
			VectPlan Movevector = m_position;
			float dx = m_dest.position.x - m_position.x;
			float dy = m_dest.position.y - m_position.y;
			Movevector.theta = atan2f(dy, dx);
			//Vérifie la présence d'objet static sur le chemin
			float xmin = t->m_detection->computeFrontObject(DETECTION_STATIC_OBJ, Movevector, &a_table, &b_table);

			float dist2 = dx * dx +  dy * dy;
			float xmin2 = xmin * xmin;

			//un objet static se trouve sur le chemin donc on utilise le graph sinon on se déplace directement à la position
			if( xmin2 < dist2)
			{
				// trajectoire impossible, on passe par le graph
				log(LOG_INFO, "goto - obstacle statique, utilisation du graph");
				computeGraph(DETECTION_STATIC_OBJ);
				return ;
			}
		}
		VectPlan dest = m_dest.position;

		///Calcul du delta pour la future destination
		VectPlan u = m_dest - m_position;
		float ds = u.norm();


		log_format(LOG_INFO, "Move to dest");
		//Par défaut on se déplace puis on fait une rotation
		TrajectoryType traj_type = TRAJECTORY_AXIS_XYA;

		//Si la distance est plus grande que ESPSILON
		if( fabsf(ds) > EPSILON )
		{
			///Bizzard plus la distance est grande moins on tolère une approximation de distance  ????
			//Ce n'est pas un calcul de la sorte
			//if(u.x < 0)
			// dest.x -= m_dest.approxDisti
			//else
			//dest.x -= m_dest.approxDist ? ->idem en Y
			if( t->m_type == TRAJECTORY_GOTO_XY )
			{
				traj_type = TRAJECTORY_AXIS_XY;
			}
			dest.x -= t->m_approxDist * u.x / ds;
			dest.y -= t->m_approxDist * u.y / ds;
		}
		//Si c'est une petite trajectoire c'est forcement un de rotation
		else if( m_dest->type == TRAJECTORY_ROTATE )
		{
			traj_type = TRAJECTORY_AXIS_A;
		}
		//Ajout d'une position à atteindre
		motionAddGoTo(true, dest, VectPlan(), m_dest->way, traj_type);

		t->m_motion->startTrajectory(t->m_linearParam, t->m_angularParam);
		t->m_trajectoryState = TRAJECTORY_STATE_MOVING_TO_DEST;

}

void TrajectoryMoveToDest::computeGraph(enum detection_type type)
{
	int startId = findWayToGraph(m_position, type);
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



int TrajectoryMoveToDest::findWayToGraph(VectPlan pos, enum detection_type detect_type)
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
unsigned int TrajectoryMoveToDest::transition(void* data)
{
	Trajectory* traj = (Trajectory*) data;

	if(traj->m_newRequest)
	{
		return TRAJECTORY_STATE_MOVE_TO_DEST;
	}
	else
	{
		return TRAJECTORY_STATE_MOVING_TO_DEST;
	}
	// sinon dans les autres cas on ne change pas d'état change d'etat
	return m_stateId;
}

void TrajectoryMoveToDest::motionAddGoToCurvilinear(bool newTrajectory, VectPlan dest, VectPlan cp, TrajectoryWay way, TrajectoryType type)
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
	float u[6] = { n, 0, 0, 0, 0, 0};
	if( type == TRAJECTORY_CURVILINEAR_XYA )
	{
		u[2] = n;
	}

	if( way == WAY_ANY )
	{
		// test en marche avant
		poly7f_full(start, wantedDest, a1, b1, u);

		// test en marche arriere
		start.theta += M_PI;
		wantedDest.theta += M_PI;
		poly7f_full(start, wantedDest, a2, b2, u);
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
		poly7f_full(start, wantedDest, a1, b1, u);
	}
	else
	{
		// test en marche arriere
		start.theta += M_PI;
		wantedDest.theta += M_PI;
		poly7f_full(start, wantedDest, a2, b2, u);
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
		pt.pos.theta = atan2f(y - pt.pos.y, x - pt.pos.x);
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

void TrajectoryMoveToDest::motionAddGoTo(bool newTrajectory, VectPlan dest, VectPlan cp, TrajectoryWay way, TrajectoryType type)
{
	switch(type)
	{
		case TRAJECTORY_AXIS_XYA:
			//type = TRAJECTORY_CURVILINEAR_XYA;
			//motionAddGoToCurvilinear(newTrajectory, dest, cp, way, type);
			//break;
		case TRAJECTORY_AXIS_A:
		case TRAJECTORY_AXIS_XY:
			motionAddGoToStraightRotate(newTrajectory, dest, cp, way, type);
			//type = TRAJECTORY_CURVILINEAR_XY;
			//motionAddGoToCurvilinear(newTrajectory, dest, cp, way, type);
			break;
		case TRAJECTORY_CURVILINEAR_XY:
		case TRAJECTORY_CURVILINEAR_XYA:
			motionAddGoToCurvilinear(newTrajectory, dest, cp, way, type);
			break;
		default:
			log(LOG_ERROR, "unknown trajectory type");
			break;
	}
}

