/*
 * TrajectoryMoveToDest.h
 *
 *  Created on: 18 nov. 2015
 *      Author: jul
 */

#ifndef TRAJECTORYMOVETODEST_H_
#define TRAJECTORYMOVETODEST_H_
#include "middleware/trajectory/Trajectory.h"
#include "middleware/motion/Motion.h"
#include "kernel/location/location.h"
#include "middleware/detection.h"
#include "Graph.h"

struct trajectory_dest
{
	trajectory_cmd_type cmdType;             //!< type de trajectoire
	TrajectoryType type;             //!< type de trajectoir
	avoidance_type avoidance;   //!< type d'évitement
	TrajectoryWay way;              //!< sens
	VectPlan position;			   //!<destination x,y
	float approxDist;				   //Précision
}

class TrajectoryMoveToDest: public StateMachineState
{
	protected:
		trajectory_dest	m_dest;
		VectPlan	m_position;
		bool m_hokuyoEnableCheck; //!< utilisation ou non des hokuyos
		bool m_staticCheckEnable; //!< verification des éléments statiques
		KinematicsParameters m_linearParam;
		KinematicsParameters m_angularParam;
		Graph m_graph;
		Location* m_plocation;
		Detection* m_pdetection;
        Motion * m_pmotion;
		bool computeGraph(enum detection_type type);


	public:
		TrajectoryMoveToDest(Motion pmotion, Detection pdetection, Location plocation);
		virtual ~TrajectoryMoveToDest();
		uint32_t transition(void* data);
		void run(void* data);
		void entry(void* data);


		uint32_t findWayToGraph( enum detection_type detect_type);
		bool computeGraph(enum detection_type type);

		//Ajout d'une destination pour state move
		void motionAddGoToCurvilinear(bool newTrajectory);
		void motionAddGoToStraightRotate(bool newTrajectory);
		void motionAddGoTo(bool newTrajectory);

};

#endif /* TRAJECTORYMOVETODEST_H_ */
