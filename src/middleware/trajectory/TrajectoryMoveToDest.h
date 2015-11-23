/*
 * TrajectoryMoveToDest.h
 *
 *  Created on: 18 nov. 2015
 *      Author: jul
 */

#ifndef TRAJECTORYMOVETODEST_H_
#define TRAJECTORYMOVETODEST_H_



struct trajectory_dest
{
	trajectory_cmd_type type;             //!< type de trajectoire
	avoidance_type avoidance;   //!< type d'évitement
	TrajectoryWay way;              //!< sens
	VectPlan position;			   //!<destination x,y
	float approxDist;				   //Précision
}

class TrajectoryMoveToDest: public StateMachineState
{
	private:
		trajectory_dest	m_dest;
		VectPlan	m_position;
	public:
		TrajectoryMoveToDest();
		virtual ~TrajectoryMoveToDest();
		unsigned int transition(void* data);
		void run(void* data);
		void entry(void* data);
		int findWayToGraph(VectPlan pos, enum detection_type detect_type);



		void computeGraph(enum detection_type type);
		//Ajout d'une destination pour state move
		motionAddGoToCurvilinear();
		void motionAddGoTo
};

#endif /* TRAJECTORYMOVETODEST_H_ */
