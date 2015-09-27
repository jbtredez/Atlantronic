#ifndef TRAJECTORY_H
#define TRAJECTORY_H

//! @file trajectory.h
//! @brief Generation de trajectoire
//! @author Atlantronic

#include "motion.h"
#include "kernel/location/location.h"
#include "middleware/detection.h"
#include "graph.h"

enum trajectory_cmd_type
{
	TRAJECTORY_FREE,
	TRAJECTORY_STRAIGHT,
	TRAJECTORY_ROTATE,
	TRAJECTORY_ROTATE_TO,
	TRAJECTORY_GOTO_XY,
	TRAJECTORY_GOTO_XYA,
	TRAJECTORY_GOTO_GRAPH,
};

enum avoidance_type
{
	AVOIDANCE_STOP,        //!< arrêt en cas d'obstacle
	AVOIDANCE_GRAPH,       //!< passage par le graph en cas d'obstacle
};

enum trajectory_state
{
	TRAJECTORY_STATE_NONE = 0,
	TRAJECTORY_STATE_UPDATING_TRAJECTORY,
	TRAJECTORY_STATE_MOVE_TO_DEST,
	TRAJECTORY_STATE_MOVING_TO_DEST,
	TRAJECTORY_STATE_MOVE_TO_GRAPH,
	TRAJECTORY_STATE_USING_GRAPH,
	TRAJECTORY_STATE_TARGET_REACHED,
	TRAJECTORY_STATE_TARGET_NOT_REACHED,
	TRAJECTORY_STATE_COLISION,
};

struct trajectory_cmd_arg
{
	uint16_t type;             //!< type de trajectoire
	uint16_t avoidance_type;   //!< type d'évitement
	uint16_t way;              //!< sens
	VectPlan dest;
	float dist;
} __attribute__ (( packed ));

class Trajectory
{
	public:
		Trajectory();

		int init();

		void getKinematicsParam(KinematicsParameters* linParam, KinematicsParameters* angParam);

		void setKinematicsParam(KinematicsParameters linParam, KinematicsParameters angParam);

		//!< roue libre
		void freeWheel();

		//!< rejoindre le graph
		void goToGraph();

		void goToGraphNode(uint32_t node_id, float dist, enum motion_way way, enum avoidance_type avoidance_type);

		void goToNearXy(float x, float y, float dist, enum motion_way way, enum avoidance_type avoidance_type);

		void goToNear(VectPlan dest, float dist, enum motion_way way, enum avoidance_type avoidance_type);

		void goTo(VectPlan dest, enum motion_way way, enum avoidance_type avoidance_type);

		void rotate(float theta);

		void rotateTo(float theta);

		void straight(float dist);

		void straightToWall();

		//!< activation  /desactivation de l'arrêt sur obstacle statique
		inline void enableStaticCheck(bool enable)
		{
			m_staticCheckEnable = enable;
		}

		//!< activation / desactivation de l'arrêt sur obstacle détecté par hokuyo
		inline void enableHokuyo(bool enable)
		{
			m_hokuyoEnableCheck = enable;
		}

		inline enum trajectory_state getState()
		{
			return m_trajectoryState;
		}
		////////////////////////////////////////////////
		/// function    : trajectory_wait()
		/// descrition  : Waiting function of trajectory move functions
		/// param       : wanted_state = enum trajectory_state
		/// param       : timeout = uint32_t time_out (<0 no time-out but buffer overflow!!!!)
		/// retrun      : -1 if fail or 0 if sucess
		////////////////////////////////////////////////
		int wait(enum trajectory_state wanted_state, uint32_t timeout);

		static void trajectory_task(void* arg);

	protected:
		friend void trajectoryCmd(void* arg);
		void trajectoryTask();
		void simplifyPath(enum detection_type type);
		void computeGraph(enum detection_type type);
		void update();
		int findWayToGraph(VectPlan pos, enum detection_type detect_type);
		void updateRequest();

		// requete pour la tache trajectory + mutex
		struct trajectory_cmd_arg m_request;
		bool m_newRequest;
		xSemaphoreHandle m_mutex;

		// donnees privees a la tache
		VectPlan m_pos; //!< position du robot au moment du reveil de la tache
		VectPlan m_dest;
		float m_approxDist;
		enum motion_way m_way;
		enum trajectory_cmd_type m_type;
		enum avoidance_type m_avoidanceType;
		enum trajectory_state m_trajectoryState;
		bool m_hokuyoEnableCheck; //!< utilisation ou non des hokuyos
		bool m_staticCheckEnable; //!< verification des éléments statiques
		uint8_t m_graphWayId;
		KinematicsParameters m_linearParam;
		KinematicsParameters m_angularParam;
		Graph m_graph;
};

extern Trajectory trajectory;

#endif
