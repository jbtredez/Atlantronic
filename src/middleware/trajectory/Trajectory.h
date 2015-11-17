#ifndef TRAJECTORY_H
#define TRAJECTORY_H

//! @file Trajectory.h
//! @brief Generation de trajectoire
//! @author Atlantronic

#include "middleware/motion/Motion.h"
#include "kernel/location/location.h"
#include "middleware/detection.h"
#include "Graph.h"

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
	TRAJECTORY_STATE_MOVING_TO_DEST,
	TRAJECTORY_STATE_TARGET_REACHED,
	TRAJECTORY_STATE_TARGET_NOT_REACHED,
	TRAJECTORY_STATE_COLISION,
};

enum TrajectoryWay
{
	WAY_BACKWARD = -1,    //!< marche arriere
	WAY_ANY  = 0,         //!< marche avant ou marche arriere (selon le plus rapide)
	WAY_FORWARD  = 1,     //!< marche avant
};

enum TrajectoryType
{
	TRAJECTORY_AXIS_XYA = 0,     //!< aller a la position x,y, alpha en ligne droite (=> rotation puis avance puis rotation)
	TRAJECTORY_AXIS_A,           //!< rotation sur place
	TRAJECTORY_AXIS_XY,          //!< aller a la position x,y en ligne droite (=> rotation puis avance)
	TRAJECTORY_CURVILINEAR_XY,   //!< deplacement curviligne vers x,y
	TRAJECTORY_CURVILINEAR_XYA,  //!< deplacement curviligne vers x,y,a
};

struct trajectory_cmd_arg
{
	uint16_t type;             //!< type de trajectoire
	uint16_t avoidance_type;   //!< type d'évitement
	uint16_t way;              //!< sens
	VectPlan dest;
	float dist;
} __attribute__ (( packed ));

#ifndef LINUX
class Trajectory
{
	public:
		int init(Detection* detection, Motion* motion, Location* location);

		void getKinematicsParam(KinematicsParameters* linParam, KinematicsParameters* angParam);

		void setKinematicsParam(KinematicsParameters linParam, KinematicsParameters angParam);

		//!< roue libre
		void freeWheel();

		//!< rejoindre le graph
		void goToGraph();

		void goToGraphNode(uint32_t node_id, float dist, enum TrajectoryWay way, enum avoidance_type avoidance_type);

		void goToNearXy(float x, float y, float dist, enum TrajectoryWay way, enum avoidance_type avoidance_type);

		void goToNear(VectPlan dest, float dist, enum TrajectoryWay way, enum avoidance_type avoidance_type);

		void goTo(VectPlan dest, enum TrajectoryWay way, enum avoidance_type avoidance_type);

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
		static void trajectoryCmd(void* arg, void* data);
		static void detectionCallback(void* arg);
		void trajectoryTask();
		void simplifyPath(enum detection_type type);
		void computeGraph(enum detection_type type);
		void update();
		int findWayToGraph(VectPlan pos, enum detection_type detect_type);
		void updateRequest();
		void motionAddGoTo(bool newTrajectory, VectPlan dest, VectPlan cp, enum TrajectoryWay way, enum TrajectoryType type);
		void motionAddGoToCurvilinear(bool newTrajectory, VectPlan dest, VectPlan cp, enum TrajectoryWay way, enum TrajectoryType type);
		void motionAddGoToStraightRotate(bool newTrajectory, VectPlan dest, VectPlan cp, enum TrajectoryWay way, enum TrajectoryType type);

		// requete pour la tache trajectory + mutex
		struct trajectory_cmd_arg m_request;
		bool m_newRequest;
		xSemaphoreHandle m_mutex;

		// donnees privees a la tache
		VectPlan m_pos; //!< position du robot au moment du reveil de la tache
		VectPlan m_dest;
		float m_approxDist;
		enum TrajectoryWay m_way;
		enum trajectory_cmd_type m_type;
		enum avoidance_type m_avoidanceType;
		enum trajectory_state m_trajectoryState;
		bool m_hokuyoEnableCheck; //!< utilisation ou non des hokuyos
		bool m_staticCheckEnable; //!< verification des éléments statiques
		KinematicsParameters m_linearParam;
		KinematicsParameters m_angularParam;
		Graph m_graph;
		Location* m_location;
		Detection* m_detection;
		Motion* m_motion;
};

#endif
#endif
