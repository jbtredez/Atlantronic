#include "kernel/log.h"
#include "kernel/motion/new_state/MotionVar.h"
#include "kernel/motion/new_state/MotionEtat.h"

#ifndef STATE_MOTION_TRAJECTORY_H
#define STATE_MOTION_TRAJECTORY_H


class StateMotionTrajectory : public MotionEtat
{
	private:
		systime m_motion_target_not_reached_start_time;

		Etat * m_pMotionDisable;
		Etat * m_pMotionInterrupting;

		Pid * m_pmotion_x_pid;
		Pid * m_pmotion_theta_pid; 
		motion_goto_parameter * m_pgotoparam;
		MotionSpeedCheck m_motion_linear_speed_check;
		Kinematics m_motion_curvilinearKinematics;
		VectPlan   m_motion_speed_cmd;
		float m_motion_ds[3];
	public:
		StateMotionTrajectory();
		~StateMotionTrajectory();


		void InitState(Etat * pMotionInterrupting, Etat * pMotionDisable, Pid * pmotion_x_pid,Pid * pmotion_theta_pid , motion_goto_parameter * pgotoparam);

		////////////////////////////////////////
		//méthode virtuelle Effectue l'action de l'etat
		//Param :
		//retourne: Réussite de l'action
		bool run();

		////////////////////////////////////////
		//méthode virtuelle Effectue l'action de l'etat
		//Param :
		//retourne: Réussite de l'action
		bool entry();

		////////////////////////////////////////
		//méthode recupere l'etat suivant
		//Param :
		//retourne: l'etat suivant
		//          null si on ne change pas etat
		Etat * getProchainEtat();
		void motion_update_motors();
		void motion_goto(VectPlan dest, VectPlan cp, enum motion_way way, enum motion_trajectory_type type, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam);
		float motion_compute_time(float ds, KinematicsParameters param);


};

#endif /* STATE_MOTION_TRAJECTORY_H */
