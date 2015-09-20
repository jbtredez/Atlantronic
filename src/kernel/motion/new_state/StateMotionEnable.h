#ifndef STATE_MOTION_ENABLE_H
#define STATE_MOTION_ENABLE_H

class StateMotionEnable : public MotionEtat
{
	private:
		Etat * m_pMotionSpeed;
		Etat * m_pMotionTrajectory;
		Etat * m_pMotionActuorKinematics;
		Etat * m_pMotionDisable;
		motion_cmd_set_actuator_kinematics_arg * m_pmotion_wanted_kinematics;
		motion_goto_parameter * m_pgotoparam;
	public:
		StateMotionEnable();

		void initState(Etat * pMotionTrajectory, Etat * pMotionSpeed, Etat * pMotionActuorKinematics, Etat * pMotionDisable,motion_cmd_set_actuator_kinematics_arg *pmotion_wanted_kinematics,motion_goto_parameter * pgotoparam);
		////////////////////////////////////////
		//méthode virtuelle Effectue l'action de l'etat
		//Param :
		//retourne: Réussite de l'action		
		bool run();

		////////////////////////////////////////
		//méthode recupere l'etat suivant
		//Param :
		//retourne: l'etat suivant
		//          null si on ne change pas etat
		Etat * getProchainEtat();
		void motion_goto(VectPlan dest, VectPlan cp, enum motion_way way, enum motion_trajectory_type type, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam);

		void motion_set_actuator_kinematics(struct motion_cmd_set_actuator_kinematics_arg cmd);
};

#endif
