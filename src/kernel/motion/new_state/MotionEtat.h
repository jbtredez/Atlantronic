//
//Classe mere d'un etat
//Cette classe contient le nom de l'etat

#include "kernel/log.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/can/can_id.h"
#include "kernel/canopen.h"
#include "kernel/control.h"
#include "kernel/location/location.h"
#include "kernel/kinematics_model/kinematics_model.h"
#include "kernel/driver/usb.h"
#include "kernel/driver/pwm.h"
#include "kernel/fault.h"
#include "kernel/driver/power.h"
#include "kernel/state_machine/state_machine.h"
#include "kernel/robot_parameters.h"
#include "kernel/detection.h"
#include "kernel/match.h"
#include "kernel/motion/motion_speed_check.h"
#include "kernel/asm/asm_base_func.h"
#include "kernel/can_motor_mip.h"
#include "kernel/motion/new_state/MotionVar.h"
#include "kernel/state_machine/new_state_machine/Etat.h"

#ifndef MOTION_ETAT_H
#define MOTION_ETAT_H


class MotionEtat :public Etat
{
	public :
		///////////////
		///Constructeur
		MotionEtat(const char* name);

		motion_state m_motion_State;
		int motion_module_init();
		static xSemaphoreHandle m_motion_mutex;
		//Liste des etats liée au contexte
		static motion_state m_motion_Wanted_State;

		static Kinematics m_motion_kinematics[CAN_MOTOR_MAX];
		static Kinematics m_motion_kinematics_mes[CAN_MOTOR_MAX];
		static enum motion_status m_motion_status;
		static VectPlan m_motion_u;
		static float m_motion_v;
		static enum motion_trajectory_step m_motion_traj_step;
		static VectPlan m_motion_pos_mes;
		static VectPlan m_motion_speed_mes;
		static VectPlan m_motion_dest;
		static VectPlan m_motion_pos_cmd_th;




		static bool m_motion_antico_on;
		void motion_update_motors();
		void motion_enable_antico(bool enable);
		void motion_cmd_set_param(motion_cmd_param_arg* arg);
		void motion_cmd_goto(void* arg);
		void motion_cmd_set_speed(void* arg);
		void motion_cmd_set_max_current(void* arg);
		void motion_cmd_enable(void* arg);
		void motion_cmd_set_actuator_kinematics(void* arg);
		void motion_enable(bool enable);
		void motion_set_max_driving_current(float maxCurrent);
		virtual void motion_goto(VectPlan dest, VectPlan cp, enum motion_way way, enum motion_trajectory_type type, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam);
		void motion_set_speed(VectPlan u, float v);
		void motion_stop();
		void motion_set_actuator_kinematics(struct motion_cmd_set_actuator_kinematics_arg cmd);
		void motion_get_state(enum motion_state* state, enum motion_status* status, enum motion_trajectory_step* step, enum motion_state* wanted_state);
		void motion_compute();
		void motion_update_usb_data(struct control_usb_data* data);
		void TakeSem();
		void GiveSem();


		static float motion_find_rotate(float start, float end);


		////////////////////////////////////////
		//méthode virtuelle Effectue l'action de l'etat
		//Param :
		//retourne: Réussite de l'action
		bool run(){return true;};

		////////////////////////////////////////
		//méthode virtuelle Effectue l'action de l'etat
		//Param :
		//retourne: Réussite de l'action
		bool entry();
		////////////////////////////////////////
		//méthode virtuelle Effectue l'action de l'etat
		//Param :
		//retourne: Réussite de l'action
		bool out();

		////////////////////////////////////////
		//méthode recupere l'etat suivant
		//Param :
		//retourne: l'etat suivant
		//          null si on ne change pas etat
		Etat * getProchainEtat(){return 0;};
};




#endif
