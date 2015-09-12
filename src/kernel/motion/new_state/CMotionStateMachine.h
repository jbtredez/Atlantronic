/*
 * CMotionStateMachine.h
 *
 *  Created on: 2 sept. 2015
 *      Author: jul
 */


#ifndef CMOTIONSTATEMACHINE_H_
#define CMOTIONSTATEMACHINE_H_
#include "kernel/log.h"
#include "kernel/motion/pid.h"

#include "kernel/motion/new_state/MotionVar.h"
#include "kernel/state_machine/new_state_machine/CContexteEtat.h"

#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/math/vect_plan.h"
#include "kernel/motion/new_state/CStateMotionActuatorKinematic.h"
#include "kernel/motion/new_state/CStateMotionDisable.h"
#include "kernel/motion/new_state/CStateMotionEnable.h"
#include "kernel/motion/new_state/CStateMotionSpeed.h"
#include "kernel/motion/new_state/CStateMotionTrajectory.h"
#include "kernel/motion/new_state/CStateMotionTryEnable.h"
#include "kernel/motion/new_state/CStateMotionInterrupting.h"
class CMotionStateMachine : public ContextEtat
{
	public:
		Pid m_motion_x_pid;
		Pid m_motion_theta_pid;

	public:
		CMotionStateMachine(){};
		~CMotionStateMachine();
		void Init();
		void motion_cmd_print_param(void* /*arg*/);
		void motion_cmd_set_param(void* arg);
		void motion_cmd_goto(void * arg);
		void motion_cmd_set_speed(void* arg);
		void motion_cmd_set_max_current(void* arg);
		void motion_cmd_enable(void* arg);
		void motion_cmd_set_actuator_kinematics(void* arg);
		void motion_enable(bool enable);
		void motion_enable_antico(bool enable);
		void motion_goto(VectPlan dest, VectPlan cp, enum motion_way way, enum motion_trajectory_type type, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam);
		void motion_stop();
		void motion_get_state(enum motion_state* state, enum motion_status* status, enum motion_trajectory_step* step, enum motion_state* wanted_state);
		void motion_update_usb_data(struct control_usb_data* data);
		void motion_compute();

	public :
		//Declaration des States
		CStateMotionActuatorKinematic 		m_StateMotionActuatorKinematic;
		CStateMotionDisable 			m_StateMotionDisable;
		CStateMotionEnable 			m_StateMotionEnable;
		CStateMotionSpeed 			m_StateMotionSpeed;
		CStateMotionTrajectory 			m_StateMotionTrajectory;
		CStateMotionTryEnable 			m_StateMotionTryEnable;
		CStateMotionInterrupting		m_StateMotionInterrupting;

		//declaration des paramétres commun des états
		motion_goto_parameter m_gotoparam;
		motion_cmd_set_actuator_kinematics_arg m_motion_cmd_set_actuator_kinematics_arg;

};

 extern CMotionStateMachine * MotionStateMachine;
#endif /* CMOTIONSTATEMACHINE_H_ */
