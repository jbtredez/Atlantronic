#ifndef MOTION_STATE_MACHINE_H
#define MOTION_STATE_MACHINE_H

#include "kernel/log.h"
#include "kernel/motion/pid.h"

#include "MotionVar.h"
#include "kernel/state_machine/new_state_machine/ContexteEtat.h"

#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/math/vect_plan.h"
#include "StateMotionActuatorKinematic.h"
#include "StateMotionDisable.h"
#include "StateMotionEnable.h"
#include "StateMotionSpeed.h"
#include "StateMotionTrajectory.h"
#include "StateMotionTryEnable.h"
#include "StateMotionInterrupting.h"

class MotionStateMachine : public ContextEtat
{
	private:
		Pid m_motion_x_pid;
		Pid m_motion_theta_pid;

	public:
		MotionStateMachine(){};
		~MotionStateMachine();
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

	private :
		//Declaration des States
		StateMotionActuatorKinematic 		m_StateMotionActuatorKinematic;
		StateMotionDisable 			m_StateMotionDisable;
		StateMotionEnable 			m_StateMotionEnable;
		StateMotionSpeed 			m_StateMotionSpeed;
		StateMotionTrajectory 			m_StateMotionTrajectory;
		StateMotionTryEnable 			m_StateMotionTryEnable;
		StateMotionInterrupting		m_StateMotionInterrupting;

		//declaration des paramétres commun des états
		motion_goto_parameter m_gotoparam;
		motion_cmd_set_actuator_kinematics_arg m_motion_cmd_set_actuator_kinematics_arg;
};

 extern MotionStateMachine * motionStateMachine;
#endif /* MOTION_STATE_MACHINE_H */
