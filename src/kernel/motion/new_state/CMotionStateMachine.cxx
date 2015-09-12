/*
 * CMotionStateMachine.cpp
 *
 *  Created on: 2 sept. 2015
 *      Author: jul
 */


#include "CMotionStateMachine.h"

CMotionStateMachine * MotionStateMachine;

static void motion_cmd_print_param(void* arg/*arg*/)
{
	MotionStateMachine->motion_cmd_print_param(arg);
}

static void motion_cmd_set_param(void* arg)
{
	MotionStateMachine->motion_cmd_set_param(arg);
}

static void motion_cmd_goto(void * arg)
{
	MotionStateMachine->motion_cmd_goto(arg);
}

static void motion_cmd_set_speed(void* arg)
{
	MotionStateMachine->motion_cmd_goto(arg);
}

static void motion_cmd_set_max_current(void* arg)
{
	MotionStateMachine->motion_cmd_set_max_current(arg);
}


static void motion_cmd_enable(void* arg)
{
	MotionStateMachine->motion_cmd_enable(arg);
}

static void motion_cmd_set_actuator_kinematics(void* arg)
{
	MotionStateMachine->motion_cmd_set_actuator_kinematics(arg);
}
static int motion_module_init()
{

	MotionStateMachine = new CMotionStateMachine();
	MotionStateMachine->Init();
	MotionEtat * pMotionCourant = (MotionEtat *)MotionStateMachine->getCurrentState();
	pMotionCourant->motion_module_init();	
	usb_add_cmd(USB_CMD_MOTION_GOTO, &motion_cmd_goto);
	usb_add_cmd(USB_CMD_MOTION_SET_SPEED, &motion_cmd_set_speed);
	usb_add_cmd(USB_CMD_MOTION_SET_MAX_CURRENT, &motion_cmd_set_max_current);
	usb_add_cmd(USB_CMD_MOTION_ENABLE, &motion_cmd_enable);
	usb_add_cmd(USB_CMD_MOTION_SET_ACTUATOR_KINEMATICS, &motion_cmd_set_actuator_kinematics);
	usb_add_cmd(USB_CMD_MOTION_PRINT_PARAM, &motion_cmd_print_param);
	usb_add_cmd(USB_CMD_MOTION_PARAM, &motion_cmd_set_param);

	return 0;
}

module_init(motion_module_init, INIT_MOTION);
void CMotionStateMachine::Init()
{
	// TODO Auto-generated constructor stub
	m_motion_x_pid.kp = 2;
	m_motion_x_pid.ki = 1;
	m_motion_x_pid.kd = 0;
	m_motion_x_pid.max_integral = 100;
	m_motion_x_pid.max_out = 100;


	m_motion_theta_pid.kp = 8;
	m_motion_theta_pid.ki = 1;
	m_motion_theta_pid.kd = 0;
	m_motion_theta_pid.max_integral = 1;
	m_motion_theta_pid.max_out = 1;


	//Initalisation des etats

	m_StateMotionDisable.InitState(&m_StateMotionTryEnable);

	m_StateMotionTryEnable.InitState(&m_StateMotionEnable,&m_StateMotionDisable);

	m_StateMotionEnable.InitState(&m_StateMotionTrajectory,&m_StateMotionSpeed,&m_StateMotionActuatorKinematic,&m_StateMotionDisable, &m_motion_cmd_set_actuator_kinematics_arg,&m_gotoparam);

	m_StateMotionSpeed.InitState(&m_StateMotionEnable,&m_StateMotionDisable);

	m_StateMotionActuatorKinematic.InitState(&m_StateMotionEnable,&m_StateMotionDisable,&m_motion_cmd_set_actuator_kinematics_arg);


	m_StateMotionTrajectory.InitState(&m_StateMotionInterrupting,&m_StateMotionDisable,&m_motion_x_pid,&m_motion_theta_pid,&m_gotoparam);

	m_StateMotionInterrupting.InitState(&m_StateMotionEnable,&m_StateMotionDisable);

	//Mise en place du premier Etat
	mp_EtatCourant = &m_StateMotionDisable;

}


CMotionStateMachine::~CMotionStateMachine()
{
	// TODO Auto-generated destructor stub
}


void CMotionStateMachine::motion_cmd_print_param(void* /*arg*/)
{
	log_format(LOG_INFO, "axe x     : kp %d ki %d kd %d", (int)(this->m_motion_x_pid.kp), (int)(m_motion_x_pid.ki), (int)(m_motion_x_pid.kd));
	log_format(LOG_INFO, "axe theta : kp %d ki %d kd %d", (int)(this->m_motion_theta_pid.kp), (int)(m_motion_theta_pid.ki), (int)(m_motion_theta_pid.kd));

}

void CMotionStateMachine::motion_cmd_set_param(void* arg)
{
	struct motion_cmd_param_arg* cmd = (struct motion_cmd_param_arg*) arg;
	this->m_motion_x_pid.kp = cmd->kp_av;
	this->m_motion_x_pid.ki = cmd->ki_av;
	this->m_motion_x_pid.kd = cmd->kd_av;
	this->m_motion_theta_pid.kp = cmd->kp_rot;
	this->m_motion_theta_pid.ki = cmd->ki_rot;
	this->m_motion_theta_pid.kd = cmd->kd_rot;
}

void CMotionStateMachine::motion_enable_antico(bool enable)
{
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_enable_antico(enable);	
}
void CMotionStateMachine::motion_cmd_goto(void* arg)
{
	struct motion_cmd_goto_arg* cmd = (struct motion_cmd_goto_arg*) arg;
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_goto(cmd->dest, cmd->cp, (enum motion_way)cmd->way, (enum motion_trajectory_type)cmd->type, cmd->linearParam, cmd->angularParam);
}

void CMotionStateMachine::motion_enable(bool enable)
{
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_enable(enable);
}
void CMotionStateMachine::motion_goto(VectPlan dest, VectPlan cp, enum motion_way way, enum motion_trajectory_type type, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam)
{

	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_goto(dest, cp, way, type,linearParam, angularParam);
}

void CMotionStateMachine::motion_cmd_set_speed(void* arg)
{
	struct motion_cmd_set_speed_arg* cmd = (struct motion_cmd_set_speed_arg*) arg;
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_set_speed(cmd->u, cmd->v);
}
void CMotionStateMachine::motion_get_state(enum motion_state* state, enum motion_status* status, enum motion_trajectory_step* step, enum motion_state* wanted_state)
{
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_get_state(state, status,step,wanted_state);
}

void CMotionStateMachine::motion_cmd_set_max_current(void* arg)
{
	struct motion_cmd_set_max_driving_current_arg* cmd = (struct motion_cmd_set_max_driving_current_arg*) arg;
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_set_max_driving_current(cmd->maxDrivingCurrent);
}

void CMotionStateMachine::motion_cmd_enable(void* arg)
{
	struct motion_cmd_enable_arg* cmd_arg = (struct motion_cmd_enable_arg*) arg;
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_enable(cmd_arg->enable != 0);
}

void CMotionStateMachine::motion_cmd_set_actuator_kinematics(void* arg)
{
	struct motion_cmd_set_actuator_kinematics_arg* cmd = (struct motion_cmd_set_actuator_kinematics_arg*) arg;
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_set_actuator_kinematics(*cmd);
}

void CMotionStateMachine::motion_compute()
{

	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;

	pMotionCourant->TakeSem();
	pMotionCourant->motion_compute();
	execute();

	pMotionCourant->GiveSem();
}

void CMotionStateMachine::motion_update_usb_data(struct control_usb_data* data)
{
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_update_usb_data(data);

}
