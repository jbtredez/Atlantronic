#include "MotionStateMachine.h"

MotionStateMachine * motionStateMachine;

static void motion_cmd_print_param(void* arg/*arg*/)
{
	motionStateMachine->motion_cmd_print_param(arg);
}

static void motion_cmd_set_param(void* arg)
{
	motionStateMachine->motion_cmd_set_param(arg);
}

static void motion_cmd_goto(void * arg)
{
	motionStateMachine->motion_cmd_goto(arg);
}

static void motion_cmd_set_speed(void* arg)
{
	motionStateMachine->motion_cmd_goto(arg);
}

static void motion_cmd_set_max_current(void* arg)
{
	motionStateMachine->motion_cmd_set_max_current(arg);
}


static void motion_cmd_enable(void* arg)
{
	motionStateMachine->motion_cmd_enable(arg);
}

static void motion_cmd_set_actuator_kinematics(void* arg)
{
	motionStateMachine->motion_cmd_set_actuator_kinematics(arg);
}
static int motion_module_init()
{

	motionStateMachine = new MotionStateMachine();
	motionStateMachine->init();
	MotionEtat * pMotionCourant = (MotionEtat *)motionStateMachine->getCurrentState();
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

void MotionStateMachine::init()
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

	m_StateMotionDisable.initState(&m_StateMotionTryEnable);
	m_StateMotionTryEnable.initState(&m_StateMotionEnable,&m_StateMotionDisable);
	m_StateMotionEnable.initState(&m_StateMotionTrajectory,&m_StateMotionSpeed,&m_StateMotionActuatorKinematic,&m_StateMotionDisable, &m_motion_cmd_set_actuator_kinematics_arg,&m_gotoparam);
	m_StateMotionSpeed.initState(&m_StateMotionEnable,&m_StateMotionDisable);
	m_StateMotionActuatorKinematic.initState(&m_StateMotionEnable,&m_StateMotionDisable,&m_motion_cmd_set_actuator_kinematics_arg);
	m_StateMotionTrajectory.initState(&m_StateMotionInterrupting,&m_StateMotionDisable,&m_motion_x_pid,&m_motion_theta_pid,&m_gotoparam);
	m_StateMotionInterrupting.initState(&m_StateMotionEnable,&m_StateMotionDisable);

	//Mise en place du premier Etat
	mp_EtatCourant = &m_StateMotionDisable;

}

void MotionStateMachine::motion_cmd_print_param(void* /*arg*/)
{
	log_format(LOG_INFO, "axe x     : kp %d ki %d kd %d", (int)(this->m_motion_x_pid.kp), (int)(m_motion_x_pid.ki), (int)(m_motion_x_pid.kd));
	log_format(LOG_INFO, "axe theta : kp %d ki %d kd %d", (int)(this->m_motion_theta_pid.kp), (int)(m_motion_theta_pid.ki), (int)(m_motion_theta_pid.kd));

}

void MotionStateMachine::motion_cmd_set_param(void* arg)
{
	struct motion_cmd_param_arg* cmd = (struct motion_cmd_param_arg*) arg;
	this->m_motion_x_pid.kp = cmd->kp_av;
	this->m_motion_x_pid.ki = cmd->ki_av;
	this->m_motion_x_pid.kd = cmd->kd_av;
	this->m_motion_theta_pid.kp = cmd->kp_rot;
	this->m_motion_theta_pid.ki = cmd->ki_rot;
	this->m_motion_theta_pid.kd = cmd->kd_rot;
}

void MotionStateMachine::motion_enable_antico(bool enable)
{
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_enable_antico(enable);	
}
void MotionStateMachine::motion_cmd_goto(void* arg)
{
	struct motion_cmd_goto_arg* cmd = (struct motion_cmd_goto_arg*) arg;
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_goto(cmd->dest, cmd->cp, (enum motion_way)cmd->way, (enum motion_trajectory_type)cmd->type, cmd->linearParam, cmd->angularParam);
}

void MotionStateMachine::motion_enable(bool enable)
{
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_enable(enable);
}
void MotionStateMachine::motion_goto(VectPlan dest, VectPlan cp, enum motion_way way, enum motion_trajectory_type type, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam)
{

	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_goto(dest, cp, way, type,linearParam, angularParam);
}

void MotionStateMachine::motion_cmd_set_speed(void* arg)
{
	struct motion_cmd_set_speed_arg* cmd = (struct motion_cmd_set_speed_arg*) arg;
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_set_speed(cmd->u, cmd->v);
}
void MotionStateMachine::motion_get_state(enum motion_state* state, enum motion_status* status, enum motion_trajectory_step* step, enum motion_state* wanted_state)
{
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_get_state(state, status,step,wanted_state);
}

void MotionStateMachine::motion_cmd_set_max_current(void* arg)
{
	struct motion_cmd_set_max_driving_current_arg* cmd = (struct motion_cmd_set_max_driving_current_arg*) arg;
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_set_max_driving_current(cmd->maxDrivingCurrent);
}

void MotionStateMachine::motion_cmd_enable(void* arg)
{
	struct motion_cmd_enable_arg* cmd_arg = (struct motion_cmd_enable_arg*) arg;
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_enable(cmd_arg->enable != 0);
}

void MotionStateMachine::motion_cmd_set_actuator_kinematics(void* arg)
{
	struct motion_cmd_set_actuator_kinematics_arg* cmd = (struct motion_cmd_set_actuator_kinematics_arg*) arg;
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_set_actuator_kinematics(*cmd);
}

void MotionStateMachine::motion_compute()
{

	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;

	pMotionCourant->TakeSem();
	pMotionCourant->motion_compute();
	execute();

	pMotionCourant->GiveSem();
}

void MotionStateMachine::motion_update_usb_data(struct control_usb_data* data)
{
	MotionEtat * pMotionCourant = (MotionEtat *)mp_EtatCourant;
	pMotionCourant->motion_update_usb_data(data);

}
