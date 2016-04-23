#include "MotionSpeedState.h"
#include "kernel/log.h"
#include "disco/robot_parameters.h"
#include "kernel/control.h"

MotionSpeedState::MotionSpeedState() :
	MotionMoveState("MOTION_SPEED",MOTION_SPEED)
{

}

void MotionSpeedState::entry(void* data)
{
	Motion* m = (Motion*) data;
	m->m_status = MOTION_IN_MOTION;

	//Satisfaction de la volonte operateur
	m->m_wantedState = MOTION_UNKNOWN_STATE;

	log(LOG_INFO, "IN_MOTION");
}

void MotionSpeedState::run(void* data)
{
	Motion* m = (Motion*) data;
	m->m_kinematicsModel->computeActuatorCmd(m->m_u, m->m_v, CONTROL_DT, m->m_kinematics, true);
	m->motionUpdateMotors();
}

