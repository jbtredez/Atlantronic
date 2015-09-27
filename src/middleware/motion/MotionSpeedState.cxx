#include "MotionSpeedState.h"
#include "kernel/log.h"
#include "disco/robot_parameters.h"
#include "kernel/control.h"
#include "kernel/kinematics_model/kinematics_model.h"

MotionSpeedState::MotionSpeedState() :
	StateMachineState("MOTION_SPEED")
{

}

void MotionSpeedState::entry(void* data)
{
	Motion* m = (Motion*) data;
	m->m_status = MOTION_IN_MOTION;
	log(LOG_INFO, "IN_MOTION");
}

void MotionSpeedState::run(void* data)
{
	Motion* m = (Motion*) data;
	kinematics_model_compute_actuator_cmd(VOIE_MOT, m->m_u, m->m_v, CONTROL_DT, m->m_kinematics);
	m->motionUpdateMotors();
}

unsigned int MotionSpeedState::transition(void* data, unsigned int currentState)
{
	Motion* m = (Motion*) data;
	return m->motionStateGenericPowerTransition(currentState);
}
