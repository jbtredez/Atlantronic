#include "action.h"


////////////////////////////////////////////////
/// function    : action()
/// descrition  : constructor
/// param       : firstcheckpoint : VectPlan first checkpoint of the action
/// retrun      : none
////////////////////////////////////////////////	
Action::Action(VectPlan firstcheckpoint, const char * name,void* robotState)
{
	m_name = name;
	m_retry = 1;
	m_actiontype = -1;
	m_firstcheckpoint = firstcheckpoint;
	initialized = false;
	m_robotState = robotState;
	m_state = ACTION_NOT_DONE;
}

int Action::do_action()
{
	return -1;
};

void Action::Initialise(int stratcolor)
{
	if( ! initialized )
	{
		m_firstcheckpoint = m_firstcheckpoint.symetric(stratcolor);
		initialized = true;
	}
}

bool Action::Ready()
{

	return ( ((m_state == ACTION_NOT_DONE) || (m_state == ACTION_FAILED)) && (m_retry >= 0));



}
