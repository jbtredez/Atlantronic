#include "action.h"


////////////////////////////////////////////////
/// function    : action()
/// descrition  : constructor
/// param       : firstcheckpoint : VectPlan first checkpoint of the action
/// retrun      : none
////////////////////////////////////////////////	
Action::Action(VectPlan firstcheckpoint, const char * name)
{
	m_name = name;
	m_try = 0;
	m_actiontype = -1;
	m_firstcheckpoint = firstcheckpoint;
	initialized = false;
}

int Action::do_action()
{
	log_format(LOG_INFO , m_name);
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
