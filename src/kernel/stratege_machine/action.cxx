#include "action.h"


////////////////////////////////////////////////
/// function    : action()
/// descrition  : constructor
/// param       : firstcheckpoint : VectPlan first checkpoint of the action
/// retrun      : none
////////////////////////////////////////////////	
Action::Action(VectPlan firstcheckpoint, const char * name)
{
	actionDone = false;
	m_name = name;
	m_try = 0;
	m_actiontype = -1;
	m_firstcheckpoint = firstcheckpoint;
}

int Action::do_action()
{
	log_format(LOG_INFO , m_name);
	return -1;
};
