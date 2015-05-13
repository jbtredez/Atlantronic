#include "action.h"


////////////////////////////////////////////////
/// function    : action()
/// descrition  : constructor
/// param       : firstcheckpoint : VectPlan first checkpoint of the action
/// retrun      : none
////////////////////////////////////////////////	
action::action(VectPlan firstcheckpoint, const char * name)
{
	m_name = 0;
	if( name != 0)
	{
		m_name = name;	
	}
	m_try = 0;
	m_actiontype = -1;
	m_firstcheckpoint = firstcheckpoint;
}
