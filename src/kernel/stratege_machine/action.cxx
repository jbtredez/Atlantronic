

#include "action.h"





////////////////////////////////////////////////
/// function    : action()
/// descrition  : constructor
/// param       : firstcheckpoint : VectPlan first checkpoint of the action
/// retrun      : none
////////////////////////////////////////////////	
action::action(VectPlan firstcheckpoint)
{
	m_try = 0;
	m_firstcheckpoint = firstcheckpoint;
}
