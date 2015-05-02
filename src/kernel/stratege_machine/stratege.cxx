
#include "stratege.h"




////////////////////////////////////////////////
/// function    : add_action()
/// descrition  : add action in the list of action
/// param       : pointeur sur l'objet action
/// retrun      : result of the operation -1 fail, 0 sucess
////////////////////////////////////////////////
int strategy::add_action(action * p_action)
{
	if( (p_action == 0) || (m_size_actionlist >= NB_ACTION_MAX) )
	{
		return -1;
	}

	m_list_action[m_size_actionlist] = p_action;
	m_size_actionlist++;
	return 0;
}
