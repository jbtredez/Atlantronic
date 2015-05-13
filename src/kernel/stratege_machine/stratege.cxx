
#include "stratege.h"

////////////////////////////////////////////////
/// function    : strategy()
/// descrition  : constructor
/// param       : none
/// retrun      : none
////////////////////////////////////////////////	
strategy::strategy()
{
	m_size_actionlist = 0;
	m_strat_color = 0;
	for(int i= 0;i<NB_ACTION_MAX;i++)
	{
		m_list_action[i] = 0;
	}
}

////////////////////////////////////////////////
/// function    : add_action()
/// descrition  : add action in the list of action
/// param       : pointeur sur l'objet action
/// retrun      : result of the operation -1 fail, 0 sucess
////////////////////////////////////////////////
int strategy::add_action(Action * p_action)
{
	if( (p_action == 0) || (m_size_actionlist >= NB_ACTION_MAX) )
	{
		return -1;
	}

	m_list_action[m_size_actionlist] = p_action;
	m_size_actionlist++;
	return 0;
}
