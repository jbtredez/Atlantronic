#include "action.h"
#include "action_composite.h"

///////////////////////////////////////////////
/// function    : actioncomposite()
/// descrition  : constructor
/// param       : firstcheckpoint : VectPlan first checkpoint of the action
/// retrun      : none
////////////////////////////////////////////////	
ActionComposite::ActionComposite(VectPlan firstcheckpoint,char * name):Action(firstcheckpoint,name)
{
	m_try = 0;
	for( int i = 0 ; i< NB_MAX_COMPO_ACTION ; i++)
	{
		m_list_action[i] = 0;
	}
	
	m_size_actionlist = 0;
}

////////////////////////////////////////////////
/// function    : add_action()
/// descrition  : add action in the list of action
/// param       : pointeur sur l'objet action
/// retrun      : result of the operation -1 fail, 0 sucess
////////////////////////////////////////////////
int ActionComposite::add_action(Action * p_action)
{
	if( (p_action == 0) || (m_size_actionlist >= NB_MAX_COMPO_ACTION) )
	{
		return -1;
	}

	m_list_action[m_size_actionlist] = p_action;
	m_size_actionlist++;
	return 0;
}

////////////////////////////////////////////////
/// function    : find_action_not_done()
/// descrition  : find the nearest a type action wich are not done
/// param       :int : type of the action
/// retrun      : -1 if fail or Number of the action in the list  if sucess
////////////////////////////////////////////////
Action * ActionComposite::find_action_not_done(int type, VectPlan position)
{
	
	float distance = 99999;
	VectPlan Vectdistance;
	float result = 0;
	Action * p_actiontodo = 0;
	for( int i = 0 ; i< NB_MAX_COMPO_ACTION ; i++)
	{

		
		if( (m_list_action[i]->m_actiontype == type) 
			&& m_list_action[i]->m_retry > -1)
		{
			Vectdistance = (position - (m_list_action[i]->get_firstcheckpoint()));
			distance = Vectdistance.norm();
			if(result < distance )
			{
				distance = result;
				p_actiontodo = m_list_action[i];
			}
 
			

		}

	}
	return p_actiontodo;
}
