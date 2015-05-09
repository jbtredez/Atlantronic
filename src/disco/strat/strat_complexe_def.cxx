#include "strat_simple.h"
#include "kernel/log.h"
#include "kernel/motion/trajectory.h"


#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"



////////////////////////////////////////////////
/// function    : Initialise()
/// descrition  : intialise the checkpoint color
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
void stratcomplexedef::Initialise(int stratcolor)
{
	m_stratcolor = stratcolor;
	for(int i = 0 ; i < m_size_actionlist ; i++)
	{
		if( m_list_action[i] != 0 )
		{
			m_list_action[i]->Initialise(stratcolor);
		}
	}

}


////////////////////////////////////////////////
/// function    : run()
/// descrition  : execute the strategie
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int stratcomplexedef::run()
{
	int result =0;
	if(m_size_actionlist == 0)
	{
		return -1;
	}
	///Strategie complexe  sans recherche de meilleur action
	for(int i = 0 ; i < m_size_actionlist ; i++)
	{
		if( m_list_action[i] != 0 )
		{
			if(  m_list_action[i]->get_try() >= 0)
			{
				
				log_format(LOG_INFO , "Action %d",(int)i);

				result = m_list_action[i]->do_action();
				if(result == -1)
				{
					m_list_action[i]->m_try++;
				}
				else
				{
					m_list_action[i]->m_try = -1;
				}
			}
		}
	}
	return 0;

}

