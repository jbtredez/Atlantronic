#include "strat_simple.h"
#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "disco/wing.h"
#include "action/clapet.h"

#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"

////////////////////////////////////////////////
/// function    : Initialise()
/// descrition  : intialise the checkpoint color
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
void homologation::Initialise(int stratcolor)
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
int homologation::run()
{
	int result =0;
	if(m_size_actionlist == 0)
	{
		return -1;
	}

	// sortie case depart en marche arriere
	VectPlan dest(1000 * m_stratcolor , 0, 0);

	log_format(LOG_INFO , "couleur %d", (int)m_stratcolor);
	do
	{
	    trajectory_goto(dest, WAY_BACKWARD, AVOIDANCE_STOP);
	
	}while( trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0); 
		
	log_format(LOG_INFO , "lancement des actions");
	///Strategie simple  sans recherche de meilleur action
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

