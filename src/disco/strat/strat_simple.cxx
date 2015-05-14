#include "strat_simple.h"
#include "kernel/log.h"
#include "kernel/motion/trajectory.h"

#include "kernel/log.h"

#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"


#include "disco/robot_state.h"
#include "kernel/stratege_machine/action.h"
#include "disco/action/Move.h"

////////////////////////////////////////////////
/// function    : Initialise()
/// descrition  : intialise the checkpoint color
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
void stratsimple::Initialise(int stratcolor)
{
	m_stratcolor = stratcolor;
	log_format(LOG_INFO, "couleur %d", (int)stratcolor );
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
int stratsimple::run()
{
	int result =0;
	bool allDone = false;
	bool busefullaction = false;
	if(m_size_actionlist == 0)
	{
		return -1;
	}

	log_format(LOG_INFO , "lancement des actions");

	// Strategie simple  sans recherche de meilleur action
	do
	{
		allDone = true;
		busefullaction = false;
		for(int i = 0 ; i < m_size_actionlist ; i++)
		{
			if( m_list_action[i] != 0 )
			{
				int actionTry =  m_list_action[i]->get_try();
				if( actionTry >= 0)
				{
					log_format(LOG_INFO , "Action %d, try %d", i, actionTry);

					result = m_list_action[i]->do_action();
					if(result == -1)
					{
						allDone = false;
						m_list_action[i]->m_try++;
						///Cas ou 10 echec
						if( (ACTION_DROP != (m_list_action[i])->get_actiontype() )
						|| ACTION_DROPZONE != (m_list_action[i])->get_actiontype()  )
						{
							busefullaction = true;
						}						
					}
					else
					{
						m_list_action[i]->m_try = -1;
					}
				}

			}

		}
		vTaskDelay(1000);
	


	}while( ! allDone
		&&  false == busefullaction );
	
	// Idle 
	Move Idle(VectPlan(390, 200, 0.0f), "Idle");
	Idle.do_action();
	return 0;
}

