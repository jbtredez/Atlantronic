#include "strat_simple.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "disco/bot.h"
#include "disco/action/Move.h"

////////////////////////////////////////////////
/// function    : Initialise()
/// descrition  : intialise the checkpoint color
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
void StratSimple::Initialise(int stratcolor)
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
int StratSimple::run()
{
	int result =0;
	bool allDone = false;
	if(m_size_actionlist == 0)
	{
		return -1;
	}

	log_format(LOG_INFO , "lancement des actions");

	// Strategie simple  sans recherche de meilleur action
	do
	{
		allDone = true;
		for(int i = 0 ; i < m_size_actionlist ; i++)
		{
			if( m_list_action[i] != 0 )
			{
				int actionTry =  m_list_action[i]->get_try();
				if( actionTry >= 0)
				{
					log_format(LOG_INFO , "Action %d (%s), try %d", i, m_list_action[i]->get_name(), actionTry);

					result = m_list_action[i]->do_action();
					if(result == -1)
					{
						allDone = false;
						m_list_action[i]->m_retry--;
						log_format(LOG_INFO,"Action %s failed, decrementing m_retry to %d", m_list_action[i]->m_name, m_list_action[i]->m_retry);
					}
					else
					{
						m_list_action[i]->m_retry = -1;
					}
				}

			}

		}
		vTaskDelay(1000);
	}while( ! allDone );
	
	// Idle
	/*VectPlan pos(390, -200, 0.0f);
	Move Idle(pos.symetric(m_stratcolor), "Idle");
	Idle.do_action();*/
	return 0;
}

