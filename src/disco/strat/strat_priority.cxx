/*
 * strat_priority.cxx
 *
 *  Created on: May 5, 2017
 *      Author: herzaeone
 */

#include "strat_priority.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "disco/bot.h"
#include "disco/action/Move.h"

void StratPriority::Initialise(int stratcolor)
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


int StratPriority::add_action(Action * p_action, uint8_t priority)
{
	if( (p_action == 0) || (m_size_actionlist >= NB_ACTION_MAX) )
	{
		return -1;
	}


	m_list_action_prioritised[m_size_actionlist].pAction = p_action;
	m_list_action_prioritised[m_size_actionlist].priority = priority;
	m_list_action_prioritised[m_size_actionlist].state = WAIT;
	m_size_actionlist++;
	return 0;
}

PrioritisedAction StratPriority::getNextAction()
{
	PrioritisedAction nextAction = {NULL, 0, FAILED};
	uint8_t currentPriority = 0;
	int count = 0;

	for (count = 0; count < m_size_actionlist; count++)
	{
		if(m_list_action_prioritised[count].pAction)
		{
			if (m_list_action_prioritised[count].state == WAIT ||
					(m_list_action_prioritised[count].state == FAILED && m_list_action_prioritised[count].pAction->m_retry >= 0))
			{
				if (m_list_action_prioritised[count].priority > currentPriority)
				{
					nextAction = m_list_action_prioritised[count];
					currentPriority = m_list_action_prioritised[count].priority;
				}
			}
		}
	}

	return nextAction;
}


// Recherche de l'action la plus urgente selon la priorité
int StratPriority::run()
{
	int result =0;
	bool allDone = false;
	PrioritisedAction nextAction;
	int i = 0;
	if(m_size_actionlist == 0)
	{
		return -1;
	}

	log_format(LOG_INFO , "lancement des actions");


	do {
		 nextAction = getNextAction();
		 if (nextAction.pAction != NULL)
		 {
			 nextAction.state = IN_PROGRESS;
			 log_format(LOG_INFO , "Action %d (%s), try %d", ++i, nextAction.pAction->get_name(), nextAction.pAction->get_try());

			 result = nextAction.pAction->do_action();

			 if(result == -1)
			{
				nextAction.pAction->m_retry--;
				log_format(LOG_INFO,"Action %s failed, decrementing m_retry to %d", m_list_action[i]->m_name, m_list_action[i]->m_retry);
				nextAction.state = FAILED;
			}
			else
			{
				nextAction.pAction->m_retry = -1;
				nextAction.state = DONE;
			}

		 } else
		 {
			 allDone = true;
		 }
		 vTaskDelay(1000);
	}while(! allDone);

	return 0;
}
