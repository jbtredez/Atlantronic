/*
 * strat_priority.h
 *
 *  Created on: May 5, 2017
 *      Author: herzaeone
 */

#ifndef SRC_DISCO_STRAT_STRAT_PRIORITY_H_
#define SRC_DISCO_STRAT_STRAT_PRIORITY_H_

#include "middleware/stratege_machine/stratege.h"
#include "middleware/stratege_machine/action.h"


typedef enum
{
	WAIT = 0,
	IN_PROGRESS,
	DONE,
	FAILED
} ActionState;

typedef struct
{
		Action *pAction;
		uint8_t priority;	// 255 is highest
		ActionState state;
} PrioritisedAction;

class StratPriority:
		public strategy
{
	private:
		PrioritisedAction m_list_action_prioritised[NB_ACTION_MAX];
		int m_stratcolor;

	public:
		void Initialise(int stratcolor);
		int add_action(Action * p_action, uint8_t priority = 1);
		int run();

	private:
		PrioritisedAction *getNextAction();
};

#endif /* SRC_DISCO_STRAT_STRAT_PRIORITY_H_ */
