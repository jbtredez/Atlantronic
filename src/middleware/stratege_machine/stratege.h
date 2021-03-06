#ifndef STRATEGY_H
#define STRATEGY_H

//! @file strategy.h
//! @brief mother class of STRATEGY
//! @author Atlantronic

#include "action.h"

#define NB_ACTION_MAX  20

class strategy
{
	public:
		//color of the party
		int m_strat_color ;

		Action * m_list_action[NB_ACTION_MAX];
		int m_size_actionlist;

		////////////////////////////////////////////////
		/// function    : strategy()
		/// descrition  : constructor
		/// param       : none
		/// retrun      : none
		////////////////////////////////////////////////
		strategy();

		////////////////////////////////////////////////
		/// function    : add_action()
		/// descrition  : add action in the list of action
		/// param       : pointeur sur l'objet action
		/// retrun      : result of the operation -1 fail, 0 sucess
		////////////////////////////////////////////////
		int add_action(Action * p_action);

		////////////////////////////////////////////////
		/// function    : run()
		/// descrition  : execute the strategy
		/// param       : none
		/// retrun      : -1 if fail or 0 if sucess
		////////////////////////////////////////////////
		int run(){return 0;};

	
		////////////////////////////////////////////////
		/// function    : affiche()
		/// descrition  : print the strategy
		/// param       : none
		/// retrun      : -1 if fail or 0 if sucess
		////////////////////////////////////////////////
		void affiche();
};

#endif
