#ifndef STRATEGY_H
#define STRATEGY_H

//! @file strategy.h
//! @brief mother class of STRATEGY
//! @author Atlantronic

#include "action.h"

#define NB_ACTION_MAX  20

class strategy
{
	private:
	//color of the party
	int m_strat_color ;

	action * m_list_action[NB_ACTION_MAX];
	int m_size_actionlist;		
	
	public :


	////////////////////////////////////////////////
	/// function    : strategy()
	/// descrition  : constructor
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////	
	strategy(){ m_size_actionlist = 0; };

	////////////////////////////////////////////////
	/// function    : ~strategy()
	/// descrition  : destructor
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	~strategy(){};

	////////////////////////////////////////////////
	/// function    : add_action()
	/// descrition  : add action in the list of action
	/// param       : pointeur sur l'objet action
	/// retrun      : result of the operation -1 fail, 0 sucess
	////////////////////////////////////////////////
	int add_action(action * p_action);

	////////////////////////////////////////////////
	/// function    : run()
	/// descrition  : execute the strategie
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int run(){return 0;};
	
};

#endif
