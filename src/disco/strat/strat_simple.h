#ifndef STRAT_SIMPLE_H
#define STRAT_SIMPLE_H

#include "middleware/stratege_machine/stratege.h"

class StratSimple : public strategy
{
	public:
		int m_stratcolor;
		////////////////////////////////////////////////
		/// function    : Initialise()
		/// descrition  : intialise the checkpoint color
		/// param       : none
		/// retrun      : -1 if fail or 0 if sucess
		////////////////////////////////////////////////
		void Initialise(int stratcolor);
	
		////////////////////////////////////////////////
		/// function    : run()
		/// descrition  : execute the strategie
		/// param       : none
		/// retrun      : -1 if fail or 0 if sucess
		////////////////////////////////////////////////
		int run();
}; 

#endif
