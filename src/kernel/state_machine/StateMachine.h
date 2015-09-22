#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>

#include "StateMachineState.h"

class StateMachine
{
	public:
		StateMachine(StateMachineState** states, unsigned int size, void* data = 0);

		int execute();
		inline int getCurrentState()
		{
			return m_currentStateId;
		}

	protected:
		void* m_data;
		StateMachineState** m_states;
		unsigned int m_size;
		unsigned int m_currentStateId;
		unsigned int m_lastStateId;
};

#endif
