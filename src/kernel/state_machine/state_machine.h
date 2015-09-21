#ifndef STATE_MACHINE
#define STATE_MACHINE

#include <stdint.h>

class StateMachineState
{
	public:
		StateMachineState(const char* name);
		virtual void entry(void* data);
		virtual void run(void* data);
		virtual unsigned int transition(void* data, unsigned int currentState);

		const char* m_name;
};

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
