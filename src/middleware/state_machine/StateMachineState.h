#ifndef STATE_MACHINE_STATE_H
#define STATE_MACHINE_STATE_H

#include <stdint.h>

class StateMachineState
{
	public:
		StateMachineState(const char* name);
		virtual void entry(void* data);
		virtual void run(void* data);
		virtual void exit(void* data);
		virtual unsigned int transition(void* data, unsigned int currentState);

		const char* m_name;
};

#endif
