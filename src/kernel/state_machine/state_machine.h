#ifndef STATE_MACHINE
#define STATE_MACHINE

typedef void (*StateMachineEntry_t) ();
typedef void (*StateMachineRun_t) ();
typedef unsigned int (*StateMachineTransition_t) (unsigned int currentState);

struct StateMachineState
{
	const char* name;
	StateMachineEntry_t entry;
	StateMachineRun_t run;
	StateMachineTransition_t transition;
};

class StateMachine
{
	public:
		StateMachine(StateMachineState* states, unsigned int size);

		int execute();
		inline int getCurrentState()
		{
			return currentStateId;
		}

	protected:
		StateMachineState* states;
		unsigned int size;
		unsigned int currentStateId;
		unsigned int lastStateId;
};

#endif
