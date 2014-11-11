#include "state_machine.h"
#include "kernel/log.h"

StateMachine::StateMachine(StateMachineState* States, unsigned int Size)
{
	states = States;
	size = Size;
	currentStateId = 0;
	lastStateId = size;
}

int StateMachine::execute()
{
	if( currentStateId >= size )
	{
		log_format(LOG_ERROR, "invalid state machine id %d >= size %d", currentStateId, size);
		return -1;
	}

	if( lastStateId != currentStateId)
	{
		states[currentStateId].entry();
		lastStateId = currentStateId;
	}

	states[currentStateId].run();

	unsigned int wantedStateId = states[currentStateId].transition(currentStateId);
	if( wantedStateId >= size)
	{
		log_format(LOG_ERROR, "invalid state machine transition to id %d >= size %d", currentStateId, size);
		return -1;
	}

	if( wantedStateId != currentStateId )
	{
		currentStateId = wantedStateId;
		log_format(LOG_INFO, "%s", states[currentStateId].name);
		states[currentStateId].entry();
		lastStateId = currentStateId;
	}

	return 0;
}
