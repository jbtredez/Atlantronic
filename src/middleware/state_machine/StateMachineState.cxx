#include "StateMachineState.h"

StateMachineState::StateMachineState(const char* name)
{
	m_name = name;
}

void StateMachineState::entry(void* /*data*/)
{

}

void StateMachineState::run(void* /*data*/)
{

}

unsigned int StateMachineState::transition(void* /*data*/, unsigned int currentState)
{
	return currentState;
}
