#include "StateMachineState.h"

StateMachineState::StateMachineState(const char* name,unsigned int stateId)
{
	m_name = name;
	m_stateId = stateId;
}

void StateMachineState::entry(void* /*data*/)
{

}

void StateMachineState::run(void* /*data*/)
{

}

void StateMachineState::exit(void* /*data*/)
{

}

unsigned int StateMachineState::transition(void* /*data*/)
{
	return m_stateId;
}

