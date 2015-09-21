#include "state_machine.h"
#include "kernel/log.h"

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

StateMachine::StateMachine(StateMachineState** states, unsigned int size, void* data)
{
	m_data = data;
	m_states = states;
	m_size = size;
	m_currentStateId = 0;
	m_lastStateId = m_size;
}

int StateMachine::execute()
{
	if( m_currentStateId >= m_size )
	{
		log_format(LOG_ERROR, "invalid state machine id %d >= size %d", m_currentStateId, m_size);
		return -1;
	}

	if( m_lastStateId != m_currentStateId)
	{
		m_states[m_currentStateId]->entry(m_data);
		m_lastStateId = m_currentStateId;
	}

	m_states[m_currentStateId]->run(m_data);

	unsigned int wantedStateId = m_states[m_currentStateId]->transition(m_data, m_currentStateId);
	if( wantedStateId >= m_size)
	{
		log_format(LOG_ERROR, "invalid state machine transition to id %d >= size %d", m_currentStateId, m_size);
		return -1;
	}

	if( wantedStateId != m_currentStateId )
	{
		m_currentStateId = wantedStateId;
		log_format(LOG_INFO, "%s", m_states[m_currentStateId]->m_name);
		m_states[m_currentStateId]->entry(m_data);
		m_lastStateId = m_currentStateId;
	}

	return 0;
}
