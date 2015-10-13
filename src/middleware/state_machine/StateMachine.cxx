#include "kernel/log.h"
#include "StateMachine.h"

StateMachine::StateMachine(StateMachineState** states, unsigned int size, void* data)
{
	m_data = data;
	m_states = states;
	m_size = size;

	//Initialisation pour effectuer l'entry du premier Etat (0)
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

	unsigned int nextStateId = m_states[m_currentStateId]->transition(m_data);
	if( nextStateId >= m_size)
	{
		log_format(LOG_ERROR, "invalid state machine transition to id %d >= size %d", m_currentStateId, m_size);
		return -1;
	}

	if( nextStateId != m_currentStateId )
	{
		m_currentStateId = nextStateId;
		log_format(LOG_INFO, "%s", m_states[m_currentStateId]->m_name);
		m_states[m_currentStateId]->out(m_data);
		m_lastStateId = m_currentStateId;
	}

	return 0;
}
