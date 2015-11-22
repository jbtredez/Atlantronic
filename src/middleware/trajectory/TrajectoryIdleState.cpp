/*
 * TrajectoryIdleState.cpp
 *
 *  Created on: 17 nov. 2015
 *      Author: jul
 */

#include "TrajectoryIdleState.h"

TrajectoryIdleState::TrajectoryIdleState():
StateMachineState("TRAJECTORY_IdleState",TRAJECTORY_STATE_IDLE)
{
	// TODO Auto-generated constructor stub

}

TrajectoryIdleState::~TrajectoryIdleState()
{
	// TODO Auto-generated destructor stub
}



void TrajectoryIdleState::entry(void* data)
{
	Trajectory* t = (Trajectory*) data;

	//Satisfaction de la volonte operateur ou de la volonte automatique
	t->m_wantedState = TRAJECTORY_STATE_NONE;
}

void TrajectoryIdleState::run(void* data)
{
	Trajectory* t = (Trajectory*) data;



}

unsigned int TrajectoryIdleState::transition(void* data)
{
	Trajectory* j = (Trajectory*) data;

	if(j->m_newRequest)
	{
		return TRAJECTORY_STATE_MOVE_TO_DEST;
	}

	// sinon dans les autres cas on change d'etat
	return m_stateId;
}
