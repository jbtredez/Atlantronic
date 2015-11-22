/*
 * TrajectoryIdleState.h
 *
 *  Created on: 17 nov. 2015
 *      Author: jul
 */

#ifndef TRAJECTORYIDLESTATE_H_
#define TRAJECTORYIDLESTATE_H_

class TrajectoryIdleState: public StateMachineState
{
	public:
		TrajectoryIdleState();
		virtual ~TrajectoryIdleState();
		unsigned int transition(void* data);
		void run(void* data);
		void entry(void* data);
};

#endif /* TRAJECTORYIDLESTATE_H_ */
