#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

//! @file robot_state.h
//! @brief describe the state of the robot
//! @author Atlantronic

#include "kernel/log.h"

enum Eelevator_state
{
	ELEVATOR_EMPTY = 0,
	ELEVATOR_FEET,
	ELEVATOR_LIGHT,
	ELEVATOR_GOBELET,

};
#define MAX_ELEMENT 5

class robotstate 
{
	private:
	//indicats what's in the elevator
	Eelevator_state m_elevator;
	//Number element store in the elevator
	int m_numberelement;



	public:
	Eelevator_state getelevatorstate(){return m_elevator;};
	int getnumberelement(){return m_numberelement;};

	void setelevatorstate(Eelevator_state elevator){m_elevator = elevator;};
	void setnumberelement(int numberelement){m_numberelement = numberelement;};
	robotstate(){m_numberelement = 0; m_elevator = ELEVATOR_EMPTY;} ;

};

#endif
