#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

//! @file robot_state.h
//! @brief describe the state of the robot
//! @author Atlantronic

#include "kernel/log.h"
#include "disco/star/wing.h"
#include "disco/star/carpet.h"

#define	ACTION_FEET		0
#define ACTION_LIGHT		1
#define	ACTION_CLAP		2
#define	ACTION_SPOTLIGHT	3
#define	ACTION_CARPET		4
#define	ACTION_GOBELET		5
#define	ACTION_STAIR		6
#define	ACTION_DROPZONE		7
#define	ACTION_DROP		8
#define ACTION_MOVE		9
#define	ACTION_ENDLIST		10
#define ACTION_HUT		11

enum Eelevator_state
{
	ELEVATOR_EMPTY = 0,
	ELEVATOR_FEET,
	ELEVATOR_LIGHT,
	ELEVATOR_GOBELET,

};

#define MAX_ELEMENT 5

class RobotState 
{
	private:
		//indicats what's in the elevator
		Eelevator_state m_elevator;
		//Number element store in the elevator
		int m_numberelement;

		wing_cmd_type m_leftwing;
		wing_cmd_type m_rightwing;

		carpet_type m_carpet;

	public:
		Eelevator_state getelevatorstate(){return m_elevator;};
		int getnumberelement(){return m_numberelement;};

		carpet_type getcarpetstate(){return m_carpet;};
		void setcarpetstate(carpet_type carpet){m_carpet = carpet;};


		void setelevatorstate(Eelevator_state elevator){m_elevator = elevator;};
		void setnumberelement(int numberelement){m_numberelement = numberelement;};
		RobotState(){m_numberelement = 0; m_elevator = ELEVATOR_EMPTY;} ;

		wing_cmd_type getwingopen(){return (m_leftwing == WING_OPEN || m_leftwing == WING_OPEN ) ? WING_OPEN : WING_PARK;};
		void setwingstate(wing_cmd_type leftwing,wing_cmd_type rightwing ){m_leftwing = leftwing;m_rightwing = rightwing;};
};

#endif
