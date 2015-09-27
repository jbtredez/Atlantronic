#include "kernel/log.h"
#include "middleware/motion/trajectory.h"
#include "disco/robot_state.h"
#include "elevator.h"

#include "disco/finger.h"
#include "kernel/location/location.h"
#include "disco/action/dropzone.h"

DropZone::DropZone(VectPlan firstcheckpoint, const char * name, RobotState * elevator):Action(firstcheckpoint, name)
{
	if(elevator != 0)
	{
		m_elevator = elevator;	
	}
	m_dropposition = 350;
	m_actiontype = ACTION_DROPZONE;
}


////////////////////////////////////////////////
/// function    : Exit()
/// descrition  : action effectue pour sortir de l'action proprement
/// param       : none
/// retrun      : none
////////////////////////////////////////////////
void DropZone::Exit()
{
	do 
	{
		vTaskDelay(100);
		trajectory.goToNear(m_firstcheckpoint, DROPSTART_APPROX_DIST, WAY_ANY, AVOIDANCE_STOP) ;

	}while( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0) ;
	
}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int DropZone::do_action()
{
	int result = -1;
	int essai = 0;

	if(m_elevator == 0)
	{
		return -1;
	}

	Action::do_action();

	Eelevator_state state = m_elevator->getelevatorstate();
	VectPlan dropzone = m_firstcheckpoint;
	//Si la reserve est vide ou contient seulement une lumière on quitte la fonction
	if( ( state == ELEVATOR_EMPTY) || (state == ELEVATOR_LIGHT) )
	{
		return -1;
	}

	//On va dans la zone d'entrée de départ
	do 
	{
		trajectory.goToNear(m_firstcheckpoint, DROPSTART_APPROX_DIST, WAY_FORWARD, AVOIDANCE_STOP) ;
		
		if ( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) == 0)
		{
			result = 0;
		}

		essai++;

		if(essai == 3)
		{
			return -1;
		}
		vTaskDelay(100);
	} while( result ==-1 ) ;

	if(dropzone.x <0 )
	{
		dropzone.x -= m_dropposition;
		dropzone.theta = 3.14;
	}
	else
	{
		dropzone.x += m_dropposition;
		dropzone.theta = 0.0f;
	}

	essai = 0;
	result = -1;
	do 
	{
		//on se déplace dans la zone de départ
		trajectory.goToNear(dropzone, DROPSTART_APPROX_DIST, WAY_FORWARD, AVOIDANCE_STOP) ;

		if ( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) == 0 )
		{
			result = 0;
		}

		// on n'a pas besoin de precision ici. Si on est sur la zone, c'est bon.
		// TODO on regarde le target not reached (pour eliminer collision) => verifier position actuelle a la place
		if( TRAJECTORY_STATE_TARGET_NOT_REACHED == trajectory.getState() )
		{
			result = 0;
		}

		essai++;

		if(3 == essai)
		{
			Exit();
			return -1;
		}
		vTaskDelay(100);
	} while(  0 != result ) ;

	essai = 0;

	finger_set_pos(FINGER_OPEN, FINGER_OPEN);
	vTaskDelay(400);

	Exit();

	vTaskDelay(100);

	finger_set_pos(FINGER_CLOSE, FINGER_OPEN);
	vTaskDelay(100);
	finger_set_pos(FINGER_CLOSE, FINGER_HALF_CLOSE);
	m_elevator->setnumberelement(0);
 	m_elevator->setelevatorstate(ELEVATOR_EMPTY);
	m_dropposition = m_dropposition - 150;

// on retourne -1 pour que l'action reste dans la pile de strat
	return -1;//result;
}
