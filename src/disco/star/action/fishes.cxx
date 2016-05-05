#include "fishes.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "disco/star/star.h"
#include "disco/star/servos.h"

////////////////////////////////////////////////
/// function    : Fishes()
/// description  : constructor
/// param       : firstcheckpoint: The action starting point
/// param       : name: The name of the action
/// param       : robot: The instance of the robot which executes the action
/// retrun      : none
////////////////////////////////////////////////
Fishes::Fishes(VectPlan firstcheckpoint, const char * name, RobotState * robot):
	Action(firstcheckpoint, name),
		m_fishingAction(firstcheckpoint, "Fishing Action", robot),
		m_dropFishesAction(firstcheckpoint, "Fishing Action", robot)
{
	if(robot != 0)
	{
		m_robot =  robot;
	}

	m_actiontype = ACTION_FISHES;
	m_retry = 4;
	m_state = FISHES_IDLE;
	m_stratColor = 0;
}

void Fishes::Initialise(int stratcolor)
{
	m_stratColor = stratcolor;
	m_fishingAction.Initialise(m_stratColor);
	m_dropFishesAction.Initialise(m_stratColor);
}

int Fishes::do_action()
{
	int result = 0;
	int run = 1;
	VectPlan actionStart;

	while(run)
	{
		switch(m_state)
		{
			case FISHES_IDLE:
				m_state = FISHES_GRAB;
				break;

			case FISHES_GRAB:
				result = m_fishingAction.do_action();
				if (result == -1)
				{
					m_state = FISHES_IDLE;
					run = 0;
				} else
				{
					m_state = FISHES_DROP;
				}
				break;

			case FISHES_DROP:
				actionStart.x = 400;
				actionStart.y = -850;
				actionStart.theta = M_PI;
				m_dropFishesAction.m_firstcheckpoint = actionStart.symetric(m_stratColor);

				result = m_dropFishesAction.do_action();
				if(result == -1)
				{
					// On change pas d'état (on souhaite redémarrer ici au prochain essai)
					run = 0;
				} else
				{
					m_state = FISHES_FINISHED;
				}
				break;

			case FISHES_FINISHED:
				// Etat puit, on reste là
				run = 0;
				break;

			default:
				result = -1;
				run = 0;
				m_state = FISHES_IDLE;
				log_format(LOG_ERROR, "Fishes: default state");
				break;
		}
	}

	return result;
}

