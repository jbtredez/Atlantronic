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
	m_retry = 10;
	m_state = FISHES_IDLE;
	m_stratColor = 0;
	m_finshingPos[0] = VectPlan(940, -848.5, M_PI);
	m_finshingPos[1] = VectPlan(840, -848.5, M_PI);
	m_finshingPos[2] = VectPlan(755, -848.5, M_PI);
	m_droppingPos[0] = VectPlan(400, -848.5, M_PI);
	m_droppingPos[1] = VectPlan(300, -848.5, M_PI);
	m_droppingPos[2] = VectPlan(200, -848.5, M_PI);
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
	int fishesPos = 0;
	int i = 0;

	while(run)
	{
		switch(m_state)
		{
			case FISHES_IDLE:
				actionStart.x = 1075;
				actionStart.y = -100;
				actionStart = actionStart.symetric(m_stratColor);
				vTaskDelay(150);
				trajectory.goToNearXy(actionStart.x, actionStart.y, 100, WAY_ANY, AVOIDANCE_GRAPH);
				if (trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000))
				{
					run = 0;
					result = 0;
					break;
				}
				m_state = FISHES_GRAB;
				break;

			case FISHES_GRAB:
				m_fishingAction.m_firstcheckpoint = m_finshingPos[fishesPos];
				result = m_fishingAction.do_action();
				if (result == -1)
				{
					m_state = FISHES_IDLE;
					run = 0;
				} else
				{
					m_state = FISHES_TRANSIT;
				}
				break;

			case FISHES_TRANSIT:
				actionStart.x = 572;
				actionStart.y = -830;
				actionStart.theta = M_PI;
				trajectory.goTo(actionStart.symetric(m_stratColor), WAY_FORWARD, AVOIDANCE_STOP);
				trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000);
				m_state = FISHES_DROP;
				break;

			case FISHES_DROP:
				m_dropFishesAction.m_firstcheckpoint = m_droppingPos[fishesPos];
				log_format(LOG_INFO, "drop a %d", fishesPos);
				fishesPos++;
				if (fishesPos == 3)
				{
					fishesPos = 1;
				}

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
				i++;
				if (i == 10)
				{
					run = 0;
					result = 0;
					break;
				}

				actionStart.x = 572;
				actionStart.y = -830;
				actionStart.theta = M_PI;
				trajectory.goTo(actionStart.symetric(m_stratColor), WAY_ANY, AVOIDANCE_STOP);
				trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000);
				m_state = FISHES_GRAB;
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

