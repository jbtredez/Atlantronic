#ifndef FISHES_H_
#define FISHES_H_

#include "middleware/stratege_machine/action.h"
#include "disco/star/robot_state.h"
#include "disco/star/action/fishing.h"
#include "disco/star/action/dropFishes.h"

typedef enum
{
	FISHES_IDLE = 0,
	FISHES_GRAB,
	FISHES_DROP,
	FISHES_FINISHED
} FishesState;

class Fishes  : public Action
{
	private :
		RobotState * m_robot;

	public:
		Fishes(VectPlan firstcheckpoint, const char * name, RobotState * robot);
		void Initialise(int stratcolor);
		int do_action();
		void Exit();

	private:
		int m_stratColor;
		Fishing m_fishingAction;
		DropFishes m_dropFishesAction;
		FishesState m_state;


};

#endif /* FISHES_H_ */
