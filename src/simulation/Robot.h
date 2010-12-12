#ifndef ROBOT_H
#define ROBOT_H

#include "ArmCm3.h"

class Robot
{
public:
	Robot();
	~Robot();

	enum
	{
		MODEL_MOT_RIGHT_I,
		MODEL_MOT_RIGHT_THETA,
		MODEL_MOT_RIGHT_W,
		MODEL_MOT_LEFT_I,
		MODEL_MOT_LEFT_THETA,
		MODEL_MOT_LEFT_W,
		MODEL_POS_X,
		MODEL_POS_Y,
		MODEL_POS_ALPHA,
		MODEL_ODO_RIGHT_THETA,
		MODEL_ODO_LEFT_THETA,
		MODEL_SIZE
	};

	double x[MODEL_SIZE]; //!< etat du robot

private:
	ArmCm3 cpu;

//	void update_model();
};


#endif
