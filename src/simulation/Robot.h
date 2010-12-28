#ifndef ROBOT_H
#define ROBOT_H

#include <irrlicht/irrlicht.h>
#include <Newton.h>
#include "ArmCm3.h"
#include "Model.h"
#include "Motor.h"

class Robot : public Model
{
public:
	Robot(NewtonWorld *newtonWorld, irr::scene::ISceneManager* smgr, const char* fichier);
	~Robot();

	void start(const char* pipe_name, const char* prog);
	void setPosition(float x, float y, float alpha);

	enum
	{
		MODEL_MOT_RIGHT_I = 0,
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

	double X[MODEL_SIZE]; //!< etat du robot

private:
	static void transformCallback(const NewtonBody *nbody, const float* mat, int);
	void update(uint64_t vm_clk);
	void compute_dx(double *x, double* dx);

	NewtonWorld* newtonWorld;
	ArmCm3 cpu;
	Motor motor[4];
	uint64_t model_time;
	irr::scene::IAnimatedMesh *mesh;
	irr::scene::IAnimatedMeshSceneNode *node;
	NewtonBody* body;
};


#endif