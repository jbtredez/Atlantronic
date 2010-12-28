#include "Robot.h"
#include "robot_parameters.h"
#include <memory.h>

//! Multiplicateur de la fréquence du noyau
//! La fréquence du modèle 1000 * MODEL_FREQ_MULT Hz
#define MODEL_FREQ_MULT     10
#define MODEL_KHZ           72000

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

Robot::Robot(NewtonWorld *newtonWorld, irr::scene::ISceneManager* smgr, const char* fichier) :
	newtonWorld(newtonWorld),
	cpu(this)
{
	float m = 10;
	// grosse inertie au pif (c'est pas un pion qui va nous faire tourner)
	float Iyy = 10000;
	float Ixx = 10000;
	float Izz = 10000;
	float dx = 0.3f;
	float dy = 0.3f;
	float dz = 0.3f;
	matrix4 offset;

	memset(X, 0x00, sizeof(X));
	model_time = 0;

	mesh = smgr->getMesh( fichier );

	if(mesh)
	{
		node = smgr->addAnimatedMeshSceneNode( mesh );
		node->setMaterialFlag(EMF_BACK_FACE_CULLING, true);

		offset.makeIdentity();
		offset.setTranslation( vector3df(0, dy/2.0f, 0) );
		NewtonCollision* treeCollision = NewtonCreateBox(newtonWorld,  dx, dy, dz, 0, offset.pointer());

		body = NewtonCreateBody(newtonWorld, treeCollision);
		NewtonReleaseCollision(newtonWorld, treeCollision);
		NewtonBodySetUserData(body, this);
		NewtonBodySetMassMatrix(body, m, Ixx, Iyy, Izz);
		NewtonBodySetTransformCallback(body, transformCallback);
		// FIXME : voir si on met la gravité sur le robot
	}
	else
	{
		meslog(_erreur_, "impossible de charger le fichier 'media/pion.3ds'");
	}
}

Robot::~Robot()
{

}

void Robot::transformCallback(const NewtonBody *nbody, const float* mat, int)
{
	Robot* p = (Robot*) NewtonBodyGetUserData(nbody);
	matrix4 m;
	memcpy(m.pointer(), mat, sizeof(float)*16);
	p->node->setRotation(m.getRotationDegrees());
	// attention, conversion en m => mm
	p->node->setPosition(m.getTranslation()*1000);
}

void Robot::setPosition(float x, float y, float alpha)
{
	// FIXME bug sur l'orientation ?

	matrix4 m;
	// newton en m et non mm
	m.setTranslation( vector3df(x/1000, 0, y/1000));
	m.setRotationDegrees( vector3df(0, -alpha, 0));
	NewtonBodySetMatrix(body, m.pointer());
	node->setPosition( vector3df(x, 0, y) );
	node->setRotation( vector3df(0, -alpha, 0) );
	X[MODEL_POS_X] = x;
	X[MODEL_POS_Y] = y;
	X[MODEL_POS_ALPHA] = alpha*M_PI/180;
}

void Robot::start(const char* pipe_name, const char* prog)
{
	cpu.start(pipe_name, prog);
}

void Robot::update(	uint64_t vm_clk )
{
	const double te = 1.0f/(1000 * MODEL_FREQ_MULT);

	int i,j;

	double x[MODEL_SIZE];
	double k1[MODEL_SIZE];
	double k2[MODEL_SIZE];
	double k3[MODEL_SIZE];
	double k4[MODEL_SIZE];

	motor[0].pwm = cpu.TIM1.getPwm(0);
	motor[1].pwm = cpu.TIM1.getPwm(1);

	while(model_time < (uint64_t) (vm_clk / MODEL_KHZ))
	{
		for(i=0; i<MODEL_FREQ_MULT; i++)
		{
			double xprec = X[MODEL_POS_X];
			double yprec = X[MODEL_POS_Y];
			// intégration runge-kutta 4
			compute_dx(X, k1);

			for(j=0; j<MODEL_SIZE; j++)
			{
				x[j] = X[j] + k1[j] * te / 2;
			}
			compute_dx(x, k2);

			for(j=0; j<MODEL_SIZE; j++)
			{
				x[j] = X[j] + k2[j] * te / 2;
			}
			compute_dx(x, k3);

			for(j=0; j<MODEL_SIZE; j++)
			{
				x[j] = X[j] + k3[j] * te;
			}
			compute_dx(x, k4);

			for(j=0; j<MODEL_SIZE; j++)
			{
				X[j] += (k1[j] + 2* k2[j] + 2*k3[j] + k4[j]) * te / 6.0f;
			}

			// FIXME : vitesses divisées par 10 pour que ca marche ...
			float speedT[3] = {(X[MODEL_POS_X] - xprec) / (te*10000), 0, (X[MODEL_POS_Y] - yprec) / (te*10000)};
			float speedA[3] = {0,-(k1[MODEL_POS_ALPHA] + 2* k2[MODEL_POS_ALPHA] + 2*k3[MODEL_POS_ALPHA] + k4[MODEL_POS_ALPHA]) / 60.0f,0};
			NewtonBodySetVelocity(body, speedT);
			NewtonBodySetOmega(body, speedA);
			NewtonUpdate(newtonWorld, te);
		}
		model_time++;

//		fprintf(model_log_file, "%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", model_time, X[MODEL_MOT_RIGHT_I], X[MODEL_MOT_RIGHT_THETA], X[MODEL_MOT_RIGHT_W], X[MODEL_MOT_LEFT_I], X[MODEL_MOT_LEFT_THETA], X[MODEL_MOT_LEFT_W], X[MODEL_POS_X], X[MODEL_POS_Y], X[MODEL_POS_ALPHA], X[MODEL_ODO_RIGHT_THETA], X[MODEL_ODO_LEFT_THETA]);
//		printf( "%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", model_time, X[MODEL_MOT_RIGHT_I], X[MODEL_MOT_RIGHT_THETA], X[MODEL_MOT_RIGHT_W], X[MODEL_MOT_LEFT_I], X[MODEL_MOT_LEFT_THETA], X[MODEL_MOT_LEFT_W], X[MODEL_POS_X], X[MODEL_POS_Y], X[MODEL_POS_ALPHA], X[MODEL_ODO_RIGHT_THETA], X[MODEL_ODO_LEFT_THETA]);
	}

//	env->update();

	cpu.TIM3.setEncoder((uint16_t) X[MODEL_ODO_RIGHT_THETA]);
	cpu.TIM4.setEncoder((uint16_t) X[MODEL_ODO_LEFT_THETA]);
//	printf("pos : %f\t%f\t%f\n", X[MODEL_POS_X], X[MODEL_POS_Y], X[MODEL_POS_ALPHA]);
//	printf("codeurs : %i     %i\n", (uint16_t) X[MODEL_ODO_RIGHT_THETA], (uint16_t) X[MODEL_ODO_LEFT_THETA]);
}

void Robot::compute_dx(double *x, double* dx)
{
	motor[0].compute_dx(x, dx); // moteur droit
	motor[1].compute_dx(x+3, dx+3); // moteur gauche

	double v_right = x[MODEL_MOT_RIGHT_W] * PARAM_RIGHT_MOT_WHEEL_RADIUS * PARAM_RIGHT_MOT_WHEEL_WAY;
	double v_left = x[MODEL_MOT_LEFT_W] * PARAM_LEFT_MOT_WHEEL_RADIUS  * PARAM_LEFT_MOT_WHEEL_WAY;

	double v_d = PARAM_DIST_MOD_GAIN * (v_right + v_left);
	double v_r = PARAM_ROT_MOD_GAIN * (v_right - v_left);

	dx[MODEL_POS_X] = v_d * cos(x[MODEL_POS_ALPHA]);
	dx[MODEL_POS_Y] = v_d * sin(x[MODEL_POS_ALPHA]);
	dx[MODEL_POS_ALPHA] = v_r;

	dx[MODEL_ODO_RIGHT_THETA] = (v_d / PARAM_DIST_ODO_GAIN + v_r / PARAM_ROT_ODO_GAIN )/(2 * PARAM_RIGHT_ODO_WHEEL_RADIUS * PARAM_RIGHT_ODO_WHEEL_WAY);
	dx[MODEL_ODO_LEFT_THETA] = (v_d / PARAM_DIST_ODO_GAIN - v_r / PARAM_ROT_ODO_GAIN )/(2 * PARAM_LEFT_ODO_WHEEL_RADIUS  * PARAM_LEFT_ODO_WHEEL_WAY);
}
