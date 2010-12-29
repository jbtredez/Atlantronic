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

Robot::Robot(NewtonWorld *newtonWorld, irr::scene::ISceneManager* smgr, const char* fichier, const char* fichierFanion) :
	cpu(this)
{
	float m = 10;
	// grosse inertie au pif (c'est pas un pion qui va nous faire tourner)
	float Iyy = 10000;
	float Ixx = 10000;
	float Izz = 10000;
	matrix4 offset;

	memset(X, 0x00, sizeof(X));
	model_time = 0;
	pthread_mutex_init(&mutex, NULL);
	pthread_cond_init(&cond, NULL);
	newtonUpdateReq = 0;

	mesh = smgr->getMesh( fichier );
	fanionMesh = smgr->getMesh( fichierFanion );

	if(!mesh)
	{
		meslog(_erreur_, "impossible de charger le fichier %s", fichier);
	}
	else if(!fanionMesh)
	{
		meslog(_erreur_, "impossible de charger le fichier %s", fichierFanion);
	}
	else
	{
		node = smgr->addAnimatedMeshSceneNode( mesh );
		node->setMaterialFlag(EMF_BACK_FACE_CULLING, true);

		fanionNode = smgr->addAnimatedMeshSceneNode( fanionMesh );
		fanionNode->setMaterialFlag(EMF_BACK_FACE_CULLING, true);

		vector3df min = node->getBoundingBox().MinEdge;
		vector3df max = node->getBoundingBox().MaxEdge;
		vector3df size = max - min;
		vector3df center = (max + min)/2.0f;

		origin.X = 0;
		origin.Y = min.Y;
		origin.Z = center.Z;

		vector3df minFanion = fanionNode->getBoundingBox().MinEdge;
		vector3df maxFanion = fanionNode->getBoundingBox().MaxEdge;
		vector3df sizeFanion = maxFanion - minFanion;
		vector3df centerFanion = (maxFanion + minFanion)/2.0f;

		fanionOffset.X = 0 - centerFanion.X;
		fanionOffset.Y = size.Y - minFanion.Y;
		fanionOffset.Z = 10 - centerFanion.Z;

		offset.makeIdentity();
		offset.setTranslation( center/1000.0f );
		NewtonCollision* treeCollision = NewtonCreateBox(newtonWorld,  size.X/1000.0f, size.Y/1000.0f, size.Z/1000.0f, 0, offset.pointer());

		body = NewtonCreateBody(newtonWorld, treeCollision);
		NewtonReleaseCollision(newtonWorld, treeCollision);
		NewtonBodySetUserData(body, this);
		NewtonBodySetMassMatrix(body, m, Ixx, Iyy, Izz);
		float dir[3] = {0,1,0};
		NewtonConstraintCreateUpVector(newtonWorld, dir, body);
		NewtonBodySetTransformCallback(body, transformCallback);
		NewtonBodySetForceAndTorqueCallback(body, forceAndTorqueCallback);
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
	vector3df tr = m.getTranslation()*1000;
	vector3df rot = m.getRotationDegrees();
	p->node->setRotation(rot);
	// attention, conversion en m => mm
	p->node->setPosition(tr - p->origin);
	p->fanionNode->setRotation(rot);
	p->fanionNode->setPosition( tr + p->fanionOffset);
}

void Robot::forceAndTorqueCallback(const NewtonBody *nbody, float, int)
{
	float m, ixx, iyy, izz;
	matrix4 mat;
	float force[3];

	NewtonBodyGetMassMatrix(nbody, &m, &ixx, &iyy, &izz);
	NewtonBodyGetForce(nbody, force);
	// TODO : possibilité de compenser la force latérale du robot (repère robot)
	force[0] = 0;
	force[1] = -9.81 * m;
	force[2] = 0;
	NewtonBodyAddForce(nbody, force);
}

void Robot::setPosition(float x, float y, float alpha)
{
	// TODO : voir bug avec alpha == 180 ou alpha == -180
	// bug lié à la réalisation
	//   matrix4 m;
	//   m.setRotationDegrees( vector3df(0, -alpha, 0));
	//   m.getRotationDegrees().Y pas bon à 180 degré près
	if(alpha == 180)
	{
		alpha -= 0.01;
	}

	matrix4 m;
	X[MODEL_POS_X] = x;
	X[MODEL_POS_Y] = y;
	X[MODEL_POS_ALPHA] = alpha*M_PI/180;

	// newton en m et non mm
	m.setTranslation( vector3df(x/1000, 0, y/1000));
	m.setRotationDegrees( vector3df(0, -alpha, 0));
	NewtonBodySetMatrix(body, m.pointer());
	node->setPosition( vector3df(x, 0, y) - origin);
	node->setRotation( vector3df(0, -alpha, 0) );
	fanionNode->setRotation( vector3df(0, -alpha, 0) );
	fanionNode->setPosition( vector3df(x, 0, y) + fanionOffset);
}

void Robot::waitNewtonUpdate()
{
	bool wait;
	pthread_mutex_lock(&mutex);
	newtonUpdateReq = 1;
	pthread_cond_broadcast(&cond);
	do
	{
		wait = (newtonUpdateReq != 0);
		if(wait)
		{
			pthread_cond_wait(&cond, &mutex);	
		}
	}while(wait);
	pthread_mutex_unlock(&mutex);
}

void Robot::waitRobotUpdate()
{
	bool wait;
	pthread_mutex_lock(&mutex);
	do
	{
		wait = (newtonUpdateReq == 0);
		if(wait)
		{
			pthread_cond_wait(&cond, &mutex);	
		}
	}while(wait);
	pthread_mutex_unlock(&mutex);
}

void Robot::setNewtonUpdated()
{
	pthread_mutex_lock(&mutex);
	newtonUpdateReq = 0;
	pthread_cond_broadcast(&cond);
	pthread_mutex_unlock(&mutex);
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
	double xprec = X[MODEL_POS_X];
	double yprec = X[MODEL_POS_Y];
	double aprec = X[MODEL_POS_ALPHA];
	float speedT[3] = {0,0,0};
	float speedA[3] = {0,0,0};

	motor[0].pwm = cpu.TIM1.getPwm(0);
	motor[1].pwm = cpu.TIM1.getPwm(1);

	while(model_time < (uint64_t) (vm_clk / MODEL_KHZ))
	{
		for(i=0; i<MODEL_FREQ_MULT; i++)
		{
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
		}
		model_time++;

		// limitation à 1000 Hz de NewtonUpdate (cf doc)
		// attention, conversions en m/s pour newton
		speedT[0] = (X[MODEL_POS_X] - xprec) / (0.001f*1000);
		speedT[2] = (X[MODEL_POS_Y] - yprec) / (0.001f*1000);
		speedA[1] = (aprec - X[MODEL_POS_ALPHA]) / 0.001f;
		NewtonBodySetVelocity(body, speedT);
		NewtonBodySetOmega(body, speedA);

		waitNewtonUpdate();

		NewtonBodyGetOmega(body, speedA);
		xprec = X[MODEL_POS_X];
		yprec = X[MODEL_POS_Y];
		// TODO reprendre les vitesses / newton et mettre à jour  l' état X
//		X[MODEL_POS_ALPHA] = (aprec - speedA[1])*1000;
		aprec = X[MODEL_POS_ALPHA];

//		fprintf(model_log_file, "%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", model_time, X[MODEL_MOT_RIGHT_I], X[MODEL_MOT_RIGHT_THETA], X[MODEL_MOT_RIGHT_W], X[MODEL_MOT_LEFT_I], X[MODEL_MOT_LEFT_THETA], X[MODEL_MOT_LEFT_W], X[MODEL_POS_X], X[MODEL_POS_Y], X[MODEL_POS_ALPHA], X[MODEL_ODO_RIGHT_THETA], X[MODEL_ODO_LEFT_THETA]);
//		printf( "%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", model_time, X[MODEL_MOT_RIGHT_I], X[MODEL_MOT_RIGHT_THETA], X[MODEL_MOT_RIGHT_W], X[MODEL_MOT_LEFT_I], X[MODEL_MOT_LEFT_THETA], X[MODEL_MOT_LEFT_W], X[MODEL_POS_X], X[MODEL_POS_Y], X[MODEL_POS_ALPHA], X[MODEL_ODO_RIGHT_THETA], X[MODEL_ODO_LEFT_THETA]);
	}

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
