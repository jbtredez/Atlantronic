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

Robot::Robot(NewtonWorld *newtonWorld, irr::scene::ISceneManager* smgr, const char* fichier, const char* fichierFanion, int Color) :
	cpu(this)
{
	ax12[0] = new Ax12(0);
	cpu.USART3.connect(ax12[0]);
	ax12[1] = new Ax12(1);
	cpu.USART3.connect(ax12[1]);

	float m = 10;
	matrix4 offset;

	memset(X, 0x00, sizeof(X));
	memset(Xold, 0x00, sizeof(Xold));
	model_time = 0;
	pthread_mutex_init(&mutex, NULL);
	pthread_cond_init(&cond, NULL);
	newtonUpdateReq = 0;

	setColor(Color);

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

		float Ixx = m*size.X*size.X/(6.0f*1000.0f*1000.0f);
		float Iyy = Ixx;
		float Izz = Iyy;

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

		offset.setTranslation( (center - origin)/1000.0f );
		NewtonCollision* treeCollision = NewtonCreateBox(newtonWorld,  size.X/1000.0f, size.Y/1000.0f, size.Z/1000.0f, 0, offset.pointer());

		matrix4 identity;

		body = NewtonCreateBody(newtonWorld, treeCollision, identity.pointer());
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
// FIXME : à voir. Pour le moment, robot non perturbé
	force[0] = - force[0];
	force[1] = -9.81 * m;
	force[2] = - force[2];
	NewtonBodyAddForce(nbody, force);
}

void Robot::setColor(int Color)
{
	color = Color;
	cpu.gpioD.setInput(color, GPIO_IDR_IDR9);
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
	Xold[MODEL_POS_X] = X[MODEL_POS_X];
	Xold[MODEL_POS_Y] = X[MODEL_POS_Y];
	Xold[MODEL_POS_ALPHA] = X[MODEL_POS_ALPHA];

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

void Robot::start(const char* pipe_name, const char* prog, int gdb_port)
{
	cpu.start(pipe_name, prog, gdb_port);
}

void Robot::stop()
{
	cpu.stop();
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
	float speedT[3] = {0,0,0};
	float speedA[3] = {0,0,0};
	double v_d;
	double v_r;
	double dx;
	double dy;
	matrix4 m;

	if( cpu.gpioE.getOutput(GPIO_ODR_ODR8) )
	{
		motor[0].pwm = cpu.tim1.getPwm(0) * PARAM_RIGHT_MOT_WHEEL_WAY;
	}
	else
	{
		motor[0].pwm = -cpu.tim1.getPwm(0) * PARAM_RIGHT_MOT_WHEEL_WAY;
	}

	if( cpu.gpioE.getOutput(GPIO_ODR_ODR10) )
	{
		motor[1].pwm = cpu.tim1.getPwm(1) * PARAM_LEFT_MOT_WHEEL_WAY;
	}
	else
	{
		motor[1].pwm = -cpu.tim1.getPwm(1) * PARAM_LEFT_MOT_WHEEL_WAY;
	}

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
		speedT[0] = (X[MODEL_POS_X] - Xold[MODEL_POS_X]) / (0.001f*1000);
		speedT[2] = (X[MODEL_POS_Y] - Xold[MODEL_POS_Y]) / (0.001f*1000);
		speedA[1] = (Xold[MODEL_POS_ALPHA] - X[MODEL_POS_ALPHA]) / 0.001f;
		NewtonBodySetVelocity(body, speedT);
		NewtonBodySetOmega(body, speedA);

		waitNewtonUpdate();

		NewtonBodyGetOmega(body, speedA);
		NewtonBodyGetVelocity(body, speedT);

		NewtonBodyGetMatrix(body, m.pointer());
		vector3df tr = m.getTranslation()*1000;
		vector3df rot = m.getRotationDegrees();

		X[MODEL_POS_X] = tr.X;
		X[MODEL_POS_Y] = tr.Z;
		X[MODEL_POS_ALPHA] = atan2(m[2], m[0]);

		v_r = fmod(X[MODEL_POS_ALPHA] - Xold[MODEL_POS_ALPHA] + M_PI, 2*M_PI);
		if(v_r < 0)
		{
			v_r += M_PI;
		}
		else
		{
			v_r -= M_PI;
		}

		dx = X[MODEL_POS_X] - Xold[MODEL_POS_X];
		dy = X[MODEL_POS_Y] - Xold[MODEL_POS_Y];
		v_d = sqrt(dx*dx + dy*dy);
		double speed_alpha = atan2(dy, dx);
		if( fabs(fmod(speed_alpha - Xold[MODEL_POS_ALPHA] + M_PI, 2*M_PI) - M_PI) > M_PI/2)
		{
			v_d = - v_d;
		}
/*
		if(color == 1)
			printf("%f\t%f\t%f ------ %f\t%f\t%f -------- %f\t%f\t%f\n", X[MODEL_POS_ALPHA], Xold[MODEL_POS_ALPHA], v_r, dx, dy, v_d, X[MODEL_POS_X], X[MODEL_POS_Y], X[MODEL_POS_ALPHA]);
*/
		X[MODEL_ODO_RIGHT_THETA] = Xold[MODEL_ODO_RIGHT_THETA] + (v_d / PARAM_DIST_ODO_GAIN + v_r / PARAM_ROT_ODO_GAIN )/(2 * PARAM_RIGHT_ODO_WHEEL_RADIUS * PARAM_RIGHT_ODO_WHEEL_WAY);
		X[MODEL_ODO_LEFT_THETA] = Xold[MODEL_ODO_LEFT_THETA] + (v_d / PARAM_DIST_ODO_GAIN - v_r / PARAM_ROT_ODO_GAIN )/(2 * PARAM_LEFT_ODO_WHEEL_RADIUS  * PARAM_LEFT_ODO_WHEEL_WAY);

		double v_right = (v_d / PARAM_DIST_MOD_GAIN + v_r / PARAM_ROT_MOD_GAIN) / 2.0f;
		double v_left = (v_d / PARAM_DIST_MOD_GAIN - v_r / PARAM_ROT_MOD_GAIN ) / 2.0f;

		X[MODEL_MOT_RIGHT_W] = v_right / (PARAM_RIGHT_MOT_WHEEL_RADIUS) * 1000.0f;
		X[MODEL_MOT_LEFT_W]  = v_left  / (PARAM_LEFT_MOT_WHEEL_RADIUS ) * 1000.0f;

		X[MODEL_MOT_RIGHT_THETA] = Xold[MODEL_MOT_RIGHT_THETA] + X[MODEL_MOT_RIGHT_W] * 0.001f;
		X[MODEL_MOT_LEFT_THETA] = Xold[MODEL_MOT_LEFT_THETA] + X[MODEL_MOT_LEFT_W] * 0.001f;

		memcpy(Xold, X, sizeof(X));

//		fprintf(model_log_file, "%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", model_time, X[MODEL_MOT_RIGHT_I], X[MODEL_MOT_RIGHT_THETA], X[MODEL_MOT_RIGHT_W], X[MODEL_MOT_LEFT_I], X[MODEL_MOT_LEFT_THETA], X[MODEL_MOT_LEFT_W], X[MODEL_POS_X], X[MODEL_POS_Y], X[MODEL_POS_ALPHA], X[MODEL_ODO_RIGHT_THETA], X[MODEL_ODO_LEFT_THETA]);
//		printf( "%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", model_time, X[MODEL_MOT_RIGHT_I], X[MODEL_MOT_RIGHT_THETA], X[MODEL_MOT_RIGHT_W], X[MODEL_MOT_LEFT_I], X[MODEL_MOT_LEFT_THETA], X[MODEL_MOT_LEFT_W], X[MODEL_POS_X], X[MODEL_POS_Y], X[MODEL_POS_ALPHA], X[MODEL_ODO_RIGHT_THETA], X[MODEL_ODO_LEFT_THETA]);
	}

	cpu.tim4.setEncoder((uint16_t) X[MODEL_ODO_RIGHT_THETA]);
	cpu.tim2.setEncoder((uint16_t) X[MODEL_ODO_LEFT_THETA]);

//	if(color == 1)
//	{
//		printf("pos : %f\t%f\t%f\n", X[MODEL_POS_X], X[MODEL_POS_Y], X[MODEL_POS_ALPHA]);
//		printf("codeurs : %i     %i\n", (uint16_t) X[MODEL_ODO_RIGHT_THETA], (uint16_t) X[MODEL_ODO_LEFT_THETA]);
//	}
}

void Robot::compute_dx(double *x, double* dx)
{
	motor[0].compute_dx(x, dx); // moteur droit
	motor[1].compute_dx(x+3, dx+3); // moteur gauche

	double v_right = x[MODEL_MOT_RIGHT_W] * PARAM_RIGHT_MOT_WHEEL_RADIUS;
	double v_left = x[MODEL_MOT_LEFT_W] * PARAM_LEFT_MOT_WHEEL_RADIUS;

	double v_d = PARAM_DIST_MOD_GAIN * (v_right + v_left);
	double v_r = PARAM_ROT_MOD_GAIN * (v_right - v_left);

	dx[MODEL_POS_X] = v_d * cos(x[MODEL_POS_ALPHA]);
	dx[MODEL_POS_Y] = v_d * sin(x[MODEL_POS_ALPHA]);
	dx[MODEL_POS_ALPHA] = v_r;

	dx[MODEL_ODO_RIGHT_THETA] = (v_d / PARAM_DIST_ODO_GAIN + v_r / PARAM_ROT_ODO_GAIN )/(2 * PARAM_RIGHT_ODO_WHEEL_RADIUS * PARAM_RIGHT_ODO_WHEEL_WAY);
	dx[MODEL_ODO_LEFT_THETA] = (v_d / PARAM_DIST_ODO_GAIN - v_r / PARAM_ROT_ODO_GAIN )/(2 * PARAM_LEFT_ODO_WHEEL_RADIUS  * PARAM_LEFT_ODO_WHEEL_WAY);
}
