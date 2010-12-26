#include "Environnement.h"
#include <unistd.h>

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

Environnement::Environnement() :
	robotQemu1(this)
{
	m_ready = false;
	bool init = true;
	pthread_mutex_init(&mutexUpdateLoop, NULL);
	pthread_cond_init(&condUpdate, NULL);

	device = createDevice(EDT_OPENGL, dimension2d<u32>(1080, 960),
		16, false, false, false, this);

	if(device)
	{
		driver = device->getVideoDriver();
		smgr = device->getSceneManager();
		guienv = device->getGUIEnvironment();

		device->setWindowCaption(L"Atlantronic - Simulation");
		camera = smgr->addCameraSceneNode();
		camera->setFarValue(20000.f);
		camera->setTarget(vector3df(0,0,0));
		camera->setPosition(vector3df(0,1000,-2200));

		smgr->addLightSceneNode(0, vector3df(0,1000,0), SColorf(1.0f,1.0f,1.0f),2000);
		smgr->setAmbientLight(SColorf(0.3f,0.3f,0.3f));

		tableMesh = smgr->getMesh( "media/table.3ds");

		if(tableMesh)
		{
			table = smgr->addAnimatedMeshSceneNode( tableMesh );
			table->setPosition(vector3df(0, -10, 0));
			table->setRotation(vector3df(0, 180, 0));
		}
		else
		{
			device->drop();
			init = false;
		}

		robot1Mesh = smgr->getMesh( "media/robot2009.3ds");

		if(robot1Mesh)
		{
			robot1 = smgr->addAnimatedMeshSceneNode( robot1Mesh );
		}
		else
		{
			device->drop();
			init = false;
		}

		robot2Mesh = smgr->getMesh( "media/robot2009.3ds");

		if(robot2Mesh)
		{
			robot2 = smgr->addAnimatedMeshSceneNode( robot2Mesh );
		}
		else
		{
			device->drop();
			init = false;
		}

		m_ready = init;

		robotQemu1.start();
	}
}

void Environnement::setPositionRobot1(double x, double y, double alpha)
{
	robot1->setPosition( vector3df(x, 0, y));
	robot1->setRotation( vector3df(0, - alpha, 0) );
	robotQemu1.X[Robot::MODEL_POS_X] = x;
	robotQemu1.X[Robot::MODEL_POS_Y] = y;
	robotQemu1.X[Robot::MODEL_POS_ALPHA] = - alpha * M_PI / 180.0f;
}

void Environnement::setPositionRobot2(double x, double y, double alpha)
{
	robot2->setPosition( vector3df(x, 0, y));
	robot2->setRotation( vector3df(0, -alpha, 0) );
}

bool Environnement::OnEvent(const irr::SEvent& event)
{
	return false;
}

void Environnement::update()
{
	// attention, fonction appelée depuis une autre tache que la tache "loop"
	pthread_mutex_lock(&mutexUpdateLoop);
	robot1->setPosition( vector3df(robotQemu1.X[Robot::MODEL_POS_X], 0, robotQemu1.X[Robot::MODEL_POS_Y]));
	robot1->setRotation( vector3df(0, - robotQemu1.X[Robot::MODEL_POS_ALPHA] * 180 / M_PI, 0) );
	pthread_mutex_unlock(&mutexUpdateLoop);
	// on débloque la tache "loop" pour rafraichir
//	pthread_cond_broadcast(&condUpdate);
}

void Environnement::loop()
{
	struct timespec timeout;
	timeout.tv_sec  = 1;
	timeout.tv_nsec = 100000000;

	//pthread_mutex_lock(&mutexUpdateLoop);
	while( device->run() )
	{
		pthread_mutex_lock(&mutexUpdateLoop);
		driver->beginScene(true, true, SColor(255,100,101,140));
		smgr->drawAll();
		guienv->drawAll();

		driver->endScene();
		pthread_mutex_unlock(&mutexUpdateLoop);
		usleep(100000);
		//pthread_cond_timedwait(&condUpdate, &mutexUpdateLoop, &timeout);
	}
	//pthread_mutex_unlock(&mutexUpdateLoop);
}

Environnement::~Environnement()
{
	if(device)
	{
		device->drop();
	}
}
