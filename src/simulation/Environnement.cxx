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

		robot2Mesh = smgr->getMesh( "media/robot2011.3ds");

		if(robot2Mesh)
		{
			robot2 = smgr->addAnimatedMeshSceneNode( robot2Mesh );
		}
		else
		{
			device->drop();
			init = false;
		}

		pionMesh = smgr->getMesh( "media/pion.3ds");

		if(pionMesh)
		{
			for(int i=0; i< 15; i++)
			{
				pions[i] = smgr->addAnimatedMeshSceneNode( pionMesh );
			}
		}
		else
		{
			device->drop();
			init = false;
		}

		roiMesh = smgr->getMesh( "media/roi.3ds");

		if(roiMesh)
		{
			roi[0] = smgr->addAnimatedMeshSceneNode( roiMesh );
			roi[1] = smgr->addAnimatedMeshSceneNode( roiMesh );
		}
		else
		{
			device->drop();
			init = false;
		}

		reineMesh = smgr->getMesh( "media/reine.3ds");

		if(roiMesh)
		{
			reine[0] = smgr->addAnimatedMeshSceneNode( reineMesh );
			reine[1] = smgr->addAnimatedMeshSceneNode( reineMesh );
		}
		else
		{
			device->drop();
			init = false;
		}

		m_ready = init;
	}
}

void Environnement::start()
{
	setPositionRobot1(-1300, -850, 0);
//	setPositionRobot1(0, 0, 0);
	setPositionRobot2( 1300, -850, 180);
	robotQemu1.start();
}

bool Environnement::configure(unsigned int a, unsigned int b, unsigned int c)
{
	bool res = true;

	confCarte[0] = a;
	confCarte[1] = b;
	confCarte[2] = c;

	config[0] = vector3df(0,0,0);

	#define V1  vector3df(1300, 0,  760)
	#define V2  vector3df(1300, 0,  480)
	#define V3  vector3df(1300, 0,  200)
	#define V4  vector3df(1300, 0,  -80)
	#define V5  vector3df(1300, 0, -360)

	switch(a)
	{
		case 0:
			config[15] = V1;
			config[17] = V2;
			config[1]  = V3;
			config[2]  = V4;
			config[3]  = V5;
			break;
		case 1:
			config[15] = V1;
			config[1]  = V2;
			config[17] = V3;
			config[2]  = V4;
			config[3]  = V5;
			break;
		case 2:
			config[15] = V1;
			config[1]  = V2;
			config[2]  = V3;
			config[17] = V4;
			config[3]  = V5;
			break;
		case 3:
			config[15] = V1;
			config[1]  = V2;
			config[2]  = V3;
			config[3]  = V4;
			config[17] = V5;
			break;
		case 4:
			config[17] = V1;
			config[15] = V2;
			config[1]  = V3;
			config[2]  = V4;
			config[3]  = V5;
			break;
		case 5:
			config[1]  = V1;
			config[15] = V2;
			config[17] = V3;
			config[2]  = V4;
			config[3]  = V5;
			break;
		case 6:
			config[1]  = V1;
			config[15] = V2;
			config[2]  = V3;
			config[17] = V4;
			config[3]  = V5;
			break;
		case 7:
			config[1]  = V1;
			config[15] = V2;
			config[2]  = V3;
			config[3]  = V4;
			config[17] = V5;
			break;
		case 8:
			config[17] = V1;
			config[1]  = V2;
			config[15] = V3;
			config[2]  = V4;
			config[3]  = V5;
			break;
		case 9:
			config[1]  = V1;
			config[17] = V2;
			config[15] = V3;
			config[2]  = V4;
			config[3]  = V5;
			break;
		case 10:
			config[1]  = V1;
			config[2]  = V2;
			config[15] = V3;
			config[17] = V4;
			config[3]  = V5;
			break;
		case 11:
			config[1]  = V1;
			config[2]  = V2;
			config[15] = V3;
			config[3]  = V4;
			config[17] = V5;
			break;
		case 12:
			config[17] = V1;
			config[1]  = V2;
			config[2]  = V3;
			config[15] = V4;
			config[3]  = V5;
			break;
		case 13:
			config[1]  = V1;
			config[17] = V2;
			config[2]  = V3;
			config[15] = V4;
			config[3]  = V5;
			break;
		case 14:
			config[1]  = V1;
			config[2]  = V2;
			config[17] = V3;
			config[15] = V4;
			config[3]  = V5;
			break;
		case 15:
			config[1]  = V1;
			config[2]  = V2;
			config[3]  = V3;
			config[15] = V4;
			config[17] = V5;
			break;
		case 16:
			config[17] = V1;
			config[1]  = V2;
			config[2]  = V3;
			config[3]  = V4;
			config[15] = V5;
			break;
		case 17:
			config[1]  = V1;
			config[17] = V2;
			config[2]  = V3;
			config[3]  = V4;
			config[15] = V5;
			break;
		case 18:
			config[1]  = V1;
			config[2]  = V2;
			config[17] = V3;
			config[3]  = V4;
			config[15] = V5;
			break;
		case 19:
			config[1]  = V1;
			config[2]  = V2;
			config[3]  = V3;
			config[17] = V4;
			config[15] = V5;
			break;
		default:
			// TODO : log erreur
			res = false;
			break;
	}

	config[16] = config[15];
	config[16].X = - config[16].X;

	config[18] = config[17];
	config[18].X = - config[18].X;

	#define I11 vector3df(700, 0,   700)
	#define I12 vector3df(700, 0,   350)
	#define I13 vector3df(700, 0,     0)
	#define I14 vector3df(700, 0,  -350)
	#define I15 vector3df(700, 0,  -700)

	switch(b)
	{
		case 0:
			config[4] = I11;
			config[5] = I12;
			break;
		case 1:
			config[4] = I11;
			config[5] = I13;
			break;
		case 2:
			config[4] = I11;
			config[5] = I14;
			break;
		case 3:
			config[4] = I11;
			config[5] = I15;
			break;
		case 4:
			config[4] = I12;
			config[5] = I13;
			break;
		case 5:
			config[4] = I12;
			config[5] = I14;
			break;
		case 6:
			config[4] = I12;
			config[5] = I15;
			break;
		case 7:
			config[4] = I13;
			config[5] = I14;
			break;
		case 8:
			config[4] = I13;
			config[5] = I15;
			break;
		case 9:
			config[4] = I14;
			config[5] = I15;
			break;
		default:
			// TODO : log erreur
			res = false;
			break;
	}

	#define I21 vector3df(350, 0,   700)
	#define I22 vector3df(350, 0,   350)
	#define I23 vector3df(350, 0,     0)
	#define I24 vector3df(350, 0,  -350)
	#define I25 vector3df(350, 0,  -700)

	switch(c)
	{
		case 0:
			config[6] = I21;
			config[7] = I22;
			break;
		case 1:
			config[6] = I21;
			config[7] = I23;
			break;
		case 2:
			config[6] = I21;
			config[7] = I24;
			break;
		case 3:
			config[6] = I21;
			config[7] = I25;
			break;
		case 4:
			config[6] = I22;
			config[7] = I23;
			break;
		case 5:
			config[6] = I22;
			config[7] = I24;
			break;
		case 6:
			config[6] = I22;
			config[7] = I25;
			break;
		case 7:
			config[6] = I23;
			config[7] = I24;
			break;
		case 8:
			config[6] = I23;
			config[7] = I25;
			break;
		case 9:
			config[6] = I24;
			config[7] = I25;
			break;
		default:
			// TODO : log erreur
			res = false;
			break;
	}

	for(int i=8; i<15; i++)
	{
		config[i] = config[i-7];
		config[i].X = -config[i].X;
	}

	for(int i=0; i<15; i++)
	{
		pions[i]->setPosition(config[i]);
	}

	roi[0]->setPosition(config[15]);
	roi[1]->setPosition(config[16]);
	reine[0]->setPosition(config[17]);
	reine[1]->setPosition(config[18]);

	return true;
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
