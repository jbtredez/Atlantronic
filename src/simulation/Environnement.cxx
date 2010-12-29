#include "Environnement.h"
#include <unistd.h>
#include "log.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

Environnement::Environnement() :
	device(NULL),
	driver(NULL),
	smgr(NULL),
	guienv(NULL),
	camera(NULL),
	newtonWorld(NULL)
{
	initLog();
	for(int i=0; i<19; i++)
	{
		pions[i] = NULL;
	}
	table = NULL;
	robot[0] = NULL;
	robot[1] = NULL;

	confCarte[0] = -1;
	confCarte[1] = -1;
	confCarte[2] = -1;

	irrlichtInit();
	newtonInit();
	loadAll();
}

Environnement::~Environnement()
{
	for(int i=0; i<19; i++)
	{
		if(pions[i])
		{
			delete pions[i];
		}
	}

	if(table)
	{
		delete table;
	}

	if(robot[0])
	{
		delete robot[0];
	}

	if(robot[1])
	{
		delete robot[1];
	}

	if(newtonWorld)
	{
		NewtonDestroy(newtonWorld);
	}

	if(device)
	{
		device->drop();
	}
}

void Environnement::irrlichtInit()
{
	device = createDevice(EDT_OPENGL, dimension2d<u32>(1080, 960), 16, false, false, false, this);

	if(device)
	{
		driver = device->getVideoDriver();
		smgr = device->getSceneManager();
		guienv = device->getGUIEnvironment();

		device->setWindowCaption(L"Atlantronic - Simulation");

		camera = smgr->addCameraSceneNodeFPS();
		camera->setFarValue(20000.f);
		camera->setTarget(vector3df(0,0,0));
		camera->setPosition(vector3df(0,1000,-2200));

		smgr->addLightSceneNode(0, vector3df(0,1000,0), SColorf(1.0f,1.0f,1.0f),2000);
		smgr->setAmbientLight(SColorf(0.5f,0.5f,0.5f));
	}
	else
	{
		meslog(_erreur_, "device == NULL");
	}
}

void Environnement::newtonInit()
{
	newtonWorld = NewtonCreate();
	NewtonSetSolverModel(newtonWorld, 0);

	float min[3] = {-2, -2, -2};
	float max[3] = { 2,  2,  2};

	NewtonSetWorldSize( newtonWorld, min, max );

	int i = NewtonMaterialGetDefaultGroupID(newtonWorld);
	NewtonMaterialSetDefaultFriction   (newtonWorld, i, i, 0.8f, 0.4f);
	NewtonMaterialSetDefaultElasticity (newtonWorld, i, i, 0.0f);
	NewtonMaterialSetDefaultSoftness   (newtonWorld, i, i, 0.0f);
	NewtonMaterialSetCollisionCallback (newtonWorld, i, i, NULL, NULL, NULL);
}

void Environnement::loadAll()
{
	if(device && newtonWorld)
	{
		table = new Table(newtonWorld, smgr);
		for(int i=0; i<15; i++)
		{
			pions[i] = new Pion(newtonWorld, smgr, "media/pion.3ds");
		}
		pions[15] = new Pion(newtonWorld, smgr, "media/roi.3ds");
		pions[16] = new Pion(newtonWorld, smgr, "media/roi.3ds");
		pions[17] = new Pion(newtonWorld, smgr, "media/reine.3ds");
		pions[18] = new Pion(newtonWorld, smgr, "media/reine.3ds");

		robot[0] = new Robot(newtonWorld, smgr, "media/robot2011.3ds", "media/fanion_bleu.3ds");
		robot[1] = new Robot(newtonWorld, smgr, "media/robot2011.3ds", "media/fanion_rouge.3ds");
	}
	else
	{
		meslog(_erreur_, "Environnement non initialisé");
	}
}

bool Environnement::OnEvent(const irr::SEvent& event)
{
	(void) event;

	return false;
}

void Environnement::loop()
{
	if(device)
	{
		while( device->run() )
		{
//			NewtonUpdate(newtonWorld, 0.1f);
			driver->beginScene(true, true, SColor(255,100,101,140));
			smgr->drawAll();
			guienv->drawAll();

			driver->endScene();
			usleep(100000);
		}
	}
	else
	{
		meslog(_erreur_, "device == NULL");
	}
}

void Environnement::start(const char* prog1, const char* prog2)
{
	robot[0]->setPosition(-1300, -850,   0);
	robot[1]->setPosition( 1300, -850, 180);
	robot[0]->start("/tmp/robot0", prog1);
//FIXME : voir comment faire / NewtonUpdate
//	robot[1]->start("/tmp/robot1", prog2);
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
			meslog(_erreur_, "configuration (carte verte) erronée : %i", a);
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
			meslog(_erreur_, "configuration (linge 1) erronée : %i", b);
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
			meslog(_erreur_, "configuration (linge 1) erronée : %i", c);
			res = false;
			break;
	}

	for(int i=8; i<15; i++)
	{
		config[i] = config[i-7];
		config[i].X = -config[i].X;
	}

	for(int i=0; i<19; i++)
	{
		config[i].Y = 500;
		pions[i]->setPosition(config[i], vector3df(0,0,0) );
	}

	return true;
}
