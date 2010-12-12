#include "Environnement.h"
#include <unistd.h>

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

Environnement::Environnement()
{
	ready = 0;

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

		table = smgr->getMesh( "media/table.3ds");

		if(table)
		{
			smgr->addAnimatedMeshSceneNode( table );
		}
		else
		{
			device->drop();
		}

		robot = smgr->getMesh( "media/robot2009.3ds");
		if(robot)
		{
			smgr->addAnimatedMeshSceneNode( robot );
		}
		else
		{
			device->drop();
		}
	}
}

bool Environnement::OnEvent(const irr::SEvent& event)
{
	return false;
}

bool Environnement::update()
{
	if( device->run() )
	{
		driver->beginScene(true, true, SColor(255,100,101,140));
		smgr->drawAll();
		guienv->drawAll();

		driver->endScene();

		return true;
	}

	return false;
}

Environnement::~Environnement()
{
	if(device)
	{
		device->drop();
	}
}
