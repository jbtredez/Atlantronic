#ifndef ENVIRONNEMENT_H
#define ENVIRONNEMENT_H

#include <irrlicht/irrlicht.h>

class Environnement : public irr::IEventReceiver
{
public:
	Environnement();
	~Environnement();

	bool update();

protected:
	bool OnEvent(const irr::SEvent& event);

	irr::IrrlichtDevice* device;
	irr::video::IVideoDriver* driver;
	irr::scene::ISceneManager* smgr;
	irr::gui::IGUIEnvironment* guienv;

	irr::scene::IAnimatedMesh* table;
	irr::scene::IAnimatedMesh* robot;
	irr::scene::ICameraSceneNode* camera;

	int ready;
};

#endif
