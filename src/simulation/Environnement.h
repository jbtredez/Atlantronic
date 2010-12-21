#ifndef ENVIRONNEMENT_H
#define ENVIRONNEMENT_H

#include <irrlicht/irrlicht.h>
#include "Robot.h"
#include "Model.h"
#include "EnvironnementInterface.h"
#include <pthread.h>

class Environnement : public irr::IEventReceiver, public EnvironnementInterface
{
public:
	Environnement();
	~Environnement();

	void setPositionRobot1(double x, double y, double alpha);
	void setPositionRobot2(double x, double y, double alpha);

	void loop();

	inline bool ready();

protected:
	bool OnEvent(const irr::SEvent& event);
	void update();

	irr::IrrlichtDevice* device;
	irr::video::IVideoDriver* driver;
	irr::scene::ISceneManager* smgr;
	irr::gui::IGUIEnvironment* guienv;

	irr::scene::IAnimatedMeshSceneNode *table;
	irr::scene::IAnimatedMeshSceneNode *robot1;
	irr::scene::IAnimatedMeshSceneNode *robot2;

	irr::scene::IAnimatedMesh* tableMesh;
	irr::scene::IAnimatedMesh* robot1Mesh;
	irr::scene::IAnimatedMesh* robot2Mesh;
	irr::scene::ICameraSceneNode* camera;

	Robot robotQemu1;

	pthread_mutex_t mutexUpdateLoop;
	pthread_cond_t condUpdate;

	bool m_ready;
};

inline bool Environnement::ready()
{
	return m_ready;
}

#endif
