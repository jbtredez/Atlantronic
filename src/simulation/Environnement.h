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

	bool configure(unsigned int a, unsigned int b, unsigned int c);

	void start();
	void loop();

	inline bool ready();

protected:
	bool OnEvent(const irr::SEvent& event);
	void update();

	int confCarte[3]; // numéro de la configuration (cf spec TODO mettre à jour les spec)
	irr::core::vector3df config[19];

	irr::IrrlichtDevice* device;
	irr::video::IVideoDriver* driver;
	irr::scene::ISceneManager* smgr;
	irr::gui::IGUIEnvironment* guienv;

	irr::scene::IAnimatedMeshSceneNode *table;
	irr::scene::IAnimatedMeshSceneNode *pions[15];
	irr::scene::IAnimatedMeshSceneNode *roi[2];
	irr::scene::IAnimatedMeshSceneNode *reine[2];
	irr::scene::IAnimatedMeshSceneNode *robot1;
	irr::scene::IAnimatedMeshSceneNode *robot2;

	irr::scene::IAnimatedMesh* tableMesh;
	irr::scene::IAnimatedMesh* pionMesh;
	irr::scene::IAnimatedMesh* roiMesh;
	irr::scene::IAnimatedMesh* reineMesh;
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
