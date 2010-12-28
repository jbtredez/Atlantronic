#ifndef PION_H
#define PION_H

#include <irrlicht/irrlicht.h>
#include <Newton.h>

class Pion
{
public:
	Pion(NewtonWorld *newtonWorld, irr::scene::ISceneManager* smgr,  const char* fichier);
	~Pion();

	void setPosition(irr::core::vector3df pos, irr::core::vector3df rot);

protected:
	static void forceAndTorqueCallback(const NewtonBody *nbody, float, int);
	static void transformCallback(const NewtonBody *nbody, const float* mat, int);

	irr::scene::IAnimatedMesh *mesh;
	irr::scene::IAnimatedMeshSceneNode *node;
	NewtonBody* body;
};

#endif
