#ifndef TABLE_H
#define TABLE_H

#include <irrlicht/irrlicht.h>
#include <Newton.h>

class Table
{
public:
	Table(NewtonWorld *newtonWorld, irr::scene::ISceneManager* smgr);
	~Table();

protected:
	void addMeshToTreeCollisionStandard(irr::scene::IMeshBuffer* meshBuffer, NewtonCollision* treeCollision, irr::core::vector3df scale);
	void addMeshToTreeCollision2TCoords(irr::scene::IMeshBuffer* meshBuffer, NewtonCollision* treeCollision, irr::core::vector3df scale);
	void addMeshToTreeCollisionTangents(irr::scene::IMeshBuffer* meshBuffer, NewtonCollision* treeCollision, irr::core::vector3df scale);
	NewtonBody* createNewtonTree(NewtonWorld *world, irr::scene::IAnimatedMeshSceneNode *node, int id);

	irr::scene::IAnimatedMesh *mesh;
	irr::scene::IAnimatedMeshSceneNode *node;
	NewtonBody* body;
	irr::core::vector3df origin; // erreur de centrage du 3ds
};

#endif
